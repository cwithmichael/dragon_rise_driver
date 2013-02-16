#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/syslog.h>
#include <sys/fcntl.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#define SNES_USB_BUF_SIZE (1 << 15)
#define SNES_USB_IFQ_MAX_LEN 8

#define UREQ_GET_PORT_STATUS 0x01
#define UREQ_SOFT_RESET      0x02

#define UP     0x7f00
#define DOWN   0x7fff
#define LEFT   0x00ff
#define RIGHT  0xff7f
#define X      0x1f
#define Y      0x8f
#define A      0x2f
#define B      0x4f
#define SELECT 0x10
#define START  0x20
#define LEFT_T 0x01
#define RIGHT_T 0x02



enum{
	SNES_USB_INTR_DT_RD,
	SNES_USB_STATUS_DT_RD,
	SNES_USB_N_TRANSFER
};

struct snes_usb_softc{
	device_t sc_dev;
	struct usb_device *sc_usb_device;
	struct mtx sc_mutex;
	struct usb_callout sc_watchdog;
	uint8_t sc_iface_num;
	struct usb_xfer *sc_transfer[SNES_USB_N_TRANSFER];
	struct usb_fifo_sc sc_fifo;
	struct usb_fifo_sc sc_fifo_no_reset;
	int sc_fflags;
	struct usb_fifo *sc_fifo_open[2];
	uint8_t sc_zero_length_packets;
	uint8_t sc_previous_status;
};

static device_probe_t snes_usb_probe;
static device_attach_t snes_usb_attach;
static device_detach_t snes_usb_detach;

static usb_fifo_open_t snes_usb_open;
static usb_fifo_close_t snes_usb_close;
static usb_fifo_ioctl_t snes_usb_ioctl;
static usb_fifo_cmd_t snes_usb_start_read;
static usb_fifo_cmd_t snes_usb_stop_read;

static void snes_usb_reset(struct snes_usb_softc *);
static void snes_usb_watchdog(void *);

static usb_callback_t snes_usb_read_callback;
static usb_callback_t snes_usb_status_callback;

static struct usb_fifo_methods snes_usb_fifo_methods = {
	.f_open = &snes_usb_open,
	.f_close = &snes_usb_close,
	.f_ioctl = &snes_usb_ioctl,
	.f_start_read =&snes_usb_start_read,
	.f_stop_read = &snes_usb_stop_read,
	.basename[0] = "snes_usb"
};

static const struct usb_config snes_usb_config[SNES_USB_N_TRANSFER] =
	{[SNES_USB_INTR_DT_RD] = {
	.callback = &snes_usb_read_callback,
	.bufsize = sizeof(struct usb_device_request) +1,
	.flags = {.short_xfer_ok = 1, .short_frames_ok = 1, .pipe_bof =1, .proxy_buffer =1},
	.type = UE_INTERRUPT,
	.endpoint = 0x81,
	.direction = UE_DIR_IN
	},
	[SNES_USB_STATUS_DT_RD] = {
	.callback = &snes_usb_status_callback,
	.bufsize = sizeof(struct usb_device_request) + 1,
	.timeout = 1000,
	.type = UE_CONTROL,
	.endpoint = 0x00,
	.direction = UE_DIR_ANY
	}
};

static int
snes_usb_open(struct usb_fifo *fifo, int fflags)
{
	uprintf("OPENING SNES USB\n");
	struct snes_usb_softc *sc = usb_fifo_softc(fifo);
	int error;
	
	if(sc->sc_fflags & fflags){
		snes_usb_reset(sc);
		return (EBUSY);	
	}
	
	mtx_lock(&sc->sc_mutex);
	usbd_xfer_set_stall(sc->sc_transfer[SNES_USB_INTR_DT_RD]);
	mtx_unlock(&sc->sc_mutex);

	error = usb_fifo_alloc_buffer(fifo,
		usbd_xfer_max_len(sc->sc_transfer[SNES_USB_INTR_DT_RD]),
		SNES_USB_IFQ_MAX_LEN);
	if(error)
		return (ENOMEM);

	sc->sc_fifo_open[USB_FIFO_RX] = fifo;
	
	//sc->sc_fflags |= fflags & FREAD;
	return (0);
}	 


static void
snes_usb_reset(struct snes_usb_softc *sc)
{
	struct usb_device_request req;
	int error;
		
	req.bRequest = UREQ_SOFT_RESET;
	USETW(req.wValue, 0);
	USETW(req.wIndex, sc->sc_iface_num);
	USETW(req.wLength, 0);

	mtx_lock(&sc->sc_mutex);

	error = usbd_do_request_flags(sc->sc_usb_device, &sc->sc_mutex,
			&req, NULL, 0, NULL, 2 * USB_MS_HZ);

	if(error){
		usbd_do_request_flags(sc->sc_usb_device, &sc->sc_mutex,
			&req, NULL, 0, NULL, 2 * USB_MS_HZ);
	}

	mtx_unlock(&sc->sc_mutex);
}

static void
snes_usb_close(struct usb_fifo *fifo, int fflags)
{
	uprintf("CLOSING SNES USB\n");
	struct snes_usb_softc *sc = usb_fifo_softc(fifo);
	
	sc->sc_fflags &= ~(fflags & FREAD);
	usb_fifo_free_buffer(fifo);
}

static int
snes_usb_ioctl(struct usb_fifo *fifo, u_long cmd, void *data, int fflags)
{
	uprintf("SOMEONE CAME\n");
	struct snes_usb_softc *sc = usb_fifo_softc(fifo);
	device_printf(sc->sc_dev, "SOMEONE CAME\n");
	return (ENODEV);
}

static void
snes_usb_watchdog(void *arg)
{
	struct snes_usb_softc *sc = arg;
	mtx_assert(&sc->sc_mutex, MA_OWNED);

	if(sc->sc_fflags == 0)
		usbd_transfer_start(sc->sc_transfer[SNES_USB_STATUS_DT_RD]);

	usb_callout_reset(&sc->sc_watchdog, hz, &snes_usb_watchdog, sc);
}

static void 
snes_usb_start_read(struct usb_fifo *fifo)
{
	struct snes_usb_softc *sc = usb_fifo_softc(fifo);
	usbd_transfer_start(sc->sc_transfer[SNES_USB_INTR_DT_RD]);
}

static void
snes_usb_stop_read(struct usb_fifo *fifo)
{
	struct snes_usb_softc *sc = usb_fifo_softc(fifo);
	
	usbd_transfer_stop(sc->sc_transfer[SNES_USB_INTR_DT_RD]);
}

static void
snes_usb_read_callback(struct usb_xfer *transfer, usb_error_t error)
{
	struct snes_usb_softc *sc = usbd_xfer_softc(transfer);
	struct usb_fifo *fifo = sc->sc_fifo_open[USB_FIFO_RX];
	struct usb_page_cache *pc;
	int actual, max;
	uint8_t current_status[8];
	usbd_xfer_status(transfer, &actual, NULL, NULL, NULL);
	if(fifo == NULL)
		return;
	switch(USB_GET_STATE(transfer)){
	
		case USB_ST_TRANSFERRED:
			if(actual == 0){
				if(sc->sc_zero_length_packets == 4)
					/*Throttle transfers. */
					usbd_xfer_set_interval(transfer, 500);
				else
					sc->sc_zero_length_packets++;

			}else{
				/*disable throttling. */
				usbd_xfer_set_interval(transfer, 0);
				sc->sc_zero_length_packets = 0;
			}
			pc = usbd_xfer_get_frame(transfer, 0);
			while(actual >= 4){
			usbd_copy_out(pc, 0, current_status, 8);

			/*BUTTON PRESSED*/
			if(current_status[1] == 0x7f && current_status[2] == 0x7f && current_status[3] == 0x7f
				&& current_status[4] == 0x7f){
				if(current_status[5] == X){
					device_printf(sc->sc_dev,"X BUTTON WAS PRESSED\n");
				}
				if(current_status[5] == Y){
					device_printf(sc->sc_dev,"Y BUTTON WAS PRESSED\n");
				}
		        if(current_status[5] == A){
					device_printf(sc->sc_dev,"A BUTTON WAS PRESSED\n");
				}
				if(current_status[5] == B){
					device_printf(sc->sc_dev,"B BUTTON WAS PRESSED\n");
				}
				if(current_status[6] == SELECT){
					device_printf(sc->sc_dev,"SELECT BUTTON WAS PRESSED\n");
				}
				if(current_status[6] == START){
					device_printf(sc->sc_dev,"START BUTTON WAS PRESSED\n");
				}
				if(current_status[6] == LEFT_T){
					device_printf(sc->sc_dev,"LEFT TRIGGER WAS PRESSED\n");
				}
				if(current_status[6] == RIGHT_T){
					device_printf(sc->sc_dev,"RIGHT TRIGGER WAS PRESSED\n");
				}	
			}

			usb_fifo_put_data_linear(fifo, current_status + 1,  actual, 1);
			actual -=4;
			}
			/*FALLTHROUGH*/

		case USB_ST_SETUP:
setup:
			if(usb_fifo_put_bytes_max(fifo) != 0){
				max = usbd_xfer_max_len(transfer);
				usbd_xfer_set_frame_len(transfer, 0, max);
				usbd_transfer_submit(transfer);
			}
			break;

		default:
			/*disable throttling. */
			usbd_xfer_set_interval(transfer, 0);
			sc->sc_zero_length_packets = 0;

			if(error != USB_ERR_CANCELLED){
				/* Issue a clear-stall request. */
				usbd_xfer_set_stall(transfer);
				goto setup;
			}

			break;
	}
}

static void
snes_usb_status_callback(struct usb_xfer *transfer, usb_error_t error)
{
	struct snes_usb_softc *sc = usbd_xfer_softc(transfer);
	struct usb_device_request req;
	struct usb_page_cache *pc;
	uint8_t current_status, new_status;

	switch(USB_GET_STATE(transfer)){
		case USB_ST_SETUP:
			req.bmRequestType = UT_READ_CLASS_INTERFACE;
			req.bRequest = UREQ_GET_PORT_STATUS;
			USETW(req.wValue, 0);
			req.wIndex[0] = sc->sc_iface_num;
			req.wIndex[1] = 0;
			USETW(req.wLength, 1);

			pc = usbd_xfer_get_frame(transfer, 0);
			usbd_copy_in(pc, 0, &req, sizeof(req));
			usbd_xfer_set_frame_len(transfer, 0, sizeof(req));
			usbd_xfer_set_frame_len(transfer, 1, 1);
			usbd_xfer_set_frames(transfer, 2);
			usbd_transfer_submit(transfer);

			break;
		case USB_ST_TRANSFERRED:
			pc = usbd_xfer_get_frame(transfer, 1);
			usbd_copy_out(pc, 0, &current_status, 1);
			new_status = current_status & ~sc->sc_previous_status;
			sc->sc_previous_status = current_status;
			break;
		default:
			break;
		}
			
}

static int
snes_usb_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	
	if(uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);

	if ((uaa->info.idVendor == 0x0079)){
		return (BUS_PROBE_SPECIFIC); 
	}
	return (ENXIO);
}

static int
snes_usb_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct snes_usb_softc *sc = device_get_softc(dev);
	struct usb_interface_descriptor *idesc;
	struct usb_config_descriptor *cdesc;
	uint8_t alt_index, iface_index = uaa->info.bIfaceIndex;
	int error,unit = device_get_unit(dev);

	sc->sc_dev = dev;
	sc->sc_usb_device = uaa->device;
	device_set_usb_desc(dev);
	mtx_init(&sc->sc_mutex, "snes_usb", NULL, MTX_DEF | MTX_RECURSE);
	usb_callout_init_mtx(&sc->sc_watchdog, &sc->sc_mutex, 0);
	
	idesc = usbd_get_interface_descriptor(uaa->iface);
	alt_index = -1;
	for(;;){
		if(idesc == NULL)
			break;
	
		if((idesc->bDescriptorType == UDESC_INTERFACE) &&
		   (idesc->bLength >= sizeof(*idesc))){
				if(idesc->bInterfaceNumber != uaa->info.bIfaceNum)
					break;
				else{
					alt_index++;
					if(idesc->bInterfaceClass == UICLASS_HID) 
						goto found;
				}
		}
		
		cdesc = usbd_get_config_descriptor(uaa->device);
		idesc = (void *)usb_desc_foreach(cdesc, (void *)idesc);
	}
	goto detach;

found:
		if(alt_index){
			error = usbd_set_alt_interface_index(uaa->device, iface_index, alt_index);
			if(error)
				goto detach;
		}

		sc->sc_iface_num = idesc->bInterfaceNumber;
		
		error = usbd_transfer_setup(uaa->device, &iface_index,
			sc->sc_transfer, snes_usb_config, SNES_USB_N_TRANSFER, sc,
			&sc->sc_mutex);

		if(error)
			goto detach;

		error = usb_fifo_attach(uaa->device, sc, &sc->sc_mutex,
			&snes_usb_fifo_methods, &sc->sc_fifo, unit, -1,
			iface_index, UID_ROOT, GID_OPERATOR, 0644);

		if(error)
			goto detach;

		mtx_lock(&sc->sc_mutex);
		snes_usb_watchdog(sc);
		mtx_unlock(&sc->sc_mutex);
		return (0);

detach:
		snes_usb_detach(dev);
		return (ENOMEM);
}

static int
snes_usb_detach(device_t dev)
{
	struct snes_usb_softc *sc = device_get_softc(dev);

	usb_fifo_detach(&sc->sc_fifo);
	usb_fifo_detach(&sc->sc_fifo_no_reset);
	
	mtx_lock(&sc->sc_mutex);
	usb_callout_stop(&sc->sc_watchdog);
	mtx_unlock(&sc->sc_mutex);

	usbd_transfer_unsetup(sc->sc_transfer, SNES_USB_N_TRANSFER);
	usb_callout_drain(&sc->sc_watchdog);
	mtx_destroy(&sc->sc_mutex);

	return (0);
}

static device_method_t snes_usb_methods[] = {
		//Device interface. 
		DEVMETHOD(device_probe, snes_usb_probe),
		DEVMETHOD(device_attach, snes_usb_attach),
		DEVMETHOD(device_detach, snes_usb_detach),
		{0,0}
};

static driver_t snes_usb_driver = {
	"snes_usb",
	snes_usb_methods,
	sizeof(struct snes_usb_softc)
};

static devclass_t snes_usb_devclass;

DRIVER_MODULE(snes_usb, uhub, snes_usb_driver, snes_usb_devclass, NULL, 0);
MODULE_DEPEND(snes_usb, usb, 1, 1, 1);


