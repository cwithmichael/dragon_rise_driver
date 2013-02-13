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
#define SNES_USB_IFQ_MAX_LEN 2

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
	.bufsize = SNES_USB_BUF_SIZE,
	.flags = {.short_xfer_ok = 1, .pipe_bof =1, .proxy_buffer =1},
	.type = UE_INTERRUPT,
	.endpoint = UE_ADDR_ANY,
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
	//todo
	return 0;
}	 

static void
snes_usb_reset(struct snes_usb_softc *sc)
{
	//todo
}

static void
snes_usb_close(struct usb_fifo *fifo, int fflags)
{
	//todo
}

static int
snes_usb_ioctl(struct usb_fifo *fifo, u_long cmd, void *data, int fflags)
{
	//todo
	return 0;
}

static void
snes_usb_watchdog(void *arg)
{
	//todo
}

static void 
snes_usb_start_read(struct usb_fifo *fifo)
{
	//todo
}

static void
snes_usb_stop_read(struct usb_fifo *fifo)
{
	//todo
}

static void
snes_usb_read_callback(struct usb_xfer *transfer, usb_error_t error)
{
	//todo
}

static void
snes_usb_status_callback(struct usb_xfer *transfer, usb_error_t error)
{
	//todo
}

static int
snes_usb_probe(device_t dev)
{
	//todo
	return 0;
}

static int
snes_usb_attach(device_t dev)
{
	//todo
	return 0;
}

static int
snes_usb_detach(device_t dev)
{
	//todo
	return 0;
}

static device_method_t snes_usb_methods[] = {
		/*Device interface. */
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

DRIVER_MODULE(snes_usb, uhub, snes_usb_driver, snes_usb_devclass, 0, 0);
MODULE_DEPEND(snes_usb, usb, 1, 1, 1);
MODULE_DEPEND(snes_usb, ucom, 1, 1, 1);


