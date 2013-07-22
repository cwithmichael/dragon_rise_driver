KMOD= joy 
SRCS= snes_usb.c bus_if.h device_if.h opt_usb.h usbdevs.h


.include <bsd.kmod.mk>
