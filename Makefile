KMOD= snes_usb 
SRCS= snes_usb.c bus_if.h device_if.h opt_usb.h

.include <bsd.kmod.mk>
