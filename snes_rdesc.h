/* Fixed report descriptor for PID 0x011 joystick */
#define UHID_SNES_USB_REPORT_DESCR(...)\
	0x05, 0x01,         /*  Usage Page (Desktop),           */\
	0x09, 0x04,         /*  Usage (Joystik),                */\
	0xA1, 0x01,         /*  Collection (Application),       */\
	0xA1, 0x02,         /*      Collection (Logical),       */\
	0x14,               /*          Logical Minimum (0),    */\
	0x75, 0x08,         /*          Report Size (8),        */\
	0x95, 0x03,         /*          Report Count (3),       */\
	0x81, 0x01,         /*          Input (Constant),       */\
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */\
	0x95, 0x02,         /*          Report Count (2),       */\
	0x09, 0x30,         /*          Usage (X),              */\
	0x09, 0x31,         /*          Usage (Y),              */\
	0x81, 0x02,         /*          Input (Variable),       */\
	0x75, 0x01,         /*          Report Size (1),        */\
	0x95, 0x04,         /*          Report Count (4),       */\
	0x81, 0x01,         /*          Input (Constant),       */\
	0x25, 0x01,         /*          Logical Maximum (1),    */\
	0x95, 0x0A,         /*          Report Count (10),      */\
	0x05, 0x09,         /*          Usage Page (Button),    */\
	0x19, 0x01,         /*          Usage Minimum (01h),    */\
	0x29, 0x0A,         /*          Usage Maximum (0Ah),    */\
	0x81, 0x02,         /*          Input (Variable),       */\
	0x95, 0x0A,         /*          Report Count (10),      */\
	0x81, 0x01,         /*          Input (Constant),       */\
	0xC0,               /*      End Collection,             */\
	0xC0                /*  End Collection                  */\

