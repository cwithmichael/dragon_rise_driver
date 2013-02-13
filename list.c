#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <libusb.h>

uint16_t VENDOR = 121;
uint16_t PRODUCT = 17;

libusb_device_handle** find_snes(libusb_device **devs){
	libusb_device *dev;
	libusb_device_handle **handle;
	int i = 0;

	printf("Trying to find Snes controller.\n");
	while((dev = devs[i++]) != NULL){
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if(r < 0){
			fprintf(stderr, "failed");
			return NULL;
		}

		/*printf("%04x:%04x (bus %d, device %d)\n",
			desc.idVendor, desc.idProduct,
			libusb_get_bus_number(dev), libusb_get_device_address(dev));*/
		
		if(desc.idVendor==VENDOR && desc.idProduct==PRODUCT){
			printf("Found SNES\n");
			int usb_open = libusb_open(dev,handle);
			if(usb_open==0){
				printf("open succes\n");
				return handle;
			}
			fprintf(stderr, "open failed: code %d\n", usb_open);
			return handle;
		}

	}
	
	return NULL;
}

int
main(void)
{
	libusb_device **devs;
	int r;
	ssize_t cnt;
	
	r = libusb_init(NULL);
	if(r < 0)
		return r;

	cnt = libusb_get_device_list(NULL, &devs);
	if(cnt <= 0)
		return(int) cnt;

	libusb_device_handle **handle=find_snes(devs);

	if(handle==NULL){
		fprintf(stderr, "No snes found\n");
		libusb_free_device_list(devs,1);
	        libusb_exit(NULL);
                return -1;
	}
	unsigned char ENDPOINT_UP = 0x81;
	int actual_length;
	unsigned char dataUp[64];
	for(int i = 0; i < 10000; i++){
		r = libusb_interrupt_transfer(handle[0], ENDPOINT_UP, dataUp,
									  sizeof(dataUp), &actual_length, 1000);
		if(r == 0 && actual_length == sizeof(dataUp)){
			if(dataUp[0] != 0 || dataUp[1] !=0){
				printf("Data: ");
				for(int j=0; j <64; j++){
					printf("%02x ", dataUp[j]);
				}
				printf("\n");
			}
		sleep(1);
		}

		else{
			printf("Someting went wrong (r == %i, actual_length == %i, sizeof(data) == %lu).\n",
					r, actual_length, sizeof(dataUp));
		}
	}


        libusb_free_device_list(devs,1);
	libusb_exit(NULL);	
	return 0;
}

