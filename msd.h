#ifndef MSD_H_
#define MSD_H_

#include "ohci.h"
#include "ucos_ii.h"

typedef struct {

   unsigned char bLength;           
   unsigned char bDescriptorType;   
   unsigned short bcdUSB;            
   unsigned char bDeviceClass;      
   unsigned char bDeviceSubClass;   
   unsigned char bDeviceProtocol;   
   unsigned char bMaxPacketSize0;   
   unsigned short idVendor;          
   unsigned short idProduct;         
   unsigned short bcdDevice;         
   unsigned char iManufacturer;     
   unsigned char iProduct;          
   unsigned char iSerialNumber;     
   unsigned char bNumConfigurations;

} __attribute__ ((packed)) USBDeviceDescriptor; // GCC

typedef struct {
	unsigned char bLength;
	unsigned char bDescriptorType;
	unsigned short wTotalLength;
	unsigned char bNumInterfaces;
	unsigned char bConfigurationValue;
	unsigned char iConfiguration;
	unsigned char bmAttributes;
	unsigned char bMaxPower;
} __attribute__ ((packed)) USBConfigDescriptor;

typedef struct {

   unsigned char bLength;
   unsigned char bDescriptorType;
   unsigned char bInterfaceNumber;
   unsigned char bAlternateSetting;
   unsigned char bNumEndpoints;
   unsigned char bInterfaceClass;
   unsigned char bInterfaceSubClass;
   unsigned char bInterfaceProtocol;
   unsigned char iInterface;

} __attribute__ ((packed)) USBInterfaceDescriptor;

typedef struct {

   unsigned char bLength;
   unsigned char bDescriptorType;
   unsigned char bEndpointAddress;
   unsigned char bmAttributes;
   unsigned short wMaxPacketSize;
   unsigned char bInterval;

} __attribute__ ((packed)) USBEndpointDescriptor;


typedef struct {

   unsigned char bLength;
   unsigned char bDescriptorType;
   unsigned short bcdUSB;
   unsigned char bDeviceClass;
   unsigned char bDeviceSubClass;
   unsigned char bDeviceProtocol;
   unsigned char bMaxPacketSize0;
   unsigned char bNumConfigurations;
   unsigned char bReserved;

} __attribute__ ((packed)) USBDeviceQualifierDescriptor;


typedef struct {

    unsigned char bLength;
    unsigned char bDescriptorType;

} __attribute__ ((packed)) USBGenericDescriptor;


typedef struct {
	void 		*pED;
	unsigned int direction;
	unsigned int speed;
	unsigned int max_packet;
	unsigned int present;
	unsigned int type;
	unsigned int address;
} __attribute__ ((packed)) dev_ed_t;

typedef struct {
	USBDeviceDescriptor 	device_descriptor;
	USBConfigDescriptor 	config_descriptor;
	USBInterfaceDescriptor 	interface_descriptor;
	USBEndpointDescriptor 	ed_descriptor[4];
	unsigned int 			ed_num;
	dev_ed_t 				dev_eds[MAX_EDS];
	unsigned int 			state;
	OS_EVENT				*lock_sem;
	unsigned int 			fun_addr;
	void					*pDevDrv;
	unsigned int			speed;
	unsigned int 			dev_num;
	unsigned int 			present;
} __attribute__ ((packed)) USBDevice_t, *pUSBDevice_t;

extern USBDevice_t USBDev[MAX_USB_DEV];

#endif /*MSD_H_*/
