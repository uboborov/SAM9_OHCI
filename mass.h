#ifndef MASS_H_
#define MASS_H_

#include "msd.h"

typedef struct CBW{   //command block
	unsigned int dCBWSignature;
	unsigned int dCBWTag;
	unsigned int dCBWDataTransfereLength;
	unsigned char bmCBWFlags;
	unsigned char bCBWLUN;
	unsigned char bCBWCBLength;
	unsigned char CBWCB[16];
} __attribute__ ((packed)) CBW_t;

typedef struct CSW{ //command status
	unsigned int dCSWSignature;
	unsigned int dCSWTag;
	unsigned int dCSWDataResidue;
	unsigned char  bCSWStatus;
}__attribute__ ((packed)) CSW_t;


typedef struct _MsdDrv {
	unsigned int msd_capacity;
	unsigned int block_size;
	unsigned int block_num;
}__attribute__ ((packed)) MsdDrv_t;

int usb_flash_inqury(USBDevice_t *pD, void *pOut);
int usb_flash_get_capacity(USBDevice_t *pD, void *pOut);
int usb_flash_mode_sense(USBDevice_t *pD, void *pIn);
int usb_flash_test_unit_ready(USBDevice_t *pD);
int usb_flash_read(USBDevice_t *pD, void *pOut, unsigned int address, unsigned int size);
int usb_flash_write(USBDevice_t *pD, void *pIn, unsigned int address, unsigned int size);
PS_UHP_ED get_ed(USBDevice_t *pD, unsigned int type, unsigned int dir);

int msd_init(unsigned int Unit);
#endif /*MASS_H_*/
