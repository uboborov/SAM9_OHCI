#ifndef AT91_OHCI_H
#define AT91_OHCI_H

//#define OHCI_POLING

#define UHC_DBG

#ifdef UHC_DBG
#include "log.h"
#define OHCI_DBG 	LOG
#else 
#define OHCI_DBG	if (0) printf
#endif

// *************** Base address of OHCI on MCU address space ************
#define OHCI_REGS_BASE	   0x00500000
// **********************************************************************

#define OHCI_SO		(1 << 0)
#define OHCI_WDH	(1 << 1)
#define OHCI_SF		(1 << 2)
#define OHCI_RD		(1 << 3)
#define OHCI_UE		(1 << 4)
#define OHCI_FNO	(1 << 5)
#define OHCI_RHSC	(1 << 6)
#define OHCI_OC		(1 << 30)
#define OHCI_MIE	(1 << 31)
// HcCommandStatus bits
#define OHCI_CS_HCR (1 << 0)
#define OHCI_CS_CLF (1 << 1)
#define OHCI_CS_BLF (1 << 2)
#define OHCI_CS_OCR (1 << 3)

#define OHCI_RH_LSDA	(1 << 9)

#define USBRESET    	(0x00 << 6)
#define USBRESUME		(0x01 << 6)
#define USBOPERATIONAL	(0x02 << 6)
#define USBSUSPEND 		(0x03 << 6)

#define OHCI_CL_PLE 	(1 << 2)
#define OHCI_CL_BLE 	(1 << 5)
#define OHCI_CL_CLE 	(1 << 4)
//#define OHCI_CL_CBSR 	()

#define ED_IN			0x02
#define ED_OUT			0x01
#define ED_GET_FROM_TD	0x00
#define ED_FUL_SPEED	0x00
#define ED_LOW_SPEED	0x01
#define ED_SKIP			0x01
#define ED_GEN_FORMAT	0x00
#define ED_ISO_FORMAT	0x01
#define ED_HALTED		0x01
#define ED_

#define MAX_USB_DEV	2
#define MAX_TDS		32
#define MAX_EDS		32

#define 	OHCI_IDLE			1
#define		OHCI_DEVICE			2
#define		OHCI_ERROR			3
#define		OHCI_OPER			4

#define		OHCI_MSG_DEVICE_POWERED_1	0x01
#define		OHCI_MSG_DEVICE_POWERED_2	0x02
#define		OHCI_MSG_DEVICE_SUSPENDED_1	0x03
#define		OHCI_MSG_DEVICE_SUSPENDED_2	0x04
#define		OHCI_MSG_DEVICE_RESUMED_1	0x05
#define		OHCI_MSG_DEVICE_RESUMED_2	0x06
#define		OHCI_MSG_DEVICE_DETACHED_1	0x07
#define		OHCI_MSG_DEVICE_DETACHED_2	0x08

#define		OHCI_DEV_LOW_SPEED			0x80

#define 	OHCI_CONTROL_ED				0x00
#define		OHCI_BULK_ED				0x01
#define		OHCI_ISO_ED					0x02
#define		OHCI_INT_ED					0x03

#define     ED_TYPE_CONTROL				OHCI_CONTROL_ED
#define     ED_TYPE_BULK				OHCI_BULK_ED
#define     ED_TYPE_ISO					OHCI_ISO_ED
#define     ED_TYPE_INT					OHCI_INT_ED	

#define     DEVICE_STATE_DETACHED		0
#define     DEVICE_STATE_POWERED		1
#define     DEVICE_STATE_CONFIGURED		2
#define     DEVICE_STATE_EROOR			3

#define OHCI_PAGE_SIZE 0x1000
#define OHCI_PAGE(x) ((x) &~ 0xfff)
#define OHCI_PAGE_MASK(x) ((x) & 0xfff)

typedef struct _OHCI_S_UHP_ED {
	volatile unsigned int Control;
	volatile unsigned int TailP;
	volatile unsigned int HeadP;
	volatile unsigned int NextEd;
} OHCI_S_UHP_ED, *OHCI_PS_UHP_ED;

typedef struct _OHCI_S_UHP_TD {
	volatile unsigned int Control;
	volatile unsigned int CBP;
	volatile unsigned int NextTD;
	volatile unsigned int BE;
} OHCI_S_UHP_TD, *OHCI_PS_UHP_TD;

typedef struct _OHCI_S_UHP_ITD {
	volatile unsigned int Control;
	volatile unsigned int BP0;
	volatile unsigned int NextTD;
	volatile unsigned int BE;
	volatile unsigned int PSW0;
	volatile unsigned int PSW1;
	volatile unsigned int PSW2;
	volatile unsigned int PSW3;
	volatile unsigned int PSW4;
	volatile unsigned int PSW5;
	volatile unsigned int PSW6;
	volatile unsigned int PSW7;
} OHCI_S_UHP_ITD, *OHCI_PS_UHP_ITD;

typedef struct _OHCI_S_UHP_HCCA {
	volatile unsigned int    UHP_HccaInterruptTable[32];
	volatile unsigned short  UHP_HccaFrameNumber;
	volatile unsigned short  UHP_HccaPad1;
	volatile unsigned int    UHP_HccaDoneHead;
	volatile unsigned int    reserved[30];
} OHCI_S_UHP_HCCA, *OHCI_PS_UHP_HCCA;

typedef struct _UHP_TD {
	unsigned int TDAddress;
	unsigned int Data_Toggle;
	unsigned int DelayInterrupt;
	unsigned int Direction;
	unsigned int Rounding;
	unsigned int CBP;
	unsigned int NextTD;
	unsigned int Buf_Len;
	unsigned int Flags;
	unsigned int callback;
	unsigned int arg;
} UHP_TD, *PS_UHP_TD;

typedef struct _UHP_ED {
	unsigned int Max_packet;
	unsigned int TD_format;
	unsigned int Skip;
	unsigned int Speed;
	unsigned int Direction;
	unsigned int Endpoint;
	unsigned int Func_Address;
	unsigned int TDQTailPtr;
	unsigned int TDQHeadPtr;
	unsigned int ToggleCarry;
	struct _UHP_ED 	 *NextED;
	unsigned int EDAddress;
	OHCI_PS_UHP_ED phED;
	unsigned int Type;
	unsigned int Trigger;
	unsigned int Data_amount;
	unsigned int stop_poling;
} UHP_ED, *PS_UHP_ED;

typedef struct {
  volatile unsigned int    HcRevision;
  volatile unsigned int    HcControl;
  volatile unsigned int    HcCommandStatus;
  volatile unsigned int    HcInterruptStatus;
  volatile unsigned int    HcInterruptEnable;
  volatile unsigned int    HcInterruptDisable;
  volatile unsigned int    HcHCCA;             // phys ptr to HCCA
  volatile unsigned int    HcPeriodCurrentED;  // phys ptr to HC_ENDPOINT_DESCRIPTOR
  volatile unsigned int    HcControlHeadED;    // phys ptr to HC_ENDPOINT_DESCRIPTOR
  volatile unsigned int    HcControlCurrentED; // phys ptr to HC_ENDPOINT_DESCRIPTOR
  volatile unsigned int    HcBulkHeadED;       // phys ptr to HC_ENDPOINT_DESCRIPTOR
  volatile unsigned int    HcBulkCurrentED;    // phys ptr to HC_ENDPOINT_DESCRIPTOR
  volatile unsigned int    HcDoneHead;         // phys ptr to HC_TRANSFER_DESCRIPTOR
  volatile unsigned int    HcFmInterval;
  volatile unsigned int    HcFmRemaining;
  volatile unsigned int    HcFmNumber;
  volatile unsigned int    HcPeriodicStart;
  volatile unsigned int    HcLSThreshold;
                       // root hub stuff
  volatile unsigned int    HcRhDescriptorA;
  volatile unsigned int    HcRhDescriptorB;
  volatile unsigned int    HcRhStatus;
  volatile unsigned int    HcRhPortStatus[2];  // actual number in low 8 bits of HcRhDescriptorA
} OHCIS_t, *OHCIPS_t;

#define OHCI_STAT

#ifdef OHCI_STAT
extern volatile unsigned int ohci_in_packet_count;
extern volatile unsigned int ohci_out_packet_count;
#endif

void InitUHP(void);
int ohci_bulk_write(PS_UHP_ED pED, void *pOut, unsigned int size, unsigned int tm);
int ohci_bulk_read(PS_UHP_ED pED, void *pIn, unsigned int size, unsigned int tm);
int ohci_int_read(PS_UHP_ED pED, void *pIn, unsigned int size, unsigned int tm);
int ohci_make_td_chain(PS_UHP_ED pED, unsigned int len, unsigned int slice, unsigned int ed_dir, void *pBuf);
int ohci_clear_future_request(PS_UHP_ED pED, void *out, unsigned int future, unsigned int ep_addr);

int usbh_test(char *command, char *operation, int iter, int size);
int usbh_clear_ep(char *command, int epnum, int param2, int param3);
int usbh_restart_uhp(char *command, int param1, int param2, int param3);
#endif
