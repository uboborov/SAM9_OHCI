#include "ioat91sam9260.h"
#include "ohci.h"
#include "includes.h"
#include "msd.h"
#include "mass.h"
#include "prio.h"

// =====================================
// Memory allocation for UHP:
// =====================================

// UHP HCCA memory location:
OHCIPS_t ohci = (OHCIPS_t)OHCI_REGS_BASE;

#define ohci_wr_reg(reg, val) {ohci->(reg) = (val)}
#define ohci_rd_reg(reg) {ohci->(reg)}

OHCI_S_UHP_HCCA HCCA __attribute__ ((aligned(256)));
PS_UHP_ED psUHPED[MAX_EDS];
volatile unsigned int o_sem = 0;
 
#ifdef OHCI_STAT
volatile unsigned int ohci_in_packet_count;
volatile unsigned int ohci_out_packet_count;
#endif

USBDevice_t USBDev[MAX_USB_DEV];
static unsigned int cur_dev_num = 0;

static unsigned int cur_td_num = 0;
static unsigned int cur_ed_num = 0;

static OS_EVENT *ohci_sem;
static OS_EVENT *ohci_int_sem;
static OS_EVENT *ohci_lock;
static OS_EVENT *OHCI_msg;

#define FRINTERVAL	0x2EDF
#define FSMAXPKTSZ	(((FRINTERVAL - 210) * 6) / 7)
#define FMINTERVAL	((FSMAXPKTSZ << 16) | FRINTERVAL)
#define PRDSTRT		9*FRINTERVAL/10;

#define IMASK	(OHCI_RHSC | OHCI_WDH)

//*********************** prototypes **********************************************
void print_dev_data(void *pDevData);
static int isr_process(unsigned int mask);
static void UHP_Isr_Handler();
static void ohci_hub_isr();
static int start_trans(PS_UHP_ED pED);
static void ohci_skip_ed(PS_UHP_ED pED);

PS_UHP_TD ohci_create_td(unsigned int td_type, 
						 unsigned int del_int, 
						 unsigned int direction,
						 unsigned int rounding,
						 unsigned int cbp,
						 unsigned int buf_len,
						 unsigned int next_td,
						 unsigned int cb,
						 unsigned int arg);
void ohci_add_td(PS_UHP_ED pED, PS_UHP_TD pTD);

/*
 * 
 */
 static void ll_CreateEd(
	unsigned int EDAddr,
	unsigned int MaxPacket,
	unsigned int TDFormat,
	unsigned int Skip,
	unsigned int Speed,
	unsigned int Direction,
	unsigned int EndPt,
	unsigned int FuncAddress,
	unsigned int TDQTailPntr,
	unsigned int TDQHeadPntr,
	unsigned int ToggleCarry,
	unsigned int NextED)
{
	OHCI_PS_UHP_ED pED = (OHCI_PS_UHP_ED) EDAddr;
	pED->Control = (MaxPacket << 16) | (TDFormat << 15) |
	               (Skip << 14) | (Speed << 13) | (Direction << 11) |
	               (EndPt << 7) | FuncAddress;
	pED->TailP   = (TDQTailPntr & 0xFFFFFFF0);
	pED->HeadP   = (TDQHeadPntr & 0xFFFFFFF0) | (ToggleCarry << 1);
	pED->NextEd  = (NextED & 0xFFFFFFF0);
}

/*
 * 
 */
 static void ll_CreateGenTd(
	unsigned int GenTdAddr,
	unsigned int DataToggle,
	unsigned int DelayInterrupt,
	unsigned int Direction,
	unsigned int BufRnding,
	unsigned int CurBufPtr,
	unsigned int NextTD,
	unsigned int BuffLen)
{
	OHCI_PS_UHP_TD pTD = (OHCI_PS_UHP_TD) GenTdAddr;
	pTD->Control = (DataToggle << 24) | (DelayInterrupt << 21) | (Direction << 19) | (BufRnding << 18);
	pTD->CBP     = CurBufPtr;
	pTD->NextTD  = (NextTD & 0xFFFFFFF0);
	pTD->BE      = (BuffLen) ? CurBufPtr + BuffLen - 1 : CurBufPtr;
}

/*
 * 
 */
 static void ll_CreateGenITd(
	unsigned int GenTdAddr,
	unsigned int CondCode,
	unsigned int FrameCount,
	unsigned int DelayInterrupt,
	unsigned int StartFrame,
	unsigned int BuffPage0,
	unsigned int NextTD,
	unsigned int BufEnd,
	unsigned int PswOffset0,
	unsigned int PswOffset1,
	unsigned int PswOffset2,
	unsigned int PswOffset3,
	unsigned int PswOffset4,
	unsigned int PswOffset5,
	unsigned int PswOffset6,
	unsigned int PswOffset7)
{
	OHCI_PS_UHP_ITD pITD = (OHCI_PS_UHP_ITD) GenTdAddr;

	pITD->Control = (CondCode << 28) | (FrameCount << 24) | (DelayInterrupt << 21) | StartFrame;
	pITD->BP0 = (BuffPage0 << 12);
	pITD->NextTD = (NextTD << 4);
	pITD->BE   = BufEnd;
	pITD->PSW0 = PswOffset0;
	pITD->PSW1 = PswOffset1;
	pITD->PSW2 = PswOffset2;
	pITD->PSW3 = PswOffset3;
	pITD->PSW4 = PswOffset4;
	pITD->PSW5 = PswOffset5;
	pITD->PSW6 = PswOffset6;
	pITD->PSW7 = PswOffset7;

}

typedef struct _S_TIMEOUT {
		unsigned int tick;
		unsigned int second;
} S_TIMEOUT, *PS_TIMEOUT;


static void InitTimeout(PS_TIMEOUT pTimeout, unsigned int second)
{
	pTimeout->tick = OSTimeGet();
	pTimeout->second = second;
}

static int TestTimeout(PS_TIMEOUT pTimeout)
{
	if (pTimeout->second != 0) {
		if (((pTimeout->tick + (OS_TICKS_PER_SEC * pTimeout->second))) < OSTimeGet()) {
			pTimeout->second = 0;
			return 0;
		}
	}
	return pTimeout->second;
}

//********* Platform dependent functions *********************************

inline void AT91F_PMC_EnablePeriphClock (
	AT91PS_PMC pPMC, // \arg pointer to PMC controller
	unsigned int periphIds)  // \arg IDs of peripherals to enable
{
	pPMC->PMC_PCER = periphIds;
}

inline void AT91F_UHP_CfgPMC (void)
{
	AT91F_PMC_EnablePeriphClock(
		AT91C_BASE_PMC, // PIO controller base address
		((unsigned int) 1 << AT91C_ID_UHP));
}
//**************************************************************************
void CPU_ConfigureUHPClock() {
	// for atmel CPU
	AT91F_UHP_CfgPMC();
	// enable UHP clock in PMC
	AT91C_BASE_PMC->PMC_SCER |= (AT91C_PMC_UHP);
}

void CPU_DisableUHPClock() {
	AT91C_BASE_PMC->PMC_SCDR |= (AT91C_PMC_UHP);
}

void ohci_clock_disable() {
	CPU_DisableUHPClock();
}

static void lock_ohci() {
	unsigned char err;
	OSSemPend(ohci_lock, 0, &err);
}

static void unlock_ohci() {
	OSSemPost(ohci_lock);
}



/*
 * 
 */
static void AT91_UHP_Isr_Init() {
	AT91C_BASE_AIC->AIC_SVR[ AT91C_ID_UHP ] = ( unsigned int )UHP_Isr_Handler;
	AT91C_BASE_AIC->AIC_SMR[ AT91C_ID_UHP ] = 0;
	AT91C_BASE_AIC->AIC_ICCR = 1 << AT91C_ID_UHP;
	    
	AT91C_BASE_AIC->AIC_IECR = 1 << AT91C_ID_UHP;
}

/*
 * 
 */
static void UHP_Isr_Init() {
	AT91_UHP_Isr_Init();
}

static void CPU_ClearUHPPendingBit() {
	// for atmel cpu
	AT91C_BASE_AIC->AIC_ICCR = 1 << AT91C_ID_UHP;
	AT91C_BASE_AIC->AIC_EOICR = 0;
}

/*
 * 
 */
static void UHP_Isr_Handler() {
	unsigned int status, status_s;
	
	status = ohci->HcInterruptStatus;
	status_s = status & IMASK;
	
	if (!status_s) {
		ohci->HcInterruptStatus = status;
		CPU_ClearUHPPendingBit();
		return;
	}
	isr_process(status_s);
	ohci->HcInterruptStatus = status;
	
	CPU_ClearUHPPendingBit();
}

/*
 * 
 */
static int isr_process(unsigned int mask) {
	unsigned int intrs, eintrs;
	unsigned int done;
	OHCI_PS_UHP_HCCA uhp;

	intrs = 0;
	uhp = (OHCI_PS_UHP_HCCA)ohci->HcHCCA;
	done = uhp->UHP_HccaDoneHead;
	
		if (done != 0) {
#ifndef OHCI_POLING			
			OSSemPost(ohci_sem);
#endif			
			o_sem = 1;
		
			uhp->UHP_HccaDoneHead = 0;
				intrs = OHCI_WDH;
		} else {
			intrs = mask;
		}

		if (intrs == 0) {
			/* nothing to be done ?! */
			return (0);
		}

		intrs &= IMASK;	/* mask out Master Interrupt Enable */

		/* Acknowledge any interrupts that have happened */
		ohci->HcInterruptStatus = intrs;

		/* Any interrupts we had enabled? */
		eintrs = intrs ;//& sc->sc_eintrs;
		if (!eintrs)
			return (0);

		
		if (eintrs & OHCI_SO) {
			OHCI_DBG("scheduling overrun\r\n");
			/* XXX do what */
			intrs &= ~OHCI_SO;
		}
		if (eintrs & OHCI_WDH) {			
			//ohci_process_done(sc, done);
			//printf("!!!DONE: 0x%08X\r\n", uhp->UHP_HccaDoneHead );
			intrs &= ~OHCI_WDH;
		}
		if (eintrs & OHCI_RD) {
			OHCI_DBG("resume detect\r\n");
			/* XXX process resume detect */
		}
		if (eintrs & OHCI_UE) {
			OHCI_DBG("unrecoverable error, controller halted\r\n");
//			OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
			/* XXX what else */
		}
		if (eintrs & OHCI_RHSC) {
			//ohci_rhsc(sc, sc->sc_intrxfer);
			ohci_hub_isr();
			intrs &= ~OHCI_RHSC;
		}


		/* Block unprocessed interrupts. XXX */
		ohci->HcInterruptDisable = intrs;
		return 0;
}


void print_dev_data(void *pDevData) {
	USBDeviceDescriptor *p = (USBDeviceDescriptor *)pDevData;
	printf("\r\n");
	printf("bDescriptorType: %d\r\n",p->bDescriptorType);
	printf("bcdUSB: 0x%04X\r\n",p->bcdUSB);
	printf("bDeviceClass: %d\r\n",p->bDeviceClass);
	printf("bDeviceSubClass: %d\r\n",p->bDeviceSubClass);
	printf("bDeviceProtocol: %d\r\n",p->bDeviceProtocol);
	printf("bMaxPacketSize0: %d\r\n",p->bMaxPacketSize0);
	printf("idVendor: 0x%04X\r\n",p->idVendor);
	printf("idProduct: 0x%04X\r\n",p->idProduct);
	printf("bcdDevice: 0x%04X\r\n",p->bcdDevice);
	printf("iManufacturer: %d\r\n",p->iManufacturer);
	printf("iProduct: %d\r\n",p->iProduct);
	printf("iSerialNumber: %d\r\n",p->iSerialNumber);
	printf("bNumConfigurations: %d\r\n",p->bNumConfigurations);
	printf("\r\n");
}

char *parse_subclass(unsigned int s_class) {
	switch(s_class) {
		case 0x01: return "RBC";
		case 0x02: return "MMC-2 (ATAPI)";	
		case 0x03: return "QIC-157";
		case 0x04: return "UFI";
		case 0x05: return "SFF-8070i";
		case 0x06: return "SCSI";
		default: return "Unknown";
	};
}

char *parse_protocol(unsigned int protocol) {
	switch(protocol) {
		case 0x00: return "Control/Bulk/Interrupt (with cc)";
		case 0x01: return "Control/Bulk/Interrupt (without cc)";	
		case 0x50: return "Bulk-Only Transport";
		default: return "Unknown";
	};
}

void print_dev_config(void *pDevData) {
	
	USBConfigDescriptor *p = (USBConfigDescriptor *)pDevData;
	USBInterfaceDescriptor *e = (USBInterfaceDescriptor *)(pDevData + sizeof(USBConfigDescriptor));
	USBEndpointDescriptor *ed1 = (USBEndpointDescriptor *)(pDevData + sizeof(USBConfigDescriptor) + 
			sizeof(USBInterfaceDescriptor));
	USBEndpointDescriptor *ed2 = (USBEndpointDescriptor *)(pDevData + sizeof(USBConfigDescriptor) + 
			sizeof(USBInterfaceDescriptor) + sizeof(USBEndpointDescriptor));
	
	printf("\r\n");
	printf("bLength: %d\r\n",p->bLength);
	printf("bDescriptorType: %d (%s)\r\n",p->bDescriptorType, "Configuration");
	printf("wTotalLength: %d\r\n",p->wTotalLength);
	printf("bNumInterfaces: %d\r\n",p->bNumInterfaces);
	printf("bConfigurationValue: %d\r\n",p->bConfigurationValue);
	printf("iConfiguration: %d\r\n",p->iConfiguration);
	printf("bmAttributes: 0x%02X\r\n",p->bmAttributes);
	printf("bMaxPower: %d\r\n",p->bMaxPower);
	printf("\r\n");
	if (p->wTotalLength >= (sizeof(USBConfigDescriptor) + sizeof(USBInterfaceDescriptor))) {
		 printf("\r\n");
		 printf("bLength: %d\r\n",e->bLength);
		 printf("bDescriptorType: %d (%s)\r\n",e->bDescriptorType, "Interface");
		 printf("bInterfaceNumber: %d\r\n",e->bInterfaceNumber);
		 printf("bAlternateSetting: %d\r\n",e->bAlternateSetting);
		 printf("bNumEndpoints: %d\r\n",e->bNumEndpoints);
		 printf("bInterfaceClass: %s\r\n",(e->bInterfaceClass == 0x08)?"MASS STORAGE":"OTHER");
		 printf("bInterfaceSubClass: %s\r\n",parse_subclass(e->bInterfaceSubClass));
		 printf("bInterfaceProtocol: %s\r\n",parse_protocol(e->bInterfaceProtocol));
		 printf("iInterface: %d\r\n",e->iInterface);
		 printf("\r\n");
	}
	if (p->wTotalLength >= (sizeof(USBConfigDescriptor) + sizeof(USBInterfaceDescriptor) + 
			sizeof(USBEndpointDescriptor))) { 
		printf("\r\n");
		printf("bLength: %d\r\n",ed1->bLength);
		printf("bDescriptorType: %d (%s)\r\n",ed1->bDescriptorType, "Endpoint");
		printf("bEndpointAddress: %d (%s)\r\n",ed1->bEndpointAddress & 0x7F,
				(ed1->bEndpointAddress & 0x80)?"OUT":"IN");
		printf("bmAttributes: 0x%02X (%s)\r\n",ed1->bmAttributes, (ed1->bmAttributes & 0x02)?
				"Bulk": "Other");
		printf("wMaxPacketSize: %d\r\n",ed1->wMaxPacketSize);
		printf("bInterval: %d\r\n",ed1->bInterval);
		printf("\r\n");
	}
	if (p->wTotalLength >= (sizeof(USBConfigDescriptor) + sizeof(USBInterfaceDescriptor) + 
			sizeof(USBEndpointDescriptor) + sizeof(USBEndpointDescriptor))) { 
		printf("\r\n");
		printf("bLength: %d\r\n",ed2->bLength);
		printf("bDescriptorType: %d (%s)\r\n",ed2->bDescriptorType, "Endpoint");
		printf("bEndpointAddress: %d (%s)\r\n",ed2->bEndpointAddress & 0x7F,
						(ed2->bEndpointAddress & 0x80)?"OUT":"IN");
		printf("bmAttributes: 0x%02X (%s)\r\n",ed2->bmAttributes, (ed2->bmAttributes & 0x02)?
						"Bulk": "Other");
		printf("wMaxPacketSize: %d\r\n",ed2->wMaxPacketSize);
		printf("bInterval: %d\r\n",ed2->bInterval);
		printf("\r\n");
	} 
}

/*
 * 
 */
void print_interface (void *pDevData) {
	USBInterfaceDescriptor *p = (USBInterfaceDescriptor*)pDevData;
	   printf("\r\n");
	   printf("bLength: %d\r\n",p->bLength);
	   printf("bDescriptorType: %d\r\n",p->bDescriptorType);
	   printf("bInterfaceNumber: %d\r\n",p->bInterfaceNumber);
	   printf("bAlternateSetting: %d\r\n",p->bAlternateSetting);
	   printf("bNumEndpoints: %d\r\n",p->bNumEndpoints);
	   printf("bInterfaceClass: %d\r\n",p->bInterfaceClass);
	   printf("bInterfaceSubClass: 0x%02X\r\n",p->bInterfaceSubClass);
	   printf("bInterfaceProtocol: %d\r\n",p->bInterfaceProtocol);
	   printf("iInterface: %d\r\n",p->iInterface);
	   printf("\r\n");
}

/*
 * 
 */

void print_cc(unsigned int cc) {
	switch (cc) {
	case 1:
		printf("EP ERROR: CRC\r\n");
		return;
	case 2:
		printf("EP ERROR: BITSTUFFING\r\n");
		return;
	case 3:
		printf("EP ERROR: DATATOGGLEMISMATCH\r\n");
		return;
	case 4:
		printf("EP ERROR: STALL\r\n");
		return;
	case 5:
		printf("EP ERROR: DEVICENOTRESPONDING\r\n");
		return;
	case 6:
		printf("EP ERROR: PIDCHECKFAILURE\r\n");
		return;
	case 7:
		printf("EP ERROR: UNEXPECTEDPID\r\n");
		return;
	case 8:
		printf("EP ERROR: DATAOVERRUN\r\n");
		return;
	case 9:
		printf("EP ERROR: DATAUNDERRUN\r\n");
		return;				
	case 12:
		printf("EP ERROR: BUFFEROVERRUN\r\n");
		return;	
	case 13:
		printf("EP ERROR: BUFFERUNDERRUN\r\n");
		return;	
	default:
		printf("EP ERROR: RESERVED 0x%02X\r\n", cc);
		return;	
	}
}

/*
 * 
 */
PS_UHP_TD ohci_create_td(unsigned int td_type, 
						 unsigned int del_int, 
						 unsigned int direction,
						 unsigned int rounding,
						 unsigned int cbp,
						 unsigned int buf_len,
						 unsigned int next_td,
						 unsigned int cb,
						 unsigned int arg) {
	PS_UHP_TD pTD = NULL;
	OHCI_PS_UHP_TD pUTD ;
	
	if (cur_td_num > MAX_TDS) {
		OHCI_DBG("UH ERROR: MAX TD\r\n");
		return NULL;
	}
	
	pTD = (PS_UHP_TD)safe_malloc(sizeof(UHP_TD));
	if(!pTD) {
		OHCI_DBG("UH ERROR: Not enough memory\r\n");
		return NULL;
	}
	memset(pTD, 0, sizeof(UHP_TD));
	
	pUTD = (OHCI_PS_UHP_TD)aligned_malloc(sizeof(OHCI_S_UHP_TD), 16);
	if(!pUTD) {
		OHCI_DBG("UH ERROR: Not enough memory\r\n");
		return NULL;
	}
	memset(pUTD, 0, sizeof(OHCI_S_UHP_TD));
	
	pTD->Buf_Len = buf_len;
	pTD->CBP = cbp;
	pTD->Data_Toggle = td_type;
	pTD->DelayInterrupt = del_int;
	pTD->Direction = direction;
	pTD->NextTD = 0;
	pTD->Rounding = rounding;
	pTD->TDAddress = (unsigned int) pUTD;
	pTD->callback = cb;
	pTD->arg = arg;
	
	ll_CreateGenTd(
					pTD->TDAddress,
					pTD->Data_Toggle,
					pTD->DelayInterrupt,
					pTD->Direction,
					pTD->Rounding,
					pTD->CBP,
					0,
					pTD->Buf_Len);

	cur_td_num++;
	return pTD;
}

/*
 * 
 */
void ohci_add_td(PS_UHP_ED pED, PS_UHP_TD pTD) {
	OHCI_PS_UHP_TD psTD;
	OHCI_PS_UHP_ED phED;
	
	phED = pED->phED;
	
	if (!phED) return;
	
	if (!pTD) {
		OHCI_DBG("UH ERROR: Invalid TD\r\n");
		return;
	}
	
	if (cur_td_num > MAX_TDS) {
		OHCI_DBG("UH ERROR: MAX TD\r\n");
		return;
	}
			
	if(!phED->HeadP) {
		phED->HeadP = (((unsigned int) pTD->TDAddress) & 0xFFFFFFF0);
	} else {
		psTD = (OHCI_PS_UHP_TD)phED->HeadP;
		for (;psTD->NextTD;psTD = (OHCI_PS_UHP_TD)psTD->NextTD);
		psTD->NextTD = pTD->TDAddress;
	}
	
	phED->TailP = (((unsigned int) pTD->TDAddress) & 0xFFFFFFF0);
}

/*
 * 
 */
void ohci_free_td(PS_UHP_TD pTD) {
	
	if (!pTD) {
		OHCI_DBG("UH ERROR: TD is not valid\r\n");
		return;
	}
	
	aligned_free((void *)(pTD->TDAddress));
	safe_free(pTD);
	cur_td_num --;
}

/*
 * 
 */
int ohci_make_td_chain(PS_UHP_ED pED, 
	unsigned int len, 
	unsigned int slice, 
	unsigned int ed_dir,
	void *pBuf) {
	
	void *p = pBuf;
	PS_UHP_TD pTD[(len/4096) + 2];
	OHCI_PS_UHP_ED phED;	
	int i = 0;
	unsigned int trigger = pED->Trigger;
	
	if (!pED) return -1;
	
	phED = pED->phED;
	if (!phED) return -1;
	 
	while (len > 4096) {		
		pTD[i] = ohci_create_td(trigger, 0, ed_dir, 1, (unsigned int)p, 4096, 0, 0, 0);
		if (!pTD[i]) return -1;
		ohci_add_td(pED, pTD[i]);
		len -= 4096;
		p += 4096;
		i++;
#ifdef OHCI_STAT
	if (ed_dir == ED_IN)
		ohci_in_packet_count += 4096;
	else
		ohci_out_packet_count += 4096;
#endif				
	}
	
	pTD[i] = ohci_create_td(trigger, 0, ed_dir, 1, (unsigned int)p, len, 0, 0, 0);
	if (!pTD[i]) return -1;
	ohci_add_td(pED, pTD[i]);
	i++;

/*	
	for (i = 0;i < (len/slice);i++) {
		pTD[i] = ohci_create_td(trigger, 0, ed_dir, 1, (unsigned int)p, slice, 0, 0, 0);
		if (!pTD[i]) {
			//unlock_ohci();
			return -1;
		}
		ohci_add_td(pED, pTD[i]);
		//trigger = (((trigger ^ 1) & 0x1) | 0x2);
		p += slice;
	}
*/		
	pTD[i] = ohci_create_td(trigger, 0, ed_dir, 1, (unsigned int)0, 0, 0, 0, 0);
	if (!pTD[i]) return -1;
	ohci_add_td(pED, pTD[i]);

	
	if (start_trans(pED)) {
/*		for (i = 0;i < ((len/slice)+1);i++) {
			ohci_free_td(pTD[i]);
		}*/
		for (;i >= 0;i--) {
			ohci_free_td(pTD[i]);
		}
		OHCI_DBG("OHCI: Trans ERROR\r\n");
		return -1;
	}
#ifdef OHCI_STAT
	if (ed_dir == ED_IN)
		ohci_in_packet_count += len;
	else
		ohci_out_packet_count += len;
#endif		
	phED->TailP   = 0x00000000;
	phED->HeadP   = 0x00000000;
/*	for (i = 0;i < ((len/slice)+1);i++) {
		ohci_free_td(pTD[i]);
	}*/
	for (;i >= 0;i--) {
		ohci_free_td(pTD[i]);
	}
	return 0;	
}

/*
 * 
 */
PS_UHP_ED ohci_create_ed(unsigned int max_packet, 
						 unsigned int td_format, 
						 unsigned int skip,
						 unsigned int speed,
						 unsigned int direction,
						 unsigned int ed_num,
						 unsigned int func,
						 unsigned int td_head,
						 unsigned int td_tail,
						 unsigned int toggle,
						 unsigned int next_ed,
						 unsigned int type,
						 unsigned int amount) {
	
	PS_UHP_ED pED = NULL;
	OHCI_PS_UHP_ED pUED ;
	
	if (cur_ed_num > MAX_EDS) {
		OHCI_DBG("UH ERROR: MAX ED\r\n");
		return NULL;
	}
	
	pED = (PS_UHP_ED)safe_malloc(sizeof(UHP_ED));
	if(!pED) {
		OHCI_DBG("UH ERROR: Not enough memory\r\n");
		return NULL;
	}
	memset(pED, 0, sizeof(PS_UHP_ED));
	
	pUED = (OHCI_PS_UHP_ED)aligned_malloc(sizeof(OHCI_S_UHP_ED), 16);		
	if(!pUED) {
		OHCI_DBG("UH ERROR: Not enough memory\r\n");
		safe_free(pED);
		return NULL;
	}
	memset(pUED, 0, sizeof(OHCI_PS_UHP_ED));
	
	psUHPED[cur_ed_num] = pED;
	
	pED->Max_packet = max_packet;
	pED->TD_format = td_format;
	pED->Skip = skip;
	pED->Speed = speed;
	pED->Direction = direction;
	pED->Endpoint = ed_num;
	pED->Func_Address = func;
	pED->TDQHeadPtr = td_head;
	pED->TDQTailPtr = td_tail;
	pED->ToggleCarry = toggle;
	pED->NextED = NULL;
	pED->EDAddress = (unsigned int)pUED;
	pED->phED = pUED;
	pED->Type = type;
	pED->Trigger = 0;
	pED->Data_amount = amount;
	
	ll_CreateEd(
			pED->EDAddress, // ED Address
			pED->Max_packet,      // Max packet
			pED->TD_format,      // TD format
			pED->Skip,      // Skip
			pED->Speed,      // Speed
			pED->Direction,    // Direction
			pED->Endpoint,    // Endpoint
			pED->Func_Address,    // Func Address
			pED->TDQHeadPtr,//(unsigned int) &pUHPTd[3],    // TDQTailPointer
			pED->TDQTailPtr,//(unsigned int) &pUHPTd[0],    // TDQHeadPointer
			pED->ToggleCarry,      // ToggleCarry
			0);   // NextED
	cur_ed_num++;
	return pED;
}

/*
 * 
 */
void ohci_add_ed(PS_UHP_ED pED) {
	OHCI_PS_UHP_ED ppED = NULL;
	PS_UHP_ED pfED = NULL;
	unsigned int i;
	
	
	if (pED->Type == OHCI_CONTROL_ED) {
		ppED = (OHCI_PS_UHP_ED) ohci->HcControlHeadED;
	} else if (pED->Type == OHCI_BULK_ED) {
		ppED = (OHCI_PS_UHP_ED) ohci->HcBulkHeadED;
	} else if (pED->Type == OHCI_INT_ED) {
		for (i = 0;i < MAX_EDS;i++) {
			if ((psUHPED[i]->Type == OHCI_INT_ED) && (psUHPED[i] != pED)) {
				pfED = psUHPED[i];
				break;
			}
		}
	}
	// Find first phisical ED in virtual ED list
	if (pED->Type != OHCI_INT_ED) {
		for (i = 0;i < MAX_EDS;i++) {
			if (psUHPED[i]->phED == ppED) {
				pfED = psUHPED[i];
				break;
			}		
		}
	}
	// Return if we did not find appropriate virtual ED
	if (!pfED) return;
	// Roll the list to find the last ED in the queue
	for (;pfED->NextED;pfED = pfED->NextED);
	
	pfED->NextED = pED;
	pfED->phED->NextEd = (((unsigned int)pED->phED) & 0xFFFFFFF0);
	pED->phED->NextEd = 0;
}

/*
 * 
 */
static int ohci_remove_ed(PS_UHP_ED pED) {
	OHCI_PS_UHP_ED pfED = NULL;
	unsigned int i;
	
	if (!pED) return -1;

	if (pED->Type == ED_TYPE_CONTROL) {
		pfED = (OHCI_PS_UHP_ED)ohci->HcControlHeadED; 
	} else if (pED->Type == ED_TYPE_BULK) {
		pfED = (OHCI_PS_UHP_ED)ohci->HcBulkHeadED;
	} else if (pED->Type == ED_TYPE_INT) {
		for (i = 0;i < MAX_EDS;i++) {
			if (psUHPED[i] == pED) {
				pfED = psUHPED[i]->phED;
			}
		}
	}
	
	// Remove logical ED
	for (i = 0;i < MAX_EDS;i++) {
		if (psUHPED[i]->NextED == pED) {
			psUHPED[i]->NextED = pED->NextED;			
			break;
		}		
	}
	
	// Remove phisical ED
	for (;pfED->NextEd;pfED = (OHCI_PS_UHP_ED)pfED->NextEd) {
		if (pfED->NextEd == pED->EDAddress) {
			pfED->NextEd = pED->phED->NextEd;
			return 0;
		}
	}
	
	return 0;
}

/*
 * 
 */
void ohci_free_ed(PS_UHP_ED pED) {
	unsigned int i;
	int j = -1;
	
	if (!pED) {
		OHCI_DBG("UH ERROR: TD is not valid\r\n");
		return;
	}
	// free hw pointer
	aligned_free(pED->phED);
	// find ed in pointer list
	for (i = 0;i < MAX_EDS;i++) {
		if (pED == psUHPED[i]) {
			//psUHPED[i] = 0;
			j = i;
			break;
		}
	}	
	if (j < 0) return;
	// free ed
	safe_free(pED);
	// clear ed in pointer list
	if (j >= 0)
		psUHPED[j] = 0;
	// decrement ed number
	cur_ed_num--;
}

/*
 * 
 */
static void ohci_skip_ed(PS_UHP_ED pED) {
	OHCI_PS_UHP_ED phED = pED->phED;
	if (!phED) return;
	phED->Control |= (1 << 14);
}

/*
 * 
 */
int ohci_new_control_trans(PS_UHP_ED pED, int request_type, void *pIn, void *pSetup, unsigned int len) {
	PS_UHP_TD pTDin, pTDout, pTDempty;
	
	OHCI_PS_UHP_ED phED = pED->phED;
	
	//lock_ohci();
	if (!phED) {
		//unlock_ohci();
		return -1;
	}
		
	// create OUT transaction
	pTDout = ohci_create_td(2, 0, 0, 1, (unsigned int)pSetup, 8, 0, 0, 0);
	if (!pTDout) {
		//unlock_ohci();
		return -1;
	}
	ohci_add_td(pED, pTDout);
	// create IN transaction
	pTDin = ohci_create_td(3, 0, ED_IN, 1, (unsigned int)pIn, len, 0, 0, 0);
	if (!pTDin) {
		//unlock_ohci();
		return -1;
	}
	ohci_add_td(pED, pTDin);
	 //create EMTY transaction
	pTDempty = ohci_create_td(2, 0, 0, 1, 0, 0, 0, 0, 0);
	if (!pTDempty) {
		//unlock_ohci();
		return -1;	
	}
	ohci_add_td(pED, pTDempty);
	//unlock_ohci();
	
	if (start_trans(pED)) {
		ohci_free_td(pTDout);
		ohci_free_td(pTDin);
		ohci_free_td(pTDempty);
		////unlock_ohci();
		return -1;
	}
	
	phED->TailP   = 0x00000000;
	phED->HeadP   = 0x00000000;
	ohci_free_td(pTDout);
	ohci_free_td(pTDin);
	ohci_free_td(pTDempty);
	////unlock_ohci();
	return 0;
}
/*
 * 
 */
int ohci_device_config_request(PS_UHP_ED pED, void *out) {
	unsigned char setup[] = {0x80,0x06,0x00,0x01,0x00,0x00,0x40,0x00};
	
	return ohci_new_control_trans(pED, 0, out, setup, 18);
}

/*
 * 
 */
int ohci_config_request(PS_UHP_ED pED, void *out) {
	unsigned char setup[] = {0x80,0x06,0x00,0x02,0x00,0x00,0x40,0x00};
	
	return ohci_new_control_trans(pED, 0, out, setup, 64);
}

/*
 * 
 */
int ohci_interface_descriptor_request(PS_UHP_ED pED, void *out) {
	unsigned char setup[] = {0x80,0x06,0x00,0x04,0x00,0x00,0x40,0x00};
	
	return ohci_new_control_trans(pED, 0, out, setup, 64);
}

/*
 * 
 */
int ohci_set_address_request(PS_UHP_ED pED, void *out, unsigned int address) {
	unsigned char setup[] = {0x00,0x05,(unsigned char)address,0x02,0x00,0x00,0x00,0x00};
	int result = -1;
	OHCI_PS_UHP_ED phED = pED->phED;
	
	result = ohci_new_control_trans(pED, 0, out, setup, 0);
	phED->Control |= (unsigned int) address;
	pED->Func_Address = address;
	return result;
}

/*
 * 
 */
int ohci_set_configured_request(PS_UHP_ED pED, void *out) {
	unsigned char setup[] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	
	return ohci_new_control_trans(pED, 0, out, setup, 0);
}

/*
 * 
 */
int ohci_clear_future_request(PS_UHP_ED pED, void *out, unsigned int future, unsigned int ep_addr) {
	unsigned char setup[] = {0x02,0x01,future,0x00,ep_addr,0x00,0x00,0x00};
	
	return ohci_new_control_trans(pED, 0, out, setup, 0);
}

int ohci_usb_clear_halt(PS_UHP_ED pED, unsigned int epnum) {
	return ohci_clear_future_request(pED, NULL, 0x00, epnum);
}

/*
 * 
 */
int ohci_bulk_write(PS_UHP_ED pED, void *pOut, unsigned int size, unsigned int tm) {
	PS_UHP_TD pTDout;
	OHCI_PS_UHP_ED phED = pED->phED;
	int trans = 0;
	
	//lock_ohci();
	
	if (!phED) {
		//unlock_ohci();
		return -1;
	}
			
	// create OUT transaction
	pTDout = ohci_create_td(pED->Trigger, 0, ED_OUT, 1, (unsigned int)pOut, size, 0, 0, 0);
	if (!pTDout) {
		//unlock_ohci();
		return -1;
	}
	ohci_add_td(pED, pTDout);
	// create OUT transaction
	phED->TailP = 0;	
	//unlock_ohci();
	if ((trans = start_trans(pED)) != 0) {
		ohci_free_td(pTDout);
		phED->TailP   = 0x00000000;
		phED->HeadP   = 0x00000000;
		////unlock_ohci();
		return trans;
	}
#ifdef OHCI_STAT
	ohci_out_packet_count += size;
#endif
	phED->TailP   = 0x00000000;
	phED->HeadP   = 0x00000000;
	ohci_free_td(pTDout);
	////unlock_ohci();
	return 0;
}

/*
 * 
 */
int ohci_bulk_read(PS_UHP_ED pED, void *pIn, unsigned int size, unsigned int tm) {
	PS_UHP_TD pTDin;
	OHCI_PS_UHP_ED phED = pED->phED;
	int trans = 0;
	//lock_ohci();
	
	if (!phED) {
		//unlock_ohci();
		return -1;
	}
	
	// create OUT transaction
	pTDin = ohci_create_td(pED->Trigger, 0, ED_IN, 1, (unsigned int)pIn, size, 0, 0, 0);
	if (!pTDin) {
		//unlock_ohci();
		return -1;
	}
	ohci_add_td(pED, pTDin);
	// create IN transaction
	phED->TailP = 0;
	//unlock_ohci();
	
	if ((trans = start_trans(pED)) != 0) {
		ohci_free_td(pTDin);
		phED->TailP   = 0x00000000;
		phED->HeadP   = 0x00000000;
		////unlock_ohci();
		return trans;
	}
#ifdef OHCI_STAT
	ohci_in_packet_count += size;
#endif	
	phED->TailP   = 0x00000000;
	phED->HeadP   = 0x00000000;
	ohci_free_td(pTDin);
	////unlock_ohci();
	return 0;
}

/*
 * 
 */
int ohci_int_read(PS_UHP_ED pED, void *pIn, unsigned int size, unsigned int tm) {
	PS_UHP_TD pTDin;	
	OHCI_PS_UHP_ED phED = pED->phED;

	//lock_ohci();

	if (!phED) {
		//unlock_ohci();
		return -1;
	}
		
	// create IN transaction
	pTDin = ohci_create_td(pED->Trigger, 0, ED_IN, 1, (unsigned int)pIn, size, 0, 0, 0);
	if (!pTDin) {
		//unlock_ohci();
		return -1;
	}
	ohci_add_td(pED, pTDin);
	phED->TailP = 0;
	//unlock_ohci();
	
	if (start_trans(pED)) {
		ohci_free_td(pTDin);
		phED->TailP   = 0x00000000;
		phED->HeadP   = 0x00000000;
				
		return -1;
	}
	
	phED->TailP   = 0x00000000;
	phED->HeadP   = 0x00000000;
	ohci_free_td(pTDin);
	return 0;
}


/*
 * 
 */
static int start_trans(PS_UHP_ED pED) {
	OHCI_PS_UHP_ED phED;
	OHCI_PS_UHP_TD phTD;
#ifndef OHCI_POLING	
	unsigned char err;
#endif	
	S_TIMEOUT timeout;
	volatile unsigned int *tControl = NULL;
	
	phED = pED->phED;
	phTD = NULL;
	
	if (!phED) return -1;
	
	if (pED->Type == OHCI_INT_ED)		
		InitTimeout(&timeout, 1);
	
	phED->Control &= ~(1 << 14);
	ohci->HcControl |= 0x80; // USB operational
	
	if (phED->TailP > 0) {
		phTD = (OHCI_PS_UHP_TD)phED->TailP;			
	} else {
		phTD = (OHCI_PS_UHP_TD)phED->HeadP;
	}
	tControl = &phTD->Control;
	
	if (pED->Type == ED_TYPE_CONTROL) {			
		ohci->HcCommandStatus |= OHCI_CS_CLF; //CLF
	} else if (pED->Type == ED_TYPE_BULK) {		
		ohci->HcCommandStatus |= OHCI_CS_BLF; //BLF
	} else {
		
	}
	
#ifndef OHCI_POLING		
	// not interrupt or ISO transaction
	if (pED->Type != OHCI_INT_ED) {
		OSSemPend(ohci_sem, 100, &err);
			
		if (err == OS_NO_ERR) {
			
			phED->Control |= (1 << 14); // Set sKip
			
			if ((phED->HeadP & 0x01)!=0) {
				   OHCI_DBG("HALTED: 0x%02X\r\n", (phED->HeadP & 0x03));
				   //print_cc(((*tControl) >> 28) & 0x0F);
				   return -1;
			}
			if ((phED->HeadP & 0xFFFFFFF0) != phED->TailP) {
				OHCI_DBG("HALTED: 0x%02X\r\n", (phED->HeadP & 0x03));
				//print_cc(((*tControl) >> 28) & 0x0F);
				return -1;
			}
	
			if (pED->Type == ED_TYPE_BULK) {
				pED->Trigger = ((((*tControl) >> 24) & 0x00000003) | 0x00000002);
			}
			return 0;		
		} else if (err == OS_TIMEOUT) {
		   phED->Control |= (1 << 14);
		   OHCI_DBG("OHCI TIMEOUT: 0x%02X\r\n", (phED->HeadP & 0x03));
		   if ((phED->HeadP & 0x01)!=0) {
			   OHCI_DBG("HALTED TIMEOUT: 0x%02X\r\n", (phED->HeadP & 0x03));
			   //print_cc(((*tControl) >> 28) & 0x0F);
			   // Clear HALT bit
			   phED->HeadP &= ~(1 << 0);		 
		       return pED->Endpoint;
		   }
			return -1;
		}
	} else {
		// only for interrupt and ISO transactions
		do {
			
		   if (pED->stop_poling) {
			   return -1;
		   }

			// HALTED
		   if ((phED->HeadP & 0x01)!=0) {
			   phED->Control |= (1 << 14);
			   OHCI_DBG("HALTED: 0x%02X\r\n", (phED->HeadP & 0x03));
			   //print_cc(((*tControl) >> 28) & 0x0F);
			   // Clear HALT bit
			   phED->HeadP &= ~(1 << 0);		 
		       return pED->Endpoint;
		   }

		   if ((phED->HeadP & 0xFFFFFFF0) == phED->TailP) {
			   phED->Control |= (1 << 14);
			   
			   if (pED->Type == ED_TYPE_BULK) {
				   pED->Trigger = ((((*tControl) >> 24) & 0x00000003) | 0x00000002); 
			   }
		       return 0;
		   }
		}  while (1);
	}
#else	
	do {
		if (pED->Type != OHCI_INT_ED) {
			if (!TestTimeout(&timeout)) {
				OHCI_DBG("Timeout\r\n");
				print_cc(((*tControl) >> 28) & 0x0F);
				return -1;
			}
		} else {
			if (pED->stop_poling) {
				return -1;
			}
		}
		// HALTED
	   if ((phED->HeadP & 0x01)!=0) {
		   phED->Control |= (1 << 14);
		   OHCI_DBG("HALTED: 0x%02X\r\n", (phED->HeadP & 0x03));
		   print_cc(((*tControl) >> 28) & 0x0F);
		   // Clear HALT bit
		   phED->HeadP &= ~(1 << 0);		 
	       return pED->Endpoint;
	   }

	   if ((phED->HeadP & 0xFFFFFFF0) == phED->TailP) {
		   phED->Control |= (1 << 14);
		   
		   if (pED->Type == ED_TYPE_BULK) {
			   pED->Trigger = ((((*tControl) >> 24) & 0x00000003) | 0x00000002); 
		   }
	       return 0;
	   }
	}  while (1);
#endif	
	
	return -1;
}




unsigned char *pFileBuf;


/*
 * 
 */
static void *ohci_wait_event(OS_EVENT *mbox, unsigned int timeout) {
	unsigned char err;
	void *msg;
	
	if (mbox == NULL) return NULL;
	
	msg = OSMboxPend(mbox, timeout, &err);
	if (err == OS_NO_ERR) {
		return msg;
	} else 
		return NULL;
}

/*
 * 
 */
static void ohci_post_event(OS_EVENT *mbox, unsigned int msg) {
	if (mbox == NULL) return;
	
	OSMboxPost(mbox, (void *)msg);
}

/*
 * 
 */
static int ohci_create_control_ed(USBDevice_t *pD) {
	
	PS_UHP_ED pED;
	OHCI_PS_UHP_ED phED;
	
	pD->dev_eds[OHCI_CONTROL_ED].direction = ED_GEN_FORMAT; //control
	pD->dev_eds[OHCI_CONTROL_ED].type = OHCI_CONTROL_ED;
	
	pED = ohci_create_ed(64,				// Max packet
						ED_GEN_FORMAT,		// TD format
						ED_SKIP,					// Skip
						(pD->speed & 0x80)? ED_LOW_SPEED: ED_FUL_SPEED,		// Speed
						ED_GET_FROM_TD,	// Direction
						OHCI_CONTROL_ED,					// Endpoint number
						0,					// Func Address
						0,					// TDQTailPointer
						0,					// TDQHeadPointer
						0,					// ToggleCarry
						0,					// NextED
						OHCI_CONTROL_ED,
						64);				// Type (Control)
	if (!pED) return -1;
	
	phED = pED->phED;
	if (!phED) return -1;

	
	pD->dev_eds[OHCI_CONTROL_ED].pED = (void *)pED;
	pD->dev_eds[OHCI_CONTROL_ED].max_packet = 64;
	pD->dev_eds[OHCI_CONTROL_ED].present = 1;
	
	ohci_add_ed(pED);
	return 0;
}

/*
 * 
 */
static void ohci_parse_descriptor(USBDevice_t *pD, void *pDin) {
	unsigned char *p;
	unsigned char desc_len, tot_len = 9, desc_type, cur_len = 0, ep_num = 0;
	
#define DESC_DEVICE		1	
#define DESC_CONFIG		2
#define DESC_STRING		3
#define DESC_INTERF		4
#define DESC_EP			5
#define DESC_DQ			6
#define DESC_OSC		7
#define DESC_IP			8
#define DESC_OTG		9
	
	p = pDin;
	
	while (1) {
		desc_len = *p;
		desc_type = *(p + 1);
	
		if (desc_type == DESC_CONFIG) {
			tot_len = *(p + 2);
			memcpy(&pD->config_descriptor, p, desc_len);
		} else if (desc_type == DESC_INTERF) {
			memcpy(&pD->interface_descriptor, p, desc_len);
		} else if (desc_type == DESC_EP) {
			memcpy(&pD->ed_descriptor[ep_num++], p, desc_len);
		} else {
			
		}
		
		p += desc_len;
		cur_len += desc_len;
		if (cur_len >= tot_len) break;		
	}
}

/*
 * 
 */
static void ohci_set_ed_size(PS_UHP_ED pED, unsigned int size) {
	pED->phED->Control &= ~(0xFF << 16); 
	pED->phED->Control |= (size << 16);
}

/*
 * 
 */
static int ohci_get_device_info(USBDevice_t *pD) {
	
	PS_UHP_ED pED;
	unsigned char bbuf[64];
	
	if (!pD) return -1;
	
	pED = (PS_UHP_ED)(pD->dev_eds[OHCI_CONTROL_ED].pED);
	if (!pED) return -1;
	
	if (ohci_device_config_request(pED, (void *)&(pD->device_descriptor))) {
		OHCI_DBG("!!!ERROR: Get device descriptor\r\n");
		return -1;
	}	
	
	if (!pD->device_descriptor.bDescriptorType) return -1;
	
	if (pED->Max_packet != pD->device_descriptor.bMaxPacketSize0) {
		ohci_set_ed_size(pED, pD->device_descriptor.bMaxPacketSize0);		
		if (ohci_device_config_request(pED, (void *)&(pD->device_descriptor))) {
			OHCI_DBG("!!!ERROR: Get device descriptor\r\n");
			return -1;
		}	
	}
	
	//print_dev_data((void *)&(pD->device_descriptor));
	if (ohci_set_address_request(pED, (void *)bbuf, pD->dev_num + 1)) {
		OHCI_DBG("!!!ERROR: Set address[%d]\r\n", pD->dev_num + 1);
		return -1;
	}
	pD->fun_addr = pD->dev_num + 1;
	OSTimeDly(20);
	if (ohci_config_request(pED, (void *)bbuf)) {
		OHCI_DBG("!!!ERROR: Get configuration descriptor\r\n");
		return -1;
	}
	ohci_parse_descriptor(pD, (void *)bbuf);
	
	printf("\r\n");
	pD->ed_num = pD->interface_descriptor.bNumEndpoints;
	//print_dev_config((void *)&(pD->config_descriptor));

	if (ohci_set_configured_request(pED, (void *)bbuf)) {
		OHCI_DBG("!!!ERROR: Set configured\r\n");
		return -1;
	}	
	return 0;	
}

/*
 * 
 */
static int ohci_set_device_configuration(USBDevice_t *pD) {
	unsigned int i;
	PS_UHP_ED pED;
	OHCI_PS_UHP_ED phED;
	unsigned int dir;
	unsigned int addr;
	unsigned int type = 0;
	unsigned int amount;
		
	for (i = 0;i < pD->ed_num;i++) {
		dir = (pD->ed_descriptor[i].bEndpointAddress & 0x80)? ED_IN: ED_OUT;
		addr = pD->ed_descriptor[i].bEndpointAddress & 0x0F;
		
		if ((pD->ed_descriptor[i].bmAttributes & 0x03) == 2)
			type = OHCI_BULK_ED;
		else if ((pD->ed_descriptor[i].bmAttributes & 0x03) == 3)
			type = OHCI_INT_ED;

		amount = pD->ed_descriptor[i].wMaxPacketSize;

		pED = ohci_create_ed(pD->ed_descriptor[i].wMaxPacketSize,				// Max packet
				 			ED_GEN_FORMAT,		// TD format
				 			ED_SKIP,			// Skip
				 			(pD->speed & 0x80)? ED_LOW_SPEED: ED_FUL_SPEED,		// Speed
				 			dir,				// Direction
				 			addr,				// Endpoint number
				 			pD->fun_addr,		// Func Address
				 			0,					// TDQTailPointer
				 			0,					// TDQHeadPointer
				 			0,					// ToggleCarry
				 			0,					// NextED
				 			type,				// Type (Bulk, Int, Iso)
				 			amount);			// EP max packet size	
		if (!pED) return -1;
			
		phED = pED->phED;
		if (!phED) return -1;
		
		pD->dev_eds[i + 1].pED = (void *)pED;
		pD->dev_eds[i + 1].max_packet = pD->ed_descriptor[i].wMaxPacketSize;
		pD->dev_eds[i + 1].direction = dir; 
		pD->dev_eds[i + 1].present = 1;
		
		ohci_add_ed(pED);
	}
	return 0;
}

/*
 * 
 */
int ohci_init_ed_list() {
	PS_UHP_ED pControl, pBulk, pInt;
	OHCI_PS_UHP_HCCA pHcca = &HCCA;
	// Create empty EDs to start ed list
	
	// Create Control ED
	pControl = ohci_create_ed(64,				// Max packet
							ED_GEN_FORMAT,		// TD format
							ED_SKIP,					// Skip
							ED_FUL_SPEED,		// Speed
							ED_GET_FROM_TD,	// Direction
							0,					// Endpoint number
							0,					// Func Address
							0,					// TDQTailPointer
							0,					// TDQHeadPointer
							0,					// ToggleCarry
							0,					// NextED
							OHCI_CONTROL_ED,
							64);				// Type (Control)
	if (!pControl) 
		return -1;
	
	ohci->HcControlHeadED = (unsigned int) pControl->phED;
	ohci->HcControlCurrentED = (unsigned int) pControl->phED;
	// Create Bulk ED	
	pBulk = ohci_create_ed(64,				// Max packet
 							ED_GEN_FORMAT,		// TD format
 							ED_SKIP,			// Skip
 							ED_FUL_SPEED,		// Speed
 							ED_GET_FROM_TD,		// Direction
 							0,					// Endpoint number
 							0,					// Func Address
 							0,					// TDQTailPointer
 							0,					// TDQHeadPointer
 							0,					// ToggleCarry
 							0,					// NextED
 							OHCI_BULK_ED,
 							64);	
	if (!pBulk)
		return -1;
	
	ohci->HcBulkHeadED = (unsigned int) pBulk->phED;
	ohci->HcBulkCurrentED = (unsigned int) pBulk->phED;
	// Create Interrupt ED
	pInt = ohci_create_ed(64,				// Max packet
 							ED_GEN_FORMAT,		// TD format
 							ED_SKIP,			// Skip
 							ED_LOW_SPEED,		// Speed
 							ED_IN,		// Direction
 							0,					// Endpoint number
 							0,					// Func Address
 							0,					// TDQTailPointer
 							0,					// TDQHeadPointer
 							0,					// ToggleCarry
 							0,					// NextED
 							OHCI_INT_ED,
 							64);	
	if (!pInt)
		return -1;
	
	ohci->HcPeriodCurrentED = (unsigned int) pInt->phED;
	pHcca->UHP_HccaInterruptTable[0] = (unsigned int) pInt->phED;	
	return 0;
}

void ohci_os_init() {
	ohci_sem = OSSemCreate(0);
	ohci_int_sem = OSSemCreate(0);
	OHCI_msg = OSMboxCreate(NULL);
	ohci_lock = OSSemCreate(1);
}
/*
 * 
 */
void ohci_cpu_init() {
	int i;	
	// Open clocks for USB host
	CPU_ConfigureUHPClock();
	// Forcing UHP_Hc to reset
	// Disabling port power
	ohci->HcRhPortStatus[0] = (1 << 9);
	ohci->HcRhPortStatus[1] = (1 << 9);
	OSTimeDly(100);
	ohci->HcControl = 0;	
	OSTimeDly(10);	
	ohci->HcCommandStatus = 0x01;
	
	for (i = 0; i < 10; i++) {	
		int hcr;
		OSTimeDly(1);
		hcr = ohci->HcCommandStatus & 0x01;
		if (!hcr) break;
	}
	ohci->HcInterruptEnable = (IMASK | OHCI_MIE);		
	// Writing the UHP_HCCA
	ohci->HcHCCA = (unsigned int) &HCCA;
	// Enabling list processing
	ohci->HcControl = 0;
	// Set the frame interval
	ohci->HcFmInterval = FMINTERVAL;
	ohci->HcPeriodicStart = PRDSTRT;
	// Create a default endpoint descriptors
	ohci_init_ed_list();
	// Initializing the UHP_HcDoneHead
	ohci->HcDoneHead = 0;
	HCCA.UHP_HccaDoneHead = 0x0000;

	// Forcing UHP_Hc to Operational State
	ohci->HcControl = 0x80;
	ohci->HcRhStatus = (1 << 16);
	// Enabling port power
	ohci->HcRhPortStatus[0] = 0x00000100;
	ohci->HcRhPortStatus[1] = 0x00000100;

	ohci->HcRhStatus = 0x00010000;

	UHP_Isr_Init();
}

/*
 * 
 */
static void ohci_free_all_eds(USBDevice_t *pD) {
	unsigned int i;
	
	for (i = 0;i < MAX_EDS;i++) {
		if (pD->dev_eds[i].present) {
			ohci_skip_ed((PS_UHP_ED)pD->dev_eds[i].pED);
			if (ohci_remove_ed((PS_UHP_ED)pD->dev_eds[i].pED)) {
				OHCI_DBG("ERROR: Ed is not in phisical Ed list\r\n");
			}
			ohci_free_ed((PS_UHP_ED)pD->dev_eds[i].pED);
			memset(&pD->dev_eds[i], 0, sizeof(dev_ed_t));
		}
	}
}

void ohci_int_disable() {
	AT91C_BASE_AIC->AIC_IDCR = 1 << AT91C_ID_UHP;
	AT91C_BASE_AIC->AIC_SVR[ AT91C_ID_UHP ] = 0;
	AT91C_BASE_AIC->AIC_SMR[ AT91C_ID_UHP ] = 0;	
}

void ohci_free_ed_list() {
	unsigned int i;
	
	for (i = 0;i < 3;i++) {
		if (psUHPED[i]) {
			ohci_skip_ed((PS_UHP_ED)psUHPED[i]);
			ohci_free_ed((PS_UHP_ED)psUHPED[i]);
		}
	}
}

void ohci_os_shutdown() {
	unsigned char err;
	OSSemDel(ohci_sem, OS_DEL_ALWAYS, &err);
	OSSemDel(ohci_int_sem, OS_DEL_ALWAYS, &err);
	OSSemDel(OHCI_msg, OS_DEL_ALWAYS, &err);
	OSSemDel(ohci_lock, OS_DEL_ALWAYS, &err);
}

void ohci_shutdown() {
	unsigned char err;
	// Disable INT and CLOCK
	ohci_int_disable();
	// Disable port power
	ohci->HcRhPortStatus[0] = (1 << 9);
	ohci->HcRhPortStatus[1] = (1 << 9);
	ohci->HcRhStatus = (1 << 0);
	//ohci_os_shutdown();
	// Kill ED list
	for (err = 0;err < 2;err++) {
		if (USBDev[err].present) {
			ohci_free_all_eds(&USBDev[err]);
		}
	}		
	ohci_free_ed_list();
	ohci_clock_disable();
}

/*
 * 
 */
static unsigned int hub_status = 0;

static void ohci_hub_isr() {
	unsigned int hub_msg = 0;
	
	if ( (ohci->HcRhPortStatus[0] & 0x01) && (!(hub_status & (1 << 0))) ) {		
		hub_msg = OHCI_MSG_DEVICE_POWERED_1;
		hub_status |= (1 << 0);
		ohci_post_event(OHCI_msg, hub_msg);
	} else if ((ohci->HcRhPortStatus[1] & 0x01) && (!(hub_status & (1 << 1)))) {		
		hub_msg = OHCI_MSG_DEVICE_POWERED_2;
		hub_status |= (1 << 1);
		ohci_post_event(OHCI_msg, hub_msg);
	} else if (!(ohci->HcRhPortStatus[0] & 0x01) && (hub_status & (1 << 0))) {
		//OHCI_DBG("-I- Device detached from port 0\n\r");
		hub_status &= ~(1 << 0);
		ohci_post_event(OHCI_msg, OHCI_MSG_DEVICE_DETACHED_1);
		ohci->HcRhPortStatus[0] = (1 << 0);
	} else if (!(ohci->HcRhPortStatus[1] & 0x01) && (hub_status & (1 << 1))) {
		//OHCI_DBG("-I- Device detached from port 1\n\r");
		hub_status &= ~(1 << 1);
		ohci_post_event(OHCI_msg, OHCI_MSG_DEVICE_DETACHED_2);
		ohci->HcRhPortStatus[1] = (1 << 0);
	}
}



/*
 * 
 */
static int ohci_attach_device(USBDevice_t *pD) {
	
	cur_dev_num = 1;
	
	pD->state = DEVICE_STATE_POWERED;
	
	if (ohci_create_control_ed(pD)) {
		cur_dev_num = 0;
		return -1;				
	}
	ohci->HcControl |= (USBOPERATIONAL | OHCI_CL_CLE );
	if (ohci_get_device_info(pD)) {
		ohci_free_all_eds(pD);
		memset(pD, 0, sizeof(USBDevice_t));
		cur_dev_num = 0;
		return -1;
	}
		
	pD->state = DEVICE_STATE_CONFIGURED;
	
	if (ohci_set_device_configuration(pD)) {
		ohci_free_all_eds(pD);
		memset(pD, 0, sizeof(USBDevice_t));
		cur_dev_num = 0;
		return -1;
	}
	ohci->HcControl |= (USBOPERATIONAL | OHCI_CL_BLE | OHCI_CL_PLE);
	pD->lock_sem = OSSemCreate(1);
	return 0;
}

/*
 * 
 */
static int ohci_remove_device(USBDevice_t *pD) {
	unsigned char err;
	OSSemDel(pD->lock_sem, OS_DEL_ALWAYS, &err);
	ohci_free_all_eds(pD);
	pD->state = DEVICE_STATE_DETACHED;
	return 0;
}

#ifndef WITH_FS
extern int msd_init(unsigned int Unit);
#endif
/*
 * 
 */
int usb_load_driver(USBDevice_t *pD) {
	if (pD->interface_descriptor.bInterfaceClass == 0x08) {
#ifdef WITH_FS		
		if (FS_Init(pD->dev_num)) {
			//printf("ERROR init FS\r\n");
			return -1;
		}
#else 	
	msd_init(pD->dev_num);
#endif
	} /*else if (pD->interface_descriptor.bInterfaceClass == 0x03) {
		if (pD->interface_descriptor.bInterfaceProtocol == 0x02) {
			if (um_create_driver(pD)) {
				//printf("ERROR init HID\r\n");
				return -1;
			}
		} 
		
	}*/ else {
		//printf("ERROR can not create unknown device driver\r\n");
		return -1;
	}
	
	return 0;
}

/*
 * 
 */
int usb_unload_driver(USBDevice_t *pD) {
	if (pD->interface_descriptor.bInterfaceClass == 0x08) {
#ifdef WITH_FS		
		FS_Exit();
#endif		
		ohci_remove_device(pD);	
		return 0;
	} /*else if (pD->interface_descriptor.bInterfaceClass == 0x03) {
		if (pD->interface_descriptor.bInterfaceProtocol == 0x02) {
			um_remove_driver(pD);
			ohci_remove_device(pD);
			return 0;
		}		
	} */else {
		ohci_remove_device(pD);
		//printf("Unknown device!\r\n");
		return -1;
	}
	return -1;
}


#define OHCI_TASK_STK_SIZE    4096

static unsigned int ohci_stack[OHCI_TASK_STK_SIZE];

void ohci_task(void *parg) {
	USBDevice_t *pDevice = NULL;
	static unsigned int ohci_state = OHCI_IDLE;
	unsigned int message;
	unsigned int device = 0;
	OS_CPU_SR cpu_sr;
	volatile unsigned int i;
	
	ohci_os_init();
	ohci_cpu_init();
	
	while (1) {		
		switch (ohci_state) {
		case OHCI_IDLE:
			
			message = (unsigned int)ohci_wait_event(OHCI_msg, 0);
			
			if ((message & 0x7F) == OHCI_MSG_DEVICE_POWERED_1)  {
				device = 0;
				OS_ENTER_CRITICAL();
				// This loop is very important
				for (i = 0;i < 10000;i++);
				OS_EXIT_CRITICAL();
				ohci->HcRhPortStatus[0] = (1 << 4); // SetPortReset
				while (ohci->HcRhPortStatus[0] & (1 << 4)); // Wait for the end of reset
				ohci->HcRhPortStatus[0] = (1 << 1); // SetPortEnable				
				if (ohci->HcRhPortStatus[0] & OHCI_RH_LSDA)
					message |= OHCI_DEV_LOW_SPEED;
				
			} else if ((message & 0x7F) == OHCI_MSG_DEVICE_POWERED_2) {
				device = 1;
				OS_ENTER_CRITICAL();
				// This loop is very important
				for (i = 0;i < 10000;i++);
				OS_EXIT_CRITICAL();
				ohci->HcRhPortStatus[1] = (1 << 4); // SetPortReset
				while (ohci->HcRhPortStatus[1] & (1 << 4)); // Wait for the end of reset
				ohci->HcRhPortStatus[1] = (1 << 1); // SetPortEnable
				if (ohci->HcRhPortStatus[1] & OHCI_RH_LSDA)
					message |= OHCI_DEV_LOW_SPEED;
			} else if (message == OHCI_MSG_DEVICE_DETACHED_1) {
				device = 0;
			} else if (message == OHCI_MSG_DEVICE_DETACHED_2) {
				device = 1;
			}
				// create new device type
			pDevice = &USBDev[device];
						
			if (((message & 0x7F) == OHCI_MSG_DEVICE_POWERED_1) || 
				((message & 0x7F) == OHCI_MSG_DEVICE_POWERED_2))  {
				if (ohci->HcRhPortStatus[0] & OHCI_RH_LSDA)
				
				OSTimeDly(50);
				
				memset(pDevice, 0, sizeof(USBDevice_t));
				pDevice->speed = (message & 0x80);
				pDevice->dev_num = device;
				pDevice->present = 1;
							
				if (ohci_attach_device(pDevice)) {
					if (device == 0) {
						ohci->HcRhPortStatus[0] = (1 << 9);
						ohci->HcRhPortStatus[0] = (1 << 0);
					} else {
						ohci->HcRhPortStatus[1] = (1 << 9);
						ohci->HcRhPortStatus[1] = (1 << 0);
					}
					continue;
				} 
				usb_load_driver(pDevice);
				
			} else if ((message == OHCI_MSG_DEVICE_DETACHED_1) ||
					(message == OHCI_MSG_DEVICE_DETACHED_2)) {
				pDevice->present = 0;
				usb_unload_driver(pDevice);
			}
			
			break;
		case OHCI_DEVICE:
			break;
		case OHCI_OPER:
			break;
		case OHCI_ERROR:
			break;
		default:
			ohci_state = OHCI_IDLE;
			break;
		} //end of switch
		
	} //end of while
}

#define UF_START_BLOCK		1048576 // 1MB
#define UF_TEST_SIZE		1048576 * 1

static int usb_flash_test(unsigned int iter, unsigned int rw, unsigned int size) {
	unsigned int i;
	double time1, time2, per;
	unsigned char *pBufw, *pBufr;
	char *wdata = "ABCDEFJHIJKLMNOPQRSTUVWXYZ";
	unsigned int size_table[] = { 512, 1024, 2048, 4096, 8192, 16384, 32768,
			49152, 65536 };
	USBDevice_t *pDevice = &USBDev[0];
	unsigned int sz = 0;
#ifdef OHCI_STAT
	unsigned int byte_cnt_in = 0;
	unsigned int byte_cnt_out = 0;
	double dt = 0, din = 0, dout = 0;
	unsigned int terr = 0;
#endif		
	if (pDevice->state != DEVICE_STATE_CONFIGURED) {
		printf("No USB Flash device 0\r\n");
		return -1;
	}
		
	for (i = 0; i < sizeof(size_table); i++) {
		if (size == size_table[i]) { 
			sz = size;			
		}
	}

	if (!sz)
		sz = 512;

	pBufw = (unsigned char *)safe_malloc(sz);
	pBufr = (unsigned char *)safe_malloc(sz);
	if (!pBufw || !pBufr) {
		printf("Not enough memory\r\n");
		return -1;
	}
	puts("Test starting\r\n");
	time1 = (double)OSTimeGet();
#ifdef OHCI_STAT				
	byte_cnt_in = ohci_in_packet_count;
	byte_cnt_out = ohci_out_packet_count;
#endif		
	for (i = 0; i < ((UF_TEST_SIZE * iter)/sz); i++) {
		memset(pBufw, wdata[(i % 26)], sz);
		if (rw & 1) {
			if (usb_flash_write(pDevice, (void *)pBufw, ((UF_START_BLOCK + (i * sz)) / 512), sz)) {
				printf("Write err\r\n");		
				terr++;
				break;
			}
		}

		if (rw & 2) {
			if (usb_flash_read(pDevice, (void *)pBufr, ((UF_START_BLOCK + (i * sz))/ 512), sz)) {
				printf("Read err\r\n");
				break;
			}
		}
		if (rw == 3) {
			if (memcmp(pBufw, pBufr, sz)) {
				printf("Data comparsion error\r\n");
				printf("Block: %d\r\n", ((UF_START_BLOCK + (i * sz))/ 512));
			}
		}
		
		per = (double)((((double)i)/((UF_TEST_SIZE * iter)/sz)) * 100);

		printf("Complete: %.f%%\r", per);

	}
	safe_free(pBufw);
	safe_free(pBufr);

	time2 = (double)OSTimeGet();
	printf("\r\n");
#ifdef OHCI_STAT	
	dt = (double)((time2 - time1)/OS_TICKS_PER_SEC);
	printf("Time: %.2f Sec\r\n", dt);
	if (rw & 2)
		din = (ohci_in_packet_count - byte_cnt_in)/dt;
	if (rw & 1)
		dout = (ohci_out_packet_count - byte_cnt_out)/dt;
	printf("USB Total speed: %.2f Bytes/s\r\n", (din + dout));
#endif
	return 0;
}

int usbh_test(char *command, char *operation, int iter, int size) {
	unsigned int rw = 0;

	if (!strcmp(operation, "rw"))
		rw = 3;
	else if (!strcmp(operation, "w"))
		rw = 1;
	else if (!strcmp(operation, "r"))
		rw = 2;
	else
		rw = 2;

	//usb_flash_test(iter, rw, size);
	usb_flash_test(iter, rw, size);
	return 0;
}

int usbh_clear_ep(char *command, int epnum, int param2, int param3) {
	PS_UHP_ED pED;
	USBDevice_t *pD = &USBDev[0];
	pED = (PS_UHP_ED)(pD->dev_eds[OHCI_CONTROL_ED].pED);
	ohci_usb_clear_halt(pED, epnum);
	return 0;
}

int usbh_restart_uhp(char *command, int param1, int param2, int param3) {
	ohci_shutdown();
	OSTimeDly(50);
	ohci_cpu_init();
	OSTimeDly(50);
	ohci_post_event(OHCI_msg, OHCI_MSG_DEVICE_POWERED_1);
	return 0;
}

/*
 *  
 */
void InitUHP(void) {
	
	OSTaskCreateExt( ohci_task,
                     ( void * ) 0,
                     ( OS_STK * ) &ohci_stack[ OHCI_TASK_STK_SIZE - 1 ],
                     OHCI_TASK_PRIORITY,
                     OHCI_TASK_PRIORITY,
                     ( OS_STK * ) &ohci_stack[ 0 ],
                     OHCI_TASK_STK_SIZE,
                     ( void * ) 0,
                     OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR );
	
		
}
