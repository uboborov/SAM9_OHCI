#include "mass.h"
#include "includes.h"
static CBW_t cbw;

OS_EVENT *msd_sem;

static void msd_lock(OS_EVENT *sem) {
	unsigned char err;
	OSSemPend(sem, 0, &err);
}

static void msd_unlock(OS_EVENT *sem) {
	OSSemPost(sem);
}

/*
 * 
 */
PS_UHP_ED get_ed(USBDevice_t *pD, unsigned int type, unsigned int dir) {
	
	unsigned int i;
	PS_UHP_ED pED = NULL;
	
	if (!pD) return NULL;
		
	for (i = 0;i < MAX_EDS;i++) {
		if ((((PS_UHP_ED)(pD->dev_eds[i].pED))->Type == type) && (((PS_UHP_ED)(pD->dev_eds[i].pED))->Direction == dir))
			pED =  ((PS_UHP_ED)pD->dev_eds[i].pED);
	}
	return pED;
}

/*
 * 
 */
int mass_read_csw(PS_UHP_ED pEDIn, CSW_t *pCsw) {
	
	if (ohci_bulk_read(pEDIn, (void *)pCsw, 13, 1))
		return -1;
	// Signature check
	if (pCsw->dCSWSignature != 0x53425355) {
		OHCI_DBG("USB Flash: Wrong signature\r\n");
		return -1;
	}
	if (pCsw->bCSWStatus == 0x01) {
		OHCI_DBG("USB Flash: Comand faild\r\n");
		return -2;
	} else if (pCsw->bCSWStatus == 0x02) {
		OHCI_DBG("USB Flash: Phase error\r\n");
		return -3;
	} else if (pCsw->bCSWStatus != 0) {
		OHCI_DBG("USB Flash: Status bits reserved\r\n");
		return 0;
	} else 
		return 0;
}

/*
 * 
 */
static CBW_t *mass_cmd(void *cmd, unsigned int cmd_size, unsigned int size, unsigned int dir) {
	CBW_t *pCBW = &cbw;
	
	memset(pCBW, 0, sizeof(CBW_t));
	
	pCBW->dCBWSignature = 0x43425355;
	pCBW->dCBWTag = 0x000000AA;//0x8730F008;
	pCBW->dCBWDataTransfereLength = size;
	pCBW->bmCBWFlags = dir?0x80:0x00;
	pCBW->bCBWLUN = 0;
	pCBW->bCBWCBLength = (unsigned char)cmd_size;
	memcpy(pCBW->CBWCB, cmd, cmd_size);
	
	return pCBW;
}

/*
 * 
 */
int usb_flash_inqury(USBDevice_t *pD, void *pOut) {
	CBW_t *pCBW;
	CSW_t pCSW;
	PS_UHP_ED EDIn, EDOut;
	unsigned char cmd = 0x12;
	
	msd_lock(pD->lock_sem);
	
	pCBW = mass_cmd((void *)&cmd, 1, 64, 1);
	if (!pCBW) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	EDIn = get_ed(pD, OHCI_BULK_ED, ED_IN);
	if (!EDIn) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	EDOut = get_ed(pD, OHCI_BULK_ED, ED_OUT);
	if (!EDOut) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_bulk_write(EDOut, (void *)pCBW, 31, 1)) {
		OHCI_DBG("BULK OUT ERROR: Inqury OUT request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	if (ohci_bulk_read(EDIn, pOut, 64, 1)) {
		OHCI_DBG("BULK IN ERROR: Inqury IN request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}	
	if (mass_read_csw(EDIn, &pCSW)) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	msd_unlock(pD->lock_sem);
	return 0;
}

/*
 * 
 */
int usb_flash_mode_sense(USBDevice_t *pD, void *pIn) {
	CBW_t *pCBW;
	CSW_t pCSW;	
	unsigned char cmd[12];
	PS_UHP_ED EDIn, EDOut;
	
	msd_lock(pD->lock_sem);
	
	memset(cmd, 0, 12);
	cmd[0] = 0x03;
	cmd[4] = 0x12;
	
	pCBW = mass_cmd((void *)cmd, 12, 18, 1);
	if (!pCBW) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	EDIn = get_ed(pD, OHCI_BULK_ED, ED_IN);
	if (!EDIn) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	EDOut = get_ed(pD, OHCI_BULK_ED, ED_OUT);
	if (!EDOut) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_bulk_write(EDOut, (void *)pCBW, 31, 1)) {
		OHCI_DBG("BULK OUT ERROR: Format OUT request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}

	if (ohci_bulk_read(EDIn, pIn, 64, 1)) {
		OHCI_DBG("BULK IN ERROR: Format IN request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (mass_read_csw(EDIn, &pCSW)) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	msd_unlock(pD->lock_sem);
	return 0;
}

/*
 * 
 */
int usb_flash_get_capacity(USBDevice_t *pD, void *pOut) {
	CBW_t *pCBW;
	CSW_t pCSW;	
	unsigned char cmd = 0x25;
	PS_UHP_ED EDIn, EDOut;
	int ret = 0;
	
	msd_lock(pD->lock_sem);
	
	pCBW = mass_cmd((void *)&cmd, 1, 64, 1);
	if (!pCBW) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	EDIn = get_ed(pD, OHCI_BULK_ED, ED_IN);
	if (!EDIn) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	EDOut = get_ed(pD, OHCI_BULK_ED, ED_OUT);
	if (!EDOut) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if ((ret = ohci_bulk_write(EDOut, (void *)pCBW, 31, 1)) != 0) {
		OHCI_DBG("BULK OUT ERROR: Capacity OUT request error\r\n");
		msd_unlock(pD->lock_sem);
		return ret;
	}

	if ((ret = ohci_bulk_read(EDIn, pOut, 64, 1)) != 0) {
		OHCI_DBG("BULK IN ERROR: Capacity IN request error\r\n");
		msd_unlock(pD->lock_sem);
		return ret;
	}
	 
	if ((ret = mass_read_csw(EDIn, &pCSW)) != 0) {
		msd_unlock(pD->lock_sem);
		return ret;
	}
	msd_unlock(pD->lock_sem);
	return 0;
	
}

/*
 * 
 */
int usb_flash_test_unit_ready(USBDevice_t *pD) {
	CBW_t *pCBW;
	CSW_t pCsw;
	PS_UHP_ED EDIn, EDOut;	
	unsigned char cmd[6];

	msd_lock(pD->lock_sem);
	
	memset(cmd, 0, 6);
	
	pCBW = mass_cmd((void *)cmd, 6, 0, 0);
	if (!pCBW) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	EDIn = get_ed(pD, OHCI_BULK_ED, ED_IN);
	if (!EDIn) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	EDOut = get_ed(pD, OHCI_BULK_ED, ED_OUT);
	if (!EDOut) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_bulk_write(EDOut, (void *)pCBW, 31, 1)) {
		OHCI_DBG("BULK OUT ERROR: Get CSW OUT request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	memset(&pCsw, 0x00, 13);
	if (ohci_bulk_read(EDIn, &pCsw, 13, 1)) {
		OHCI_DBG("BULK IN ERROR: Get CSW IN request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (pCsw.dCSWSignature != 0x53425355) {
		OHCI_DBG("USB Flash CSW: Wrong signature\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	if (pCsw.bCSWStatus == 0x01) {
		OHCI_DBG("USB Flash CSW: Comand faild\r\n");
		msd_unlock(pD->lock_sem);
		return 0;
	} else if (pCsw.bCSWStatus == 0x02) {
		OHCI_DBG("USB Flash CSW: Phase error\r\n");
		msd_unlock(pD->lock_sem);
		return 0;
	} else if (pCsw.bCSWStatus != 0) {
		OHCI_DBG("USB Flash CSW: Status bits reserved\r\n");
		msd_unlock(pD->lock_sem);
		return 0;
	} else {
		msd_unlock(pD->lock_sem);
		return 0;
	}
}


/*
 * 
 */
int usb_flash_read(USBDevice_t *pD, void *pOut, 
		unsigned int address, unsigned int size) {
	CBW_t *pCBW;
	CSW_t pCSW;	
	int i;
	PS_UHP_ED EDIn, EDOut;
		
	unsigned char cmd[9];
	//unsigned int *lba;
	unsigned short len = (size / 0x200);
	
	msd_lock(pD->lock_sem);
	
	memset(cmd, 0, 8);
	
	cmd[0] = 0x28;
	cmd[2] = address >> 24;
	cmd[3] = address >> 16;
	cmd[4] = address >> 8;
	cmd[5] = address >> 0;
	cmd[7] = (unsigned char)(len >> 8);
	cmd[8] = (unsigned char)len;
	
	pCBW = mass_cmd((void *)&cmd, 10, size, 1);
	if (!pCBW) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	EDIn = get_ed(pD, OHCI_BULK_ED, ED_IN);
	if (!EDIn){
		OHCI_DBG("OHCI: NO EDIn\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}

	EDOut = get_ed(pD, OHCI_BULK_ED, ED_OUT);
	if (!EDOut) {
		OHCI_DBG("OHCI: NO EDOut\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_bulk_write(EDOut, (void *)pCBW, 31, 1)) {
		printf("BULK OUT ERROR: CBW OUT request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_make_td_chain(EDIn, size, /*EDIn->Data_amount*/size, ED_IN, (void *)pOut)) {
		printf("BULK IN ERROR: Rear Sector[%i] IN request error\r\n",address);
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	/*for (i = 0;i < (size/EDIn->Data_amount); i++) {
		if (ohci_bulk_read(EDIn, pOut, EDIn->Data_amount, 1)) {
			OHCI_DBG("BULK IN ERROR: Rear Sector[%i] IN request error\r\n",address);
			return -1;
		}
		pOut += EDIn->Data_amount;
	}*/
	
	if (mass_read_csw(EDIn, &pCSW)) {
		OHCI_DBG("OHCI: CSW read error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	msd_unlock(pD->lock_sem);
	return 0;
}

/*
 * 
 */
int usb_flash_write(USBDevice_t *pD, void *pIn, 
		unsigned int address, unsigned int size) {
	CBW_t *pCBW;
	CSW_t pCSW;	
	int i;
	PS_UHP_ED EDIn, EDOut;
		
	unsigned char cmd[9];
	//unsigned int *lba;
	unsigned short len = (size / 0x200);
	
	msd_lock(pD->lock_sem);
	
	memset(cmd, 0, 8);
	
	cmd[0] = 0x2A;
	cmd[2] = address >> 24;
	cmd[3] = address >> 16;
	cmd[4] = address >> 8;
	cmd[5] = address >> 0;
	cmd[7] = (unsigned char)(len >> 8);
	cmd[8] = (unsigned char)len;
	
	pCBW = mass_cmd((void *)&cmd, 10, size, 0);
	if (!pCBW) {
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	EDIn = get_ed(pD, OHCI_BULK_ED, ED_IN);
	if (!EDIn){
		OHCI_DBG("OHCI: NO EDIn\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}

	EDOut = get_ed(pD, OHCI_BULK_ED, ED_OUT);
	if (!EDOut) {
		OHCI_DBG("OHCI: NO EDOut\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_bulk_write(EDOut, (void *)pCBW, 31, 1)) {
		OHCI_DBG("BULK OUT ERROR: Capacity OUT request error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
	if (ohci_make_td_chain(EDOut, size, /*EDOut->Data_amount*/size, ED_OUT, (void *)pIn)) {
		printf("BULK IN ERROR: Rear Sector[%i] IN request error\r\n",address);
		msd_unlock(pD->lock_sem);
		return -1;
	}
	
/*	for (i = 0;i < (size/EDOut->Data_amount); i++) {
		if (ohci_bulk_write(EDOut, pIn, EDOut->Data_amount, 1)) {
			OHCI_DBG("BULK OUT ERROR: Rear Sector[%i] OUT request error\r\n",address);
			return -1;
		}
		pIn += EDOut->Data_amount;
	}*/
	
	if (mass_read_csw(EDIn, &pCSW)) {
		OHCI_DBG("OHCI: CSW read error\r\n");
		msd_unlock(pD->lock_sem);
		return -1;
	}
	msd_unlock(pD->lock_sem);
	return 0;
}

int msd_init(unsigned int Unit) {
	
	unsigned char pBuf[64];
	
	USBDevice_t *pDevice = &USBDev[Unit];
	MsdDrv_t *drv;
		
	if(usb_flash_inqury(pDevice, (void *)pBuf)) {
		return -1;
	}	
	
	if(usb_flash_mode_sense(pDevice, (void *)pBuf)) {
		return -1;
	}
	
	if(usb_flash_get_capacity(pDevice, (void *)pBuf)) {
		return -1;
	}
	
	if(usb_flash_test_unit_ready(pDevice)) {
		return -1;
	}
	
	drv = (MsdDrv_t *)safe_malloc(sizeof(MsdDrv_t));
	if (!drv) {
		printf("ERROR: Can not allocate space for MSD driver\r\n");
		return -1;
	}
	drv->block_num = ((pBuf[0] << 24) | (pBuf[1] << 16) | (pBuf[2] << 8) | pBuf[3]);
	drv->block_size = ((pBuf[4] << 24) | (pBuf[5] << 16) | (pBuf[6] << 8) | pBuf[7]);
	drv->msd_capacity = drv->block_num * drv->block_size;
	pDevice->pDevDrv = (void *)drv;
	
	return 0;
}