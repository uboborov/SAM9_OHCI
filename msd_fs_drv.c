#include 	"fs_conf.h"
#include 	"fs_dev.h"

#include 	"fs_api.h"
#include 	"fs_os.h"
#include 	"fs_fsl.h"
#include 	"fs_int.h"
#include 	"api_int.h"

#include    "msd_drv.h"
#include    "ohci.h"
#include    "msd.h"
#include    "mass.h"
#include    "includes.h"

#include    <string.h>

#define MSD_BLOCKSIZE              0x200
 
static int dev_num = 0;

int msd_init(FS_u8 Unit) {
	
	unsigned char pBuf[64];
	dev_num = Unit;
	USBDevice_t *pDevice = &USBDev[dev_num];
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
	
	//printf("block_num: 0x%08X\r\n",drv->block_num );
	//printf("block_size: 0x%08X\r\n",drv->block_size );
	//printf("msd_capacity: %d\r\n",drv->msd_capacity );
	
	return 0;
}

int _FS_USB_DevStatus(FS_u8 Unit)
{
	USBDevice_t *pDevice = &USBDev[dev_num];

	if (pDevice->state != DEVICE_STATE_CONFIGURED)
		return -1;
	
	/*if (usb_flash_test_unit_ready(pDevice)) {
		return -1;
	}*/
    return(0);
}

int _FS_USB_DevIoCtl(FS_u8 Unit, FS_i32 Cmd, FS_i32 Aux, void *pBuffer)
{
  U8 c;
  
  return 0;
 /* switch (Cmd)
  {
  case FS_CMD_GET_DISK_ID:
    
    c = SD_ReadCID(Unit, &SD_CID2);
    if (c != 0) {
        cache_pos = 1;
        return -1;                              
    } 
    if (memcmp(&SD_CID1, &SD_CID2, sizeof(SD_CSD_TYPE))) {
      cache_pos = 1;
      return -2;
    }
    else {
      return 0;
    }
  default:
    return 0;
  }
*/
}

int _FS_USB_DevRead(FS_u8 Unit, FS_u32 SectorNo, void *pBuffer)
{
int x;
USBDevice_t *pDevice = &USBDev[dev_num];

	if (pDevice->state != DEVICE_STATE_CONFIGURED)
		return -1;

    if(Unit >= FS_USB_MAXUNIT) {
        return -1;  // No valid unit number
    }
    x = usb_flash_read(pDevice, pBuffer, SectorNo, ((MsdDrv_t *)pDevice->pDevDrv)->block_size);
    if(x != 0) {
        x = -1;
    }

    return(x);
}

int _FS_USB_DevWrite(FS_u8 Unit, FS_u32 SectorNo, void *pBuffer)
{
int x;
USBDevice_t *pDevice = &USBDev[dev_num];
	
	if (pDevice->state != DEVICE_STATE_CONFIGURED)
		return -1;

    if(Unit >= FS_USB_MAXUNIT) {
        return -1;  // No valid unit number
    }
    x = usb_flash_write(pDevice, pBuffer, SectorNo, ((MsdDrv_t *)pDevice->pDevDrv)->block_size);
    if(x != 0) {
        x = -1;
    }

    return(x);
}


const FS_DEVICE_TYPE 	FS__usbdevice_driver = {
  	"USB_device",
 	_FS_USB_DevStatus,
  	_FS_USB_DevRead,
  	_FS_USB_DevWrite,
  	_FS_USB_DevIoCtl
};
