Open Host Controller Interface (OHCI) for Atmel's AT91SAM9260, AT91SAM9G20
------------------------------------------------------------------------

OHCI ported to the embedded OS uCos-II. It easily can be ported to any embedded system even without OS.
The project includes the Mass Storage driver and port to the embedded file system as well.

Files:
 - **ohci.c** - OHCI low level driver 
 - **mass.c** - mass storage low level driver
 - **memory.c** - memory allocator with alignment  
 - **msd_fs_drv.c** - file system wrapper for mass storage driver

CONTACT:
Yury Bobrov
Email: ubobrov@yandex.ru
