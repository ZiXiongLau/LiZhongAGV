#ifndef _FISH_BOOTLOADER_DEF_H
#define _FISH_BOOTLOADER_DEF_H

#define BOOTLOADER_ENABLE       0x01    //是否有BootLoader


#if (BOOTLOADER_ENABLE == 0)
#define APP_START_ADDR      0x8000000   //不使用BootLoader时，app的起始地址
#define APP_VECT_TAB_OFFSET 0x00        //中断向量表偏移地址
#else
#define APP_START_ADDR      0x8020000   //使用BootLoader时，app的起始地址
#define APP_VECT_TAB_OFFSET 0x20000     //中断向量表偏移地址
#endif

#endif /* _FISH_BOOTLOADER_DEF_H */

