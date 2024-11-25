#ifndef _FISH_BOOTLOADER_DEF_H
#define _FISH_BOOTLOADER_DEF_H

#define BOOTLOADER_ENABLE       0x01    //�Ƿ���BootLoader


#if (BOOTLOADER_ENABLE == 0)
#define APP_START_ADDR      0x8000000   //��ʹ��BootLoaderʱ��app����ʼ��ַ
#define APP_VECT_TAB_OFFSET 0x00        //�ж�������ƫ�Ƶ�ַ
#else
#define APP_START_ADDR      0x8020000   //ʹ��BootLoaderʱ��app����ʼ��ַ
#define APP_VECT_TAB_OFFSET 0x20000     //�ж�������ƫ�Ƶ�ַ
#endif

#endif /* _FISH_BOOTLOADER_DEF_H */

