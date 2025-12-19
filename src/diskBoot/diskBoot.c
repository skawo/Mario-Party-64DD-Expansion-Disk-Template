#include "diskBoot.h"

void ScreenSetup(void* frameBuffer);

extern void* __IPL_Entry;
extern void* __ErrorIPL_Start;

void Disk_Boot()
{
    u32* frameBuffer = (u32*)0xA0100000;
    //ScreenSetup(frameBuffer);

    ddYaz0_Decompress((u8*)ERROR_IPL_YAZ0, (u8*)frameBuffer, ERROR_IPL_YAZ0_LEN);

    INFINITE_LOOP;
}

