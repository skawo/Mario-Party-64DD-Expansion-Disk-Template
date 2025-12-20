#include "diskBoot.h"

void ScreenSetup(void* frameBuffer);

extern void* __IPL_Entry;
extern void* __ErrorIPL_Start;

void Disk_Boot()
{
    u32* frameBuffer = (u32*)0xA0100000;
    //ScreenSetup(frameBuffer);


    INFINITE_LOOP;
}

