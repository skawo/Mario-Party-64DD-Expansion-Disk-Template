#ifndef DISKINFO_H 
#define DISKINFO_H

#include "../common.h"
#include "../diskCode/diskCode.h"

typedef struct diskInfo 
{
    /* 0x0000 */ u32 LBAStart;
    /* 0x0008 */ u32 LBAAmount;
} diskInfo;

#endif // DISKINFO_H