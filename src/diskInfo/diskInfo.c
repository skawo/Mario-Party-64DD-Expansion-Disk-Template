#include "diskInfo.h"

__attribute__((section(".diskInfo")))
diskInfo diskInfoData = 
{
    .LBAStart = 50,
    .LBAAmount = 25,
};