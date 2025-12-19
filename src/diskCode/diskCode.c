#include "diskCode.h"

extern void* __FileSystemStart;
#define FILE_OFFS(file) (((u32)&__FileSystemStart) + file)

void RebootSetup(void* frameBuffer);

void Reboot()
{
    u32* frameBuffer = (u32*)0xA0100000;
    RebootSetup(frameBuffer);

    u32* graphic = (u32*)FILE_OFFS(TITLE_YAZ0);
    ddYaz0_Decompress((u8*)graphic, (u8*)frameBuffer, TITLE_YAZ0_LEN);

    INFINITE_LOOP;
}

void RebootSetup(void* frameBuffer)
{
    #define WRITE_REG(reg, data) *((u32*)K0_TO_K1(reg)) = (u32)data

    WRITE_REG(VI_STATUS_REG, VI_CTRL_TYPE_16 | 
                             VI_CTRL_GAMMA_DITHER_ON | 
                             VI_CTRL_DIVOT_ON | 
                             VI_CTRL_ANTIALIAS_MODE_1 |
                             VI_CTRL_PIXEL_ADV_DEFAULT);
    WRITE_REG(VI_ORIGIN_REG, frameBuffer);
    WRITE_REG(VI_WIDTH_REG, WIDTH(SCREEN_WIDTH));
    WRITE_REG(VI_V_INTR_REG, VINTR(2));
    WRITE_REG(VI_V_CURRENT_LINE_REG, VCURRENT(0));
    WRITE_REG(VI_TIMING_REG, BURST(57, 34, 5, 62));
    WRITE_REG(VI_V_SYNC_REG, VSYNC(525));
    WRITE_REG(VI_H_SYNC_REG, HSYNC(3093, 0));
    WRITE_REG(VI_H_SYNC_LEAP_REG, LEAP(3093, 3093));
    WRITE_REG(VI_H_VIDEO_REG, HSTART(108, 748));
    WRITE_REG(VI_V_VIDEO_REG, VSTART(37, 511));
    WRITE_REG(VI_V_BURST_REG, BURST(4, 2, 14, 0));
    WRITE_REG(VI_X_SCALE_REG, SCALE(2, 0));
    WRITE_REG(VI_Y_SCALE_REG, SCALE(1, 0)); 
}