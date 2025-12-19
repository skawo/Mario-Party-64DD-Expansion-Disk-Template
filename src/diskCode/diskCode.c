#include "diskCode.h"

#include <libdragon.h>

extern void* __FileSystemStart;
#define FILE_OFFS(file) (((u32)&__FileSystemStart) + file)

void RebootSetup(void* frameBuffer);

static resolution_t res = RESOLUTION_320x240;
static bitdepth_t bit = DEPTH_32_BPP;

void Reboot()
{
    extern char _gp; 
    asm volatile("la $gp, %0" : : "i"(&_gp));

    debug_init_isviewer();
    debugf("Libdragon IPL3\n");
    debugf("TES1T\n");
    display_init( res, bit, 2, GAMMA_NONE, FILTERS_RESAMPLE );

    debugf("TES21T\n");

    static display_context_t disp = 0;
    disp = display_get();

    u32* test = (u32*)0x80700000;
    *test = 0xDEADBEEF;

    while (1)
    {
        debugf("TEST\n");

        graphics_fill_screen( disp, 0 );

        graphics_draw_text( disp, 20, 20, "TESTING" );

        display_show(disp);
    }
}