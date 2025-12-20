#include "diskCode.h"

#include <libdragon.h>

extern void* __FileSystemStart;
#define FILE_OFFS(file) (((u32)&__FileSystemStart) + file)

static resolution_t res = RESOLUTION_320x240;
static bitdepth_t bit = DEPTH_16_BPP;

extern void *  VI_callback;
extern int __boot_memsize;

extern void __init_interrupts();
extern void __joybus_init();
extern void __inspector_init();
extern void __rdpq_paragraph_char_check_bitfield();
extern void display_show_force(display_context_t dc);

void Disk_Main()
{   
    debug_init_isviewer();

    display_close();
    display_init( res, bit, 2, GAMMA_NONE, FILTERS_RESAMPLE );

    graphics_set_default_font();

    debugf("TEST2\n");

    int x = 20;
    int y = 20;
    int vx = 1; 
    int vy = 2;

    while (1)
    {
        display_context_t disp = 0;
        disp = display_get();

        debugf("disp: %x\n", disp);

        graphics_fill_screen( disp, 0 );
        graphics_draw_text( disp, x, y, "HELLO WORLD" );
        
        display_show_force(disp);

        x += vx;
        y += vy;

        if (x < 0 || x > display_get_width() - 8 * 11)
            vx = -vx;
        if (y < 0 || y > display_get_height() - 8)
            vy = -vy;

        wait_ms(10);


    }
}