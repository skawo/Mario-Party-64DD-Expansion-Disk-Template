#include "diskCode.h"

#include <libdragon.h>

extern void* __FileSystemStart;
#define FILE_OFFS(file) (((u32)&__FileSystemStart) + file)

static resolution_t res = RESOLUTION_320x240;
static bitdepth_t bit = DEPTH_16_BPP;

extern void* inthandler;
extern int __boot_memsize;

extern void __init_interrupts();
extern void __joybus_init();
extern void __inspector_init();
extern void __rdpq_paragraph_char_check_bitfield();
extern void display_show_force(display_context_t dc);

void Disk_Main()
{
    u32* intHandlerEntry1 = (u32*)0xA0000000;
    u32* intHandlerEntry2 = (u32*)0xA0000080;
    u32* intHandlerEntry3 = (u32*)0xA0000100;
    u32* intHandlerEntry4 = (u32*)0xA0000180;

    u32 jInstr = 0x08000000 | (((u32)&inthandler >> 2) & 0x03FFFFF);

    *intHandlerEntry1 = jInstr;
    *intHandlerEntry2 = jInstr;
    *intHandlerEntry3 = jInstr;
    *intHandlerEntry4 = jInstr;

    __init_interrupts();
    __joybus_init();
    __inspector_init();
    __rdpq_paragraph_char_check_bitfield();

    debug_init_isviewer();

    display_close();
    display_init( res, bit, 2, GAMMA_NONE, FILTERS_RESAMPLE );

    graphics_set_default_font();

    joypad_init();

    debugf("TEST2\n");

    int x = 20;
    int y = 20;

    while (1)
    {
        display_context_t disp = 0;
        disp = display_get();

        graphics_fill_screen( disp, 0 );
        //graphics_draw_text( disp, x, y, "HELLO WORLD" );

        sprite_t* sp = (sprite_t*)FILE_OFFS(MARIOSPRITE_BIN); 
        graphics_draw_sprite( disp, x, y, sp );

        display_show(disp);

        joypad_poll();
        joypad_buttons_t keys = joypad_get_buttons_held(JOYPAD_PORT_1);

        if (keys.d_up)
            y--;
        if (keys.d_down)
            y++;            
        if (keys.d_right)
            x++;     
        if (keys.d_left)
            x--; 
    }
}