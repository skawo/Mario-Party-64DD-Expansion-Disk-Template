#include "diskCode.h"
#include "dd.h"
#include "dd_rtc.h"

#include <libdragon.h>

extern void* __FileSystemStart;
extern void* __FileSystemStartROM;
#define RAM_OFFS(file) (((u32)&__FileSystemStart) + file)
#define DISK_OFFS(file) (u32)(((u32)&__FileSystemStartROM) + file)

static resolution_t res = RESOLUTION_320x240;
static bitdepth_t bit = DEPTH_16_BPP;

extern void* CART_callback;
extern int __boot_memsize;

extern void __init_interrupts();
extern void __joybus_init();
extern void __inspector_init();
extern void __rdpq_paragraph_char_check_bitfield();
extern void display_show_force(display_context_t dc);

void cartInterruptHandler();
char diskID[4];
extern uint8_t diskReadBuffer[];
extern u32* readDest;
extern int readSize;

void Disk_Main()
{
    debug_init_isviewer();
    register_CART_handler(&cartInterruptHandler);

    sprite_t* sp = (sprite_t*)RAM_OFFS(MARIOSPRITE_BIN);
    int dd_present = detect64dd_ipl();

    if (dd_present > 0)
    {
        //Set PI domain 2 settings like the 64DD IPL
        io_write(0x04600024, 3); //Set domain 2 latency
        io_write(0x04600028, 6); //Set domain 2 strobe pulse
        io_write(0x0460002C, 6); //Set domain 2 page size
        io_write(0x04600030, 2); //Set domain 2 release

        debugf("Resetting 64DD...\n");

        //RESET 64DD
        io_write(ASIC_HARD_RESET, ASIC_RESET_CODE);

        while ((io_read(ASIC_STATUS) & LEO_STAT_RESET) == LEO_STAT_RESET)
            io_write(ASIC_CMD, ASIC_CLR_RSTFLG); //clear reset flag

        io_write(ASIC_BM_CTL, BM_MECHA_INT_RESET);

        getRTC_64dd();

        if (detectdisk() == 1)
        {
            debugf("Buffer is at %x\n", diskReadBuffer);

            uint32_t idstuff = readDiskID();
            readDiskSystemData();
        
            diskID[0] = (char)((idstuff & 0xFF000000) >> 24);
            diskID[1] = (char)((idstuff & 0x00FF0000) >> 16);
            diskID[2] = (char)((idstuff & 0x0000FF00) >> 8);
            diskID[3] = (char)((idstuff & 0x000000FF));

            if (!sp)
            {
                sp = aligned_alloc(32, MARIOSPRITE_BIN_LEN);
                readDisk(DISK_OFFS(MARIOSPRITE_BIN), MARIOSPRITE_BIN_LEN, (u8*)sp);
            }
        }

    }

    display_init( res, bit, 4, GAMMA_NONE, FILTERS_RESAMPLE );
    joypad_init();

    int x = 20;
    int y = 120;

    static display_context_t disp = 0;

    char dStr[256];
    char idStr[256];

    while (1)
    {
        disp = display_get();
        graphics_fill_screen( disp, 0 );

        if (!dd_present)
            graphics_draw_text( disp, 20, 20, "No 64DD connected...?");
        else
        {
            getRTC_64dd();

            sprintf(dStr, "Date: %02x/%02x/%02x - %02x:%02x:%02x\n\n", day, month, year, hour, min, sec);
            sprintf(idStr, "Disk ID: %c%c%c%c\n", diskID[0], diskID[1], diskID[2], diskID[3]);

            graphics_draw_text( disp, 20, 20, dStr);
            graphics_draw_text( disp, 20, 30, idStr);

            if (sp)
                graphics_draw_sprite( disp, x, y, sp);
        }

        display_show(disp); 

        joypad_poll();
        joypad_buttons_t keys = joypad_get_buttons_held(JOYPAD_PORT_1);          
        joypad_buttons_t keysPress = joypad_get_buttons_pressed(JOYPAD_PORT_1);      

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

void cartInterruptHandler()
{
    u32 stat = io_read(ASIC_STATUS);

    if (stat & LEO_STAT_MECHA_INT)
    {
        io_write(ASIC_BM_CTL, BM_MECHA_INT_RESET);
    }
}

void drawDiskBuffer(display_context_t* disp)
{
    int startX = 20;
    int startY = 20;
    int bytesPerRow = 16;
    int charWidth = 8;   // width of a single character in your font
    int charHeight = 8;  // height of a single character in your font

    for (int row = 0; row < (240 + bytesPerRow - 1) / bytesPerRow; row++)
    {
        int y = startY + row * charHeight;

        char hexStr[2 * bytesPerRow + 1];  // VLA
        char asciiStr[bytesPerRow + 1];  // VLA

        memset(hexStr, 0, sizeof(hexStr));
        memset(asciiStr, 0, sizeof(asciiStr));

        for (int col = 0; col < bytesPerRow; col++)
        {
            int i = row * bytesPerRow + col;
            if (i < 240)
            {
                snprintf(&hexStr[col * 2], 4, "%02X ", diskReadBuffer[i]);
                asciiStr[col] = (diskReadBuffer[i] >= 32 && diskReadBuffer[i] <= 126) ?
                                diskReadBuffer[i] : '.';
            }
            else
            {
                // pad hex string if this row is shorter than 16 bytes
                snprintf(&hexStr[col*3], 4, "   ");
                asciiStr[col] = ' ';
            }
        }

        // Draw hex string
        graphics_draw_text(*disp, startX, y, hexStr);

        // Draw ASCII string to the right of hex
        graphics_draw_text(*disp, startX + (bytesPerRow * charWidth * 2) + 10, y, asciiStr);
    }    
}