KSEG1 = 0xA0000000;
BOOT_ORIGIN  = 0x80000400; 
LBA_LENGTH  = 0x4D08;
RAM_ORIGIN  = 0x80400000; 
ROM_LENGTH = 0x3DEC800;    
RAM_LENGTH  = 0x400000;      

BOOT_OFFS = BOOT_OFFS_VALUE;

OUTPUT_ARCH(mips)
OUTPUT_FORMAT("elf32-bigmips", "elf32-bigmips", "elf32-littlemips")

MEMORY {
    HEADER (rw) : ORIGIN = 0, LENGTH = BOOT_OFFS
    BOOT (rw) : ORIGIN = BOOT_ORIGIN, LENGTH = 40 * LBA_LENGTH

    ROM (rw) : ORIGIN = 0, LENGTH = ROM_LENGTH
    RAM (rw) : ORIGIN = RAM_ORIGIN, LENGTH = RAM_LENGTH
}

OFFSET = 0;

SECTIONS {
    .diskHeader : AT(OFFSET) ALIGN(1) 
    {
        KEEP(*(.diskHeader))
    } > HEADER   

    OFFSET += BOOT_OFFS;

    .boot : AT(OFFSET) ALIGN(1) 
    {
        . = ORIGIN(BOOT);
        __IPL_Entry = .;
        KEEP(*diskBoot.o(.text* .rodata* .data* .sdata* .bss* .sbss*))
        KEEP(*ddTool.o(.text* .rodata* .data* .sdata* .bss* .sbss*))
    } > BOOT

    OFFSET = 73 * LBA_LENGTH;

    .diskInfo 0 : AT(OFFSET) ALIGN(1) 
    {
        KEEP(*(.diskInfo))
        . = ALIGN(32);
    } > ROM   

    OFFSET = 74 * LBA_LENGTH;

    . = ORIGIN(RAM);
    .text : AT(OFFSET)
    {
        __text_start = .;
        __entry = .;
        *(.text*)
        *(.init)
        *(.fini)
        __text_end  = .;
    } > RAM

    .rodata ALIGN(32) :
    {
        *(.rodata*)
        *(.eh_frame*)
    } > RAM

    .data : {
        __data_start = .;
        *(.data)
        *(.data.*)
        KEEP(*(keep.data.*))
        *(.gnu.linkonce.d.*)
        . = ALIGN(8);
    } > RAM

    .sdata : {
        _gp = . + 0x8000;
        *(.sdata)
        *(.sdata.*)
        *(.gnu.linkonce.s.*)
        . = ALIGN(8);
    } > RAM

    .lit8 : {
        *(.lit8)
        . = ALIGN(8);
    } > RAM

    .lit4 : {
        *(.lit4)
        . = ALIGN(8);
    } > RAM

    .sbss : {
         __bss_start = .;
        *(.sbss)
        *(.sbss.*)
        *(.gnu.linkonce.sb.*)
        *(.scommon)
        *(.scommon.*)
    } > RAM

    . = ALIGN(8);
    .bss : {
        *(.bss)
        *(.bss*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(8);
         __bss_end = .;
    } > RAM

   .eh_frame_hdr : { *(.eh_frame_hdr) }
   .eh_frame : { 
        __EH_FRAME_BEGIN__ = .; /* Define symbol for accessing eh_frame section */
        KEEP (*(.eh_frame)) 
    } > RAM

    .ctors : {
        __CTOR_LIST__ = .;

        KEEP(*(.ctors))

        __CTOR_END__ = .;
    } > RAM    

    . = ALIGN(32);
    __ramEnd = .;
}
