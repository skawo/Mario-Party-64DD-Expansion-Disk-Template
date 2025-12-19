ifeq ($(OS),Windows_NT)
EXE=win
else
EXE=linux
endif

N64_GCCPREFIX ?= $(N64_INST)
N64_TARGET ?= mips64-elf
N64_ROOTDIR = $(N64_INST)
N64_BINDIR = $(N64_ROOTDIR)/bin
N64_INCLUDEDIR = $(N64_ROOTDIR)/$(N64_TARGET)/include
N64_LIBDIR = $(N64_ROOTDIR)/$(N64_TARGET)/lib
N64_GCCPREFIX_TRIPLET = $(N64_GCCPREFIX)/bin/$(addsuffix -,$(N64_TARGET))

N64_CC = $(CCACHE) $(N64_GCCPREFIX_TRIPLET)gcc
N64_CXX = $(CCACHE) $(N64_GCCPREFIX_TRIPLET)g++
N64_AS = $(N64_GCCPREFIX_TRIPLET)as
N64_AR = $(N64_GCCPREFIX_TRIPLET)gcc-ar
N64_LD = $(N64_GCCPREFIX_TRIPLET)ld
N64_OBJCOPY = $(N64_GCCPREFIX_TRIPLET)objcopy
N64_OBJDUMP = $(N64_GCCPREFIX_TRIPLET)objdump
N64_SIZE = $(N64_GCCPREFIX_TRIPLET)size
N64_NM = $(N64_GCCPREFIX_TRIPLET)nm
N64_STRIP = $(N64_GCCPREFIX_TRIPLET)strip

CC      = $(N64_CC)
LD      = $(N64_LD)
OBJCOPY = $(N64_OBJCOPY)
OBJDUMP = $(N64_OBJDUMP)

CFLAGS = -G 0 -Os -march=vr4300 -mtune=vr4300 -mabi=o64 -I$(N64_INCLUDEDIR) -lc -lm -lgcc -ldragon -ldragonsys -mips3 -mno-check-zero-division -fno-builtin -mno-explicit-relocs -mno-memcpy -fno-reorder-blocks -ffast-math -fno-unsafe-math-optimizations -mno-check-zero-division -fno-optimize-sibling-calls
LDFLAGS = -T disk.ld --emit-relocs $(N64_LIBDIR)/libdragon.a $(N64_LIBDIR)/libdragonsys.a $(N64_LIBDIR)/libc.a $(N64_LIBDIR)/libm.a

__IPL_ENTRY =
DISKREGION  = DISKREGION_USA
DISKRELEASE = DISKRELEASE_RETAIL
DISKFORMAT  = DISKFORMAT_NDD
BUILDTIME := $(shell date +'%Y %m %d %H %M %S')

YEAR1 := $(shell echo $(BUILDTIME) | awk '{print substr($$1,1,2)}')
YEAR2 := $(shell echo $(BUILDTIME) | awk '{print substr($$1,3,2)}')
MONTH := $(shell echo $(BUILDTIME) | awk '{print $$2}')
DAY   := $(shell echo $(BUILDTIME) | awk '{print $$3}')
HOUR  := $(shell echo $(BUILDTIME) | awk '{print $$4}')
MIN   := $(shell echo $(BUILDTIME) | awk '{print $$5}')
SEC   := $(shell echo $(BUILDTIME) | awk '{print $$6}')

BOOT_OFFS_VALUE = 0x738C0
TIMESTAMP = "{0x00,0x$(YEAR1),0x$(YEAR2),0x$(MONTH),0x$(DAY),0x$(HOUR),0x$(MIN),0x$(SEC)}"
OUTFILENAME = "ELBE.ndd"
PYTHON := $(shell which python3)

default: USA

USA: common

JPN: DISKREGION = DISKREGION_JPN
JPN: OUTFILENAME = "ELBJ.ndd"
JPN: common

USA-DEV: DISKRELEASE = DISKRELEASE_DEV
USA-DEV: OUTFILENAME = "ELBE-DEV.ndd"
USA-DEV: common

JPN-DEV: DISKREGION = DISKREGION_JPN
JPN-DEV: DISKRELEASE = DISKRELEASE_DEV
JPN-DEV: OUTFILENAME = "ELBJ-DEV.ndd"
JPN-DEV: common

#USA-D64: DISKFORMAT = DISKFORMAT_D64
#USA-D64: OUTFILENAME = "ELBE.d64"
#USA-D64: d64

#JPN-D64: DISKFORMAT = DISKFORMAT_D64
#JPN-D64: DISKREGION = DISKREGION_JPN
#JPN-D64: OUTFILENAME = "ELBJ.d64"
#JPN-D64: d64

OBJS = ddTool/ddTool.o \
diskInfo/diskInfo.o \
diskCode/diskCode.o \
ddTool/ddToolCode.o \
diskBoot/diskBoot.o \
diskSystem/diskSystem.o \
filesystem/filesystem.o \

common: clean-diskSystem $(OBJS)
	@rm -f ../*.disk
	@echo "Creating $(OUTFILENAME)..."
	@$(LD) -o disk.elf $(OBJS) --defsym=BOOT_OFFS_VALUE=0x738C0 $(LDFLAGS)
	@$(OBJCOPY) -R .MIPS.abiflags -O binary disk.elf ../$(OUTFILENAME)
	@dd if=/dev/zero bs=1 count=0 seek=$$((0x3DEC800)) of=../$(OUTFILENAME) 2>/dev/null
	@echo "Encrypting $(OUTFILENAME)..."
	@$(PYTHON) ../tool/encrypt.py "../$(OUTFILENAME)" "../$(OUTFILENAME)" 0x164450	

d64: clean-diskSystem $(OBJS)
	@rm -f ../*.disk
	@echo "Creating $(OUTFILENAME)..."
	@$(LD) -o disk.elf $(OBJS) --defsym=BOOT_OFFS_VALUE=0x200 $(LDFLAGS)
	@$(OBJCOPY) -R .MIPS.abiflags -O binary disk.elf ../$(OUTFILENAME)

ddTool/ddTool.o: ddTool/ddTool.c
	@echo "Compiling tools..."
	@$(CC) -DPREFIX=boot_ -c $< -o $@  $(CFLAGS)

ddTool/ddToolCode.o: ddTool/ddTool.c
	@$(CC) -DPREFIX=ram_ -c $< -o $@  $(CFLAGS)

filesystem/filesystem.o: filesystem/filesystem.s
	@echo "Compiling filesystem..."
	@$(CC) -c $< -o $@ $(CFLAGS)

diskInfo/diskInfo.o: diskInfo/diskInfo.c
	@echo "Compiling diskInfo..."
	@$(CC) -c $< -o $@ $(CFLAGS)

diskCode/diskCode.o: diskCode/diskCode.c
	@echo "Compiling diskCode..."
	@$(CC) -DPREFIX=ram_ -c $< -o $@  $(CFLAGS)

diskBoot/diskBoot.o: diskBoot/diskBoot.c
	@echo "Compiling diskBoot..."
	@$(CC) -DPREFIX=boot_ -c $< -o $@  $(CFLAGS)

diskSystem/diskSystem.o: diskSystem/diskSystem.c
	@echo "Compiling diskSystem..."
	@$(CC) -D$(DISKREGION) -D$(DISKRELEASE) -D$(DISKFORMAT) -DTIMESTAMP=$(TIMESTAMP) \
	       -D__IPL_ENTRY=0x$(__IPL_ENTRY_POINT) \
	       -c $< -o $@  $(CFLAGS)

clean-diskSystem:
	@rm -f diskSystem/*.o

clean:
	@rm -f ../*.disk ../*.ndd ../*.d64
	@rm -f ddTool/*.o diskBoot/*.o diskCode/*.o diskInfo/*.o diskSystem/*.o filesystem/*.o