#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <stdint.h>
#include <libdragon.h>

#include "dd.h"
#include "../common.h"
#include "../ddTool/ddTool.h"

#include "ddGlobals.h"

uint8_t diskReadBuffer[0x4D10];
uint8_t sysData[232];
bool sysDataRead = false;

static unsigned int stat, bm_stat, err_stat = 0;

int detect64dd_ipl(void)
{
    //Look at 0x9FF00 from the IPL.
    uint32_t test = io_read(IPL_CHECK);

    //Isolate the highest byte
    test &= 0xFF000000;

    if (test == 0xC3000000)
	return 1;	//if yes, then retail 64DD is present (JPN)
    else if (test == 0x04000000)
	return 2;	//it has found a potential US 64DD.
    else
    {
	test = io_read(ASIC_ID_REG);
	if (test == 0x00040000)
	    return 3;
	else
	    return 0;	//if not, then there are no 64DD connected.
    }
}

int detectdisk(void)
{
    uint32_t test = io_read(ASIC_STATUS);	//read status

    test &= LEO_STAT_DISK;		//mask disk status

    if (test == LEO_STAT_DISK)
	    return 1;	//disk found
    else
	    return 0;	//disk not found
}

void wait64dd_statusON(uint32_t STAT)
{
    while ((io_read(ASIC_STATUS) & STAT) != STAT);
}

void wait64dd_statusOFF(uint32_t STAT)
{
    while ((io_read(ASIC_STATUS) & STAT) == STAT);
}

uint32_t readDiskID(void)
{
    readDiskSectorLBA(14, 0, diskReadBuffer);
    uint32_t diskIDs = *(uint32_t*)diskReadBuffer;
    debugf("disk ID is %x", diskIDs);
    return diskIDs;
}

void readDiskSystemData(void)
{
    readDiskSectorLBA(0, 0, diskReadBuffer);
    memcpy(sysData, diskReadBuffer, 232);
    debugf("sysdata at %x\n", sysData);
    sysDataRead = true;
}

void BMReset(uint8_t sector) 
{
    io_write(ASIC_BM_CTL, (BM_MODE | BM_RESET | ((uint32_t)sector << 16)) );    // Set BM (Bit Micro?) Reset. Leave BM_MODE set.
    io_write(ASIC_BM_CTL, (BM_MODE | ((uint32_t)sector << 16)));  		   // Clear BM (Bit Micro?) reset. Leave BM_MODE set.
}

void StartBM(uint8_t sector) 
{
    io_write(ASIC_BM_CTL, (START_BM | BM_MODE | ((uint32_t)sector << 16)));    // Set SECTOR offset (within the Track) depending on whether the LBA is Odd / Even.
}

void sendMSEQ(uint32_t secsize)
{
    //Send MSEQ
    for (int i = 0; i < 16; i++)
    {
        if (i == 4)
            io_write((MSEQ_RAM_ADDR + i * 4), (MSEQdata[i] | (secsize << 8)));
        else
            io_write((MSEQ_RAM_ADDR + i * 4), MSEQdata[i]);
    }
}

void readDisk(u32 offset, u32 size, uint8_t* dest)
{
    debugf("reading %x, %d bytes to %x\n", offset, size, dest);

    u32 lbaSt = 0;
    u32 lbaStOffs = 0;
    u32 lbaEn = 0;
    u32 lbaEnOffs = 0;

    // Convert byte offsets to starting/ending LBA and offsets within LBA
    ConvertByteToLBAWithOffset(offset, &lbaSt, &lbaStOffs);
    ConvertByteToLBAWithOffset(offset + size, &lbaEn, &lbaEnOffs);

    debugf("lba st %x, %d bytes\n", lbaSt, lbaStOffs);
    debugf("lba end %x, %d bytes\n", lbaEn, lbaEnOffs);

    u32 bytesRemaining = size;
    uint8_t* destPtr = (uint8_t*)dest;

    for (u32 lba = lbaSt; lba <= lbaEn; lba++)
    {
        debugf("reading %x %x\n", lba, diskReadBuffer);
        int copySize = readDiskLBA(lba, diskReadBuffer);

        u32 copyStart = 0;

        if (lba == lbaSt)
        {
            copyStart = lbaStOffs;
            copySize -= lbaStOffs;
        }

        if (lba == lbaEn)
            copySize = lbaEnOffs;

        if (copySize > bytesRemaining)
            copySize = bytesRemaining;

        if (copySize != 0)
        {
            debugf("copy %x %x -> %x %x\n", lba, diskReadBuffer + copyStart, copySize, destPtr);
            memcpy(destPtr, diskReadBuffer + copyStart, copySize);
            destPtr += copySize;
            bytesRemaining -= copySize;
        }
    }
}

int readDiskLBA(uint16_t LBA, u8* buffer)
{
    int bytesRead = 0;

    for (int sector = 0; sector < 85; sector++) 
    {
        int sectorSize = getSecSizefromTrack(LBA >> 1);

        u8* buffer_ptr = buffer + (sector * sectorSize);
        readDiskSectorLBA(LBA, sector, buffer_ptr);

        bytesRead += sectorSize;
    } 

    return bytesRead;
}

void readDiskSectorLBA(uint16_t LBA, uint8_t sector, void * buffer)
{
    readDiskSector(LBA, (LBA >> 1), (sector + (ALL_SECS_PER_BLK * (((LBA & 2) >> 1) ^ (LBA & 1)))), buffer);
}

void readDiskSector(uint16_t LBA, uint16_t track, uint16_t sector, void* buffer)
{   
    //Read single sector
    if (io_read(ASIC_STATUS) & LEO_STAT_MECHA_INT)
	    io_write(ASIC_BM_CTL, BM_MECHA_INT_RESET);

    io_write(ASIC_DATA, lbatophys(LBA, sysData));	//Read Track (LBA >> 1)
    io_write(ASIC_CMD, ASIC_RD_SEEK);	//SEEK CMD
    io_write(ASIC_BM_CTL, 0x00000000);	//Clear BM CTL
    io_write(ASIC_SEQ_CTL, 0x00000000);	//Clear SEQ CTL

    wait64dd_statusON(LEO_STAT_MECHA_INT);

    if (getSecSizefromTrack(track) == 0)
	    return;

    uint32_t SecSize = getSecSizefromTrack(track) - 1;
    data_cache_hit_writeback(buffer, SecSize);

    debugf("track %x, sector %x -> %x, %d\n", track, sector, buffer, SecSize);

    sendMSEQ(SecSize);	//Send MSEQ
    io_write(ASIC_HOST_SECBYTE, (SecSize << 16));    
    io_write(ASIC_SEC_BYTE, (0x59000000 | ((SecSize + C1_Length) << 16)));

    io_write(ASIC_SEQ_CTL, MICRO_PC_ENABLE);	//MSEQ enable

    BMReset(sector);
    StartBM(sector);

    wait64dd_statusON(LEO_STAT_BM_INT);

    wait64dd_statusON(LEO_STAT_DATA_REQ);
    
    dma_read_raw_async(buffer, ASIC_SECTOR_BUFF, SecSize + 1);

    stat = io_read(ASIC_STATUS);
    bm_stat = io_read(ASIC_BM_STATUS);
    err_stat = io_read(ASIC_ERR_SECTOR);
}

uint32_t getSecSizefromTrack(uint8_t track)
{
    int Total = 0;

    for (int i = 0; i < 16; i++)
    {
        if ((Zones[DISK_TYPE][i] <= (DISK_TYPE + 2))) //ROM
        {
            //Detect Head 0/1
            if (i <= (DISK_TYPE + 2))
                Total += Tracks[Zones[DISK_TYPE][i]];
            else
                Total += Tracks[Zones[DISK_TYPE][i] + 9];
        }
        else //RAM
        {
            //Detect Head 0/1
            if ((i > (5 + (DISK_TYPE * 2))) && (i < (10 + DISK_TYPE)))
            Total += Tracks[Zones[DISK_TYPE][i]];
            else
            Total += Tracks[Zones[DISK_TYPE][i] + 9];
        }

        if (track < Total)
            return SECTOR_SIZES[Zones[DISK_TYPE][i]];
    }

    return 0;
}

int bytetolba(int startlba, int nbytes, s32* lbaout)
{
    char init_flag = 1;
    int vzone = 1;
    int pzone = 0;
    int lba = startlba;
    int lba_count = 0;
    int byte_count = nbytes;
    int blkbytes = 0;

    if (nbytes != 0)
    {
        do
        {
            if ((init_flag) || (VZONE_LBA_TBL[DISK_TYPE][vzone] == lba))
            {
                vzone = LBAToVZone(lba, DISK_TYPE);
                pzone = VZoneToPZone(vzone, DISK_TYPE);
                if (7 < pzone)
                {
                    pzone -= 7;
                }
                blkbytes = BLOCK_SIZES[pzone];
            }

            if (byte_count < blkbytes)
                byte_count = 0;
            else
                byte_count -= blkbytes;

            lba++;
            lba_count++;
            init_flag = 0;

            if ((byte_count != 0) && (lba > 0x10db))
                return LEO_ERROR_LBA_OUT_OF_RANGE;

        }
        while (byte_count != 0);
    }

    *lbaout = lba;
    return LEO_ERROR_GOOD;
}

int lbatobyte(int startlba, int nlbas, s32* bytes)
{
    int totalbytes = 0;
    char init_flag = 1;
    int vzone = 1;
    int pzone = 0;
    int lba = startlba;
    int lba_count = nlbas;
    int blkbytes = 0;

    if (nlbas != 0)
    {
        for (; lba_count != 0; lba_count--)
        {
            if ((init_flag) || (VZONE_LBA_TBL[DISK_TYPE][vzone] == lba))
            {
                vzone = LBAToVZone(lba, DISK_TYPE);
                pzone = VZoneToPZone(vzone, DISK_TYPE);
                if (7 < pzone)
                    pzone -= 7;

                blkbytes = BLOCK_SIZES[pzone];
            }

            totalbytes += blkbytes;
            lba++;
            init_flag = 0;

            if ((lba_count != 0) && (lba > 0x10db))
                return LEO_ERROR_LBA_OUT_OF_RANGE;
        }
    }
    *bytes = totalbytes;
    return LEO_ERROR_GOOD;
}

int lbatophys(int lba, u8* sys_data)
{
    int start_block = 0;
    int vzone, pzone = 0;
    int param_head, param_zone, param_cylinder = 0;
    int vzone_lba, cylinder_zone_start, defect_offset, defect_amount = 0;
    
    int disktype = sys_data[5] & 0xF;

    //Skip System Area to correspond LBAs to SDK manual
    //lba += 24;

    //Unused here, get Block 0/1 on Disk Track
    if (((lba & 3) == 0) || ((lba & 3) == 3))
        start_block = 0;
    else
        start_block = 1;
    
    //Get Virtual & Physical Disk Zones
    vzone = LBAToVZone(lba, disktype);
    pzone = VZoneToPZone(vzone, disktype);

    //Get Disk Head
    param_head = (7 < pzone);

    //Get Disk Zone
    param_zone = pzone;
    if (param_head != 0)
        param_zone = pzone - 7;
    
    //Get Virtual Zone LBA start, if Zone 0, it's LBA 0
    if (vzone == 0)
        vzone_lba = 0;
    else
        vzone_lba = VZONE_LBA_TBL[disktype][vzone - 1];
    
    //Calculate Physical Cylinder
    param_cylinder = (lba - vzone_lba) >> 1;

    //Get the start cylinder from current zone
    cylinder_zone_start = SCYL_ZONE_TBL[0][pzone];
    if (param_head != 0)
    {
        //If Head 1, count from the other way around
        param_cylinder = -param_cylinder;
        cylinder_zone_start = OUTERCYL_TBL[param_zone - 1];
    }
    param_cylinder += SCYL_ZONE_TBL[0][pzone];

    //Get the relative offset to defect tracks for the current zone (if Zone 0, then it's 0)
    if (pzone == 0)
        defect_offset = 0;
    else
        defect_offset = sys_data[8 + pzone - 1];
    
    //Get amount of defect tracks for the current zone
    defect_amount = sys_data[8 + pzone] - defect_offset;

    //Skip defect tracks
    while ((defect_amount != 0) && ((sys_data[0x20 + defect_offset] + cylinder_zone_start) <= param_cylinder))
    {
        param_cylinder++;
        defect_offset++;
        defect_amount--;
    }

    //Return Cylinder and Head data for 64DD Seek command
    return ((param_head  & 0x0F) << 28) | ((param_cylinder & 0x0FFF) << 16);
}

s32 ConvertByteToLBAWithOffset(s32 bytePos, s32* outLBA, s32* outOffset) 
{
    s32 lba;
    s32 offsetWithinLBA;
    s32 result;

    result = bytetolba(0, bytePos + 1, &lba);

    if (result != LEO_ERROR_GOOD)
        return result;

    if (lba == 0)
        offsetWithinLBA = 0;
    else 
    {
        result = lbatobyte(0, lba - 1, &offsetWithinLBA);

        if (result != LEO_ERROR_GOOD)
            return result;
    }

    *outLBA = lba;
    *outOffset = bytePos - offsetWithinLBA;

    return LEO_ERROR_GOOD;
}
