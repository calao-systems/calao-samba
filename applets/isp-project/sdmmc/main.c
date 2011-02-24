/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include "../common/applet.h"
#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <sdmmc/sdmmc_mci.h>
#include <utility/math.h>
#include <utility/trace.h>

#include <string.h>

//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------
/// Max size of read and write buffer.
#define BUFFER_SIZE  0x20000

#define BUFFER_NB_BLOCK (BUFFER_SIZE / SD_BLOCK_SIZE)

//------------------------------------------------------------------------------
//         Local structures
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Structure for storing parameters for each command that can be performed by
/// the applet.
//------------------------------------------------------------------------------
struct _Mailbox {

    /// Command send to the monitor to be executed.
    unsigned int command;
    /// Returned status, updated at the end of the monitor execution.
    unsigned int status;

    /// Input Arguments in the argument area
    union {

        /// Input arguments for the Init command.
        struct {

            /// Communication link used.
            unsigned int comType;
            /// Trace level.
            unsigned int traceLevel;

        } inputInit;

        /// Output arguments for the Init command.
        struct {

            /// Memory size.
            unsigned int memorySize;
            /// Buffer address.
            unsigned int bufferAddress;
            /// Buffer size.
            unsigned int bufferSize;
        } outputInit;

        /// Input arguments for the Write command.
        struct {

            /// Buffer address.
            unsigned int bufferAddr;
            /// Buffer size.
            unsigned int bufferSize;
            /// Memory offset.
            unsigned int memoryOffset;

        } inputWrite;

        /// Output arguments for the Write command.
        struct {

            /// Bytes written.
            unsigned int bytesWritten;
        } outputWrite;

        /// Input arguments for the Read command.
        struct {

            /// Buffer address.
            unsigned int bufferAddr;
            /// Buffer size.
            unsigned int bufferSize;
            /// Memory offset.
            unsigned int memoryOffset;

        } inputRead;

        /// Output arguments for the Read command.
        struct {

            /// Bytes read.
            unsigned int bytesRead;

        } outputRead;

        /// Input arguments for the Full Erase command.
        // NONE

        /// Output arguments for the Full Erase command.
        // NONE

        /// Input arguments for the Buffer Erase command.
        struct {

            /// Memory offset to be erase.
            unsigned int memoryOffset;

        } inputBufferErase;

        /// Output arguments for the Buffer Erase command.
        struct {

            /// erased size in bytes
            unsigned int bytesErased;

        } outputBufferErase;

    } argument;
};

//------------------------------------------------------------------------------
//         Global variables
//------------------------------------------------------------------------------

/// End of program space (code + data).
extern unsigned int end;


//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------

#if defined(MCI2_INTERFACE)
#include "dmad/dmad.h"
#endif

/// MCI driver instance.
static Mci mciDrv;

/// SDCard driver instance.
static SdCard sdDrv;

/// SD card pins.
static const Pin pinsSdmmc[] = {BOARD_SD_PINS};

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------

void SD_ReadBlock_TR(
    SdCard *pSd,
    unsigned int address,
    unsigned short nbBlocks,
    unsigned char *pData
    )
{
    TRACE_INFO("SD_ReadBlock_TR  block %03d nbBlocks %03d pData %08x\n\r", 
            address,
            nbBlocks,
            (unsigned int)pData);
            
    if( SD_ReadBlock(&sdDrv, address, nbBlocks, pData) ) {
        TRACE_ERROR("SD_ReadBlock (%d,%d)\n\r", address, nbBlocks);
    }
}


void SD_WriteBlock_TR(
    SdCard *pSd,
    unsigned int address,
    unsigned short nbBlocks,
    const unsigned char *pData
    )
{
    TRACE_INFO("SD_WriteBlock_TR block %03d nbBlocks %03d pData %08x\n\r", 
            address, 
            nbBlocks, 
            (unsigned int)pData);
            
    if( SD_WriteBlock(&sdDrv, address, nbBlocks, pData) ) {
        TRACE_ERROR("SD_ReadBlock (%d,%d)\n\r", address, nbBlocks);
    }
}

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Applet main entry. This function decodes received command and executes it.
/// \param argc  always 1
/// \param argv  Address of the argument area.
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    struct _Mailbox *pMailbox = (struct _Mailbox *) argv;
    unsigned int bufferSize, bufferAddr, memoryOffset;
    unsigned char rc;
    unsigned int sizeInMB;
    unsigned int sizeOfBlock;
    unsigned int mciSpeed;
    unsigned int mciDiv;
    // Temporary buffer used for non page aligned read/write
    unsigned int tempBufferAddr;
    unsigned char * pBuffer;
    unsigned int bufferOffset;
    unsigned int block;
    unsigned int multiBlock;
    unsigned int bytesToWrite;
    unsigned int bytesToRead;
    unsigned int packetSize;

    TRACE_CONFIGURE_ISP(DBGU_STANDARD, 115200, BOARD_MCK);
    PIO_Configure(pinsSdmmc, PIO_LISTSIZE(pinsSdmmc));

    // ----------------------------------------------------------
    // INIT:
    // ----------------------------------------------------------
    if (pMailbox->command == APPLET_CMD_INIT) {

#if (DYN_TRACES == 1)
        traceLevel = pMailbox->argument.inputInit.traceLevel;
#endif

        TRACE_INFO("-- SDMMC ISP Applet %s --\n\r", SAM_BA_APPLETS_VERSION);
        TRACE_INFO("-- %s\n\r", BOARD_NAME);
        TRACE_INFO("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
        TRACE_INFO("INIT command\n\r");

        // Initialize the MCI driver
        MCI_Init(&mciDrv, BOARD_SD_MCI_BASE, BOARD_SD_MCI_ID, BOARD_SD_SLOT, MCI_POLLING_MODE);

        // Initialize the SD card driver
        rc = SD_Init((SdCard*)&sdDrv, (SdDriver *)&mciDrv);
        if (rc) {

            TRACE_ERROR("-E- SD/MMC card initialization failed\n\r");

            pMailbox->status = APPLET_FAIL;
            pMailbox->argument.outputInit.bufferSize = 0;
            pMailbox->argument.outputInit.memorySize = 0;
            pMailbox->argument.outputInit.bufferAddress = (unsigned int) &end;
            TRACE_INFO("INIT command: No SD/MMC memory connected\n\r");

            goto exit;
        }

        if (SD_TOTAL_SIZE(&sdDrv) == 0xFFFFFFFF) {
            sizeInMB = SD_TOTAL_BLOCK(&sdDrv) / (1024 * 2);
            sizeOfBlock = 512;
        }
        else {
            sizeInMB = SD_TOTAL_SIZE(&sdDrv) / (1024 * 1024);
            sizeOfBlock = SD_TOTAL_SIZE(&sdDrv) / SD_TOTAL_BLOCK(&sdDrv);
        }

        TRACE_INFO("SD/MMC card initialization successful\n\r");
        TRACE_INFO("Card size: %u MB, %u * %dB\n\r",  sizeInMB, SD_TOTAL_BLOCK(&sdDrv), sizeOfBlock);

        SD_DisplayRegisterCID((SdCard*)&sdDrv);
        SD_DisplayRegisterCSD((SdCard*)&sdDrv);
        SD_DisplayRegisterECSD((SdCard*)&sdDrv);

        mciSpeed = 10000000;
        MCI_SetSpeed(&mciDrv, mciSpeed, sdDrv.transSpeed, BOARD_MCK);
        MCI_GetSpeed(&mciDrv, &mciDiv);
        TRACE_INFO("MCK %dK Hz, MCI Speed %dK, divisor %d. \r\n", BOARD_MCK/1000, mciSpeed/1000, mciDiv);

        // Get device parameters
        pMailbox->argument.outputInit.bufferSize = BUFFER_SIZE;
        pMailbox->argument.outputInit.memorySize = (SD_TOTAL_BLOCK(&sdDrv) * sizeOfBlock)  & 0x3FFFFFFF;
        pMailbox->argument.outputInit.bufferAddress = ((unsigned int) &end);

        TRACE_INFO("bufferSize : %d memorySize : %d bufferAddr: 0x%x \n\r",
               pMailbox->argument.outputInit.bufferSize,
               pMailbox->argument.outputInit.memorySize,
               pMailbox->argument.outputInit.bufferAddress);
    }

    // ----------------------------------------------------------
    // WRITE:
    // ----------------------------------------------------------
    else if (pMailbox->command == APPLET_CMD_WRITE) {

        // address to write in eMMC
        memoryOffset = pMailbox->argument.inputWrite.memoryOffset;
        // local buffer address
        bufferAddr = pMailbox->argument.inputWrite.bufferAddr;
        // bytes number to write in eMMC
        bufferSize = pMailbox->argument.inputWrite.bufferSize;

        TRACE_INFO("WRITE arguments : offset 0x%x, buffer at 0x%x, of 0x%x Bytes\n\r",
               memoryOffset,
               bufferAddr,
               bufferSize);

        pMailbox->argument.outputWrite.bytesWritten = 0;

        // Check word alignment
        /*if (memoryOffset % 4) {

            pMailbox->status = APPLET_ALIGN_ERROR;
            goto exit;
        }*/

        // Check block alignment
        if (memoryOffset % SD_BLOCK_SIZE) {
            // reading issue while multiBlock = 1 for eMMC Toshiba
            pMailbox->status = APPLET_ALIGN_ERROR;
            goto exit;
        }

        // remaining bytes to write
        bytesToWrite = bufferSize;
        // current pointer in write buffer
        pBuffer = (unsigned char *) bufferAddr;
        // address of the temp buffer
        tempBufferAddr = bufferAddr + BUFFER_SIZE;

        if ((memoryOffset % SD_BLOCK_SIZE) != 0) {

            // We are not block aligned, retrieve first block content to update it
            // Flush temp buffer
            memset((unsigned char *)tempBufferAddr, 0xFF, SD_BLOCK_SIZE);

            bufferOffset = (memoryOffset % SD_BLOCK_SIZE);

            // packetSize is the current number of interesting bytes to write
            if( (bytesToWrite + bufferOffset) < SD_BLOCK_SIZE) {
                packetSize = bytesToWrite;
            }
            else {
                packetSize = SD_BLOCK_SIZE - bufferOffset;
            }

            memoryOffset -= bufferOffset;

            // Read block to be updated
            block = memoryOffset / SD_BLOCK_SIZE;
            multiBlock = 1;

            SD_ReadBlock_TR(&sdDrv, block, multiBlock, (unsigned char*)tempBufferAddr);

            // Fill retrieved page with data to be programmed
            memcpy((unsigned char *)(tempBufferAddr + bufferOffset), pBuffer, packetSize);

            // Write the page contents
            SD_WriteBlock_TR(&sdDrv, block, multiBlock, (unsigned char*)tempBufferAddr);

            bytesToWrite -= packetSize;
            pBuffer += packetSize;
            memoryOffset += SD_BLOCK_SIZE;
        }

        // If it remains more than one block to write
        // Write the block contents
        block = memoryOffset / SD_BLOCK_SIZE;
        multiBlock = bytesToWrite / SD_BLOCK_SIZE;

        if(multiBlock != 0) {
            SD_WriteBlock_TR(&sdDrv, block, multiBlock, (unsigned char*)pBuffer);
            pBuffer += (multiBlock * SD_BLOCK_SIZE);
            memoryOffset += (multiBlock * SD_BLOCK_SIZE);
            bytesToWrite -= (multiBlock * SD_BLOCK_SIZE);
        }

        // Write remaining data
        if (bytesToWrite > 0) {

            // Read previous content of block
            // Read block to be updated
            block = memoryOffset / SD_BLOCK_SIZE;
            multiBlock = 1;
            SD_ReadBlock_TR(&sdDrv, block, multiBlock, (unsigned char*)tempBufferAddr);

            // Fill retrieved block with data to be programmed
            memcpy((unsigned char *)tempBufferAddr, pBuffer, bytesToWrite);

            // Write the block contents
            SD_WriteBlock_TR(&sdDrv, block, multiBlock, (unsigned char*)tempBufferAddr);

            // No more bytes to write
            bytesToWrite = 0;
        }

        pMailbox->argument.outputWrite.bytesWritten = bufferSize;
        pMailbox->status = APPLET_SUCCESS;
    }

    // ----------------------------------------------------------
    // READ:
    // ----------------------------------------------------------
    else if (pMailbox->command == APPLET_CMD_READ) {
        // address to read in eMMC
        memoryOffset = pMailbox->argument.inputRead.memoryOffset;
        // local buffer address
        bufferAddr   = pMailbox->argument.inputRead.bufferAddr;
        // bytes number to read in eMMC
        bufferSize   = pMailbox->argument.inputRead.bufferSize;

        TRACE_INFO("READ at offset: 0x%x buffer at : 0x%x of: 0x%x Bytes\n\r",
               memoryOffset,
               bufferAddr,
               bufferSize);

        bytesToRead = bufferSize;
        pBuffer = (unsigned char *) bufferAddr;
        tempBufferAddr = bufferAddr + BUFFER_SIZE;

        // Check word alignment
        /*if (memoryOffset % 4) {

            pMailbox->status = APPLET_ALIGN_ERROR;
            goto exit;
        }*/

        // Check block alignment
        if (memoryOffset % SD_BLOCK_SIZE) {
            // reading issue while multiBlock = 1 for eMMC Toshiba
            pMailbox->status = APPLET_ALIGN_ERROR;
            goto exit;
        }

        if ((memoryOffset % SD_BLOCK_SIZE) != 0) {

            // We are not Block aligned, retrieve first block content
            bufferOffset = (memoryOffset % SD_BLOCK_SIZE);
            if( (bytesToRead + bufferOffset) < SD_BLOCK_SIZE) {
                packetSize = bytesToRead;
            }
            else {
                packetSize = (SD_BLOCK_SIZE - bufferOffset);
            }

            // erase temp buffer
            memset((unsigned char *)tempBufferAddr, 0xFF, SD_BLOCK_SIZE);

            // Read block
            block = memoryOffset / SD_BLOCK_SIZE;
            multiBlock = 1;
            SD_ReadBlock_TR(&sdDrv, block, multiBlock, (unsigned char*)tempBufferAddr);

            TRACE_DumpMemory((unsigned char*)tempBufferAddr, 512, tempBufferAddr);

            TRACE_INFO("memcpy 0x%08x 0x%08x 0x%08x\n\r",
                  tempBufferAddr + bufferOffset,
                  (unsigned int)((unsigned char *)(tempBufferAddr) + bufferOffset),
                  (unsigned int)((unsigned char *)(tempBufferAddr + bufferOffset)));

            // Fill retrieved page with data to be programmed
            memcpy(pBuffer, (unsigned char *)(tempBufferAddr) + bufferOffset, packetSize);

            TRACE_DumpMemory(pBuffer, 512, (unsigned int)pBuffer);

            memoryOffset += packetSize;
            pBuffer += packetSize;
            bytesToRead -= packetSize;
        }

        // If it remains more than one block to read
        // Read block
        block = memoryOffset / SD_BLOCK_SIZE;
        multiBlock = bytesToRead / SD_BLOCK_SIZE;
        if(bytesToRead % SD_BLOCK_SIZE) {
            multiBlock += 1;
        }
        if(multiBlock) {
            SD_ReadBlock_TR(&sdDrv, block, multiBlock, pBuffer);
        }

        pMailbox->argument.outputRead.bytesRead = bufferSize;
        pMailbox->status = APPLET_SUCCESS;
    }
    // ----------------------------------------------------------
    // BUFFER ERASE:
    // ----------------------------------------------------------
    else if (pMailbox->command == APPLET_CMD_BUFFER_ERASE) {
        TRACE_INFO("BUFFER ERASE command\n\r");

        TRACE_INFO("Not yet implemented\n\r");
        pMailbox->status = APPLET_FAIL;
    }

    // ----------------------------------------------------------
    // FULL ERASE:
    // ----------------------------------------------------------
    else if (pMailbox->command == APPLET_CMD_FULL_ERASE) {
        TRACE_INFO("FULL ERASE command\n\r");

        TRACE_INFO("Not yet implemented\n\r");
        pMailbox->status = APPLET_FAIL;
    }


exit :
    // Acknowledge the end of command
    TRACE_INFO("\tEnd of Applet %x %x.\n\r", pMailbox->command, pMailbox->status);
    // Notify the host application of the end of the command processing
    pMailbox->command = ~(pMailbox->command);

    return 0;
}

//------------------------------------------------------------------------------
//         Optional: SD card detection
//------------------------------------------------------------------------------

#ifdef BOARD_SD_PIN_CD

/// SD card detection pin instance.
static const Pin pinMciCardDetect = BOARD_SD_PIN_CD;

//------------------------------------------------------------------------------
/// Waits for a SD card to be connected.
//------------------------------------------------------------------------------
void WaitSdConn(void)
{
    PIO_Configure(&pinMciCardDetect, 1);
    printf("-I- Please connect a SD card ...\n\r");
    while (PIO_Get(&pinMciCardDetect) != 0);
    printf("-I- SD card connection detected\n\r");
}

#else

//------------------------------------------------------------------------------
/// Dummy function.
//------------------------------------------------------------------------------
void WaitSdConn(void)
{
    printf("-I- SD card detection not available, assuming card is present\n\r");
}

#endif
