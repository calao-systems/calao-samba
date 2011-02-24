#  ----------------------------------------------------------------------------
#          ATMEL Microcontroller Software Support
#  ----------------------------------------------------------------------------
#  Copyright (c) 2008, Atmel Corporation
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  - Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the disclaimer below.
#
#  Atmel's name may not be used to endorse or promote products derived from
#  this software without specific prior written permission. 
#
#  DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
#  DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  ----------------------------------------------------------------------------

################################################################################
################################################################################
## NAMESPACE DATAFLASH
################################################################################
################################################################################
namespace eval DATAFLASH {
    variable StartAddress        [expr 0x0]
    # Minimal Size AT45DB011B
    variable MemorySize          [expr 132*1024]
    variable MaxBootSize         [expr 156*1024]
    
    variable AT91C_SPI0_PCS0_DATAFLASH     0xE
    variable AT91C_DATAFLASH_TIMEOUT       10000000
    
    variable bufferAddr         0x202010
    variable cmdAddr            0x202000
    variable MASTER_CLOCK       48000000
    variable DATAFLASH_CLK      8000000
    # Dataflash Setup Time
    variable DATAFLASH_TCSS     [expr 0xc << 16]
    # Dataflash Hold Time
    variable DATAFLASH_TCHS     [expr 0x0 << 24]
    
    # Read Commands
    variable DB_CONTINUOUS_ARRAY_READ   0xE8
    variable DB_PAGE_READ               0xD2
    variable DB_BUF1_READ               0xD4
    variable DB_BUF2_READ               0xD6
    variable DB_STATUS                  0xD7
    
    # Write + Erase Commands
    variable DB_BUF1_WRITE              0x84
    variable DB_BUF2_WRITE              0x87
    variable DB_BUF1_PAGE_ERASE_PGM     0x83
    variable DB_BUF2_PAGE_ERASE_PGM     0x86
    variable DB_BUF1_PAGE_PGM           0x88
    variable DB_BUF2_PAGE_PGM           0x89
    variable DB_PAGE_ERASE              0x81
    variable DB_BLOCK_ERASE             0x50
    variable DB_PAGE_PGM_BUF1           0x82
    variable DB_PAGE_PGM_BUF2           0x85
    
    # Additional Commands
    variable DB_PAGE_2_BUF1_TRF         0x53
    variable DB_PAGE_2_BUF2_TRF         0x55
    
    variable GET_STATUS                 0x0F
    variable BUSY                       0x1
    variable IDLE                       0x0
    
    variable array ArrayDeviceID
    array set ArrayDeviceID {
        0x0C    AT45DB011
        0x14    AT45DB021
        0x1C    AT45DB041
        0x24    AT45DB081
        0x2C    AT45DB161
        0x34    AT45DB321
        0x3C    AT45DB642
        0x10    AT45DB1282
        0x18    AT45DB2562
        0x20    AT45DB5122
    }
    
    variable array deviceAT45
    array set deviceAT45 {
        pages_number 0
        pages_size   0
        page_offset  0
        byte_mask    0
    }
    
    variable array DataflashDesc
    array set DataflashDesc {
        tx_cmd_pt 0
        tx_cmd_size 0
        rx_cmd_pt 0
        rx_cmd_size 0
        tx_data_pt 0
        tx_data_size 0
        rx_data_pt 0
        rx_data_size 0
        DataFlash_state 0
        command0 0
        command1 0
        command2 0
        command3 0
        command4 0
        command5 0
        command6 0
        command7 0
        command8 0
    }
    
    # Parameters used by the C binary project: pageBuf is the page content (size of a page)
    variable ComType                   0x202000
    variable df_dest                   0x202004
    variable pageNb                    0x202008
    variable pageSize                  0x20200C
    variable pageOffset                0x202010
    variable pageBuf                   0x202014
    
    # Link address & Binary name in the C project
    variable fileAddr                  0x201000
    variable fileDFWrite               "SAM-BA-DataFlashWrite.bin"
    variable fileDFRead                "SAM-BA-DataFlashRead.bin"
}


################################################################################
#  proc DATAFLASH::Init
################################################################################
proc DATAFLASH::Init { } {

    puts "-I- Init SPI for DataFlash interfaces"
    puts "-I- Configure SPI PIOs"
    
    puts "-I- Enable PMC for SPI0"
    
    puts "-I- Reset the SPI0"
    
    puts "-I- Configure SPI in Master Mode (no CS selected)"
    
    puts "-I- Configure SPI CS0 for DataFlash AT45"
    
    puts "-I- Disable the RX and TX PDC transfer requests"
    
    puts "-I- Reset all Counter register Next buffer first"
    
    puts "-I- Enable the RX and TX PDC transfer requests"
    
    puts "-I- End of Init_DataFlash"
}

################################################################################
#  proc DATAFLASH::SelectDataflash
################################################################################
proc DATAFLASH::SelectDataflash { Dataflash_CS } {
    
    DATAFLASH::Init
    
    puts "-I- Switch to the correct PCS of SPI0 Mode Register (Fixed Peripheral Selected)"

    puts "-I- Enable the SPI0"
    
    puts "-I- Wait for dataflash ready (bit7 of the status register)"
    
    set deviceAT45(pages_number) 2048
    set deviceAT45(pages_size) 264
    set deviceAT45(page_offset) 9
    set deviceAT45(byte_mask) 0x100
    
    # Necessary For sendFile and receiveFile methods
    puts "-I- End of Select DataFlash"
}

################################################################################
#  proc DATAFLASH::EraseAllDataFlash
################################################################################
proc DATAFLASH::EraseAllDataFlash {  } {
    variable deviceAT45

    
    # Wait window for loading
    waitWindows::createWindow "Erase DataFlash" "erase.gif"
    tkwait visibility .topWaitWindow
    global softwareStatusLabelVariable
    set softwareStatusLabelVariable "Erasing whole DataFlash ..."
    
    set blocknb [expr $deviceAT45(pages_number) / 8]
    while {$blocknb} {
	puts "Erasing DataFlash Block [expr $blocknb - 1]"
        set blocknb [expr $blocknb - 1]
    }
    
    waitWindows::destroyWindow
    set softwareStatusLabelVariable ""
}

################################################################################
#  proc DATAFLASH::sendFile
################################################################################
proc DATAFLASH::sendFile { name addr } {
    if { [catch {set f [open $name r]}] } {
        puts stderr "-E- Can't open file $name"
        return
    }
    
    fconfigure $f -translation binary
    
    set size [file size $name]
    puts "-I- File size = $size byte(s)"
    
    close $f
}

################################################################################
#  proc DATAFLASH::receiveFile
################################################################################
proc DATAFLASH::receiveFile {name addr size} {
    set result "0123456789abcdefghijklmnopqrstuvwxyz"

    # put data in a file
    if { [catch {set f2 [open $name w+]}] } {
        puts stderr "-E- Can't open file $name"
        return -1
    }
    fconfigure $f2 -translation binary
    puts -nonewline $f2 $result
    close $f2
}
