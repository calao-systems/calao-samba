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
## NAMESPACE NANDFLASH
################################################################################
################################################################################
namespace eval NANDFLASH {
    variable StartAddress        [expr 0x0]
    variable MemorySize          [expr 132*1024]
    
    variable array nf
    array set nf {
        ComType                  0x2013EC
        monitorArg0              0x2013F0
        monitorAddr              0x202000
        monitorArg1              0x2013F4
        monitorArg2              0x2013F8
        monitorArg3              0x2013FC
        monitorArg4              0x201400
        monitorArg5              0x201404
        monitorMbx               0x201400
        nandNbBlocks             0
        nandBlockSize            0
        nandSectorSize           0x800
        nandSpareSize            0
        nandBusWidth             0
        monitorName              "SAM-BA-nand.bin"
    }
    variable array nf_cmd
    array set nf_cmd {
        'NAND_INIT'              0
        'NAND_ERASE'             1
        'NAND_READ'              2
        'NAND_WRITE'             3
        'NAND_ID'                4
    }
    variable array nf_zone
    array set nf_zone {
        'NF_INFO_ZONE'           0x0
        'NF_BAD_BLOCK'           0xBAD
        'NF_DATA_ZONE'           0xFF
    }
}

################################################################################
#  proc NANDFLASH::Init
################################################################################
proc NANDFLASH::Init { } {

    puts "-I- Configure NandFlash PIOs"
    puts "-I- Enable the address range of CS3 in EBI user interface"
    puts "-I- Configure SMC CS3"
    puts "-I- Clock PIOC"
    puts "-I- Configure Ready/Busy signal"
    puts "-I- Configure pull-up"
    puts "-I- Enable A21=ALE, A22=CLE"
    puts "-I- Enable NandFlash"

    puts "-I- ReadId"
    set nf(nandNbBlocks)       0x800
    set nf(nandBlockSize)        0x20000
    set nf(nandSectorSize)         0x800
    set nf(nandSpareSize)           0x40
    set nf(nandBusWidth)               0
    puts "-I- NandFlash Samsung K9F2G08UOM 8 bits 256MB"
    
    set MemorySize [expr $nf(nandNbBlocks) * $nf(nandBlockSize)]
}


################################################################################
#  proc NANDFLASH::sendFile
################################################################################
proc NANDFLASH::sendFile { name address } {
    puts "-I- NANDFLASH::sendFile"
    
    # Open the file
    if { [catch {set fHandle [open $name r]}] } {
        
        puts stderr "-E- Can't open file $name"
        return
    }
    
    fconfigure $fHandle -translation binary
    
    set length [file size $name]
    puts "-I- File size = $length byte(s)"
    
    close $fHandle
}

################################################################################
#  proc NANDFLASH::receiveFile
################################################################################
proc NANDFLASH::receiveFile {name address size} {
    puts "-I- NANDFLASH::receiveFile"

    set result "0123456789abcdefghijklmnopqrstuvwxyz"

    # put data in a file
    if { [catch {set fHandle [open $name w+]}] } {
        puts stderr "-E- Can't open file $name"
        return -1
    }
    fconfigure $fHandle -translation binary
    puts -nonewline $fHandle $result
    
    close $fHandle
}

################################################################################
#  proc NANDFLASH::EraseSectorTest
################################################################################
proc NANDFLASH::EraseSectorTest { } {
    variable nf

    # Wait window for erasing
    if {$commandLineMode == 0} {
        waitWindows::createWindow "Erasing NandFlash" "erase.gif"
        tkwait visibility .topWaitWindow
    }

    global softwareStatusLabelVariable
    set softwareStatusLabelVariable "Erasing NandFlash ..."


    set badBlock 0
    set blockIdx 0
    while { $blockIdx < $nf(nandNbBlocks) } {
    puts "Block erased $blockIdx"
        incr blockIdx
    }

    puts "$badBlock block(s) not erased"

    if {$commandLineMode == 0} {
        waitWindows::destroyWindow
    }
    set softwareStatusLabelVariable ""
}
