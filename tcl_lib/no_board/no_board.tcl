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

if { [ catch { source "$libPath(extLib)/$target(board)/sram.tcl"} errMsg] } {
    messageDialg  error.gif "Memory Description File not found:\n$errMsg" "File not found" ok
    exit
}

if { [ catch { source "$libPath(extLib)/$target(board)/sdram.tcl"} errMsg] } {
    messageDialg  error.gif "Memory Description File not found:\n$errMsg" "File not found" ok
    exit
}

if { [ catch { source "$libPath(extLib)/$target(board)/nandflash.tcl"} errMsg] } {
    messageDialg  error.gif "Memory Description File not found:\n$errMsg" "File not found" ok
    exit
}

if { [ catch { source "$libPath(extLib)/$target(board)/dataflash.tcl"} errMsg] } {
    messageDialg  error.gif "Memory Description File not found:\n$errMsg" "File not found" ok
    exit
}


array set memoryAlgo {
    "SRAM"                    "::no_board_sram"
    "SDRAM"                   "::no_board_sdram"
    "DataFlash AT45DB/DCB"    "::no_board_dataflash"
    "NandFlash"               "::no_board_nandflash"
    "Peripheral"              "::no_board_peripheral"
    "ROM"                     "::no_board_rom"
    "REMAP"                   "::no_board_remap"
}

array set no_board_sram {
	dftDisplay  1
    dftDefault  0
	dftAddress  0x300000
	dftSize     0x28000
	dftSend     "SRAM::sendFile"
	dftReceive  "SRAM::receiveFile"
	dftScripts  ""
}

array set no_board_sdram {
	dftDisplay  1
        dftDefault  0
        dftAddress  0x20000000
        dftSize     0x4000000
	dftSend     "SDRAM::sendFile"
	dftReceive  "SDRAM::receiveFile"
	dftScripts  "::no_board_sdram_scripts"
}

array set no_board_sdram_scripts {
	"Enable SDRAM"   "SDRAM::initSDRAM"
}

array set no_board_dataflash {
	dftDisplay  1
        dftDefault  1
	dftAddress  0x0
	dftSize     0x420000
	dftSend     "DATAFLASH::sendFile"
	dftReceive  "DATAFLASH::receiveFile"
	dftScripts  "::no_board_dataflash_scripts"
}

array set no_board_dataflash_scripts {
	"Enable Dataflash on CS0"   "DATAFLASH::SelectDataflash AT91C_SPI0_CS0"
	"Erase All"                 "DATAFLASH::EraseAllDataFlash"
}

array set no_board_nandflash {
	dftDisplay  1
        dftDefault  0
	dftAddress  0x40000000
	dftSize     0x10000000
	dftSend     "NANDFLASH::sendFile"
	dftReceive  "NANDFLASH::receiveFile"
	dftScripts  "::no_board_nandflash_scripts"
}

array set no_board_nandflash_scripts {
	"NandFlash Init"                "NANDFLASH::Init"
        "NandFlash Erase All"           "NANDFLASH::EraseSectorTest"
}


array set no_board_peripheral {
	dftDisplay  0
        dftDefault  0
	dftAddress  0xF0000000
	dftSize     0x10000000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}

array set no_board_rom {
	dftDisplay  0
        dftDefault  0
	dftAddress  0x400000
	dftSize     0x8000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}

array set no_board_remap {
	dftDisplay  0
        dftDefault  0
	dftAddress  0x00000000
	dftSize     0x8000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}
