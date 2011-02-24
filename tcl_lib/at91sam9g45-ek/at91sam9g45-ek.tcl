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

if { [ catch { source "$libPath(extLib)/common/generic.tcl"} errMsg] } {
    messageDialg  error.gif "Common library file not found:\n$errMsg" "File not found" ok
    exit
}




################################################################################
## BOARD SPECIFIC PARAMETERS
################################################################################
namespace eval BOARD {
    variable sramSize         $AT91C_IRAM_SIZE
    variable maxBootSize      [expr 60 * 1024]

    # Vdd Memory 1.8V = 0 / Vdd Memory 3.3V = 1
    variable extRamVdd 0
    # External SDRAM = 0 / External DDR = 1
    variable extRamType 1
    # Set bus width (16 or 32)
    variable extRamDataBusWidth 16 
    # DDRAM Model (0 : Micron (on EK), 1 : Samsung (on VB)) default is EK
    variable extDDRamModel 0
}

set target(board) at91sam9g45-ek

# Source procedures for compatibility with older SAM-BA versions
if { [ catch { source "$libPath(extLib)/common/functions.tcl"} errMsg] } {
    messageDialg  error.gif "Function file not found:\n$errMsg" "File not found" ok
    exit
}

array set memoryAlgo {
    "SRAM"                    "::at91sam9g45_sram"
    "DDRAM"                   "::at91sam9g45_ddram"
    "DataFlash AT45DB/DCB"    "::at91sam9g45_dataflash"
    "SerialFlash AT25/AT26"   "::at91sam9g45_serialflash"
    "EEPROM AT24"             "::at91sam9g45_eeprom"
    "NandFlash"               "::at91sam9g45_nandflash"
    "NorFlash"                "::at91sam9g45_norflash"
    "NorFlash Map"            "::at91sam9g45_norflash_map"
    "DDR2 / SDRAM Map"        "::at91sam9g45_ddr2_sdram_map"
    "Peripheral"              "::at91sam9g45_peripheral"
    "ROM"                     "::at91sam9g45_rom"
    "REMAP"                   "::at91sam9g45_remap"
}

################################################################################
## SRAM
################################################################################
array set at91sam9g45_sram {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x00300000
    dftSize     0x10000
    dftSend     "RAM::sendFile"
    dftReceive  "RAM::receiveFile"
    dftScripts  ""
}

################################################################################
## DDRAM
################################################################################
array set at91sam9g45_ddram {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x70000000
    dftSize     "$GENERIC::memorySize"
    dftSend     "RAM::sendFile"
    dftReceive  "RAM::receiveFile"
    dftScripts  "::at91sam9g45_ddram_scripts"
}


## DDRAM Modules can have different initialization sequences and timings
## Indexes in this array correspond to cases in board_memories.h
array set at91sam9g45_ddram_modules {
    Micron_MT29F1G08           0
    Samsung_M470T6554EZ3-CE6   1
}

puts "-I- External RAM Settings :  extRamVdd=$BOARD::extRamVdd, extRamType=$BOARD::extRamType, extRamDataBusWidth=$BOARD::extRamDataBusWidth, extDDRamModel=$BOARD::extDDRamModel"

set RAM::appletAddr          0x300000
set RAM::appletMailboxAddr   0x300004
set RAM::appletFileName      "$libPath(extLib)/$target(board)/isp-extram-at91sam9g45.bin"

array set at91sam9g45_ddram_scripts {
    "Enable DDRAM"   "GENERIC::Init $RAM::appletAddr $RAM::appletMailboxAddr $RAM::appletFileName [list $::target(comType) $::target(traceLevel) $BOARD::extRamVdd $BOARD::extRamType $BOARD::extRamDataBusWidth $BOARD::extDDRamModel]"
}


# Initialize DDRAMC
if {[catch {GENERIC::Init $RAM::appletAddr $RAM::appletMailboxAddr $RAM::appletFileName [list $::target(comType) $::target(traceLevel) $BOARD::extRamVdd $BOARD::extRamType $BOARD::extRamDataBusWidth $BOARD::extDDRamModel]} dummy_err] } {
        set abort 1
        if {$commandLineMode == 0} {
            set abort [messageDialg warning.gif "External RAM initialization failed.\nExternal RAM access is required to run applets.\nContinue anyway ?" "External RAM init." yesno]
        }
        puts "-E- Error during external RAM initialization."
        puts "-E- External RAM access is required to run applets."
        # Close link
        if {$abort} {
            TCL_Close $target(handle)
            exit
        }
} else {
        puts "-I- External RAM initialized"
}

################################################################################
## DATAFLASH
################################################################################
array set at91sam9g45_dataflash {
    dftDisplay  1
    dftDefault  1
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam9g45_dataflash_scripts"
}

array set at91sam9g45_dataflash_scripts {
    "Enable Dataflash (SPI0 CS0)"                        "DATAFLASH::Init 0"
    "Set DF in Power-Of-2 Page Size mode (Binary mode)"  "DATAFLASH::BinaryPage"
    "Send Boot File"                                     "GENERIC::SendBootFileGUI"
    "Erase All"                                          "DATAFLASH::EraseAll"
}

set DATAFLASH::appletAddr          0x300000
set DATAFLASH::appletMailboxAddr   0x300004
set DATAFLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-dataflash-at91sam9g45.bin"

################################################################################
## SERIALFLASH
################################################################################
array set at91sam9g45_serialflash {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam9g45_serialflash_scripts"
}

array set at91sam9g45_serialflash_scripts {
    "Enable Serialflash (SPI0 CS0)"   "SERIALFLASH::Init 0"
    "Send Boot File"                  "GENERIC::SendBootFileGUI"
    "Erase All"                       "SERIALFLASH::EraseAll"
}

set SERIALFLASH::appletAddr          0x300000
set SERIALFLASH::appletMailboxAddr   0x300004
set SERIALFLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-serialflash-at91sam9g45.bin"

################################################################################
## EEPROM
################################################################################
array set at91sam9g45_eeprom {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam9g45_eeprom_scripts"
}

array set at91sam9g45_eeprom_scripts {
    "Enable EEPROM AT24C01x"          "EEPROM::Init 0"
    "Enable EEPROM AT24C02x"          "EEPROM::Init 1"
    "Enable EEPROM AT24C04x"          "EEPROM::Init 2"
    "Enable EEPROM AT24C08x"          "EEPROM::Init 3"
    "Enable EEPROM AT24C16x"          "EEPROM::Init 4"
    "Enable EEPROM AT24C32x"          "EEPROM::Init 5"
    "Enable EEPROM AT24C64x"          "EEPROM::Init 6"
    "Enable EEPROM AT24C128x"         "EEPROM::Init 7"
    "Enable EEPROM AT24C256x"         "EEPROM::Init 8"
    "Enable EEPROM AT24C512x"         "EEPROM::Init 9"
    "Enable EEPROM AT24C1024x"        "EEPROM::Init 10"
    "Send Boot File"                  "GENERIC::SendBootFileGUI"
}

set EEPROM::appletAddr          0x300000
set EEPROM::appletMailboxAddr   0x300004
set EEPROM::appletFileName      "$libPath(extLib)/$target(board)/isp-eeprom-at91sam9g45.bin"

################################################################################
## NANDFLASH
################################################################################
array set at91sam9g45_nandflash {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam9g45_nandflash_scripts"
}

array set at91sam9g45_nandflash_scripts {
    "Enable NandFlash"    "NANDFLASH::Init"
    "Send Boot File"      "GENERIC::SendBootFileGUI"
    "Erase All"           "NANDFLASH::EraseAll"
    "Scrub NandFlash"     "NANDFLASH::EraseAll $NANDFLASH::scrubErase"
    "List Bad Blocks"     "NANDFLASH::BadBlockList"
}

set NANDFLASH::appletAddr          0x70000000
set NANDFLASH::appletMailboxAddr   0x70000004
set NANDFLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-nandflash-at91sam9g45.bin"

TCL_Write_Int $target(handle) 0x4000000 0xfffff410
TCL_Write_Int $target(handle) 0x4000000 0xfffff430

################################################################################
## NORFLASH
################################################################################
array set at91sam9g45_norflash {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam9g45_norflash_scripts"
}

array set at91sam9g45_norflash_scripts {
    "Enable NorFlash "    "NORFLASH::Init"
    "Erase All"           "NORFLASH::EraseAll"
}
set NORFLASH::appletAddr          0x70000000
set NORFLASH::appletMailboxAddr   0x70000004
set NORFLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-norflash-at91sam9g45.bin"

################################################################################
array set at91sam9g45_norflash_map {
    dftDisplay  0
    dftDefault  0
    dftAddress  0x10000000
    dftSize     0x10000000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}

array set at91sam9g45_ddr2_sdram_map {
    dftDisplay  0
    dftDefault  0
    dftAddress  0x20000000
    dftSize     0x10000000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}

array set at91sam9g45_peripheral {
    dftDisplay  0
    dftDefault  0
    dftAddress  0xF0000000
    dftSize     0x10000000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}

array set at91sam9g45_rom {
    dftDisplay  0
    dftDefault  0
    dftAddress  0x400000
    dftSize     0x8000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}

array set at91sam9g45_remap {
    dftDisplay  0
    dftDefault  0
    dftAddress  0x00000000
    dftSize     0x8000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}





