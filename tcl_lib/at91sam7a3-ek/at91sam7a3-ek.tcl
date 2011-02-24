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
    variable sramSize         0x10000
    variable maxBootSize      0
}

# Source procedures for compatibility with older SAM-BA versions
if { [ catch { source "$libPath(extLib)/common/functions.tcl"} errMsg] } {
    messageDialg  error.gif "Function file not found:\n$errMsg" "File not found" ok
    exit
}

array set memoryAlgo {
    "SRAM"                    "::at91sam7a3_sram"
    "Flash"                   "::at91sam7a3_flash"
    "DataFlash AT45DB/DCB"    "::at91sam7a3_dataflash"
    "SerialFlash AT25/AT26"	  "::at91sam7a3_serialflash"
    "EEPROM AT24"             "::at91sam7a3_eeprom"
    "Peripheral"              "::at91sam7a3_peripheral"
    "REMAP"                   "::at91sam7a3_remap"
}

################################################################################
## SRAM
################################################################################
array set at91sam7a3_sram {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x200000
    dftSize     0x08000
    dftSend     "RAM::sendFile"
    dftReceive  "RAM::receiveFile"
    dftScripts  ""
}

################################################################################
## FLASH
################################################################################
array set at91sam7a3_flash {
    dftDisplay  1
    dftDefault  1
    dftAddress  0x100000
    dftSize     0x40000
    dftSend     "FLASH::SendFile"
    dftReceive  "FLASH::ReceiveFile"
    dftScripts  "::at91sam7a3_flash_scripts"
}
set FLASH::appletAddr          0x201000
set FLASH::appletMailboxAddr   0x201004
set FLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-flash-at91sam7a3.bin"

array set at91sam7a3_flash_scripts {
    "Erase All Flash"                      "FLASH::EraseAll"
    "Enable Flash access"                  "FLASH::Init"
}
# Initialize FLASH
if {[catch {FLASH::Init} dummy_err] } {
        if {$commandLineMode == 0} {
            messageDialg error.gif "Failed to initialize FLASH accesses" "FLASH init." ok
        }
        puts "-E- Error during FLASH initialization"
        # Close link
        TCL_Close $target(handle)
        exit
} else {
        puts "-I- FLASH initialized"
}

################################################################################
## DATAFLASH
################################################################################

array set at91sam7a3_dataflash {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam7a3_dataflash_scripts"
}

array set at91sam7a3_dataflash_scripts {
	"Enable Dataflash (SPI0 CS1)"                        "DATAFLASH::Init 0"
    "Enable Dataflash (SPI0 CS0)"                        "DATAFLASH::Init 1"
    "Erase All"                                          "DATAFLASH::EraseAll"
    "Set DF in Power-Of-2 Page Size mode (Binary mode)"  "DATAFLASH::BinaryPage"
}

set DATAFLASH::appletAddr          0x201000
set DATAFLASH::appletMailboxAddr   0x201004
set DATAFLASH::appletFileName   "$libPath(extLib)/$target(board)/isp-dataflash-at91sam7a3.bin"

################################################################################
## SERIALFLASH
################################################################################

array set at91sam7a3_serialflash {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam7a3_serialflash_scripts"
}

array set at91sam7a3_serialflash_scripts {
    "Enable Serialflash (SPI0 CS3)"   "SERIALFLASH::Init 0"
    "Erase All"                       "SERIALFLASH::EraseAll"
}

set SERIALFLASH::appletAddr          0x201000
set SERIALFLASH::appletMailboxAddr   0x201004
set SERIALFLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-serialflash-at91sam7a3.bin"


################################################################################
## EEPROM
################################################################################
array set at91sam7a3_eeprom {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam7a3_eeprom_scripts"
}

array set at91sam7a3_eeprom_scripts {
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
}

set EEPROM::appletAddr          0x201000
set EEPROM::appletMailboxAddr   0x201004
set EEPROM::appletFileName      "$libPath(extLib)/$target(board)/isp-eeprom-at91sam7a3.bin"

################################################################################
array set at91sam7a3_peripheral {
    dftDisplay  0
    dftDefault  0
    dftAddress  0xF0000000
    dftSize     0x10000000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}

array set at91sam7a3_remap {
    dftDisplay  0
    dftDefault  0
    dftAddress  0x00000000
    dftSize     0x2000
    dftSend     ""
    dftReceive  ""
    dftScripts  ""
}
