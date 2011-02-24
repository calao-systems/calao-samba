#  ----------------------------------------------------------------------------
#          ATMEL Microcontroller Software Support
#  ----------------------------------------------------------------------------
#  Copyright (c) 2010, Atmel Corporation
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

namespace eval BOARD {
    variable sramSize         0x00010000
    variable maxBootSize      0
    # Vdd Memory 1.8V = 0 / Vdd Memory 3.3V = 1
    variable extRamVdd 1
    # External SDRAM = 0 / External DDR = 1 External PSRAM = 2 
    variable extRamType 2
    # Set bus width (16 or 32)
    variable extRamDataBusWidth 16
    # DDRAM Model (not used)
    variable extDDRamModel 0
}

################################################################################
## PROCEDURES FOR COMPATIBILITY WITH OLDER SAM-BA VERSIONS AND USER DEFINED
################################################################################
if { [ catch { source "$libPath(extLib)/common/functions.tcl"} errMsg] } {
    messageDialg  error.gif "Function file not found:\n$errMsg" "File not found" ok
    exit
}

array set memoryAlgo {
    "SRAM"         "::at91sam3n1_sram"
    "Flash"        "::at91sam3n1_flash"
    "SerialFlash AT25/AT26"   "::at91sam3n1_serialflash"
    "Peripheral"   "::at91sam3n1_peripheral"
    "REMAP"        "::at91sam3n1_remap"
}

################################################################################
## SRAM
################################################################################
array set at91sam3n1_sram {
	dftDisplay  1
    dftDefault  0
	dftAddress  0x20000000
	dftSize     0x4000
	dftSend     "RAM::sendFile"
	dftReceive  "RAM::receiveFile"
	dftScripts  ""
}



################################################################################
## FLASH
################################################################################
array set at91sam3n1_flash {
	dftDisplay  1
    dftDefault  1
    dftAddress  0x400000
    dftSize     0x10000
    dftSend     "FLASH::SendFile"
    dftReceive  "FLASH::ReceiveFile"
    dftScripts  "::at91sam3n1_flash_scripts"
}
array set at91sam3n1_flash_scripts {
        "Enable Security Bit (GPNVM0)"         "FLASH::ScriptGPNMV 0"
        "Boot from Flash (GPNVM1)"             "FLASH::ScriptGPNMV 2"
        "Boot from ROM (GPNVM1)"               "FLASH::ScriptGPNMV 3"
        "Erase All Flash"                      "FLASH::EraseAll"
        "Enable Flash access"                  "FLASH::Init "
        "Read unique ID"                       "FLASH::ReadUniqueID"
}

set FLASH::appletAddr             0x20000800
set FLASH::appletMailboxAddr      0x20000840
set FLASH::appletFileName         "$libPath(extLib)/$target(board)/isp-flash-atsam3n1.bin"

# Initialize FLASH
if {[catch {FLASH::Init} dummy_err]} {
        if {$commandLineMode == 0} {
            messageDialg error.gif "Failed to initialize FLASH access" "FLASH init." ok
        }
        puts "-E- Error during FLASH initialization"
        # Close link
        TCL_Close $target(handle)
        exit
} else {
        puts "-I- FLASH initialized"
}

################################################################################
## SERIALFLASH
################################################################################
array set at91sam3n1_serialflash {
    dftDisplay  1
    dftDefault  0
    dftAddress  0x0
    dftSize     "$GENERIC::memorySize"
    dftSend     "GENERIC::SendFile"
    dftReceive  "GENERIC::ReceiveFile"
    dftScripts  "::at91sam3n1_serialflash_scripts"
}

array set at91sam3n1_serialflash_scripts {
    "Enable Serialflash (SPI CS3)"    "SERIALFLASH::Init 0"
    "Erase All"                       "SERIALFLASH::EraseAll"
}

set SERIALFLASH::appletAddr          0x20001000
set SERIALFLASH::appletMailboxAddr   0x20001040
set SERIALFLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-serialflash-atsam3n1.bin"

################################################################################
array set at91sam3n1_peripheral {
	dftDisplay  0
    dftDefault  0
	dftAddress  0x40000000
	dftSize     0x10000000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}

array set at91sam3n1_remap {
	dftDisplay  0
    dftDefault  0
	dftAddress  0x00000000
	dftSize     0x40000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}

