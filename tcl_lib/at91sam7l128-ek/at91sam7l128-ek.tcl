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
    variable sramSize         0x0001800
    variable maxBootSize      0
}

# Source procedures for compatibility with older SAM-BA versions
if { [ catch { source "$libPath(extLib)/common/functions.tcl"} errMsg] } {
    messageDialg  error.gif "Function file not found:\n$errMsg" "File not found" ok
    exit
}

array set memoryAlgo {
  "SRAM"         "::at91sam7l128_sram"
  "Flash"        "::at91sam7l128_flash"
  "Peripheral"   "::at91sam7l128_peripheral"
  "REMAP"        "::at91sam7l128_remap"
}

################################################################################
## SRAM
################################################################################
array set at91sam7l128_sram {
	dftDisplay  1
    dftDefault  0
	dftAddress  0x2FF000
	dftSize     0x1800
	dftSend     "RAM::sendFile"
	dftReceive  "RAM::receiveFile"
	dftScripts  ""
}

################################################################################
## FLASH
################################################################################
array set at91sam7l128_flash {
	dftDisplay  1
	dftDefault  1
	dftAddress  0x100000
	dftSize     0x20000
	dftSend     "FLASH::SendFile"
	dftReceive  "FLASH::ReceiveFile"
	dftScripts  "::at91sam7l128_flash_scripts"
}
#set FLASH::appletAddr       0x2FF000
set FLASH::appletAddr          0x2FFB40
set FLASH::appletMailboxAddr   0x2FFB44
set FLASH::appletFileName      "$libPath(extLib)/$target(board)/isp-flash-at91sam7l128.bin"

array set at91sam7l128_flash_scripts {
    "Enable Flash access"                  "FLASH::Init"
    "Erase All Flash"                      "FLASH::EraseAll"
    "Enable Security Bit"                  "FLASH::ScriptGPNMV 0"
    "Boot from Flash (GPNVM1)"             "FLASH::ScriptGPNMV 2"
    "Boot from ROM   (GPNVM1)"             "FLASH::ScriptGPNMV 3"
}

# Disable PIO interrupt (set by ROM code for autobaudrate) (fix for ROM code v1.3)

# Reinitialize AIC
TCL_Write_Int $target(handle) 0xFFFFFFFF $AT91C_AIC_IDCR dummy_err
TCL_Write_Int $target(handle) 0x0 $AT91C_AIC_SVR dummy_err
# Clear AIC source
TCL_Write_Int $target(handle) $AT91C_ID_PIOC $AT91C_AIC_ICCR dummy_err 
# Clear current IT on PIOC
# Acknowledge the IT with PIO_ISR
TCL_Read_Int $target(handle) $AT91C_PIOC_ISR dummy_err
# Disable IT DRXD on PIOC
TCL_Write_Int $target(handle) $AT91C_PC16_DRXD $AT91C_PIOC_IDR dummy_err

# Initialize FLASH
if {[catch {FLASH::Init } dummy_err] } {
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
array set at91sam7l128_peripheral {
	dftDisplay  0
    dftDefault  0
	dftAddress  0xF0000000
	dftSize     0x10000000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}

array set at91sam7l128_remap {
	dftDisplay  0
    dftDefault  0
	dftAddress  0x00000000
	dftSize     0x80000
	dftSend     ""
	dftReceive  ""
	dftScripts  ""
}
