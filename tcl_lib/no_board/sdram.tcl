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
## NAMESPACE SDRAM
################################################################################
################################################################################
namespace eval SDRAM {
}

################################################################################
#  proc SDRAM::initSDRAM
################################################################################
proc SDRAM::initSDRAM {} {

    puts "-I- Configure PIOA as peripheral PIOA23-29"
    
    puts "-I- Configure PIOB as peripheral (A0/A17)"

    puts "-I- Configure PIOC as peripheral (D0/D15)"

    puts "-I- Init the EBI for SDRAM"

    puts "-I- Init SDRAM"
    puts "-I- 1. A minimum pause of 200us is provided to precede any signal toggle"
    
    puts "-I- 1. SDRAM Initialization Step"
    puts "-I- *pSDRAM = 0;"

    puts "-I- 2. A Precharge All command is issued to the SDRAM"
    puts "-I- *pSDRAM = 0;"

    puts "-I- 3. Eight Auto-refresh are provided"
    puts "-I- *pSDRAM = 0;"

    puts "-I- 4. A mode register cycle is issued to program the SDRAM parameters"
    puts "-I- *(pSDRAM+0x20) = 0;"

    puts "-I- 5. Write refresh rate into SDRAMC refresh timer COUNT register"

    puts "-I- 6. A Normal Mode Command is provided, 3 clocks after tMRD is set"
    puts "-I- *pSDRAM = 0;"

    puts "-I- End of Init_SDRAM"
}

################################################################################
#  proc SDRAM::sendFile
################################################################################
proc SDRAM::sendFile { name addr } {
    
    global valueOfDataForSendFile
    if { [catch {set f [open $name r]}] } {
        set valueOfDataForSendFile 0
        puts stderr "-E- Can't open file $name"
        return -1
    }
    
    fconfigure $f -translation binary
    
    set size [file size $name]
    puts "-I- File size = $size byte(s)"
    set valueOfDataForSendFile [read $f $size]
    
    close $f

    set valueOfDataForSendFile 0
}

################################################################################
#  proc SDRAM::receiveFile
################################################################################
proc SDRAM::receiveFile {name addr size} {
    set result "0123456789abcdefghijklmnopqrstuvwxyz"

    #read data from target
    # put data in a file
    if { [catch {set f2 [open $name w+]}] } {
        puts stderr "-E- Can't open file $name"
        return -1
    }
    fconfigure $f2 -translation binary
    puts -nonewline $f2 $result
    close $f2
}
