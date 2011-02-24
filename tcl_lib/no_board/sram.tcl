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
## NAMESPACE SRAM
################################################################################
################################################################################
namespace eval SRAM {
}

################################################################################
#  proc SRAM::sendFile
################################################################################
proc SRAM::sendFile { name addr } {
    global valueOfDataForSendFile
    if { [catch {set f [open $name r]}] } {
        set valueOfDataForSendFile 0
        puts stderr "-E- Can't open file $name"
        return
    }
    
    fconfigure $f -translation binary
    
    set size [file size $name]
    puts "-I- File size = $size byte(s)"
    set valueOfDataForSendFile [read $f $size]
    
    close $f

#     global target
#     set dummy_err 0
#     if {[catch {TCL_Write_Data $target(handle) $addr valueOfDataForSendFile $size dummy_err}]} {
#         puts stderr "-E- Can't send data, error in connection"
#         set valueOfDataForSendFile 0
#         return
#     }
    
    set valueOfDataForSendFile 0
}

################################################################################
#  proc SRAM::receiveFile
################################################################################
proc SRAM::receiveFile {name addr size} {
    #read data from target

    set result "0123456789abcdefghijklmnopqrstuvwxyz"

#     global target
#     set dummy_err 0
#     if {[catch {set result [TCL_Read_Data $target(handle) $addr $size dummy_err]}]} {
#         puts stderr "-E- Can't read data, error in connection"
#         return -1
#     }

    # put data in a file
    if { [catch {set f2 [open $name w+]}] } {
        puts stderr "-E- Can't open file $name"
        return -1
    }
    fconfigure $f2 -translation binary
    puts -nonewline $f2 $result
    close $f2
}

