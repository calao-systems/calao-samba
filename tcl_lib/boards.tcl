#  ----------------------------------------------------------------------------
#          ATMEL Microcontroller Software Support
#  ----------------------------------------------------------------------------
#  Copyright (c) 2009, Atmel Corporation
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
# List of target supported by SAM-BA software
# Board folder MUST have the same name as the board

array set boards {
    "at91sam3n1-ek"     "at91sam3n1-ek/at91sam3n1-ek.tcl"
    "at91sam3n2-ek"     "at91sam3n2-ek/at91sam3n2-ek.tcl"
    "at91sam3n4-ek"     "at91sam3n4-ek/at91sam3n4-ek.tcl"
    "at91sam3s1-ek"     "at91sam3s1-ek/at91sam3s1-ek.tcl"
    "at91sam3s2-ek"     "at91sam3s2-ek/at91sam3s2-ek.tcl"
    "at91sam3s4-ek"     "at91sam3s4-ek/at91sam3s4-ek.tcl"
    "at91sam3u1-ek"     "at91sam3u1-ek/at91sam3u1-ek.tcl"
    "at91sam3u2-ek"     "at91sam3u2-ek/at91sam3u2-ek.tcl"
    "at91sam3u4-ek"     "at91sam3u4-ek/at91sam3u4-ek.tcl"
    "at91sam7a3-ek"     "at91sam7a3-ek/at91sam7a3-ek.tcl"
    "at91sam7l64-ek"    "at91sam7l64-ek/at91sam7l64-ek.tcl"
    "at91sam7l128-ek"   "at91sam7l128-ek/at91sam7l128-ek.tcl"
    "at91sam7s161-ek"   "at91sam7s161-ek/at91sam7s161-ek.tcl"
    "at91sam7s16-ek"    "at91sam7s161-ek/at91sam7s161-ek.tcl"
    "at91sam7s32-ek"    "at91sam7s32-ek/at91sam7s32-ek.tcl"
    "at91sam7s321-ek"   "at91sam7s321-ek/at91sam7s321-ek.tcl"
    "at91sam7s64-ek"    "at91sam7s64-ek/at91sam7s64-ek.tcl"
    "at91sam7s128-ek"   "at91sam7s128-ek/at91sam7s128-ek.tcl"
    "at91sam7s256-ek"   "at91sam7s256-ek/at91sam7s256-ek.tcl"
    "at91sam7s512-ek"   "at91sam7s512-ek/at91sam7s512-ek.tcl"
    "at91sam7se32-ek"   "at91sam7se32-ek/at91sam7se32-ek.tcl"
    "at91sam7se256-ek"  "at91sam7se256-ek/at91sam7se256-ek.tcl"
    "at91sam7se512-ek"  "at91sam7se512-ek/at91sam7se512-ek.tcl"
    "at91sam7x128-ek"   "at91sam7x128-ek/at91sam7x128-ek.tcl"
    "at91sam7x256-ek"   "at91sam7x256-ek/at91sam7x256-ek.tcl"
    "at91sam7x512-ek"   "at91sam7x512-ek/at91sam7x512-ek.tcl"
    "at91sam7xc128-ek"  "at91sam7x128-ek/at91sam7x128-ek.tcl"
    "at91sam7xc256-ek"  "at91sam7x256-ek/at91sam7x256-ek.tcl"
    "at91sam7xc512-ek"  "at91sam7x512-ek/at91sam7x512-ek.tcl"
    "at91sam9260-ek"    "at91sam9260-ek/at91sam9260-ek.tcl"
    "at91sam9261-ek"    "at91sam9261-ek/at91sam9261-ek.tcl"
    "at91sam9263-ek"    "at91sam9263-ek/at91sam9263-ek.tcl"
    "at91sam9g10-ek"    "at91sam9g10-ek/at91sam9g10-ek.tcl"
    "at91sam9g20-ek"    "at91sam9g20-ek/at91sam9g20-ek.tcl"
    "at91sam9g45-ekes"    "at91sam9g45-ek/at91sam9g45-ek.tcl"
    "at91sam9m10-ekes"    "at91sam9m10-ek/at91sam9m10-ek.tcl"
    "at91sam9m10-g45-ek" "at91sam9m10-ek/at91sam9m10-ek.tcl"
    "at91sam9rl64-ek"   "at91sam9rl64-ek/at91sam9rl64-ek.tcl"
    "at91sam9xe128-ek"  "at91sam9xe128-ek/at91sam9xe128-ek.tcl"
    "at91sam9xe256-ek"  "at91sam9xe256-ek/at91sam9xe256-ek.tcl"
    "at91sam9xe512-ek"  "at91sam9xe512-ek/at91sam9xe512-ek.tcl"
    "at91cap9-dk-mem33" "at91cap9-dk/at91cap9-dk-mem33.tcl"
    "at91cap9-dk-mem18" "at91cap9-dk/at91cap9-dk-mem18.tcl"
    "at91cap9-stk"      "at91cap9-stk/at91cap9-stk.tcl"
    "at91cap7-dk-mem33" "at91cap7-dk/at91cap7-dk-mem33.tcl"
    "at91cap7-stk"      "at91cap7-stk/at91cap7-stk.tcl"
    "qil_at91sam9260"   "qil_at91sam9260/qil_at91sam9260.tcl"
    "qil_at91sam9g20"   "qil_at91sam9g20/qil_at91sam9g20.tcl"
    "sbc35_at91sam9260" "sbc35_at91sam9260/sbc35_at91sam9260.tcl"
    "sbc35_at91sam9g20" "sbc35_at91sam9g20/sbc35_at91sam9g20.tcl"
    "tny_at91sam9260"   "tny_at91sam9260/tny_at91sam9260.tcl"
    "tny_at91sam9g20"   "tny_at91sam9g20/tny_at91sam9g20.tcl"
    "tny_lpw_at91sam9g20" "tny_lpw_at91sam9g20/tny_lpw_at91sam9g20.tcl"
    "usb_at91sam9260"	"usb_at91sam9260/usb_at91sam9260.tcl"
    "usb_at91sam9g20"	"usb_at91sam9g20/usb_at91sam9g20.tcl"
    "usb_lpw_at91sam9g20" "usb_lpw_at91sam9g20/usb_lpw_at91sam9g20.tcl"
    "usb_at91sam9263"	"usb_at91sam9263/usb_at91sam9263.tcl"
    "tny_at91sam9263"	"tny_at91sam9263/tny_at91sam9263.tcl"
    "haba-knx_at91sam9g20"	"haba-knx_at91sam9g20/haba-knx-explorer.tcl"
    "no_board"          "no_board/no_board.tcl"
}

