# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support 
# ----------------------------------------------------------------------------
# Copyright (c) 2010, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

# 	Makefile for compiling isp-project flash applet

#-------------------------------------------------------------------------------
#		User-modifiable options
#-------------------------------------------------------------------------------

# Chip & board used for compilation
# (can be overriden by adding CHIP=chip and BOARD=board to the command-line)
CHIP  = at91sam7s256
BOARD = at91sam7s-ek

# Optimization level, put in comment for debugging
OPTIMIZATION = -Os

# define default install directory
BOARD_DIR = $(CHIP)-ek
ifeq ($(BOARD),at91sam7sl-stk)
BOARD_DIR = $(CHIP)-stk
endif

INSTALLDIR = "../tcl_lib/$(BOARD_DIR)/"

# AT91 library directory
AT91LIB = ../../at91lib

# External library
EXT_LIBS= ../../external_libs

# Output file basename
OUTPUT = isp-flash-$(CHIP)

# Compile with chip specific features
include $(AT91LIB)/boards/$(BOARD)/$(CHIP)/chip.mak

# Compile for all memories available on the board (this sets $(MEMORIES))
include $(AT91LIB)/boards/$(BOARD)/board.mak

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy

# Flags
INCLUDES += -I$(AT91LIB)/boards/$(BOARD)
INCLUDES += -I$(AT91LIB)/peripherals
INCLUDES += -I$(AT91LIB)/memories
INCLUDES += -I$(AT91LIB)

ifeq (cortexm3,$(CHIP_CORE))
INCLUDES += -I$(EXT_LIBS)/cmsis
endif

ifeq (cortexm3,$(CHIP_CORE))
    # For Cortex-M3: thumb code only mthumb32
    TARGET_OPTS = -mcpu=cortex-m3 -mthumb 
else
    # ARM7 or 9
    TARGET_OPTS =
endif


CFLAGS += $(TARGET_OPTS)
CFLAGS += -Wall -mlong-calls -ffunction-sections
CFLAGS += -g $(OPTIMIZATION) -mthumb $(INCLUDES) -D$(CHIP) -DDYN_TRACES -DSAM_BA
# For devices with EFC (not EEFC)
CFLAGS += -DEFC_EVEN_ODD_PROG
ASFLAGS = $(TARGET_OPTS) -Wall -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = -g $(OPTIMIZATION) -nostdlib $(TARGET_OPTS) -Wl,-Map=$(OUTPUT).map,--gc-sections #-nostartfiles 


#-------------------------------------------------------------------------------
# Trace level used for compilation
#-------------------------------------------------------------------------------
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0

# Define if traces are allowed depending on the chip
#-------------------------------------------------------------------------------
# Disable trace capability for all devices
TR_LEV = 0
CFLAGS += -UDYN_TRACES

CFLAGS += -DTRACE_LEVEL=$(TR_LEV)


#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found
ISP = ..
PERIPH = $(AT91LIB)/peripherals
BOARDS = $(AT91LIB)/boards
MEM = $(AT91LIB)/memories
UTILITY = $(AT91LIB)/utility

VPATH += $(ISP)/common
VPATH += $(MEM)/flash
VPATH += $(UTILITY)
VPATH += $(PERIPH)/dbgu
VPATH += $(PERIPH)/pio
VPATH += $(PERIPH)/efc
VPATH += $(PERIPH)/eefc
VPATH += $(PERIPH)/cp15
VPATH += $(BOARDS)/$(BOARD)
VPATH += $(BOARDS)/$(BOARD)/$(CHIP)
VPATH += $(EXT_LIBS)/cmsis

# Objects built from C source files
C_OBJECTS += main.o
C_OBJECTS += string.o
C_OBJECTS += math.o
C_OBJECTS += trace.o
C_OBJECTS += dbgu.o
C_OBJECTS += stdio.o
C_OBJECTS += cp15.o
C_OBJECTS += pio.o
C_OBJECTS += board_lowlevel.o
C_OBJECTS += board_memories.o 

ifeq ($(CHIP_IP_EFC),EFC)
C_OBJECTS += efc.o
C_OBJECTS += flashd_efc.o
else ifeq ($(CHIP_IP_EFC),EEFC)
C_OBJECTS += eefc.o
C_OBJECTS += flashd_eefc.o
endif

ifeq (cortexm3,$(CHIP_CORE))
    # Cortex M3
    C_OBJECTS += isp_cstartup.o
    C_OBJECTS += exceptions.o
else
	# Objects built from Assembly source files
	ASM_OBJECTS = isp_cstartup.o
endif

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------
all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	mkdir $@

define RULES
C_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1)) 
	$(CC) $(LDFLAGS) -T"$(AT91LIB)/boards/$(BOARD)/$(CHIP)/$$@_samba.lds" -o $(OUTPUT).elf $$^
	$(OBJCOPY) -O binary $(OUTPUT).elf $(OUTPUT).bin
	$(SIZE) $$^ $(OUTPUT).elf
	@[ -e $(INSTALLDIR) ] || mkdir $(INSTALLDIR)
	@cp -f $(OUTPUT).bin $(INSTALLDIR)

$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ) $(BIN)
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile $(OBJ) $(BIN)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

debug:
	perl ../../resources/gdb/debug.pl $(OUTPUT).elf

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.bin $(BIN)/*.elf $(BIN)/*.map

