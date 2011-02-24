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

# Makefile for compiling isp-nandflash

#-------------------------------------------------------------------------------
# User-modifiable options
#-------------------------------------------------------------------------------


# Chip & board used for compilation
# (can be overriden by adding CHIP=chip and BOARD=board to the command-line)
SERIE = atsam3s
CHIP  = $(SERIE)4
BOARD = $(SERIE)-ek

ifeq (atsam3s4,$(CHIP))
BOARD_DIR = at91sam3s4-ek
endif

ifeq (atsam3s2,$(CHIP))
BOARD_DIR = at91sam3s2-ek
endif

ifeq (atsam3s1,$(CHIP))
BOARD_DIR = at91sam3s1-ek
endif

MEMORIES = sram

# Trace level used for compilation
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
ifdef DEBUG
TRACE_LEVEL = 3
else
TRACE_LEVEL = 0
endif

# Optimization level
# -O1 Optimize
# -O2 Optimize even more
# -O3 Optimize yet more
# -O0 Reduce compilation time and make debugging produce the expected results
# -Os Optimize for size

OPTIMIZATION = -Os


#-------------------------------------------------------------------------------
# Path
#-------------------------------------------------------------------------------

# Output file basename
OUTPUT = isp-nandflash-$(CHIP)

# Output directories
BIN = bin
OBJ = obj

# Libraries
PATH_ATML_LIB = ../../libraries
PATH_ATML_LIB_BOARD = $(PATH_ATML_LIB)/libboard_atsam3s-ek
PATH_ATML_LIB_CHIP = $(PATH_ATML_LIB)/libchip_atsam3s
PATH_RESOURCES = $(PATH_ATML_LIB_BOARD)/resources/gcc
ISP = ..
INSTALLDIR = "../tcl_lib/$(BOARD_DIR)/"

#-------------------------------------------------------------------------------
# Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Compilation tools
AR = $(CROSS_COMPILE)ar
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

# Flags
INCLUDES  = -I$(PATH_ATML_LIB_BOARD)
INCLUDES += -I$(PATH_ATML_LIB_BOARD)/include
INCLUDES += -I$(PATH_ATML_LIB_CHIP)
INCLUDES += -I$(PATH_ATML_LIB_CHIP)/include

CFLAGS += -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -Wall -DNDEBUG 
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -DTRACE_LEVEL=$(TRACE_LEVEL)
ASFLAGS = -mcpu=cortex-m3 -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = -g $(OPTIMIZATION) -nostartfiles -mcpu=cortex-m3 -mthumb -Wl,-Map=$(OUTPUT).map,--cref,--gc-sections -L$(PATH_ATML_LIB_BOARD)/lib -L$(PATH_ATML_LIB_CHIP)/lib 

LDLIBBOARD   = -lboard_atsam3s_ek_gcc_rel
LDLIBCHIP = -lchip_$(CHIP)_gcc_rel

#-------------------------------------------------------------------------------
# Files
#-------------------------------------------------------------------------------
VPATH += $(PATH_ATML_LIB_BOARD)/lib
VPATH += $(PATH_ATML_LIB_CHIP)/lib
VPATH += $(ISP)/common
VPATH += $(PATH_ATML_LIB)/memories/nandflash

C_OBJECTS += main.o
C_OBJECTS += isp_cstartup.o
C_OBJECTS += RawNandFlash.o EccNandFlash.o SkipBlockNandFlash.o
C_OBJECTS += NandFlashModel.o NandFlashModelList.o NandSpareScheme.o

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT)

MEMORIES =sram

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	mkdir $@

define RULES
C_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1))
	$(CC) $(LDFLAGS) -T"$(PATH_RESOURCES)/$(CHIP)/$$@_samba.lds" -o $(OUTPUT).elf $$^ $(LDLIBBOARD) $(LDLIBCHIP)
	$(OBJCOPY) -O binary $(OUTPUT).elf $(OUTPUT).bin
	$(SIZE) $$^ $(OUTPUT).elf
	@[ -e $(INSTALLDIR) ] || mkdir $(INSTALLDIR)
	@cp -f $(OUTPUT).bin $(INSTALLDIR)
	
$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ) $(BIN)
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile $(OBJ) $(BIN)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.bin $(BIN)/*.elf $(BIN)/*.map
	
	
