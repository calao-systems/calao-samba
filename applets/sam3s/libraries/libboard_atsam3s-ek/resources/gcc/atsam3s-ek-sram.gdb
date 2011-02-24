#*************************************************
#
#  Connect to J-Link and debug application in sram on SAM3S
#
# Note:
#     First,users should do Step1 and Step2 according to your project,
#     then do Step3.

# Step1: Connect to the J-Link gdb server
#target remote localhost:2331
#mon reset

# Step2: Load file(eg. getting-started project)
#load bin/getting-started-at91sam3s-ek-at91sam3s4-sram.elf
#symbol-file bin/getting-started-at91sam3s-ek-at91sam3s4-sram.elf

# Step3: Initializing PC and stack pointer
# Reset perpheral  (RSTC_CR)
set *0x400e1400 = 0xA5000004
# Modify pc value to even before writing pc register
mon reg sp=(0x20000000)
set *0x20000004 = *0x20000004 & 0xFFFFFFFE
mon reg pc=(0x20000004)
info reg