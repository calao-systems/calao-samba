#######################
## HABA-KNX-EXPLORER ##
#######################
NANDFLASH::Init
NANDFLASH::EraseAll
GENERIC::SendBootFileGUI
send_file {NandFlash} "/home/ghermant/calao/buildroot-binaries/HABA-KNX-EXPLORER/u-boot.bin" 0x20000 0
send_file {NandFlash} "/home/ghermant/calao/buildroot-binaries/HABA-KNX-EXPLORER/uImage.bin" 0xA0000 0
