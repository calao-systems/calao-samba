###############
## QIL-A9G20
###############
NANDFLASH::Init
NANDFLASH::EraseAll
GENERIC::SendBootFileGUI
send_file {NandFlash} "~/calao-buildroot/output/images/u-boot.bin" 0x20000 0
send_file {NandFlash} "~/calao-buildroot/output/images/uImage.bin" 0xA0000 0
