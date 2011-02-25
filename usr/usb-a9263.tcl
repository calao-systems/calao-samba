##################
## USB-A9263
##################
DATAFLASH::Init 0
DATAFLASH::EraseAll
GENERIC::SendBootFileGUI
send_file {DataFlash AT45DB/DCB} "~/calao-buildroot/output/images/u-boot.bin" 0x4000 0
NANDFLASH::Init
NANDFLASH::EraseAll
send_file {NandFlash} "~/calao-buildroot/output/images/uImage.bin" 0xA0000 0
