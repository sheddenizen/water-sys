#!/bin/bash
HERE=$(dirname $0)
REMOTE=pi@coeurtendre
DST=/tmp
scp \
	$IDF_PATH/components/esptool_py/esptool/esptool.py \
	$HERE/build/bootloader/bootloader.bin $HERE/build/water-sys.bin \
	$HERE/build/partitions.bin \
	$REMOTE:$DST || exit 1

ssh $REMOTE python $DST/esptool.py --chip esp32 --port /dev/ttyUSB0 \
	--baud 115200 --before default_reset --after hard_reset write_flash \
	-z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 \
	$DST/bootloader.bin 0x10000 $DST/water-sys.bin \
	0x8000 $DST/partitions.bin
