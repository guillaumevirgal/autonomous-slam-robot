idf.py build

sudo $(which esptool.py) -p /dev/ttyACM0   --before usb_reset --after hard_reset --chip esp32s3   write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB   0x0     build/bootloader/bootloader.bin   0x8000  build/partition_table/partition-table.bin   0x10000 build/idefix_firmware.bin

mkdir -p ~/robot_ws/logs/stage2

stdbuf -oL cat /dev/ttyACM0 | tee ~/robot_ws/logs/stage2/stage2_pid_$(date +%Y%m%d_%H%M).csv

while true; do   sudo stdbuf -oL cat /dev/ttyACM0; done | tee "$LOGFILE"