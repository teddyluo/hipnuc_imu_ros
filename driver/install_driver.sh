#!/bin/bash
make clean
sudo apt install libelf-dev
make all
# copy to kernel folder
sudo cp ./cp210x.ko /lib/modules/$(uname -r)/kernel/drivers/serial
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/usb/serial/usbserial.ko
sudo insmod ./cp210x.ko
echo 
echo 'Please reboot your computer to make kernel take effect'
echo 
make clean
