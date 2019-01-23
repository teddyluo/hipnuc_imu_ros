#!/bin/bash
gcc ./imu_data_decode.c ./packet.c ./serial.c -o ./testimu
echo 'please run command before runing test: sudo chmod 666 /dev/ttyUSB0'
echo 'please plugin IMU hardware and run ./testimu'
