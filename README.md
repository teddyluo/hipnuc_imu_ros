hipnuc_imu_ros
===================================

Description
----
A ROS wrapper for HiPNUC HI216 IMU Device.

The HiPNUC HI216 is a 6 DoF (Degree of Freedom) IMU device which consists of 2 sensors: gyro and acceleration sensor.
More information about this sensor can be found in the official `HiPNUC` website: [http://www.hipnuc.com](http://www.hipnuc.com/).

Install Linux Driver for HiPNUC HI216
-------------------------
1) Install dependencies:

``` bash
$ sudo apt install libelf-dev
```

2) Download code and compile linux driver:
``` bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/teddyluo/hipnuc_imu_ros.git
$ cd ~/catkin_ws/src/hipnuc_imu_ros/driver
$ make all
``` 
where `catkin_ws` is your **catkin workspace**.

3) Copy `cp210x.ko` to the kernel folder:

``` bash
sudo cp ./cp210x.ko /lib/modules/$(uname -r)/kernel/drivers/serial
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/usb/serial/usbserial.ko
sudo insmod ./cp210x.ko
```
4) (Optional) Clean the redundant files:
``` bash
make clean
```

5) (Optional) Test IMU device:
``` bash
$ cd ~/catkin_ws/src/hipnuc_imu_ros/driver-test
$ gcc ./imu_data_decode.c ./packet.c ./serial.c -o ./imu-test
```
Connect HI216 device via a USB cable, then verify the output:
``` bash
./imu-test
```


Configure ROS Package
---------------------------------

1. Compile ROS package:

``` bash
$ cd ~/catkin_ws
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="hipnuc_imu_ros" -j1
``` 

2. Connect HI216 device via a USB cable, then test the device according to the following commands :
``` bash
#The 1st terminal:
$ roscore
#The 2nd terminal:
$ roslaunch hipnuc_imu_ros imu.launch
```

Now you should see the received IMU data in the terminal. Enjoy!


*Note:*
---------------------------------
1. If the permission problem is encountered, please run the following command:

``` bash
$ sudo chmod 666 /dev/ttyUSB0
```
