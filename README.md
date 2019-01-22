hipnuc_imu_ros: A ROS wrapper for HiPNUC HI216
===================================

The HiPNUC HI216 is 6 DoF (Degree of Freedom)  IMU device that consists of 2 sensors: gyro and acceleration sensor.
More information about this sensor can be found in the official **HiPNUC** website:  [http://www.hipnuc.com](http://www.hipnuc.com/) 


Install and Configure ROS Package
---------------------------------
1) Install dependencies:

``` bash
$ sudo apt install libelf-dev
```

2) Download code and compile ROS package:
``` bash
$ cd ~/catkin_workspace/src
$ git clone https://github.com/teddyluo/hipnuc_imu_ros.git
$ cd ..
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="hipnuc_imu_ros" -j1
``` 

3) Connect HI216 device via a USB cable, then make a test:
``` bash
#The 1st terminal:
$ roscore
#The 2nd terminal:
$ roslaunch hipnuc_imu_ros imu.launch
```

Now you should see the received IMU results in the terminal. Enjoy!

Note: 
1. If the permision problem is encountered, please run the following command:

``` bash
$ sudo chmod 666 /dev/ttyUSB0
```
