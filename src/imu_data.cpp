#include <string>
#include <ros/ros.h> /*ros header*/
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp> /*boost library*/
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h" /*String prototypde defined in ros*/
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include "packet.hpp"
#include "imu_data_decode.hpp"

using namespace std;
using namespace boost::asio; /*use a namespace to enable io operation*/

unsigned char buf[17]; /*define buffer to store raw IMU data*/

std::string string_to_hex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();
    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");  /*node initialization*/
    ros::NodeHandle n;

    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);
    /*define message name and sulv*/

    ros::Rate loop_rate(100);

    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0"); /*serial port */
    sp.set_option(serial_port::baud_rate(115200)); /*baud rate*/
    sp.set_option(serial_port::flow_control());  /*flow control*/

    sp.set_option(serial_port::parity());  /*enable parity*/
    //sp.set_option(  serial_port::parity( serial_port::parity::none ) );

    sp.set_option(serial_port::stop_bits()); /*stop bits*/
    //sp.set_option( serial_port::stop_bits(  serial_port::stop_bits::one ) );

    sp.set_option(serial_port::character_size(8)); /*character size*/

    //int fd=open_port("/dev/ttyUSB0");
    imu_data_decode_init();
    
    uint8_t ID = 0;
    int16_t Acc[3] = {0};
    int16_t Gyo[3] = {0};
    int16_t Mag[3] = {0};
    float Eular[3] = {0};
    float Quat[4]  = {0};
    int32_t Pressure = 0;

    int i;
    uint8_t buf[1024];
    while (ros::ok())
    {
        ssize_t n = read(sp,buffer(buf));
        for(i=0; i<n; i++)
        {
            Packet_Decode(buf[i]);
        }

        get_raw_acc(Acc);
        get_raw_gyo(Gyo);
        get_raw_mag(Mag);
        get_eular(Eular);
        get_quat(Quat);
        get_id(&ID);
        printf("---new data---\n");
        printf("Acc:%d %d %d\r\n",Acc[0], Acc[1], Acc[2]);
        printf("Gyo:%d %d %d\r\n",Gyo[0], Gyo[1], Gyo[2]);
        printf("Mag:%d %d %d\r\n",Mag[0], Mag[1], Mag[2]);
        printf("Eular(P R Y):%0.2f %0.2f %0.2f\r\n",Eular[0], Eular[1], Eular[2]);
        printf("quat(W X Y Z):%0.3f %0.3f %0.3f %0.3f\r\n",Quat[0], Quat[1], Quat[2], Quat[3]);
        printf("\n\n");

        // ROS_INFO(("quat(W X Y Z):%0.3f %0.3f %0.3f %0.3f\r\n",Quat[0], Quat[1], Quat[2], Quat[3]);
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        imu_data.orientation.x = Quat[1];
        imu_data.orientation.y = Quat[2];
        imu_data.orientation.z = Quat[3];
        imu_data.orientation.w = Quat[0];

        imu_data.orientation_covariance[0] = 1000000;
        imu_data.orientation_covariance[1] = 0;
        imu_data.orientation_covariance[2] = 0;
        imu_data.orientation_covariance[3] = 0;
        imu_data.orientation_covariance[4] = 1000000;
        imu_data.orientation_covariance[5] = 0;
        imu_data.orientation_covariance[6] = 0;
        imu_data.orientation_covariance[7] = 0;
        imu_data.orientation_covariance[8] = 0.000001;

        imu_data.angular_velocity.x = 0.0;
        imu_data.angular_velocity.y = 0.0;
        imu_data.angular_velocity.z = (double)-1*(Gyo[2]*3.14/(180*100));

        imu_data.linear_acceleration.x = (double)(-1*((Acc[0]+10) * 9.80665/1000.f));
        imu_data.linear_acceleration.y = (double)(-1*((Acc[1]+24) * 9.80665/1000.f));
        imu_data.linear_acceleration.z = (double)(-1*((Acc[2]-1070) * 9.80665/1000.f));

        IMU_pub.publish(imu_data);

        // usleep(20*1000);

        //    read (sp,buffer(buf));
        //    string str(&buf[0],&buf[17]);  /*convert array-type into string-type*/

        //    std_msgs::String msg;
        //    std::stringstream ss;
        //    ss << str;
        //    std_msgs::Float32  Yaw; /*yaw of robot, in degree*/
        //    char higher;
        //    char lower;
        //    higher = buf[5];
        //    lower = buf[4];
        //    Yaw.data= (float)((higher * 256 + lower))/100;
        //    cout << Yaw << endl;   //for debugging
        //    msg.data = string_to_hex(ss.str());
        //    ROS_INFO("%s", msg.data.c_str()); /*print out the received string*/
        //    IMU_pub.publish(Yaw); /*publish message*/
        
        ros::spinOnce();
        loop_rate.sleep();

    }
    iosev.run();
    return 0;
}
