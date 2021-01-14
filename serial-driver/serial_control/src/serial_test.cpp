#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
# include <stdio.h>
# include <stdlib.h>
#define pi 3.1415926

using namespace std;
using namespace boost::asio;

//        io_service iosev;
//        serial_port sp1(iosev, "/dev/ttyUSB0");
        // 设置参数
        /*sp1.set_option(serial_port::baud_rate(9600));
        sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp1.set_option(serial_port::parity(serial_port::parity::none));
        sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp1.set_option(serial_port::character_size(8));*/

int main(int argc, char* argv[])
{
        ros::init(argc, argv, "serial_test");

        ros::NodeHandle nh_;

        io_service iosev;
        //节点文件
        serial_port sp(iosev, "/dev/ttyUSB1");
        // 设置参数
        sp.set_option(serial_port::baud_rate(115200));
//        sp.set_option(serial_port::baud_rate(9600));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));
        ROS_INFO("Set Params");
        // 向串口写数据

        ros::Rate rate_(1);
        while(ros::ok())
        {
            char open[4]={0x55,0xB4,0x00,0x01};
            char cmd_vel_serial[11]={0x7B,1,0,1,1,0,0,0,0,1,0x7D};
            write(sp, buffer(cmd_vel_serial, 11));
            ROS_INFO("Write!");
            // 向串口读数据
            char buf[11];
            read(sp, buffer(buf));
            ROS_INFO("Read! ");
            cout<<"result = "<<buf<<endl;

            iosev.run();

            ros::spinOnce();

            rate_.sleep();
        }

        return 0;
}
