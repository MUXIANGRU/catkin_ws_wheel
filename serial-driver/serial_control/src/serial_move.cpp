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

		io_service iosev;
        serial_port sp1(iosev, "/dev/ttyUSB0");
        // 设置参数
        /*sp1.set_option(serial_port::baud_rate(9600));
        sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp1.set_option(serial_port::parity(serial_port::parity::none));
        sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp1.set_option(serial_port::character_size(8));*/

/*int main(int argc, char* argv[])
{
        io_service iosev;
        //节点文件
        serial_port sp(iosev, "/dev/ttyUSB0");
        // 设置参数
        sp.set_option(serial_port::baud_rate(115200));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));
        // 向串口写数据
	char open[4]={0x55,0xB4,0x00,0x01};
        write(sp, buffer("hello world", 11));

        // 向串口读数据
        char buf[11];
        read(sp, buffer(buf));
	cout<<"result = "<<buf<<endl;

        iosev.run();
        return 0;
}*/

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("I heard vx: [%f]", msg->linear.x);
	ROS_INFO("I heard vy: [%f]", msg->linear.y);
	ROS_INFO("I heard wz: [%f]", msg->angular.z);
	double vx=1000*msg->linear.x;
	double vy=1000*msg->linear.y;
	double wz=1000*msg->angular.z;
	char cmd_vel_serial[10]={0xff,0xfe,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0f};
                                              //vx      //vy      //wz      //方向
	//vx赋值
	int h8x,l8x,vxi;
	if(vx>0)cmd_vel_serial[9]&=0x0B; else vx=-vx;
	vxi=int(vx);
	h8x=vxi/256;
	l8x=vxi%256;
	//cout<<h8<<endl<<l8<<endl;
	cmd_vel_serial[3]+=l8x;
	cmd_vel_serial[4]+=h8x;
	//if(vx>0)cmd_vel_serial[9]&=0x0B;
	//vy赋值
	int h8y,l8y,vyi;
	if(vy>0)cmd_vel_serial[9]&=0x0D; else vy=-vy;
	vyi=int(vy);
	h8y=vyi/256;
	l8y=vyi%256;
	//if(vy>0)cmd_vel_serial[9]&=0x0D;
	cmd_vel_serial[5]+=l8y;
	cmd_vel_serial[6]+=h8y;
	//wz赋值
	int h8w,l8w,wzi;
	if(wz>0)cmd_vel_serial[9]&=0x0E; else wz=-wz;
	wzi=int(wz);
	h8w=wzi/256;
	l8w=wzi%256;
	//if(wz>0)cmd_vel_serial[9]&=0x0E;
	cmd_vel_serial[7]+=l8w;
	cmd_vel_serial[8]+=h8w;
	//cout<<hex<<cmd_vel_serial[9]<<endl;

          
 //节点文件
		/*io_service iosev;
        serial_port sp1(iosev, "/dev/ttyUSB0");
        // 设置参数
        sp1.set_option(serial_port::baud_rate(9600));
        sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp1.set_option(serial_port::parity(serial_port::parity::none));
        sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp1.set_option(serial_port::character_size(8));*/
        size_t data_len=write(sp1, buffer(cmd_vel_serial, 10));
		//size_t data_len=write(sp,buffer("hello world", 11));
		//write(sp, buffer(cmd_wheel, strlen(cmd_wheel)));
		cout<<data_len<<endl;
     /* char buf[1]={'\0'};
        read(sp, buffer(buf));
	cout<<"result = "<<buf<<endl;*/
		iosev.run();
		//return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "serial_move");


  ros::NodeHandle n;
        sp1.set_option(serial_port::baud_rate(115200));
        sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp1.set_option(serial_port::parity(serial_port::parity::none));
        sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp1.set_option(serial_port::character_size(8));
 
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);


  ros::spin();

  return 0;
}

