#include <ros/ros.h>
#include <serial/serial.h>
#include <stdint.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>
#include "csignal"
#include <std_msgs/Float32.h>

typedef union{
    float v;
    uint8_t v_4[4];
//    char v_4[4];
} float_union;

typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
}Vel_Pos_Data;

Vel_Pos_Data robot_pos_;
Vel_Pos_Data robot_vel_;
Vel_Pos_Data robot_vel_command_;

float_union v1, v2;

float_union r1, r2;

serial::Serial djSerial_;

double vx_command, vy_command, w_command;

float wl_command, wr_command;

float_union v1_stop, v2_stop;

//if false, use velocity command as feedback.
bool use_velocity_feedback_;

double v_feeback_, w_feedback_;

#define FRAME_HEADER 0X7B
#define FRAME_TAIL   0X7D
#define WHEEL_SPACING 0.84
#define WHEEL_DIA 0.076
#define pi 3.1415926

void sigintHandler(int sig)
{
    v1_stop.v = 0.0;
    v2_stop.v = 0.0;
    uint8_t send_stop_[8];
    send_stop_[0] = v1_stop.v_4[0];
    send_stop_[1] = v1_stop.v_4[1];
    send_stop_[2] = v1_stop.v_4[2];
    send_stop_[3] = v1_stop.v_4[3];
    send_stop_[4] = v2_stop.v_4[0];
    send_stop_[5] = v2_stop.v_4[1];
    send_stop_[6] = v2_stop.v_4[2];
    send_stop_[7] = v2_stop.v_4[3];
    djSerial_.write(send_stop_, sizeof (send_stop_));
    ROS_INFO("shutting down!!!");
    ros::shutdown();
}

void From_Wheelv_elocity_(double &vx, double &vy, double& w, double& wl, double& wr)
{
    wr = - wr; // + as robot moves forward.
    double r; // motion dia.

    r = (WHEEL_SPACING / 2) * (wr + wl) / (wr - wl);
    double v; // center linear velocity.
    v = (wl + wr) * WHEEL_DIA / 2;
    w = (wr - wl) / WHEEL_SPACING; // center angular velocity.
    vx = v; // todo.
    vy = 0.0;
    robot_vel_.X = vx;
    robot_vel_.Y = vy;
    robot_vel_.Z = w;
}

void To_Wheel_velocity_(double &vx, double &vy, double& w, float& wl, float& wr)
{
    wr = (float)(vx / WHEEL_DIA + w * WHEEL_SPACING / 2);
    wl = (float)(vx / WHEEL_DIA - w * WHEEL_SPACING / 2);
}

void command_callback(const geometry_msgs::TwistPtr& msg)
{
    ROS_INFO_THROTTLE(10, "test");
    uint8_t send_data_[8];
//    robot_vel_command_.X = msg->linear.x;
//    robot_vel_command_.Y = msg->linear.y;
//    robot_vel_command_.Z = msg->angular.z;
    double vx = msg->linear.x;
    double w = - msg->angular.z; // keep + as turn left.
    // use velocity as feedback.
    v_feeback_ = vx;
    w_feedback_ = w;
    ROS_WARN_STREAM_THROTTLE(3, "Command linear velocity " << vx << std::endl);
    ROS_WARN_STREAM_THROTTLE(3, "Command angular velocity " << w << std::endl);
    float wr_command, wl_command;
    wr_command = (float)(vx  + w * WHEEL_SPACING / 2) / WHEEL_DIA;
    wl_command = (float)(vx  - w * WHEEL_SPACING / 2) / WHEEL_DIA;
    // if rotate via a point, set wr = wl;
    //note:if command>8.0,the joystick cannot work???
    if(wl_command > 8.0)
        wl_command = 8.0;
    if(wl_command < -8.0)
        wl_command = -8.0;
    if(wr_command > 8.0)
        wr_command = 8.0;
    if(wr_command < -8.0)
        wr_command = -8.0;
    v1.v = -wr_command; // change direction.
    v2.v = wl_command;
    ROS_WARN_STREAM_THROTTLE(3, "Comand wheel velocity  " << wl_command << "  ,  " << wr_command << std::endl);
    send_data_[0] = v1.v_4[0];
    send_data_[1] = v1.v_4[1];
    send_data_[2] = v1.v_4[2];
    send_data_[3] = v1.v_4[3];
    send_data_[4] = v2.v_4[0];
    send_data_[5] = v2.v_4[1];
    send_data_[6] = v2.v_4[2];
    send_data_[7] = v2.v_4[3];

    djSerial_.write(send_data_, sizeof (send_data_));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_contact");//ROS初始化 并设置节点名称，可修改
  ROS_INFO("wheeltec_robot node has turned on ");//显示状态
  ros::NodeHandle nh_;
  ros::Rate rate_(40);
  signal(SIGINT, sigintHandler);


  ros::Subscriber velocity_commmand_subcriber_;
  velocity_commmand_subcriber_ = nh_.subscribe("/cmd_vel", 1000, command_callback);

  ros::Publisher wheel_velocity_pub_;
  wheel_velocity_pub_ = nh_.advertise<std_msgs::Float32>("/feedback_velocity",1000);

  ros::Publisher odom_publisher_;
  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 1000);
  nav_msgs::Odometry odom;//里程计话题消息数据类型
  odom.header.stamp = ros::Time::now();//当前时间
  odom.header.frame_id = "wheel_odom";
  odom.child_frame_id = "base";

  tf::TransformBroadcaster br;
  tf::Transform transform_base2odom;

//  serial::Serial djSerial_;
  try{
         djSerial_.setPort("/dev/ttyUSB0");//选择哪个口，如果选择的口没有接串口外设初始化会失败
         djSerial_.setBaudrate(115200);//设置波特率
         serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
         djSerial_.setTimeout(_time);
         djSerial_.open();//串口开启
    }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  }

  if(djSerial_.isOpen())
  {
      ROS_INFO("dJ serial is opened ");
  }

//  uint8_t send_data_[8];
  uint8_t receive_data_[10];
  uint8_t receive_data_reorder[10];


  float velocity_callback1_, velocity_callback2_;

  unsigned char Frame_Header, Frame_Tail;

  ros::Time last_time, now;
  float sampling_time = 0;
  last_time = ros::Time::now();
  double n_sample = 0;
  use_velocity_feedback_ = false;
  while(ros::ok())
  {
      sleep(0.005); // delay 5 ms.
      djSerial_.read(receive_data_, sizeof (receive_data_));
      ROS_INFO_STREAM_ONCE("receive data size " << sizeof (receive_data_) << std::endl);

      int Header_Pos, Tail_Pos;
      for(int i = 0; i < 10; i++)
      {
          if(receive_data_[i] == FRAME_HEADER)
          {
              Header_Pos = i;
              ROS_INFO_STREAM_ONCE("header pos " << Header_Pos << std::endl);
          }
          else if(receive_data_[i] == FRAME_TAIL)
          {
              Tail_Pos = i;
              ROS_INFO_STREAM_ONCE("tail pos " << Tail_Pos << std::endl);
          }
      }
      if(Tail_Pos == (Header_Pos + 9)) // correct order via 10 data.
      {
          memcpy(receive_data_reorder, receive_data_, sizeof (receive_data_));
      }
      else if (Header_Pos == (1 + Tail_Pos))
      {
          for(int j = 0; j < 10; j++)
              receive_data_reorder[j] = receive_data_[(j + Header_Pos) % 10];
      }
      else {
          ROS_WARN("Receive data has big error");
      }
      Frame_Header = receive_data_reorder[0];
      Frame_Tail  = receive_data_reorder[9];
      if(Frame_Header == FRAME_HEADER)
      {
          if(Frame_Tail == FRAME_TAIL)
          {
              r1.v_4[0] = receive_data_reorder[1];
              r1.v_4[1] = receive_data_reorder[2];
              r1.v_4[2] = receive_data_reorder[3];
              r1.v_4[3] = receive_data_reorder[4];

              r2.v_4[0] = receive_data_reorder[5];
              r2.v_4[1] = receive_data_reorder[6];
              r2.v_4[2] = receive_data_reorder[7];
              r2.v_4[3] = receive_data_reorder[8];
          }
      }

      double vx, vy, w, wl, wr;
      wl = r1.v;
      wr = r2.v;
      // trans linear velocity from wheel velocity.
      wr = - wr; // + as robot moves forward.
      double r; // motion dia.
      r = (WHEEL_SPACING / 2) * (wr + wl) / (wr - wl);
      double v; // center linear velocity.
      v = (wl + wr) * WHEEL_DIA / 2;
      w = (wr - wl)* WHEEL_DIA / WHEEL_SPACING; // center angular velocity.
      vx = v; // todo.
      vy = 0.0;
      if(use_velocity_feedback_)
      {
          ROS_WARN_ONCE("Use real velocity feedback");
          robot_vel_.X = vx;
          robot_vel_.Y = vy;
          robot_vel_.Z = w;
      }
      else
      {
          ROS_WARN_ONCE("Use idea velocity feedback");
          robot_vel_.X = v_feeback_;
          robot_vel_.Y = 0.0;
          robot_vel_.Z = -w_feedback_;
      }

      ////end.
      ROS_INFO_STREAM("Feedback left wheel velocity is " << wl << std::endl);
      ROS_INFO_STREAM("Feedback right wheel velocity is " << wr << std::endl);
      ROS_INFO_STREAM("Feedback linear velocity is " << robot_vel_.X << std::endl);
      ROS_INFO_STREAM("Feedback angular velocity is  " << robot_vel_.Z << std::endl);

      std_msgs::Float32 wheel_velocity_;
      wheel_velocity_.data = 0.36*(wl+wr)/2;
      wheel_velocity_pub_.publish(wheel_velocity_);
      now = ros::Time::now();
      sampling_time = (now - last_time).toSec();
      //todo!!!!
      robot_pos_.X += (robot_vel_.X * cos(robot_pos_.Z)) * sampling_time;
      robot_pos_.Y += -(robot_vel_.X) * sin(robot_pos_.Z) * sampling_time;
      robot_pos_.Z += robot_vel_.Z * sampling_time;
//      robot_pos_.X += (robot_vel_.X * sampling_time);
//      robot_pos_.Z += pi/2;
//      robot_pos_.X += -(robot_vel_.X * sin(robot_pos_.Z)) * sampling_time;
//      robot_pos_.Y += (robot_vel_.X * cos(robot_pos_.Z)) * sampling_time;
      //ROS_WARN_STREAM_THROTTLE(2, "n samples " << n_sample << std::endl);
      ROS_WARN_STREAM_THROTTLE(2, "robot angular z now " << -robot_pos_.Z << std::endl);
      ROS_WARN_STREAM_THROTTLE(2, "robot pos x now " << robot_pos_.X << std::endl);
      ROS_WARN_STREAM_THROTTLE(2, "robot pos y now " << robot_pos_.Y << std::endl);
      n_sample += 1;
      last_time = now;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pos_.Z);
      odom.pose.pose.position.x = robot_pos_.X;//位置
      odom.pose.pose.position.y = robot_pos_.Y;
      odom.pose.pose.orientation = odom_quat;
      //设置速度
      odom.twist.twist.linear.x =  robot_vel_.X;//X方向前进速度
      odom.twist.twist.linear.y =  robot_vel_.Y;//y方向前进速度
      odom.twist.twist.angular.z = robot_vel_.Z;  //角速度
      odom_publisher_.publish(odom);

      // send frame tf transform.
      tf::Quaternion orientation_;
          orientation_.setRPY(0, 0, -robot_pos_.Z); // to odom frame.

          transform_base2odom.setRotation(orientation_);
          transform_base2odom.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));

          br.sendTransform(tf::StampedTransform(transform_base2odom, ros::Time::now(), "wheel_odom", "base"));


      //write!!!!!!!

//      vx_command = robot_vel_command_.X; // forward speed.
//      w_command =robot_vel_command_.Z;
//      To_Wheel_velocity_(vx_command, vy_command, w_command, wl_command, wr_command);
//      if(wl_command > (float)5.0 || wr_command > (float)5.0)
//      {
//          wl_command = 1.0;
//          wr_command = 1.0;
//          ROS_WARN("Exceed max wheel angular velocity");
//          ros::shutdown();
//      }
//      float trans1 , trans2;
//      trans1 = wl_command;
//      trans2 = wr_command;
//      v1.v = wl_command;
//      v2.v = 2.0;

//      ROS_INFO_STREAM("wheel command" << wl_command << " " << v1.v << std::endl);

//      send_data_[0] = v1.v_4[0];
//      send_data_[1] = v1.v_4[1];
//      send_data_[2] = v1.v_4[2];
//      send_data_[3] = v1.v_4[3];
//      send_data_[4] = v2.v_4[0];
//      send_data_[5] = v2.v_4[1];
//      send_data_[6] = v2.v_4[2];
//      send_data_[7] = v2.v_4[3];


//      djSerial_.write(send_data_, sizeof (send_data_));

      ros::spinOnce();

      rate_.sleep();
  }
  return 0;
}
