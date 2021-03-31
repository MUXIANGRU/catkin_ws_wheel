#include "vrpn_client_ros/vrpn_client_ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>
ros::Publisher odom_pub;
void AngleCB(geometry_msgs::PoseStamped::ConstPtr pose_){



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "self_localize");
  ros::NodeHandle nh, private_nh("~");
  ros::Subscriber optitrack_sub_ = nh.subscribe("/vrpn_client_node/RigidBody_SLAM_CAR1/pose",1,&AngleCB);
  odom_pub = nh.advertise<geometry_msgs::Vector3>("/my_test_angle",1);

  ros::spin();
  return 0;
}
