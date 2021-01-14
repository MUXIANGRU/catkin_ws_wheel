#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle nodehandle_;

    ros::Rate rate_(50);

    ros::Publisher command_publisher;
    command_publisher = nodehandle_.advertise<geometry_msgs::Twist>("twist", 1000);

    geometry_msgs::Twist command_;
    while(ros::ok())
    {
        command_.linear.x = -0.8;
        command_.linear.y = 0.0;
//        command_.angular.z = 6.28/4;
        command_.angular.z = 0.0;

        command_publisher.publish(command_);

        ros::spinOnce();

        rate_.sleep();

        ROS_INFO("publish !!!!");
    }

    return 0;

}
