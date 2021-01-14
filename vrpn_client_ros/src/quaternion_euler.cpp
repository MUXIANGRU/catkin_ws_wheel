#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "vrpn_client_ros/vrpn_client_ros.h"
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Vector3.h>
geometry_msgs::PoseStamped my_test;
geometry_msgs::Vector3 my_angle;
ros::Publisher my_Pub,pose_convert_publisher;
ros::Subscriber my_sub;
Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//    std::cout << "Quaterniond2Euler result is:" <<std::endl;
//    std::cout << "x = "<< euler[2] << std::endl ;
//    std::cout << "y = "<< euler[1] << std::endl ;
//    std::cout << "z = "<< euler[0] << std::endl << std::endl;
    return euler;
}
double mapping_to_angle(double angle){
//MXR::note: how to convert quaternion to euler
    double fRad = angle;
    double Rad_to_deg = 45.0 / atan(1.0);
    double fAngle = fRad*Rad_to_deg;
    if(std::abs(fAngle)>160&&std::abs(fAngle)<200){
        fAngle=std::abs(fAngle)-180;
    }
    return fAngle;
}
void AngleCB(geometry_msgs::PoseStamped::ConstPtr pose_){
    //my_test.pose.orientation.x = static_cast<double>(Quaterniond2Euler(pose_->pose.orientation.x,pose_->pose.orientation.y,pose_->pose.orientation.z,pose_->pose.orientation.z).x());
    //my_test.pose.orientation.y = static_cast<double>(Quaterniond2Euler(pose_->pose.orientation.x,pose_->pose.orientation.y,pose_->pose.orientation.z,pose_->pose.orientation.z).y());
   //my_test.pose.orientation.z = static_cast<double>(Quaterniond2Euler(pose_->pose.orientation.x,pose_->pose.orientation.y,pose_->pose.orientation.z,pose_->pose.orientation.z).z());

    my_angle.x = mapping_to_angle(static_cast<double>(Quaterniond2Euler(pose_->pose.orientation.x,pose_->pose.orientation.y,pose_->pose.orientation.z,-pose_->pose.orientation.w).x()));
    my_angle.y = mapping_to_angle(static_cast<double>(Quaterniond2Euler(pose_->pose.orientation.x,pose_->pose.orientation.y,pose_->pose.orientation.z,-pose_->pose.orientation.w).y()));
    my_angle.z = mapping_to_angle( static_cast<double>(Quaterniond2Euler(pose_->pose.orientation.x,pose_->pose.orientation.y,pose_->pose.orientation.z,-pose_->pose.orientation.w).z()));

    my_Pub.publish(my_angle);
    //std::cout<<"wwwwwwooooo"<<std::endl;
    //the tf from motion_tracker to world_link
    Eigen::Isometry3d T1,T_pre,T_aft;
    T1.matrix()<<1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1;
    Eigen::Quaterniond q_pre(1,0,0,0);
    T_pre.rotate(q_pre);
    T_pre.pretranslate(Eigen::Vector3d(pose_->pose.position.x,pose_->pose.position.y,pose_->pose.position.z));
    T_aft = T1*T_pre;
    Eigen::Matrix4d M = T_aft.matrix();
    my_test.pose.position.x = M(0,3);
    my_test.pose.position.y = M(1,3);
    my_test.pose.position.z = M(2,3);
    pose_convert_publisher.publish(my_test);

}
int main(int argc, char **argv){
    ros::init(argc, argv, "quaternion_euler");
    ros::NodeHandle nh("~");

    my_sub = nh.subscribe("/vrpn_client_node/RigidBody_SLAM_CAR1/pose",1,&AngleCB);
    my_Pub = nh.advertise<geometry_msgs::Vector3>("/my_test_angle",1);
    pose_convert_publisher = nh.advertise<geometry_msgs::PoseStamped>("/pose_in_optitrack",1);
    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
