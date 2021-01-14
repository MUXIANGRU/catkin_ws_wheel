#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread.hpp>


class QuadrupedTeleOp
{
public:
    QuadrupedTeleOp(ros::NodeHandle& nodehandle);
    void commandUpdate();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;
    float leftStickLR ,leftStickUD,rightStickLR ,rightStickUD, crossKeyLR, crossKeyUD;//1/-1
    int keyA, keyB, keyX, keyY, LB, RB, keyBack, keyStart, keyPower,LT,RT;//0/1
    int countkey1=1;
    int countkey2=1;
    int countA_, countB_, countX_, countY_;
    int countline_ =0;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    boost::thread commandUpdateThread_;
};


QuadrupedTeleOp::QuadrupedTeleOp(ros::NodeHandle& nodehandle):
    nh_(nodehandle)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &QuadrupedTeleOp::joyCallback, this);
    commandUpdateThread_ = boost::thread(boost::bind(&QuadrupedTeleOp::commandUpdate, this));
    ROS_INFO("Initialized joy to twist node.");

}

void QuadrupedTeleOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //ROS_INFO_STREAM("in the joy callback");
    leftStickLR = joy->axes[0];//contiunus 1~-1
    leftStickUD = joy->axes[1];//contiunus -1~1
    //ROS_INFO_STREAM("value of leftstickud is" << leftStickUD);
    //LT = joy->axes[2];//contiunus 0~2
    rightStickLR = joy->axes[3];//contiunus 1~-1
    rightStickUD = joy->axes[4];//contiunus 1~-1
    //RT = joy->axes[5];//contiunus 0~2
    crossKeyLR = joy->axes[6];//1/-1
    crossKeyUD = joy->axes[7];//1/-1
    keyA = joy->buttons[0];//0/1
    keyB = joy->buttons[1];//0/1
    keyX = joy->buttons[2];//0/1
    keyY = joy->buttons[3];//0/1
    LB = joy->buttons[4];//0/1
    RB = joy->buttons[5];//0/1
    RT = joy->buttons[7];//0/1
    LT = joy->buttons[6];//0/1
    keyStart = joy->buttons[9];//0/1
    keyPower = joy->buttons[10];//0/1
    keyBack = joy->buttons[8];
    leftStickUD= leftStickUD+0.1217;
    leftStickLR= leftStickLR-0.0953;
}
void QuadrupedTeleOp::commandUpdate()
{
    ros::Rate rate(200);
    while (ros::ok()) {
        geometry_msgs::Twist wheel_velocity;
        //std::cout<<keyStart<<std::endl;
        if(keyStart == 1){
            ros::Duration five_seconds(10.0);

            //countline_++;
            //if(countline_<5000){
                wheel_velocity.linear.x = 4.2;
                wheel_velocity.linear.y = 0;
                wheel_velocity.linear.z = 0;
                wheel_velocity.angular.x = 0;
                wheel_velocity.angular.y = 0;
                wheel_velocity.angular.z = 0;
                vel_pub_.publish(wheel_velocity);
                //std::cout<<"count++++++++++++++++"<<" "<<countline_<<std::endl;
                five_seconds.sleep();
           //}else {
                //wheel_velocity.linear.x =0;
               //wheel_velocity.angular.z =0;
                //vel_pub_.publish(wheel_velocity);
            //}
        }


        if(keyA ==1){
            wheel_velocity.linear.x = -0.8;
            wheel_velocity.linear.y = 0;
            wheel_velocity.linear.z = 0;
            wheel_velocity.angular.x = 0;
            wheel_velocity.angular.y = 0;
            wheel_velocity.angular.z = 0;
        };
        if(keyB ==1){
            wheel_velocity.linear.x = 0.0;
            wheel_velocity.linear.y = 0;
            wheel_velocity.linear.z = 0;
            wheel_velocity.angular.x = 0;
            wheel_velocity.angular.y = 0;
            wheel_velocity.angular.z = 6.28/4;
        };
        if(keyX ==1){
            wheel_velocity.linear.x = 0.0;
            wheel_velocity.linear.y = 0;
            wheel_velocity.linear.z = 0;
            wheel_velocity.angular.x = 0;
            wheel_velocity.angular.y = 0;
            wheel_velocity.angular.z = -6.28/4;
        };
        if(keyY ==1){
            wheel_velocity.linear.x = 0.8;
            wheel_velocity.linear.y = 0;
            wheel_velocity.linear.z = 0;
            wheel_velocity.angular.x = 0;
            wheel_velocity.angular.y = 0;
            wheel_velocity.angular.z = 0;
        };

        if(LB==1){
            vel_pub_.publish(wheel_velocity);
            //ROS_INFO_STREAM("send the velocity " << wheel_velocity.linear.x );
            //ROS_INFO_STREAM("send the angularity " << wheel_velocity.angular.z );
        }
        else {
            wheel_velocity.linear.x =0;
            wheel_velocity.angular.z =0;
            vel_pub_.publish(wheel_velocity);
        };
        if(RB==1){
            wheel_velocity.linear.x =0;
            wheel_velocity.angular.z =0;
            vel_pub_.publish(wheel_velocity);
            ROS_WARN("STOPED!!!!!!!!!!!!!!!!!!!!!!!");

        }

        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_wheel_node");
    ros::NodeHandle nh("~");
    QuadrupedTeleOp teleop_turtle(nh);
//    ros::Rate rate(400);
//    while (ros::ok()) {
//        rate.sleep();
//        ros::spinOnce();
//    }
    ros::spin();

}
