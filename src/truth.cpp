#include <ros/ros.h>
#include <ros/console.h>
#include "tf/transform_datatypes.h"

ros::Subscriber truth_sub_;
ros::Publisher truth_euler_pub_;

void truthCallback(const geometry_msgs::PoseStamped msg)
{
    geometry_msgs::Quaternion truth_quat = msg.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(truth_quat, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    truth_euler_pub_.publish(rpy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpy_truth");
    
    ros::NodeHandle nh_;

    truth_sub_ = nh_.subscribe("mavros/local_position/pose", 100, truthCallback);
    truth_euler_pub_ = nh_.advertise<geometry_msgs::Vector3>("truth_euler", 1);

    ros::spin();

    return 0;
}