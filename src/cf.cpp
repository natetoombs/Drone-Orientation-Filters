#include <ros/ros.h>
#include "../include/cf.h"
#include <cmath>
#include <ros/console.h>
#include "tf/transform_datatypes.h"

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "complimentary_filter");
    
    ros::NodeHandle nh_;

    imu_sub_ = nh_.subscribe("mavros/imu/data_raw", 100, imuCallback);    
    estimate_pub_ = nh_.advertise<geometry_msgs::Vector3>("cf_estimate", 1);
    
    roll_prev_ = 0;
    pitch_prev_ = 0;
    yaw_prev_ = 1.62;
    alpha_ = 0.01;
   
    resetTime();   
    ros::spin();

    return 0;
}

double angleFromAccel(double a1, double a2, double a3)
// Compute an angle (roll/pitch) from the IMU linear accelerations
{
    double angle = atan2(a1, sqrt(pow(a2,2) + pow(a3,2)));

    return angle;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
// Receive the IMU data, sort, send to calculateEstimate()
{
    if (start_time_.sec == 0)
    {
        start_time_ = msg->header.stamp;
    }
    
    t_ = (msg->header.stamp - start_time_).toSec(); // Time elapsed from Start

    Vector3d lin_acc;
    lin_acc << msg->linear_acceleration.x, // East
               msg->linear_acceleration.y, // North
               msg->linear_acceleration.z; // Up
    Vector3d ang_vel;
    ang_vel << msg->angular_velocity.x, // Pitch
               msg->angular_velocity.y, // Roll
               msg->angular_velocity.z; // Yaw
    
    calculateEstimate(lin_acc, ang_vel);
}

void calculateEstimate(Vector3d lin_acc, Vector3d ang_vel)
// Implement the complimentary filter here.
{
    double dt = t_ - t_prev_;
    t_prev_ = t_;

    if (dt < 0)
    {
        resetTime();
    }

    double rollFromAccel = angleFromAccel(lin_acc[1], lin_acc[0], lin_acc[2]);
    double roll = alpha_*rollFromAccel + (1 - alpha_)*(roll_prev_ + ang_vel[0]*dt);
    roll_prev_ = roll;

    double pitchFromAccel = angleFromAccel(lin_acc[0], lin_acc[1], lin_acc[2]);
    double pitch = alpha_*pitchFromAccel + (1 - alpha_)*(pitch_prev_ + ang_vel[1]*dt);
    pitch_prev_ = pitch;

    double yaw = wrap(yaw_prev_ + ang_vel[2]*dt);
    yaw_prev_ = yaw;

    publishEstimate(roll, pitch, yaw);
}

void publishEstimate(double roll, double pitch, double yaw)
// Publishes a Vector3 orientation
{
    odom_msg_.x = roll;
    odom_msg_.y = pitch;
    odom_msg_.z = yaw;

    estimate_pub_.publish(odom_msg_);
}

double wrap(double angle)
{
    if (angle > 3.1415926535)
    {
        angle = angle - 2*3.1415926535;
    }
    else if (angle < -3.1415926535)
    {
        angle = angle + 2*3.1415926535;
    }

    return angle;
}

void resetTime()
{
    start_time_.fromSec(0.0);
    t_prev_ = 0;
}