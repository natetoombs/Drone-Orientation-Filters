#include <ros/ros.h>
#include <Eigen/Core>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

double angleFromAccel(double a1, double a2, double a3);
void imuCallback(const sensor_msgs::ImuConstPtr& msg);
void calculateEstimate(Eigen::Vector3d lin_acc, Eigen::Vector3d ang_vel);
void publishEstimate(double pitch, double roll, double yaw);
double wrap(double angle);
void resetTime();

ros::Subscriber imu_sub_;
ros::Publisher estimate_pub_;

double roll_prev_;
double pitch_prev_;
double yaw_prev_;
double alpha_;
double t_;
double t_prev_;
geometry_msgs::Vector3 odom_msg_;

ros::Time start_time_;