#include <ros/ros.h>
#include <Eigen/Dense>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"

void imuCallback(const sensor_msgs::ImuConstPtr& msg);
void calculateEstimate(Eigen::Vector3d lin_acc, Eigen::Vector3d ang_vel);
double wrap(double angle);
void resetTime();
void publishEstimate();

void propagate();
void update();
void setH();
void seth();
Eigen::VectorXd x(6); // state vector {phi theta psi phidot thetadot psidot}
Eigen::VectorXd z(6); // measurements vector {ax ay az gx gy gz}
Eigen::VectorXd y(6); // measurement pre-fit resitual {ax ay gx gy gz}
Eigen::MatrixXd F(6,6); // state transition model matrix
Eigen::MatrixXd P(6,6); // covariance matrix
Eigen::MatrixXd Q(6,6); // process noise covariance matrix
Eigen::MatrixXd K(6,6); // kalman gain matrix
Eigen::MatrixXd H(6,6); // observation model matrix
Eigen::VectorXd h(6); // Map state to measurements
Eigen::MatrixXd R(6,6); // observation noise covariance matrix
Eigen::MatrixXd S(6,6); // pre-fit residual covariance matrix
Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6); // 6x6 Identity Matrix
double g = 9.81;

double ax_prev_;
double ay_prev_;
double az_prev_;
double alpha_ = 0.0;

ros::Subscriber imu_sub_;
ros::Publisher estimate_pub_;

ros::Time time_prev_;

geometry_msgs::Vector3 odom_msg_;