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
Eigen::VectorXd x(6); // state vector {phi theta psi phidot thetadot psidot}
Eigen::VectorXd z(5); // measurements vector {ax ay gx gy gz}
Eigen::VectorXd y(5); // measurement pre-fit resitual {ax ay gx gy gz}
Eigen::MatrixXd F(6,6); // state transition model matrix
Eigen::MatrixXd P(6,6); // covariance matrix
Eigen::MatrixXd Q(6,6); // process noise covariance matrix
Eigen::MatrixXd K(6,5); // kalman gain matrix
Eigen::MatrixXd H(5,6); // observation model matrix
Eigen::MatrixXd R(5,5); // observation noise covariance matrix
Eigen::MatrixXd S(5,5); // pre-fit residual covariance matrix
Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6); // 6x6 Identity Matrix
double az = 9.81;

double ax_prev_;
double ay_prev_;
double alpha_ = 0.0;

ros::Subscriber imu_sub_;
ros::Publisher estimate_pub_;

ros::Time time_prev_;

geometry_msgs::Vector3 odom_msg_;