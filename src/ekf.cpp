#include <ros/ros.h>
#include "../include/ekf.h"
#include <ros/console.h>
#include "tf/transform_datatypes.h"
#include <thread>

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extended_kalman_filter");
    ros::NodeHandle nh_;

    imu_sub_ = nh_.subscribe("mavros/imu/data_raw", 100, imuCallback);
    estimate_pub_ = nh_.advertise<geometry_msgs::Vector3>("ekf_estimate", 1);

    std::thread propagation(propagate);

    // Initialize x
    x << 0, 0, 1.62, 0, 0, 0;

     // Initialize Q
    double process_noise = 0.000001;
    Q = I*process_noise;
    
    // Initialize P
    P = Q;
    
    // Initialize R
    Vector3d imu_lin_cov(1.2184696791468346e-1, 1.2184696791468346e-1, 1.2184696791468346e-1);
    Vector3d imu_ang_cov(0.00000009, 0.00000009, 0.00000009);
    VectorXd meas_cov(6);
    meas_cov << imu_lin_cov, imu_ang_cov;
    R = meas_cov.asDiagonal();
    
    // Initialize LPF's for accelerometer
    ax_prev_ = 0;
    ay_prev_ = 0;
    az_prev_ = 0;

    time_prev_ = ros::Time::now();
    ros::spin();

    propagation.join();

    return 0;
}


void imuCallback(const sensor_msgs::ImuConstPtr& msg)
// Receive the IMU data, sort, send to calculateEstimate()
{
    Vector3d lin_acc;
    lin_acc << msg->linear_acceleration.x, // East
               msg->linear_acceleration.y, // North
               msg->linear_acceleration.z; // Up
    Vector3d ang_vel;
    ang_vel << msg->angular_velocity.x, // Pitch
               msg->angular_velocity.y, // Roll
               msg->angular_velocity.z; // Yaw
    
    // Alpha filter
    lin_acc(0) = alpha_*ax_prev_ + (1 - alpha_)*lin_acc(0);
    lin_acc(1) = alpha_*ay_prev_ + (1 - alpha_)*lin_acc(1);
    lin_acc(2) = alpha_*az_prev_ + (1 - alpha_)*lin_acc(2);
    ax_prev_ = lin_acc(0);
    ay_prev_ = lin_acc(1);
    az_prev_ = lin_acc(2);

    VectorXd z_temp(6);
    z_temp << lin_acc, ang_vel;
    z = z_temp;

    // propagate();
    update();
    publishEstimate();
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

void propagate()
{   
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        double dt = now.toSec() - time_prev_.toSec();
        time_prev_ = now;
        Vector3d dt_vec(dt, dt, dt);
        // Build F
        F = I;
        F.topRightCorner(3, 3) = dt_vec.asDiagonal();
        x = F*x;
        P = F*P*F.transpose() + Q;
        rate.sleep();
    }
}

void update()
{
    setH();
    seth();
    y = z - h;
    S = H*P*H.transpose() + R;
    K = P*H.transpose()*S.inverse();
    x = x + K*y;
    P = (I - K*H)*P;
}

void publishEstimate()
{
    odom_msg_.x = x(0);
    odom_msg_.y = x(1);
    odom_msg_.z = wrap(x(2));

    estimate_pub_.publish(odom_msg_);
}

void setH()
{
    H << 0, -g*cos(x(1)), 0, 0, 0, 0,
         g*cos(x(1))*cos(x(0)), -g*sin(x(1))*sin(x(0)), 0, 0, 0, 0,
         -g*cos(x(1))*sin(x(0)), -g*sin(x(1))*cos(x(0)), 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
}

void seth()
{
    VectorXd h_temp(6);
    h_temp << -g*sin(x(1)), g*cos(x(1))*sin(x(0)), g*cos(x(1))*cos(x(0)),
         x(3), x(4), x(5);
    h = h_temp;
}