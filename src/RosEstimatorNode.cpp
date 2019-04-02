#include "ros/ros.h"
#include "Estimator.hpp"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

typedef double T;
Filter<T>* filter_core_ptr;

void imu_cb(const sensor_msgs::ImuConstPtr& imu_msg) {
    imu_data tmp_data;
    tmp_data.t = imu_msg->header.stamp.toSec();
    tmp_data.acc(0) = imu_msg->linear_acceleration.x;
    tmp_data.acc(1) = imu_msg->linear_acceleration.y;
    tmp_data.acc(2) = imu_msg->linear_acceleration.z;
    tmp_data.gyro(0) = imu_msg->angular_velocity.x;
    tmp_data.gyro(1) = imu_msg->angular_velocity.y;
    tmp_data.gyro(2) = imu_msg->angular_velocity.z;
    filter_core_ptr->input_imu(tmp_data);
}

void vio_cb(const nav_msgs::OdometryPtr& vio_msg) {
    vio_data tmp_data;
    tmp_data.t = vio_msg->header.stamp.toSec();
    tmp_data.pos(0) = vio_msg->pose.pose.position.x;
    tmp_data.pos(1) = vio_msg->pose.pose.position.y;
    tmp_data.pos(2) = vio_msg->pose.pose.position.z;
    tmp_data.vel(0) = vio_msg->twist.twist.linear.x;
    tmp_data.vel(1) = vio_msg->twist.twist.linear.y;
    tmp_data.vel(2) = vio_msg->twist.twist.linear.z;
    Eigen::Quaterniond tmp_q;
    tmp_q.w() = vio_msg->pose.pose.orientation.w;
    tmp_q.x() = vio_msg->pose.pose.orientation.x;
    tmp_q.y() = vio_msg->pose.pose.orientation.y;
    tmp_q.z() = vio_msg->pose.pose.orientation.z;
    // Eigen::Matrix3d tmp_R = tmp_q.toRotationMatrix();
    // Eigen::Vector3d tmp_e = tmp_R.eulerAngles(2,1,0);
    // std::cout <<"in :R"<< tmp_R << std::endl;
    tmp_data.att = tmp_q;
    filter_core_ptr->input_vio(tmp_data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_estimator_node");
    ros::NodeHandle node("~");
    filter_core_ptr = new Filter<T>(node);
    ros::Subscriber sub_imu = node.subscribe("/imu_ns/imu/imu_filter", 20, imu_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_vio = node.subscribe("/vins_estimator/odometry", 10, vio_cb);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}