#include "ros/ros.h"
#include "Estimator.hpp"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"

#include "estimator_publisher.hpp"

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
    // if (tmp_q.w() < 0) {
    //     tmp_q.w() = -tmp_q.w();
    //     tmp_q.x() = -tmp_q.x();
    //     tmp_q.y() = -tmp_q.y();
    //     tmp_q.z() = -tmp_q.z();
    // }
    tmp_q.normalize();
    // Eigen::Matrix3d tmp_R = tmp_q.toRotationMatrix();
    // Eigen::Vector3d tmp_e = tmp_R.eulerAngles(2,1,0);
    // tmp_q = Eigen::AngleAxisd(tmp_e(2), Eigen::Vector3d::UnitZ())
    //                             * Eigen::AngleAxisd(tmp_e(1), Eigen::Vector3d::UnitY())
    //                             * Eigen::AngleAxisd(tmp_e(0), Eigen::Vector3d::UnitX());
    // tmp_q = tmp_R;
    // // std::cout <<"in :R"<< tmp_R << std::endl;
    tmp_data.att = tmp_q;
    filter_core_ptr->input_vio(tmp_data);
}

void lidar_cb(const sensor_msgs::FluidPressurePtr& lidar_msg) {
    lidar_data tmp_data;
    tmp_data.t = lidar_msg->header.stamp.toSec();
    tmp_data.data = lidar_msg->fluid_pressure;
    filter_core_ptr->input_lidar(tmp_data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_estimator_node");
    ros::NodeHandle node("~");
    EstimatorPublisher _estimator_publisher;
    _estimator_publisher.PublisherRegist(node);
    filter_core_ptr = new Filter<T>();
    filter_core_ptr->set_publish(_estimator_publisher);
    ros::Subscriber sub_imu = node.subscribe("/imu_ns/imu/imu_filter", 20, imu_cb, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_imu = node.subscribe("/imu_ns/imu/imu", 20, imu_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_vio = node.subscribe("/vins_estimator/odometry", 10, vio_cb);
    ros::Subscriber sub_lidar = node.subscribe("/lidar_ns/lidar_filtered", 10, lidar_cb);
    ros::spin();
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    // ros::waitForShutdown();
    // spinner.stop();
    return 0;
}