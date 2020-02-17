#include "ros/ros.h"
#include "Estimator.hpp"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Range.h"

#include "estimator_publisher.hpp"

typedef double T;
Filter<T>* filter_core_ptr;
vio_data* old_vio_data_ptr;
Eigen::Vector3d vio_draft;

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

    Eigen::Quaterniond tmp_q;
    tmp_q.w() = vio_msg->pose.pose.orientation.w;
    tmp_q.x() = vio_msg->pose.pose.orientation.x;
    tmp_q.y() = vio_msg->pose.pose.orientation.y;
    tmp_q.z() = vio_msg->pose.pose.orientation.z;
	Eigen::Matrix3d tmp_R = tmp_q.toRotationMatrix();
	Eigen::Quaterniond tmp_rotate_y = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
								* Eigen::AngleAxisd(0.785398, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
	Eigen::Matrix3d tmp_R2 = tmp_R * tmp_rotate_y.toRotationMatrix().transpose();
	tmp_q = tmp_R2;
    tmp_q.normalize();

    Eigen::Vector3d tf_vec_ = {0.1, 0, -0.04}; // 0.08 0 -0.04
	Eigen::Vector3d odom_drift = tmp_R2*tf_vec_;
	Eigen::Matrix3d omega;
	omega << 0, -vio_msg->twist.twist.angular.z, vio_msg->twist.twist.angular.y,
			vio_msg->twist.twist.angular.z, 0, -vio_msg->twist.twist.angular.x,
			-vio_msg->twist.twist.angular.y, vio_msg->twist.twist.angular.x, 0;
	Eigen::Vector3d vel_drift = tmp_R2*omega*tf_vec_;

    tmp_data.pos(0) = vio_msg->pose.pose.position.x - odom_drift(0);
    tmp_data.pos(1) = vio_msg->pose.pose.position.y - odom_drift(1);
    tmp_data.pos(2) = vio_msg->pose.pose.position.z - odom_drift(2);
    tmp_data.vel(0) = vio_msg->twist.twist.linear.x;// - vel_drift(0);
    tmp_data.vel(1) = vio_msg->twist.twist.linear.y;// - vel_drift(1);
    tmp_data.vel(2) = vio_msg->twist.twist.linear.z;// - vel_drift(2);


    // Eigen::Matrix3d tmp_R = tmp_q.toRotationMatrix();
    // Eigen::Vector3d tmp_e = tmp_R.eulerAngles(2,1,0);
    // tmp_q = Eigen::AngleAxisd(tmp_e(2), Eigen::Vector3d::UnitZ())
    //                             * Eigen::AngleAxisd(tmp_e(1), Eigen::Vector3d::UnitY())
    //                             * Eigen::AngleAxisd(tmp_e(0), Eigen::Vector3d::UnitX());
    // tmp_q = tmp_R;
    // // std::cout <<"in :R"<< tmp_R << std::endl;
    tmp_data.att = tmp_q;
    tmp_data.pos -= vio_draft;
    if (old_vio_data_ptr == nullptr) {
        old_vio_data_ptr = new vio_data;
        *old_vio_data_ptr = tmp_data;
    } else {
        Eigen::Vector3d dp;
        double dt;
        dp(0) = tmp_data.pos(0) - old_vio_data_ptr->pos(0);
        dp(1) = tmp_data.pos(1) - old_vio_data_ptr->pos(1);
        dp(2) = tmp_data.pos(2) - old_vio_data_ptr->pos(2);
        dt = tmp_data.t - old_vio_data_ptr->t;
        dt = dt > 0? dt: 0.04f; // 20Hz
        double dp_norm = dp.norm();
        if (dp_norm/dt > 2.5f) {
            vio_draft += (dp_norm/dt - 2.5f) *dt * dp.normalized();
            tmp_data.pos -= (dp_norm/dt - 2.5f) *dt * dp.normalized();
        }
        *old_vio_data_ptr = tmp_data;
    }
    ROS_INFO_THROTTLE(1.0,
                      "\033[1;33m vio call back draft: [%.3f, %.3f, %.3f]; \033[0m",
                      vio_draft(0), vio_draft(1), vio_draft(2) );
    filter_core_ptr->input_vio(tmp_data);
}

void lidar_cb(const sensor_msgs::FluidPressurePtr& lidar_msg) {
    lidar_data tmp_data;
    tmp_data.t = lidar_msg->header.stamp.toSec();
    tmp_data.data = lidar_msg->fluid_pressure;
    filter_core_ptr->input_lidar(tmp_data);
}

void lidar2_cb(const sensor_msgs::RangePtr& lidar_msg) {
	lidar_data tmp_data;
	tmp_data.t = lidar_msg->header.stamp.toSec();
	tmp_data.data = lidar_msg->range;
    filter_core_ptr->input_lidar(tmp_data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_estimator_node");
    ros::NodeHandle node("~");
    vio_draft.setZero();
    EstimatorPublisher _estimator_publisher;
    _estimator_publisher.PublisherRegist(node);
    filter_core_ptr = new Filter<T>();
    filter_core_ptr->set_publish(_estimator_publisher);
    //ros::Subscriber sub_imu = node.subscribe("/imu_ns/imu/imu_filter", 20, imu_cb, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_imu = node.subscribe("/imu_ns/imu/imu", 20, imu_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_imu = node.subscribe("/mavros/imu/data_raw", 20, imu_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_vio = node.subscribe("/rs_lf/odometry", 10, vio_cb);
    //ros::Subscriber sub_lidar = node.subscribe("/lidar_ns/lidar_filtered", 10, lidar_cb);
    ros::Subscriber sub_lidar = node.subscribe("/mavros/distance_sensor/hrlv_ez4_pub", 10, lidar2_cb);
	ros::spin();
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    // ros::waitForShutdown();
    // spinner.stop();
    return 0;
}
