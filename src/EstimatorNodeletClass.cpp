#include <pluginlib/class_list_macros.h>
#include "EstimatorNodeletClass.h"

PLUGINLIB_EXPORT_CLASS(estimator_namespace::EstimatorNodeletClass, nodelet::Nodelet)

namespace estimator_namespace {
    EstimatorNodeletClass::EstimatorNodeletClass() {
    }
    EstimatorNodeletClass::~EstimatorNodeletClass() {
        delete filter_core_ptr;
        delete old_vio_data_ptr;
    }
    void EstimatorNodeletClass::onInit() {
        std::string initial_str = "Initializing estimator nodelet...";
        NODELET_DEBUG_STREAM(initial_str);
        initial_str += this->getName();
        ROS_INFO_STREAM(initial_str);
        ros::NodeHandle nodelet_nh = this->getNodeHandle();
        _estimator_publisher.PublisherRegist(nodelet_nh);
        filter_core_ptr = new Filter<T>();
        filter_core_ptr->set_publish(_estimator_publisher);
        sub_imu = nodelet_nh.subscribe("/mavros/imu/data_raw", 20, &EstimatorNodeletClass::imu_cb, this,
                ros::TransportHints().tcpNoDelay());
        sub_vio = nodelet_nh.subscribe("/rs_lf/odometry", 10, &EstimatorNodeletClass::vio_cb, this);
        //sub_lidar = nodelet_nh.subscribe("/lidar_ns/lidar_filtered", 10, &EstimatorNodeletClass::lidar_cb, this);
        sub_lidar = nodelet_nh.subscribe("/mavros/distance_sensor/hrlv_ez4_pub", 10, &EstimatorNodeletClass::lidar2_cb, this);
    }

    void EstimatorNodeletClass::imu_cb(const sensor_msgs::ImuConstPtr& imu_msg) {
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

    void EstimatorNodeletClass::vio_cb(const nav_msgs::OdometryPtr &vio_msg) {
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

    void EstimatorNodeletClass::lidar_cb(const sensor_msgs::FluidPressurePtr &lidar_msg) {
        lidar_data tmp_data;
        tmp_data.t = lidar_msg->header.stamp.toSec();
        tmp_data.data = lidar_msg->fluid_pressure;
        filter_core_ptr->input_lidar(tmp_data);
    }

    void EstimatorNodeletClass::lidar2_cb(const sensor_msgs::RangePtr &lidar_msg) {
        lidar_data tmp_data;
        tmp_data.t = lidar_msg->header.stamp.toSec();
        tmp_data.data = lidar_msg->range;
        filter_core_ptr->input_lidar(tmp_data);
    }
}
