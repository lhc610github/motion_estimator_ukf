#ifndef ESTIMATOR_PUBLISHER_HPP_
#define ESTIMATOR_PUBLISHER_HPP_
#include "Types.hpp"

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/FluidPressure.h>

class EstimatorPublisher {
    public:
        EstimatorPublisher() {
            has_register = false;
        }
        ~EstimatorPublisher() {

        }
        
        void PublisherRegist(ros::NodeHandle& _nh) {
            pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("/vio_data_rigid1/pos", 100);
            vel_pub = _nh.advertise<geometry_msgs::Vector3Stamped>("/vio_data_rigid1/vel", 100);
            acc_pub = _nh.advertise<geometry_msgs::Vector3Stamped>("/vio_data_rigid1/acc", 100);
            att_pub = _nh.advertise<geometry_msgs::PoseStamped>("/vio_data_rigid1/att", 100);
            ukf_pub = _nh.advertise<nav_msgs::Odometry>("/ukf/odometry", 100);
            ukf_predict_pub = _nh.advertise<nav_msgs::Odometry>("/ukf_predict/odometry", 100);
            lidar_pub = _nh.advertise<sensor_msgs::FluidPressure>("/lidar_data", 100);
            has_register = true;
        }

        void PosPub(const Eigen::Vector3d& _pos, const Eigen::Quaterniond& _q_att, double _t) {
            if (!has_register)
                return;
            geometry_msgs::PoseStamped _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(_t);
            _tmp_msg.pose.position.x = _pos(0);
            _tmp_msg.pose.position.y = -_pos(1);
            _tmp_msg.pose.position.z = -_pos(2);
            _tmp_msg.pose.orientation.w = _q_att.w();
            _tmp_msg.pose.orientation.x = _q_att.x();
            _tmp_msg.pose.orientation.y = -_q_att.y();
            _tmp_msg.pose.orientation.z = -_q_att.z();
            pos_pub.publish(_tmp_msg);
        }

        void VelPub(const Eigen::Vector3d& _vel, double _t) {
            if (!has_register)
                return;
            geometry_msgs::Vector3Stamped _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(_t);
            _tmp_msg.vector.x = _vel(0);
            _tmp_msg.vector.y = -_vel(1);
            _tmp_msg.vector.z = -_vel(2);
            vel_pub.publish(_tmp_msg);
        }

        void AccPub(const Eigen::Vector3d& _acc, double _t) {
            if (!has_register)
                return;
            geometry_msgs::Vector3Stamped _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(_t);
            _tmp_msg.vector.x = _acc(0);
            _tmp_msg.vector.y = -_acc(1);
            _tmp_msg.vector.z = -_acc(2);
            acc_pub.publish(_tmp_msg);
        }

        void AttPub(const Eigen::Quaterniond& _q_att, double _t) {
            if (!has_register)
                return;
            geometry_msgs::PoseStamped _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(_t);
            _tmp_msg.pose.orientation.w = _q_att.w();
            _tmp_msg.pose.orientation.x = _q_att.x();
            _tmp_msg.pose.orientation.y = -_q_att.y();
            _tmp_msg.pose.orientation.z = -_q_att.z();
            att_pub.publish(_tmp_msg);
        }

        void LidarPub(const double& _lidar_data, const double& _distant, const double& t) {
            if (!has_register)
                return;
            sensor_msgs::FluidPressure _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(t);
            _tmp_msg.fluid_pressure = _distant;
            _tmp_msg.variance = _lidar_data;
            lidar_pub.publish(_tmp_msg);
        }

        void UkfPub(const Eigen::Vector3d& _pos, const Eigen::Vector3d& _vel, const Eigen::Vector3d& _acc,
        const Eigen::Quaterniond& _q, double _t) {
            if (!has_register)
                return;
            nav_msgs::Odometry _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(_t);
            _tmp_msg.pose.pose.position.x = _pos(0);
            _tmp_msg.pose.pose.position.y = _pos(1);
            _tmp_msg.pose.pose.position.z = _pos(2);
            _tmp_msg.pose.pose.orientation.w = _q.w();
            _tmp_msg.pose.pose.orientation.x = _q.x();
            _tmp_msg.pose.pose.orientation.y = _q.y();
            _tmp_msg.pose.pose.orientation.z = _q.z();
            _tmp_msg.twist.twist.linear.x = _vel(0);
            _tmp_msg.twist.twist.linear.y = _vel(1);
            _tmp_msg.twist.twist.linear.z = _vel(2);
            ukf_pub.publish(_tmp_msg);
        }

        void UkfPredictPub(const Eigen::Vector3d& _pos, const Eigen::Vector3d& _vel, const Eigen::Vector3d& _acc,
        const Eigen::Quaterniond& _q, double _t) {
            if (!has_register)
                return;
            nav_msgs::Odometry _tmp_msg;
            _tmp_msg.header.frame_id = "world";
            _tmp_msg.header.stamp = ros::Time(_t);
            _tmp_msg.pose.pose.position.x = _pos(0);
            _tmp_msg.pose.pose.position.y = _pos(1);
            _tmp_msg.pose.pose.position.z = _pos(2);
            _tmp_msg.pose.pose.orientation.w = _q.w();
            _tmp_msg.pose.pose.orientation.x = _q.x();
            _tmp_msg.pose.pose.orientation.y = _q.y();
            _tmp_msg.pose.pose.orientation.z = _q.z();
            _tmp_msg.twist.twist.linear.x = _vel(0);
            _tmp_msg.twist.twist.linear.y = _vel(1);
            _tmp_msg.twist.twist.linear.z = _vel(2);
            ukf_predict_pub.publish(_tmp_msg);
            PosPub(_pos, _q, _t);
            VelPub(_vel, _t);
            AccPub(_acc, _t);
            AttPub(_q, _t);
            ROS_INFO_THROTTLE(1.0,
             "\033[1;36m pos: [%.3f, %.3f, %.3f];\n      vel: [%.2f, %.2f, %.2f];\n      att: [%.2f, %.2f, %.2f, %.2f]; \033[0m",
                            _pos(0), _pos(1), _pos(2),
                            _vel(0), _vel(1), _vel(2),
                              _q.w(), _q.x(), _q.y(), _q.z()  );

        }

    private:
        ros::Publisher ukf_pub;
        ros::Publisher ukf_predict_pub;
        ros::Publisher lidar_pub;
        ros::Publisher pos_pub;
        ros::Publisher vel_pub;
        ros::Publisher acc_pub;
        ros::Publisher att_pub;
        bool has_register;
};

#endif