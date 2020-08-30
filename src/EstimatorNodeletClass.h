#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include "Estimator.hpp"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Range.h"

#include "estimator_publisher.hpp"

typedef double T;

namespace estimator_namespace {
class EstimatorNodeletClass: public nodelet::Nodelet {
public:
    EstimatorNodeletClass();
    ~EstimatorNodeletClass();
    virtual void onInit();

    void imu_cb(const sensor_msgs::ImuConstPtr& imu_msg);
    void vio_cb(const nav_msgs::OdometryPtr& vio_msg);
    void lidar_cb(const sensor_msgs::FluidPressurePtr& lidar_msg);
    void lidar2_cb(const sensor_msgs::RangePtr& lidar_msg);

private:
    Filter<T>* filter_core_ptr;
    vio_data* old_vio_data_ptr;
    Eigen::Vector3d vio_draft;
    EstimatorPublisher _estimator_publisher;
    /* subscriber list */
    ros::Subscriber sub_imu, sub_vio, sub_lidar;
};
}