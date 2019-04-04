#ifndef FILTER_HPP_
#define FILTER_HPP_
#include "Types.hpp"
#include "imu_predict.hpp"
#include "vio_update.hpp"

struct imu_data {
    double t;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct vio_data {
    double t;
    Eigen::Vector3d pos; /* x y z*/
    Eigen::Vector3d vel;
    Eigen::Quaterniond att;
};

template<typename T>
class predict_state {
    public:
        predict_state() {}
        ~predict_state() {}
        double t;
        typedef typename Kalman::Imu_predict<T>::MotionStates X_;
        typedef typename Kalman::Imu_predict<T>::StateCovariance P_;
        X_ x;
        P_ P;
};


#endif