#include "imu_predict.hpp"
#include "vio_update.hpp"
#include "ros/ros.h"

typedef double T;
typedef typename MotionModel<T>::Im U;
typedef typename MotionModel<T>::Imn V;
typedef typename MotionModel<T>::Ms X;
typedef typename VioMeasurementModel<T>::Vm Z;
typedef typename VioMeasurementModel<T>::Vmn N;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ukf_node");
    ros::NodeHandle node("~");
    Kalman::Imu_predict<T> predict_core;
    Kalman::Vio_update<T> vio_update_core;
    U test_u;
    test_u.ax() = 1.0f;
    test_u.ay() = -1.0f;
    test_u.az() = 9.81f;
    test_u.wx() = 0.1f;
    test_u.wy() = -0.1f;
    test_u.wz() = 0.0f;
    V test_v;
    test_v.setZero();
    N test_n;
    test_n.setZero();
    Z test_z;
    test_z.setZero();
    test_z.the() = 0.5f;
    X test_x;
    test_x.setZero();
    test_x.the() = 0.5f;

    predict_core.init(test_x);

    for (int i = 0; i < 20; i++) {
        typename Kalman::Imu_predict<T>::MotionStates _x;
        typename Kalman::Imu_predict<T>::StateCovariance _P;
        if (!predict_core.predict(test_u, test_v, T(0.01), _P, _x)) {
            return 0;
        }
        std::cout << _x.transpose() << std::endl;

        vio_update_core.update(test_z, _x, _P, test_n);
        std::cout << _x.transpose() << std::endl;
        predict_core.update_P_x(_P, _x);
    }


    return 0;
}