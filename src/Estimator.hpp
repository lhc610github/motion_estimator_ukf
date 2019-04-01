#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include "imu_predict.hpp"
#include "vio_update.hpp"
#include <deque>
#include <mutex>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

struct imu_data {
    double t;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct vio_data {
    double t;
    Eigen::Vector3d pos; /* x y z*/
    Eigen::Vector3d vel;
    Eigen::Vector3d att;
};

template<typename T>
class predict_state {
    public:
        predict_state() {}
        ~predict_state() {}
        double t;
        typename Kalman::Imu_predict<T>::MotionStates x;
        typename Kalman::Imu_predict<T>::StateCovariance P;
};

template<typename T>
class Filter {
    public:
        Filter(ros::NodeHandle& n) {
            has_init = false;
            has_first_predict = false;
            ImuBuf_Size = 500;
            Delay_Size = 50;
            PredictBuf_Size = 60;
            nav_pub = n.advertise<nav_msgs::Odometry>("/ukf/odometry", 100);
            bias_pub = n.advertise<nav_msgs::Odometry>("/ukf/bias", 100);
        }
        ~Filter() {}

        void init_filter(const vio_data& tmp) {
            X _init_x;
            _init_x.setZero();
            _init_x.px() = tmp.pos(0);
            _init_x.py() = tmp.pos(1);
            _init_x.pz() = tmp.pos(2);
            _init_x.vx() = tmp.vel(0);
            _init_x.vy() = tmp.vel(1);
            _init_x.vz() = tmp.vel(2);
            _init_x.phi() = tmp.att(0);
            _init_x.the() = tmp.att(1);
            _init_x.psi() = tmp.att(2);
            filter_mutex.lock();
            predict_core.init(_init_x);
            predict_state<T> tmp_pre;
            tmp_pre.t = tmp.t;
            tmp_pre.x = _init_x;
            tmp_pre.P = predict_core.get_P();
            PredictBuf.clear();
            PredictBuf.push_back(tmp_pre);
            filter_mutex.unlock();
            std::cout << "[filter]: init x("<< tmp_pre.t << "): " << _init_x.transpose() << std::endl;
            has_init = true;
        }

        void restart_filter() {
            has_init = false;
            has_first_predict = false;
            ImuBuf.clear();
            PredictBuf.clear();
            predict_core.reset();
            vio_update_core.reset();
            // std::cout << "end restart" << std::endl;
        }

        void input_imu(imu_data tmp) {
            ImuBuf.push_back(tmp);
            if (!has_init) {
                while ( ImuBuf.size() > ImuBuf_Size ) {
                    ImuBuf.pop_front();
                }
                return;
            }
            
            /* predict */
            if (ImuBuf.size() >= Delay_Size) {
                U _u;
                int _i = ImuBuf.size() - Delay_Size;
                _u.ax() = ImuBuf[_i].acc(0);
                _u.ay() = ImuBuf[_i].acc(1);
                _u.az() = ImuBuf[_i].acc(2);
                _u.wx() = ImuBuf[_i].gyro(0);
                _u.wy() = ImuBuf[_i].gyro(1);
                _u.wz() = ImuBuf[_i].gyro(2);
                V _v;
                _v.setZero();
                filter_mutex.lock();
                typename Kalman::Imu_predict<T>::MotionStates _x;
                typename Kalman::Imu_predict<T>::StateCovariance _P;
                double _dt = ImuBuf[_i].t - PredictBuf.rbegin()->t;
                if (_dt < 0.0) {
                    std::cout << "[filter]: predict dt < 0 ? : " << _dt << std::endl;
                    std::cout << "[filter]: imu_t: "<< ImuBuf[_i].t << " vio_t: " << PredictBuf.rbegin()->t<< std::endl;
                    _dt = 0.001f;
                } else if (_dt > 1.0f) {
                    std::cout << "[filter]: predict dt to large ? : "<< _dt << std::endl;
                    std::cout << "[filter]: imu_t: "<< ImuBuf[_i].t << " vio_t: " << PredictBuf.rbegin()->t<< std::endl;
                    restart_filter();
                    filter_mutex.unlock();
                    return;
                }

                if (_dt < 0.005f) { // 200Hz
                    filter_mutex.unlock();
                    return;
                }

                if (!predict_core.predict(_u, _v, T(_dt), _P, _x)) {
                    std::cout << "[filter]: predict wrong!" << std::endl;
                    filter_mutex.unlock();
                    return;
                } else {
                    has_first_predict = true;
                    nav_msgs::Odometry pub_msg, pub_bias;
                    pub_msg.header.frame_id = "world";
                    pub_msg.header.stamp = ros::Time(ImuBuf[_i].t);
                    pub_bias = pub_msg;
                    pub_msg.pose.pose.position.x = _x(0);
                    pub_msg.pose.pose.position.y = _x(1);
                    pub_msg.pose.pose.position.z = _x(2);
                    Eigen::Quaterniond _x_q = Eigen::AngleAxisd(double(_x(8)), Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(double(_x(7)), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(double(_x(6)), Eigen::Vector3d::UnitX());
                    pub_msg.pose.pose.orientation.w = _x_q.w();
                    pub_msg.pose.pose.orientation.x = _x_q.x();
                    pub_msg.pose.pose.orientation.y = _x_q.y();
                    pub_msg.pose.pose.orientation.z = _x_q.z();
                    pub_msg.twist.twist.linear.x = _x(3);
                    pub_msg.twist.twist.linear.y = _x(4);
                    pub_msg.twist.twist.linear.z = _x(5);
                    nav_pub.publish(pub_msg);
                    pub_bias.pose.pose.position.x = _x(9);
                    pub_bias.pose.pose.position.y = _x(10);
                    pub_bias.pose.pose.position.z = _x(11);
                    bias_pub.publish(pub_bias);
                }
                predict_state<T> _tmp_predict_state;
                _tmp_predict_state.t = ImuBuf[_i].t;
                _tmp_predict_state.P = _P;
                _tmp_predict_state.x = _x;
                // std::cout << "[filter]: " << _x.transpose() << std::endl;
                PredictBuf.push_back(_tmp_predict_state);
                while (PredictBuf.size() > PredictBuf_Size) {
                    PredictBuf.pop_front();
                }
                filter_mutex.unlock();
            }
            while ( ImuBuf.size() > ImuBuf_Size ) {
                ImuBuf.pop_front();
            }
        }

        void input_vio(vio_data tmp) {
            if (!has_first_predict) {
                init_filter(tmp);
            } else {
                filter_mutex.lock();
                int _PBUF_SIZE = PredictBuf.size();
                if (_PBUF_SIZE > 1) {
                    for (int _i = 0; _i < _PBUF_SIZE; _i++) {
                        double _dt = tmp.t - PredictBuf[_PBUF_SIZE - 1 - _i].t;
                        if (_dt > 0.0f ) {
                            Z _z;
                            _z.px() = tmp.pos(0);
                            _z.py() = tmp.pos(1);
                            _z.pz() = tmp.pos(2);
                            _z.vx() = tmp.vel(0);
                            _z.vy() = tmp.vel(1);
                            _z.vz() = tmp.vel(2);
                            _z.phi() = tmp.att(0);
                            _z.the() = tmp.att(1);
                            _z.psi() = tmp.att(2);
                            N _n;
                            _n.setZero();
                            X _x = PredictBuf[_PBUF_SIZE-1-_i].x;
                            P _P = PredictBuf[_PBUF_SIZE-1-_i].P;
                            vio_update_core.update(_z, _x, _P, _n);
                            predict_core.update_P_x(_P, _x);
                            predict_state<T> _p_s;
                            _p_s.t = PredictBuf[_PBUF_SIZE - 1 - _i].t;
                            _p_s.P = _P;
                            _p_s.x = _x;
                            PredictBuf.clear();
                            PredictBuf.push_back(_p_s);

                            // std::cout << "[filter]: delay index: "<< _i << " total predict num: " << _PBUF_SIZE << std::endl;
                            break;
                            // TODO: check delay
                        }
                    }
                } else {
                    std::cout << "[filter]: has no predict between update" << std::endl;
                }
                filter_mutex.unlock();
            }
        }

    private:
        typedef typename MotionModel<T>::Im U;
        typedef typename MotionModel<T>::Imn V;
        typedef typename MotionModel<T>::Ms X;
        typedef typename VioMeasurementModel<T>::Vm Z;
        typedef typename VioMeasurementModel<T>::Vmn N;
        typedef typename Kalman::Imu_predict<T>::StateCovariance P;
        Kalman::Imu_predict<T> predict_core;
        Kalman::Vio_update<T> vio_update_core;
        bool has_init;
        bool has_first_predict;

        std::deque<imu_data> ImuBuf;
        int ImuBuf_Size;
        int Delay_Size;
        std::mutex filter_mutex;

        std::deque<predict_state<T>> PredictBuf;
        int PredictBuf_Size;

        ros::Publisher nav_pub;
        ros::Publisher bias_pub;

};

#endif