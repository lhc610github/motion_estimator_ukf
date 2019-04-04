#ifndef OUTPUT_PREDICTOR_HPP_
#define OUTPUT_PREDICTOR_HPP_
#include "Types.hpp"
#include "filter_type.hpp"
#include <iostream>
#include <deque>

#include "estimator_publisher.hpp"

// #include "ros/ros.h"
// #include "nav_msgs/Odometry.h"

struct outputSample {
    Eigen::Quaterniond q_norm;
    Eigen::Vector3d acc;
    Eigen::Vector3d vel;
    Eigen::Vector3d pos;
    double t;
};

struct imuSample {
    Eigen::Vector3d delta_ang;
    Eigen::Vector3d delta_vel;
    Eigen::Vector3d avg_acc; // a+g
    double dt;
    double t;
};

template<typename T>
class OutputPredictor {
    public:
        OutputPredictor(int _output_size, int _imu_count):
        OutputSampleSize(2 * _output_size),
        ImuCount(_imu_count) {
            has_regist_publisher = false;
            reset();
        }
        ~OutputPredictor() {

        }

        void reset() {
            OutputBuf.clear();
            ImuBuf.clear();
            DtBuf.clear();
            ImuSampleBuf.clear();
            pos_err_integ.setZero();
            vel_err_integ.setZero();
            delta_angle_corr.setZero();
            vel_gain = 0.011f/ 0.25f;
            pos_gain = 0.010f/ 0.25f;
            vel_gain_sp = vel_gain * vel_gain;
            pos_gain_sp = pos_gain * pos_gain;
        }

        void input_imu(const imu_data& _data, const int& _i, const double& _dt) {
            if (_dt < 0.0f || _dt > 0.5f) {
                std::cout << "predictor has problem! restart" << std::endl;
                reset();
            }
            imu_data _tmp = _data;
            int _case_tmp = ImuCount;
            if (_i == 1) {
                ImuBuf.clear();
                DtBuf.clear();
                ImuBuf.push_back(_tmp);
                DtBuf.push_back(_dt);
            } else if (_i == ImuCount) {
                ImuBuf.push_back(_tmp);
                DtBuf.push_back(_dt);
                if (ImuBuf.size() == ImuCount && DtBuf.size() == ImuCount) {
                    cal_ImuSample();
                }
            } else {
                if (_i == ImuBuf.size()+1) {
                    ImuBuf.push_back(_tmp);
                    DtBuf.push_back(_dt);
                }
            }
        }

        void cal_ImuSample() {
            // Eigen::Vector3d _acc_sum;
            Eigen::Vector3d _delta_vel;
            Eigen::Vector3d _delta_ang;
            // _acc_sum.setZero();
            _delta_vel.setZero();
            _delta_ang.setZero();
            double _delta_time = 0;
            for (int _i = 0; _i < ImuBuf.size(); _i ++) {
                double _dt = DtBuf[_i];
                _delta_vel += _dt * ImuBuf[_i].acc;
                _delta_ang += _dt * ImuBuf[_i].gyro;
                _delta_time += _dt;
            }
            imuSample tmp_imu_sample;
            tmp_imu_sample.delta_ang = _delta_ang;
            tmp_imu_sample.delta_vel = _delta_vel;
            tmp_imu_sample.dt = _delta_time;
            tmp_imu_sample.t = ImuBuf.rbegin()->t;
            if (_delta_time > 1e-7f)
                tmp_imu_sample.avg_acc = _delta_vel / _delta_time;
            else
                tmp_imu_sample.avg_acc = _delta_vel / 0.008f;
            ImuSampleBuf.push_back(tmp_imu_sample);
            while (ImuSampleBuf.size() > OutputSampleSize) {
                ImuSampleBuf.pop_front();
            }

            ImuBuf.clear();
            DtBuf.clear();
        }

        bool update_output(const typename predict_state<T>::X_& _x, double _t) {
            Eigen::Vector3d _acc_bias;
            Eigen::Vector3d _gyro_bias;
            _acc_bias << _x(10), _x(11), _x(12);
            _gyro_bias << _x(13), _x(14), _x(15);

            if (ImuSampleBuf.empty())
                return false;

            int _imu_sample_index = 0;
            for (int _i = 0; _i < ImuSampleBuf.size(); _i ++) {
                if (ImuSampleBuf[_i].t < _t)
                    continue;
                _imu_sample_index = _i;
                break;
            }

            int _output_update_index = 0;
            for (int _i = 0; _i < OutputBuf.size(); _i ++) {
                if (OutputBuf[_i].t < _t)
                    continue;
                _output_update_index = _i;
                break;
            }

            // std::cout << "[predictor]: imu_sample index: " << _imu_sample_index << " in " << ImuSampleBuf.size() << std::endl;
            // std::cout << "[predictor]: output_update index: " << _output_update_index << " in " << OutputBuf.size() << std::endl;

            if (ImuSampleBuf.size() != OutputSampleSize)
                return false;

            if (OutputBuf.empty()) {
                // init
                for (int _i = _imu_sample_index; _i < ImuSampleBuf.size(); _i ++) {
                    outputSample _old;
                    if (_i == _imu_sample_index) {
                        _old.pos << _x(0), _x(1), _x(2);
                        _old.vel << _x(3), _x(4), _x(5);
                        _old.acc.setZero();
                        Eigen::Quaterniond _q_tmp;
                        _q_tmp.w() = _x(6);
                        _q_tmp.x() = _x(7);
                        _q_tmp.y() = _x(8);
                        _q_tmp.z() = _x(9);
                        _old.q_norm = _q_tmp.normalized();
                        _old.t = _t;
                    } else {
                        // _old = *OutputBuf.rbegin();
                        _old = OutputBuf[OutputBuf.size()-1];
                    }
                    outputSample _new = predict_one_step(_old, ImuSampleBuf[_i], _acc_bias, _gyro_bias);
                    OutputBuf.push_back(_new);
                }
            } else {
                outputSample _old;
                // _old = *OutputBuf.rbegin();
                _old = OutputBuf[OutputBuf.size()-1];
                int _imu_update_index = _imu_sample_index;
                for (int _i = _imu_sample_index; _i < ImuSampleBuf.size(); _i ++) {
                    if (ImuSampleBuf[_i].t < _old.t)
                        continue;
                    _imu_update_index = _i;
                    break;
                }

                _imu_update_index ++;
                // std::cout << "[predictor]: imu_update index: " << _imu_update_index << " in " << ImuSampleBuf.size() << std::endl;

                for (int _i = _imu_update_index; _i < ImuSampleBuf.size(); _i ++) {
                    _old = *OutputBuf.rbegin();
                    outputSample _new = predict_one_step(_old, ImuSampleBuf[_i], _acc_bias, _gyro_bias);
                    OutputBuf.push_back(_new);
                }

                // update correct
                Eigen::Vector3d _pos;
                _pos << _x(0), _x(1), _x(2);
                Eigen::Vector3d _vel;
                _vel << _x(3), _x(4), _x(5);
                Eigen::Quaterniond _q_norm;
                _q_norm.w() = _x(6);
                _q_norm.x() = _x(7);
                _q_norm.y() = _x(8);
                _q_norm.z() = _x(9);
                _q_norm.normalized();

                Eigen::Vector3d _pos_err = _pos - OutputBuf[_output_update_index].pos;
                Eigen::Vector3d _vel_err = _vel - OutputBuf[_output_update_index].vel;

                vel_err_integ += _vel_err;
                pos_err_integ += _pos_err;
                // vel_err_integ.setZero();
                // pos_err_integ.setZero();

                // std::cout <<  "e_p: " << _pos_err.transpose() << std::endl;
                // std::cout <<  "e_v: " << _vel_err.transpose() << std::endl;

                Eigen::Vector3d _pos_correct = pos_gain * _pos_err + pos_gain_sp * 0.1f * pos_err_integ;
                Eigen::Vector3d _vel_correct = vel_gain * _vel_err + vel_gain_sp * 0.1f * vel_err_integ;

                // std::cout <<  "cor_p: " << _pos_correct.transpose() << std::endl;
                // std::cout <<  "cor_v: " << _vel_correct.transpose() << std::endl;
                // _pos_correct.setZero();
                // _vel_correct.setZero();

                Eigen::Quaterniond quat_inv = _q_norm.inverse();
                Eigen::Quaterniond _q_err = quat_inv * OutputBuf[_output_update_index].q_norm;
                _q_err.normalized();
                double _scalar;
                if (_q_err.w() >= 0.0f) {
                    _scalar = -2.0f;
                } else {
                    _scalar = 2.0f;
                }
                Eigen::Vector3d _delta_ang_err;
                _delta_ang_err << _scalar*_q_err.x(), _scalar*_q_err.y(), _scalar*_q_err.z();
                double _att_gain = 1.0f / 10.0f; 
                delta_angle_corr = _att_gain * _delta_ang_err;

                for (int _i = 0; _i < OutputBuf.size(); _i++) {
                    OutputBuf[_i].pos += _pos_correct;
                    OutputBuf[_i].vel += _vel_correct;
                }
            }

            while (OutputBuf.size() > OutputSampleSize) {
                OutputBuf.pop_front();
            }
            

            if (has_regist_publisher) {
                Eigen::Vector3d _tmp_pos, _tmp_vel, _tmp_acc;
                Eigen::Quaterniond _tmp_q;
                _tmp_pos = OutputBuf.rbegin()->pos;
                _tmp_vel = OutputBuf.rbegin()->vel;
                _tmp_q = OutputBuf.rbegin()->q_norm;
                _tmp_acc = OutputBuf.rbegin()->acc;
                estimator_publisher_ptr->UkfPredictPub(_tmp_pos, _tmp_vel, _tmp_acc, _tmp_q, OutputBuf.rbegin()->t);
            }
            return true;
        }

        outputSample predict_one_step(const outputSample& _output_old, 
        const imuSample& _imuSample, const Eigen::Vector3d& _acc_bias, const Eigen::Vector3d& _gyro_bias) {
            outputSample _output_new;
            _output_new.t = _imuSample.t;
            Eigen::Vector3d _delta_ang;
            _delta_ang = _imuSample.delta_ang - _imuSample.dt * _gyro_bias + delta_angle_corr;
            // _delta_ang = _imuSample.delta_ang + delta_angle_corr;
            Eigen::Quaterniond _dq = Eigen::AngleAxisd(_delta_ang(2), Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(_delta_ang(1), Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(_delta_ang(0), Eigen::Vector3d::UnitX());
            _output_new.q_norm = (_output_old.q_norm * _dq).normalized();

            Eigen::Matrix3d _R = _output_new.q_norm.toRotationMatrix();

            Eigen::Vector3d _delta_vel;
            _delta_vel = _imuSample.delta_vel - _imuSample.dt * _acc_bias;
            Eigen::Vector3d _delta_earth_vel = _R * _delta_vel;
            Eigen::Vector3d _earth_acc = _R * _imuSample.avg_acc;
            _delta_earth_vel(2) -= 9.80665f*_imuSample.dt;
            // std::cout << _delta_earth_vel.transpose() << std::endl;
            _earth_acc(2) -= 9.80665f;
            _output_new.vel = _output_old.vel + _delta_earth_vel;
            Eigen::Vector3d _delta_earth_pos = (_output_new.vel + _output_old.vel) * (_imuSample.dt * 0.5f);
            _output_new.pos = _output_old.pos + _delta_earth_pos;
            _output_new.acc = _earth_acc;
            // printf("dt: %.8f\n", _imuSample.dt);
            return _output_new;
        }

        void set_publish(EstimatorPublisher& _tmp) {
            has_regist_publisher = true;
            estimator_publisher_ptr = &_tmp;
        }

    private:
        const int OutputSampleSize; // num of imu sample in delay (11)
        const int ImuCount; // num of imu sample in output period (7)
        std::deque<outputSample> OutputBuf;
        std::deque<imuSample> ImuSampleBuf;
        std::deque<imu_data> ImuBuf;
        std::deque<double> DtBuf;

        Eigen::Vector3d pos_err_integ;
        Eigen::Vector3d vel_err_integ;

        Eigen::Vector3d delta_angle_corr;

        double vel_gain;// = 0.008f/ 0.25f;
        double pos_gain;// = 0.008f/ 0.25f;
        double vel_gain_sp;
        double pos_gain_sp;

        bool has_regist_publisher;
        EstimatorPublisher* estimator_publisher_ptr;
};

#endif