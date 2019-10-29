#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include "imu_predict.hpp"
#include "vio_update.hpp"
#include "lidar_update.hpp"
#include "filter_type.hpp"
#include "output_predictor.hpp"
#include <deque>
#include <mutex>


template<typename T>
class Filter {
    public:
        Filter() {
            has_init = false;
            has_first_predict = false;
            ImuBuf_Size = 500;
            has_regist_publisher = false;
            // Delay_Size = 80;
            int Delay_output_period = 13;
            PredictBuf_Size = 60;
            OutputPeriodCount = 4;
            Delay_Size = Delay_output_period * OutputPeriodCount;
            PredictIndex = 0;
            outputpredictor_filter_ptr = new OutputPredictor<T>(Delay_output_period, OutputPeriodCount);
            need_to_update_vio = 0;
            need_to_update_lidar = 0;
            terrain_vpos = 0;
            last_lidar_measurement_valid_time = 0;
            bias_vio_z = 0;
            err_vio_lidar_z_integ = 0;
        }
        ~Filter() {
            delete outputpredictor_filter_ptr;
        }

        void init_filter(const vio_data& tmp) {

            if (ImuBuf.size() < 200) {
                return;
            }

            Eigen::Vector3d _tmp_acc;
            _tmp_acc.setZero();
            Eigen::Vector3d _tmp_gyro;
            _tmp_gyro.setZero();

            for (int _i = 0; _i < ImuBuf.size(); _i++) {
                _tmp_acc += ImuBuf[_i].acc;
                _tmp_gyro += ImuBuf[_i].gyro;
            }

            _tmp_acc /= ImuBuf.size();
            _tmp_gyro /= ImuBuf.size();

            Eigen::Matrix3d _tmp_R = tmp.att.normalized().toRotationMatrix();
            Eigen::Vector3d _tmp_g{0,0,9.8655};

            Eigen::Vector3d _tmp_bias_acc = _tmp_acc - _tmp_R.transpose() * _tmp_g;
            Eigen::Vector3d _tmp_bias_gyro = _tmp_gyro;

            X _init_x;
            _init_x.setZero();
            _init_x.px() = T(tmp.pos(0));
            _init_x.py() = T(tmp.pos(1));
            _init_x.pz() = T(tmp.pos(2));
            _init_x.vx() = T(tmp.vel(0));
            _init_x.vy() = T(tmp.vel(1));
            _init_x.vz() = T(tmp.vel(2));
            _init_x.qw() = T(tmp.att.w());
            _init_x.qx() = T(tmp.att.x());
            _init_x.qy() = T(tmp.att.y());
            _init_x.qz() = T(tmp.att.z());
            _init_x.bax() = T(_tmp_bias_acc(0));
            _init_x.bay() = T(_tmp_bias_acc(1));
            _init_x.baz() = T(_tmp_bias_acc(2));
            _init_x.bwx() = T(_tmp_bias_gyro(0));
            _init_x.bwy() = T(_tmp_bias_gyro(1));
            _init_x.bwz() = T(_tmp_bias_gyro(2));
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
            PredictIndex = 0;
            outputpredictor_filter_ptr->reset();
            terrain_vpos = 0;
            last_lidar_measurement_valid_time = 0;
        }

        void restart_filter() {
            has_init = false;
            has_first_predict = false;
            ImuBuf.clear();
            PredictBuf.clear();
            predict_core.reset();
            vio_update_core.reset();
            lidar_update_core.reset();
            outputpredictor_filter_ptr->reset();
            PredictIndex = 0;

            VioBuf.clear();
            LidarBuf.clear();
            need_to_update_vio = 0;
            need_to_update_lidar = 0;

            bias_vio_z = 0;
            err_vio_lidar_z_integ = 0;
            std::cout << "[filter]: restart" << std::endl;
        }

        void input_imu(imu_data tmp) {
            ImuBuf.push_back(tmp);
            if (! has_init) {
                while (ImuBuf.size()> ImuBuf_Size) {
                    ImuBuf.pop_front();
                }
                return;
            }
            /* predict */
            if (ImuBuf.size() >= (Delay_Size + 10)) {

                int _i = ImuBuf.size() - Delay_Size;
                filter_mutex.lock();
                typename Kalman::Imu_predict<T>::MotionStates _x;
                typename Kalman::Imu_predict<T>::StateCovariance _P;
                double _dt = ImuBuf[_i].t - PredictBuf.rbegin()->t;
                if (_dt < -0.5) {
                    std::cout << "[filter]: predict dt < 0 ? : " << _dt << std::endl;
                    // std::cout << "[filter]: imu_t: "<< ImuBuf[_i].t << " vio_t: " << PredictBuf.rbegin()->t<< std::endl;
                    printf("[filter]: imu_t: %.4f,  vio_t: %.4f\n", ImuBuf[_i].t, PredictBuf.rbegin()->t);
                    // _dt = 0.001f;
                    restart_filter();
                    filter_mutex.unlock();
                    return;
                } else if (_dt > 1.7f) {
                    std::cout << "[filter]: predict dt to large ? : "<< _dt << std::endl;
                    // std::cout << "[filter]: imu_t: "<< ImuBuf[_i].t << " vio_t: " << PredictBuf.rbegin()->t<< std::endl;
                    printf("[filter]: imu_t: %.4f,  vio_t: %.4f\n", ImuBuf[_i].t, PredictBuf.rbegin()->t);
                    restart_filter();
                    filter_mutex.unlock();
                    return;
                } else if (_dt < 0.0f) {
                    _dt = 0.001f;
                } else if (_dt > 0.5f) {
                    _dt = 0.5f;
                }

                PredictIndex++;
                // input output filter
                double _dt_imu = ImuBuf[ImuBuf.size()-1].t - ImuBuf[ImuBuf.size()-2].t;
                outputpredictor_filter_ptr->input_imu(tmp, PredictIndex, _dt_imu);
                if (PredictIndex < OutputPeriodCount) { // IMU sample rate / OutputPeriodCount
                    filter_mutex.unlock();
                    return;
                } else {
                    PredictIndex = 0;
                }

                U _u;
                imu_data _mid_filter_data = ImuBuf[_i];
                for (int _j = 1; _j <= 10; _j++) {
                    _mid_filter_data.acc += ImuBuf[_i + _j].acc;
                    _mid_filter_data.gyro += ImuBuf[_i + _j].gyro;
                    _mid_filter_data.acc += ImuBuf[_i - _j].acc;
                    _mid_filter_data.gyro += ImuBuf[_i - _j].gyro;
                }

                _mid_filter_data.acc /= 20 + 1;
                _mid_filter_data.gyro /= 20 + 1;

                _u.ax() = _mid_filter_data.acc(0);//ImuBuf[_i].acc(0);
                _u.ay() = _mid_filter_data.acc(1);//[_i].acc(1);
                _u.az() = _mid_filter_data.acc(2);//[_i].acc(2);
                _u.wx() = _mid_filter_data.gyro(0);//ImuBuf[_i].gyro(0);
                _u.wy() = _mid_filter_data.gyro(1);//ImuBuf[_i].gyro(1);
                _u.wz() = _mid_filter_data.gyro(2);//ImuBuf[_i].gyro(2);
                V _v;
                _v.setZero();

                // std::cout << "[filter]: PredictBufsize:" << PredictBuf.size() << std::endl;

                if (!predict_core.predict(_u, _v, T(_dt), _P, _x)) {
                    std::cout << "[filter]: predict wrong!" << std::endl;
                    filter_mutex.unlock();
                    return;
                } else {
                    has_first_predict = true;
                    outputpredictor_filter_ptr->update_output(_x, ImuBuf[_i].t);
                    if (has_regist_publisher) {
                        Eigen::Vector3d _tmp_pos, _tmp_vel, _tmp_acc;
                        Eigen::Quaterniond _tmp_q;
                        _tmp_pos << _x(0), _x(1), _x(2);
                        _tmp_vel << _x(3), _x(4), _x(5);
                        _tmp_q.w() = _x(6);
                        _tmp_q.x() = _x(7);
                        _tmp_q.y() = _x(8);
                        _tmp_q.z() = _x(9);
                        Eigen::Matrix3d _tmp_R = _tmp_q.toRotationMatrix();
                        Eigen::Vector3d _tmp_acc_b{_x(10), _x(11), _x(12)};
                        Eigen::Vector3d _g{0,0,9.80665f};
                        _tmp_acc = _tmp_R * (ImuBuf[_i].acc - _tmp_acc_b) - _g;

                        estimator_publisher_ptr->UkfPub(_tmp_pos, _tmp_vel, _tmp_acc, _tmp_q, ImuBuf[_i].t);
                    }
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

                if (need_to_update_vio > 0) {
                    process_vio_update(VioBuf[VioBuf.size() - need_to_update_vio]);
                    need_to_update_vio--;
                    // std::cout << "[filter]: vio update after predict process" << std::endl;
                }

                if (need_to_update_lidar > 0) {
                    process_lidar_update(LidarBuf[LidarBuf.size() - need_to_update_lidar]);
                    need_to_update_lidar--;
                    // std::cout << "[filter]: lidar update after predict process" << std::endl;
                }

            }
            while ( ImuBuf.size() > ImuBuf_Size ) {
                ImuBuf.pop_front();
            }
        }

        void input_vio(vio_data tmp) {
            VioBuf.push_back(tmp);
            while (VioBuf.size() > 100) {
                VioBuf.pop_front();
            }
            process_vio_update(tmp);
        }

        void process_vio_update(vio_data tmp){

            if (!has_first_predict) {
                init_filter(tmp);
            } else {
                filter_mutex.lock();
                int _PBUF_SIZE = PredictBuf.size();
                // if (_PBUF_SIZE > 1) {
                    for (int _i = 0; _i < _PBUF_SIZE; _i++) {
                        double _dt = tmp.t - PredictBuf[_PBUF_SIZE - 1 - _i].t;
                        if (_dt > 0.0f || _i == _PBUF_SIZE-1) {
                            Z _z;
                            _z.px() = T(tmp.pos(0));
                            _z.py() = T(tmp.pos(1));
                            _z.pz() = T(tmp.pos(2));
                            _z.vx() = T(tmp.vel(0));
                            _z.vy() = T(tmp.vel(1));
                            _z.vz() = T(tmp.vel(2));
                            _z.qw() = T(tmp.att.w());
                            _z.qx() = T(tmp.att.x());
                            _z.qy() = T(tmp.att.y());
                            _z.qz() = T(tmp.att.z());
                            N _n;
                            _n.setZero();
                            _n.pz() = bias_vio_z;
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
                // } else {
                //     // std::cout << "[filter]: vio update: has no predict before update" << std::endl;
                //     need_to_update_vio ++;
                // }

                // update lidar correct 

                double _meas_dt = tmp.t - last_lidar_measurement_valid_time;
                if (_meas_dt < 0.5f) {
                    lidar_data _tmp_lidar;
                    // std::cout << "hello" << std::endl;
                    for (int _i = LidarBuf.size()-1; _i >=0 ; _i-- ) {
                        if (LidarBuf[_i].t < tmp.t) {
                            lidar_data _tmp_lidar = LidarBuf[_i];
                            break;
                        }
                    }
                    double _valid_dt = tmp.t - _tmp_lidar.t;
                    if (_valid_dt > 0 && _valid_dt < 0.5f) {
                        // std::cout << "hello2" << std::endl;
                        Eigen::Quaterniond _tmp_q;
                        _tmp_q.w() = double(tmp.att.w());
                        _tmp_q.x() = double(tmp.att.x());
                        _tmp_q.y() = double(tmp.att.y());
                        _tmp_q.z() = double(tmp.att.z());
                        _tmp_q.normalize();
                        Eigen::Matrix3d _tmp_R = _tmp_q.toRotationMatrix();
                        if (_tmp_R(2,2) > 0.7f) {
                        // std::cout << "hello3" << std::endl;
                            if (_tmp_lidar.data > 0.2) { // 0.7
                                double _meas_distant = 0.02f * _tmp_R(2,0) + 0.05f * _tmp_R(2,1) + (_tmp_lidar.data + 0.11f) * _tmp_R(2,2);
                                double _err_vio_lidar_z = tmp.pos(2) - bias_vio_z - _meas_distant - terrain_vpos;//_error_lidar;
                                // std::cout << "[filter]: err_vio_lidar: " << _err_vio_lidar_z << std::endl;
                                err_vio_lidar_z_integ += _err_vio_lidar_z;
                                double _lidar_correct_gain = 0.21f/ 0.25f;
                                double _lidar_correct_gain_sp = _lidar_correct_gain * _lidar_correct_gain;
                                bias_vio_z = _lidar_correct_gain * _err_vio_lidar_z + _lidar_correct_gain_sp * 0.2f * err_vio_lidar_z_integ;
                                // std::cout << "[filter]: bias_vio_z: " << bias_vio_z << std::endl;
                            }
                        }
                    }
                }

                filter_mutex.unlock();
            }
        }

        void input_lidar(lidar_data tmp) {
            LidarBuf.push_back(tmp);
            while(LidarBuf.size() > 100) {
                LidarBuf.pop_front();
            }
            process_lidar_update(tmp);
        }

        void process_lidar_update(lidar_data tmp) {
            if (has_first_predict) {
                filter_mutex.lock();
                int _PBUF_SIZE = PredictBuf.size();
                // if (_PBUF_SIZE > 1) {
                    for (int _i = LidarBuf.size()-1; _i >=0; _i--) {
                        if (_i == 0) {
                            std::cout << "[filter]: lidar update time wrong" << std::endl;
                            filter_mutex.unlock();
                            return;
                        }
                        if (LidarBuf[_i].t < PredictBuf.rbegin()->t) {
                            tmp = LidarBuf[_i];
                            // std::cout << _i << std::endl;
                            break;
                        }
                    }
                    Eigen::Quaterniond _tmp_q;
                    _tmp_q.w() = double(PredictBuf.rbegin()->x(6));
                    _tmp_q.x() = double(PredictBuf.rbegin()->x(7));
                    _tmp_q.y() = double(PredictBuf.rbegin()->x(8));
                    _tmp_q.z() = double(PredictBuf.rbegin()->x(9));
                    Eigen::Matrix3d _temp_R = _tmp_q.toRotationMatrix();
                    double _R_22 = _temp_R(2,2);
                    double _R_21 = _temp_R(2,1);
                    double _R_20 = _temp_R(2,0);
                    if (_R_22 > 0.7f) {
                        double _meas_height = 0.02f * _R_20 + 0.05f * _R_21 + (tmp.data + 0.11f)* _R_22;
                        estimator_publisher_ptr->LidarPub(bias_vio_z, _meas_height + terrain_vpos, tmp.t);
                        lidar_data _need_to_update;
                        _need_to_update.t = tmp.t;
                        _need_to_update.data = _meas_height;
                        if (tmp.data > 0.2f) { // 0.6f
                            double _meas_dt = tmp.t - last_lidar_measurement_valid_time;
                            if (_meas_dt < 0.5f) {
                                double _error_lidar = PredictBuf.rbegin()->x(2) - _meas_height - terrain_vpos;
                                if (_error_lidar < 0.3f && _error_lidar > -0.3f) {
                                    last_lidar_measurement_valid_time = _need_to_update.t;
                                    Z_l _z;
                                    _z(0) = T(_need_to_update.data);
                                    N_l _n;
                                    _n.setZero();
                                    X _x;
                                    _x = PredictBuf.rbegin()->x;
                                    double _old_z = double(_x(2));
                                    P _P;
                                    _P = PredictBuf.rbegin()->P;
                                    lidar_update_core.update(_z, _x, _P, _n);
                                    predict_core.update_P_x(_P, _x);
                                    predict_state<T> _p_s;
                                    _p_s.t = PredictBuf.rbegin()->t;
                                    _p_s.P = _P;
                                    _p_s.x = _x;
                                    PredictBuf.clear();
                                    PredictBuf.push_back(_p_s);
                                    // TODO: update correct bias of vio z
                                    // double _err_vio_lidar_z = _x(2) - _meas_height - terrain_vpos;//_error_lidar;
                                    // std::cout << "[filter]: err_vio_lidar: " << _err_vio_lidar_z << std::endl;
                                    // err_vio_lidar_z_integ += _err_vio_lidar_z;
                                    // double _lidar_correct_gain = 0.03f/ 0.25f;
                                    // double _lidar_correct_gain_sp = _lidar_correct_gain * _lidar_correct_gain;
                                    // bias_vio_z = _lidar_correct_gain * _err_vio_lidar_z + _lidar_correct_gain_sp * 0.1f * err_vio_lidar_z_integ;
                                    // std::cout << "[filter]: bias_vio_z: " << bias_vio_z << std::endl;
                                    filter_mutex.unlock();
                                    return;
                                }
                            } 
                            // re measurement the terrain
                            last_lidar_measurement_valid_time = _need_to_update.t;
                            terrain_vpos = PredictBuf.rbegin()->x(2) - _meas_height;
                            std::cout << "[filter]: lidar update: z: " << PredictBuf.rbegin()->x(2) << " lidar: " << _meas_height << std::endl;
                            std::cout << "[filter]: lidar update: terrain vpos: " << terrain_vpos << std::endl;
                        }
                    }

                // } else {
                //     // std::cout << "[filter]: lidar update: has no predict before update" << std::endl;
                //     need_to_update_lidar ++;
                // }
                filter_mutex.unlock();
            }
        }

        void set_publish(EstimatorPublisher& _tmp) {
            has_regist_publisher = true;
            estimator_publisher_ptr = &_tmp;
            outputpredictor_filter_ptr->set_publish(_tmp);
        }

    private:
        typedef typename MotionModel<T>::Im U;
        typedef typename MotionModel<T>::Imn V;
        typedef typename MotionModel<T>::Ms X;
        typedef typename VioMeasurementModel<T>::Vm Z;
        typedef typename VioMeasurementModel<T>::Vmn N;
        typedef typename LidarMeasurementModel<T>::Lm Z_l;
        typedef typename LidarMeasurementModel<T>::Lmn N_l;
        typedef typename Kalman::Imu_predict<T>::StateCovariance P;
        Kalman::Imu_predict<T> predict_core;
        Kalman::Vio_update<T> vio_update_core;
        Kalman::Lidar_update<T> lidar_update_core;
        bool has_init;
        bool has_first_predict;

        std::deque<imu_data> ImuBuf;
        int ImuBuf_Size;
        int Delay_Size;
        std::mutex filter_mutex;

        std::deque<predict_state<T>> PredictBuf;
        int PredictBuf_Size;
        int OutputPeriodCount;
        int PredictIndex; 

        std::deque<vio_data> VioBuf;
        std::deque<lidar_data> LidarBuf;
        int need_to_update_vio;
        int need_to_update_lidar;

        double terrain_vpos;
        double last_lidar_measurement_valid_time;
        double bias_vio_z;
        double err_vio_lidar_z_integ;

        OutputPredictor<T>* outputpredictor_filter_ptr;

        bool has_regist_publisher;
        EstimatorPublisher* estimator_publisher_ptr;

};

#endif