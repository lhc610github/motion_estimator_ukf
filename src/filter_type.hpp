#ifndef FILTER_HPP_
#define FILTER_HPP_
#include "Types.hpp"
#include "imu_predict.hpp"
#include "vio_update.hpp"
#include <deque>

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

struct lidar_data {
    double t;
    double data;
};

// class update_work_queue {
//     public:
//         update_work_queue(int _size = 100):
//         BufSize(_size) {
//             last_update_type = 0;
//             // VioHasNewData = false;
//             // LidarHasNewData = false;
//             numVioNewData = 0;
//             numLidarNewData = 0;
//         }
//         ~update_work_queue() {}

//         struct res {
//             int type;
//             vio_data viodate;
//             lidar_data lidardata;
//         };

//         res check_work_queue(const double& t) {
//             res _res;
//             _res.type = Notype;
//             if (numVioNewData == 0 && numLidarNewData == 0) {
//                 return _res;
//             }
//             if (numVioNewData > 0) {
//                 for (auto _it = VioBuf.rbegin(); _it > VioBuf.begin(); _it++) {
//                     double _dt = _it->t - t;
//                     if (_dt >= 0) {
//                         if (_dt )
//                         if (numVioNewData > 1) {
//                             std::cout << "[filter]: update work queue: vio too busy drop " << numVioNewData-1 << " datas" << std::endl;
//                         }
//                         numVioNewData = 0;
//                         _res.type = Viotype;
//                         _res.viodata = *VioBuf.rbegin();
//                         return Viotype;
//                     } else {
//                         numVioNewData = 0;
//                         return _res;
//                     }
//                 }
//             }

//             if (numLidarNewData > 0) {
//             }
//         }

//         static constexpr int Notype = 0;
//         static constexpr int Viotype = 1;
//         static constexpr int Lidartype = 2;

//         std::deque<vio_data> VioBuf;
//         std::deque<lidar_data> LidarBuf;

//     private:
//         int last_update_type;
//         int BufSize;

//         // bool VioHasNewData;
//         int numVioNewData;
//         // bool LidarHasNewData;
//         int numLidarNewData;
// };

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