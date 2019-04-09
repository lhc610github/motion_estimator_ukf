#ifndef IMU_PREDICT_HPP_
#define IMU_PREDICT_HPP_

#include "Types.hpp"
#include <iostream>

template<typename T>
class Imu_measurement : public Kalman::Vector<T, 6> {
    public:
        KALMAN_VECTOR(Imu_measurement, T, 6)

        static constexpr size_t aX = 0;
        static constexpr size_t aY = 1;
        static constexpr size_t aZ = 2;
        static constexpr size_t wX = 3;
        static constexpr size_t wY = 4;
        static constexpr size_t wZ = 5;

        T ax()      const { return (*this)[ aX ]; }
        T ay()      const { return (*this)[ aY ]; }
        T az()      const { return (*this)[ aZ ]; }
        T wx()      const { return (*this)[ wX ]; }
        T wy()      const { return (*this)[ wY ]; }
        T wz()      const { return (*this)[ wZ ]; }

        T& ax()      { return (*this)[ aX ]; }
        T& ay()      { return (*this)[ aY ]; }
        T& az()      { return (*this)[ aZ ]; }
        T& wx()      { return (*this)[ wX ]; }
        T& wy()      { return (*this)[ wY ]; }
        T& wz()      { return (*this)[ wZ ]; }
};

template<typename T>
class Imu_measurement_noise : public Kalman::Vector<T, 12> {
    public:
        KALMAN_VECTOR(Imu_measurement_noise, T, 12)

        static constexpr size_t vaX = 0;
        static constexpr size_t vaY = 1;
        static constexpr size_t vaZ = 2;
        static constexpr size_t vwX = 3;
        static constexpr size_t vwY = 4;
        static constexpr size_t vwZ = 5;
        static constexpr size_t vabX = 6;
        static constexpr size_t vabY = 7;
        static constexpr size_t vabZ = 8;
        static constexpr size_t vwbX = 9;
        static constexpr size_t vwbY = 10;
        static constexpr size_t vwbZ = 11;

        T v_ax()      const { return (*this)[ vaX ]; }
        T v_ay()      const { return (*this)[ vaY ]; }
        T v_az()      const { return (*this)[ vaZ ]; }
        T v_wx()      const { return (*this)[ vwX ]; }
        T v_wy()      const { return (*this)[ vwY ]; }
        T v_wz()      const { return (*this)[ vwZ ]; }
        T v_b_ax()      const { return (*this)[ vabX ]; }
        T v_b_ay()      const { return (*this)[ vabY ]; }
        T v_b_az()      const { return (*this)[ vabZ ]; }
        T v_b_wx()      const { return (*this)[ vwbX ]; }
        T v_b_wy()      const { return (*this)[ vwbY ]; }
        T v_b_wz()      const { return (*this)[ vwbZ ]; }

        T& v_ax()      { return (*this)[ vaX ]; }
        T& v_ay()      { return (*this)[ vaY ]; }
        T& v_az()      { return (*this)[ vaZ ]; }
        T& v_wx()      { return (*this)[ vwX ]; }
        T& v_wy()      { return (*this)[ vwY ]; }
        T& v_wz()      { return (*this)[ vwZ ]; }
        T& v_b_ax()      { return (*this)[ vabX ]; }
        T& v_b_ay()      { return (*this)[ vabY ]; }
        T& v_b_az()      { return (*this)[ vabZ ]; }
        T& v_b_wx()      { return (*this)[ vwbX ]; }
        T& v_b_wy()      { return (*this)[ vwbY ]; }
        T& v_b_wz()      { return (*this)[ vwbZ ]; }
};

template<typename T>
class Motion_state : public Kalman::Vector<T, 16> {
    public:
        KALMAN_VECTOR(Motion_state, T, 16)

        static constexpr size_t pX = 0;
        static constexpr size_t pY = 1;
        static constexpr size_t pZ = 2;
        static constexpr size_t vX = 3;
        static constexpr size_t vY = 4;
        static constexpr size_t vZ = 5;
        static constexpr size_t qW = 6;
        static constexpr size_t qX = 7;
        static constexpr size_t qY = 8;
        static constexpr size_t qZ = 9;
        static constexpr size_t baX = 10;
        static constexpr size_t baY = 11;
        static constexpr size_t baZ = 12;
        static constexpr size_t bwX = 13;
        static constexpr size_t bwY = 14;
        static constexpr size_t bwZ = 15;

        T px()      const { return (*this)[ pX ]; }
        T py()      const { return (*this)[ pY ]; }
        T pz()      const { return (*this)[ pZ ]; }
        T vx()      const { return (*this)[ vX ]; }
        T vy()      const { return (*this)[ vY ]; }
        T vz()      const { return (*this)[ vZ ]; }
        T qw()     const { return (*this)[ qW]; }
        T qx()     const { return (*this)[ qX]; }
        T qy()     const { return (*this)[ qY]; }
        T qz()     const { return (*this)[ qZ]; }
        T bax()      const { return (*this)[ baX ]; }
        T bay()      const { return (*this)[ baY ]; }
        T baz()      const { return (*this)[ baZ ]; }
        T bwx()      const { return (*this)[ bwX ]; }
        T bwy()      const { return (*this)[ bwY ]; }
        T bwz()      const { return (*this)[ bwZ ]; }

        T& px()      { return (*this)[ pX ]; }
        T& py()      { return (*this)[ pY ]; }
        T& pz()      { return (*this)[ pZ ]; }
        T& vx()      { return (*this)[ vX ]; }
        T& vy()      { return (*this)[ vY ]; }
        T& vz()      { return (*this)[ vZ ]; }
        T& qw()      { return (*this)[ qW]; }
        T& qx()      { return (*this)[ qX]; }
        T& qy()      { return (*this)[ qY]; }
        T& qz()      { return (*this)[ qZ]; }
        T& bax()     { return (*this)[ baX ]; }
        T& bay()     { return (*this)[ baY ]; }
        T& baz()     { return (*this)[ baZ ]; }
        T& bwx()     { return (*this)[ bwX ]; }
        T& bwy()     { return (*this)[ bwY ]; }
        T& bwz()     { return (*this)[ bwZ ]; }
};

template<typename T>
class MotionModel {
    public:
        typedef Motion_state<T> Ms;
        typedef Imu_measurement<T> Im;
        typedef Imu_measurement_noise<T> Imn;
        Eigen::Vector3d one_g;

        typedef Kalman::Matrix<T, Ms::RowsAtCompileTime, Ms::RowsAtCompileTime> MsCov;
        typedef Kalman::Matrix<T, Imn::RowsAtCompileTime, Imn::RowsAtCompileTime> ImnCov;
        
        MsCov P;
        ImnCov Q;

        MotionModel() {
            one_g << 0, 0, 9.871f;
            Kalman::Vector<T, Ms::RowsAtCompileTime> P_vector;
            P_vector << 0.003, 0.003, 0.003, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 0.1, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001;
            // P_vector << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001;
            P_vector /=10;
            P = P_vector.asDiagonal();
            Kalman::Vector<T, Imn::RowsAtCompileTime> Q_vector;
            Q_vector << 0.1, 0.1, 0.1, 0.03, 0.03, 0.03, 0.00000000001, 0.00000000001, 0.00000000001, 0.0000000001, 0.0000000001, 0.0000000001;
            Q = Q_vector.asDiagonal();
        }

        Ms f(const Ms& x_k, const Im& u, const Imn& v, const T& dt) const {
            Ms x_k_1;
            T a_m_x = u.ax() - x_k.bax() + v.v_ax();
            T a_m_y = u.ay() - x_k.bay() + v.v_ay();
            T a_m_z = u.az() - x_k.baz() + v.v_az();

            T w_m_x = u.wx() - x_k.bwx() + v.v_wx();
            T w_m_y = u.wy() - x_k.bwy() + v.v_wy();
            T w_m_z = u.wz() - x_k.bwz() + v.v_wz();

            // T a_m_x = u.ax() + v.v_ax();
            // T a_m_y = u.ay() + v.v_ay();
            // T a_m_z = u.az() + v.v_az();

            // T w_m_x = u.wx() + v.v_wx();
            // T w_m_y = u.wy() + v.v_wy();
            // T w_m_z = u.wz() + v.v_wz();
            Eigen::Matrix3d _R;
            // Eigen::Vector3d _att_euler;
            // _att_euler << x_k.phi(), x_k.the(), x_k.psi();
            Eigen::Quaterniond _att_q;
            _att_q.w() = double(x_k.qw());
            _att_q.x() = double(x_k.qx());
            _att_q.y() = double(x_k.qy());
            _att_q.z() = double(x_k.qz());
            _att_q.normalized();


            Eigen::Vector3d _acc_m;
            _acc_m << double(a_m_x), double(a_m_y), double(a_m_z);
            // get_dcm_from_euler(_R, _att_euler);
            // _R = Eigen::AngleAxisd(_att_euler(2), Eigen::Vector3d::UnitZ())
            //     * Eigen::AngleAxisd(_att_euler(1), Eigen::Vector3d::UnitY())
            //     * Eigen::AngleAxisd(_att_euler(0), Eigen::Vector3d::UnitX());
            // _R = _att_q.toRotationMatrix();
            
            // std::cout << _R << std::endl;

            Eigen::Vector3d _w;
            _w << double(w_m_x), double(w_m_y), double(w_m_z);

            Eigen::Quaterniond _dq = Eigen::AngleAxisd(_w(2)*double(dt), Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(_w(1)*double(dt), Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(_w(0)*double(dt), Eigen::Vector3d::UnitX());
            Eigen::Quaterniond _att_tmp = _att_q * _dq;
            _att_tmp.normalized();
            _R = _att_tmp.toRotationMatrix();

            // std::cout << _R << std::endl;
            // Eigen::Matrix3d _delta_R;
            // _delta_R(0,0) = 1;
            // _delta_R(1,1) = 1;
            // _delta_R(2,2) = 1;
            // _delta_R(0,1) = -_w(2) * double(dt);
            // _delta_R(0,2) = _w(1) * double(dt);
            // _delta_R(1,0) = _w(2) * double(dt);
            // _delta_R(1,2) = -_w(0) * double(dt);
            // _delta_R(2,0) = -_w(1) * double(dt);
            // _delta_R(2,1) = _w(0) * double(dt);

            // _R *= _delta_R;
            // _R.normalized();

            // std::cout << "after rotation" << std::endl;
            // std::cout << _R << std::endl;

            Eigen::Vector3d _a = _R * _acc_m - one_g;

            // std::cout << _acc_m.transpose() << std::endl;

            Eigen::Vector3d _P;
            _P << double(x_k.px()), double(x_k.py()), double(x_k.pz());
            Eigen::Vector3d _V;
            _V << double(x_k.vx()), double(x_k.vy()), double(x_k.vz());

            _P += dt * _V + T(0.5) * dt * dt * _a;
            _V += dt * _a;

            // Eigen::Vector3d _att_euler_1 = _R.eulerAngles(2,1,0);
            // std::cout << _att_euler_1.transpose() << std::endl;

            x_k_1.px() = T(_P(0));
            x_k_1.py() = T(_P(1));
            x_k_1.pz() = T(_P(2));
            x_k_1.vx() = T(_V(0));
            x_k_1.vy() = T(_V(1));
            x_k_1.vz() = T(_V(2));
            // x_k_1.phi() = _att_euler_1(2);
            // x_k_1.the() = _att_euler_1(1);
            // x_k_1.psi() = _att_euler_1(0);
            x_k_1.qw() = T(_att_tmp.w());
            x_k_1.qx() = T(_att_tmp.x());
            x_k_1.qy() = T(_att_tmp.y());
            x_k_1.qz() = T(_att_tmp.z());
            x_k_1.bax() = x_k.bax() + v.v_b_ax() * dt;
            x_k_1.bay() = x_k.bay() + v.v_b_ay() * dt;
            x_k_1.baz() = x_k.baz() + v.v_b_az() * dt;
            x_k_1.bwx() = x_k.bwx() + v.v_b_wx() * dt;
            x_k_1.bwy() = x_k.bwy() + v.v_b_wy() * dt;
            x_k_1.bwz() = x_k.bwz() + v.v_b_wz() * dt;
            
            return x_k_1;
        }

};

namespace Kalman {
// template<typename T, typename MotionState, typename ProcessNoiseState, typename >
template<typename T>//, template <T> typename PredictModel>
class Imu_predict {
    public:
        Imu_predict() {
            reset();
        }
        ~Imu_predict() { }

        static constexpr int SigmaPointCount = 2 * (MotionModel<T>::Ms::RowsAtCompileTime + MotionModel<T>::Imn::RowsAtCompileTime) + 1;
        static constexpr int StateRowsCount = MotionModel<T>::Ms::RowsAtCompileTime + MotionModel<T>::Imn::RowsAtCompileTime;
        static constexpr int MSRowsCount = MotionModel<T>::Ms::RowsAtCompileTime;
        static constexpr int PNSRowsCount = MotionModel<T>::Imn::RowsAtCompileTime;

        typedef Vector<T, SigmaPointCount> SigmaWeights;
        typedef Vector<T, StateRowsCount> AllStates;
        typedef Vector<T, MSRowsCount> MotionStates;
        typedef Vector<T, PNSRowsCount> NoiseStates;
        typedef Matrix<T, StateRowsCount, SigmaPointCount> SigmaPoints;
        typedef Matrix<T, MSRowsCount, MSRowsCount> StateCovariance;
        typedef Matrix<T, PNSRowsCount, PNSRowsCount> NoiseCovariance;

        void init(const MotionStates& tmp) {
            x = tmp;
            has_init = true;
            std::cout << "[predict]: init: " << x.transpose() << std::endl;
        }

        void reset() {
            // P.setIdentity();
            P = motion_model.P;
            // Q.setIdentity();
            // Q *= T(0.01f);
            Q = motion_model.Q;
            x.setZero();
            alpha = T(0.6f);    //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
            beta = T(2.0f);     //!< Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
            kappa = T(0.0f);    //!< Secondary scaling parameter (usually 0)
            computeWeights();
            has_init = false;
        }

        StateCovariance get_P() {
            return P;
        }

        MotionStates get_x() {
            return x;
        }

        void update_P_x(const StateCovariance& tmp1, const MotionStates& tmp2) {
            P = tmp1;
            x = tmp2;
        }

        bool predict(const typename MotionModel<T>::Im& u,
        const typename MotionModel<T>::Imn& v,
        const T& dt,
        StateCovariance& _tmp_P,
        MotionStates& _tmp_x) {
            // typename MotionModel<T>::Ms res;
            if (!has_init) {
                std::cout << "[predict]: has not init " << std::endl;
                return false;
            }
            //Compute sigma points
            if (!computeSigmaPoints(v)) {
                std::cout << "[predict]: SigmaPoints cal wrong" << std::endl;
                P = motion_model.P;
                Q = motion_model.Q;
                computeSigmaPoints(v);
                return false;
            }
            //Compute predicted state
            computeStatePrediction(u, dt);
            //Compute predicted covariance
            computeCovarianceFromSigmaPoints();

            _tmp_x = x;
            _tmp_P = P;
            
            return true;
        }

        bool computeSigmaPoints(const typename MotionModel<T>::Imn& v) {
            CovarianceSquareRoot<AllStates> llt;
            SquareMatrix<T, AllStates::RowsAtCompileTime> P_a;
            P_a.setZero();
            P_a.block(0,0,MSRowsCount, MSRowsCount) = P;
            P_a.block(MSRowsCount, MSRowsCount, PNSRowsCount, PNSRowsCount) = Q;
            llt.compute(P_a);
            if (llt.info() != Eigen::Success) {
                P_a.block(0,0,MSRowsCount, MSRowsCount) = motion_model.P;
                std::cout << "[imu_predict]: P matrix is not positive cannot get squareroot" << std::endl;
                // std::cout << P_a << std::endl;
                // Eigen::EigenSolver<Matrix<T, AllStates::RowsAtCompileTime, AllStates::RowsAtCompileTime>> es(P_a);
                // Matrix<T, AllStates::RowsAtCompileTime, AllStates::RowsAtCompileTime> _D = es.pseudoEigenvalueMatrix();
                // Matrix<T, AllStates::RowsAtCompileTime, AllStates::RowsAtCompileTime> _V = es.pseudoEigenvectors();
                // std::cout << "[vio_update]: P eigenvalue: " << std::endl;
                // std::cout << _D << std::endl;
                // for (int _i = 0; _i < AllStates::RowsAtCompileTime; _i ++) {
                //     if (_D(_i, _i) < T(0)) {
                //         _D(_i, _i) = T(0);
                //     }
                // }
                // std::cout << "[vio_update]: change the negative eigenvalue for squareroot: " << std::endl;
                // std::cout << _D << std::endl;
                // P_a = _V * _D * _V.inverse();
                // std::cout << "[vio_update]: get the new positive conv matrix" << std::endl;
                // std::cout << P_a << std::endl;
                llt.compute(P_a);
                if (llt.info() != Eigen::Success)
                    return false;
            }

            SquareMatrix<T, AllStates::RowsAtCompileTime> _S = llt.matrixL().toDenseMatrix();
            Vector<T, StateRowsCount> all_x;
            all_x << x, v;
            sigmaStatePoints.template leftCols<1>() = all_x;
            sigmaStatePoints.template block<StateRowsCount, StateRowsCount>(0,1)
                = (gamma * _S).colwise() + all_x;
            sigmaStatePoints.template rightCols<StateRowsCount>()
                = (-gamma * _S).colwise() + all_x;

            return true;
        }

        void computeStatePrediction(const typename MotionModel<T>::Im& u, const T& dt) {
            computeSigmaPointTransition(u, dt);
            computePredictionFromSigmaPoints();
        }

        void computeSigmaPointTransition(const typename MotionModel<T>::Im& u, const T& dt) {
            for (int i = 0; i < SigmaPointCount; ++i ) {
                MotionStates temp_motion_state = motion_model.f(sigmaStatePoints.col(i).head(MSRowsCount), u, sigmaStatePoints.col(i).tail(PNSRowsCount), dt);
                // std::cout << "sg["<<i<<"]:" <<temp_motion_state.transpose() << std::endl;
                sigmaStatePoints.col(i).head(MSRowsCount) = temp_motion_state;
            }
        }

        void computePredictionFromSigmaPoints() {
            AllStates temp_all_state = sigmaStatePoints * sigmaWm;
            // Vector<T, MotionModel<T>::Ms.phI>temp_all_state1 = sigmaStatePoints.block(0,0,MotionModel<T>::Ms.phI,SigmaPointCount) * sigmaWm;
            // Eigen::Matrix3d temp_sum_R;
            // temp_sum_R.setZero();
            // for (int i = 0; i < SigmaPointCount; ++i) {
            //     Vector<T, 3> _att = sigmaStatePoints.col(i).block(MotionModel<T>::Ms::phI, 0, 3, 1);
            //     // std::cout << temp_att_state.transpose() << std::endl;

            //     Eigen::Matrix3d _R;
            //     _R = Eigen::AngleAxisd(double(_att(2)), Eigen::Vector3d::UnitZ())
            //         * Eigen::AngleAxisd(double(_att(1)), Eigen::Vector3d::UnitY())
            //         * Eigen::AngleAxisd(double(_att(0)), Eigen::Vector3d::UnitX());
                
            //     temp_sum_R += (double)sigmaWm(i) * _R;
            // }
            // temp_sum_R.normalized();
            // Eigen::Vector3d _att_euler_1 = temp_sum_R.eulerAngles(2,1,0);
            // temp_all_state(MotionModel<T>::Ms::phI) = T(_att_euler_1(2));
            // temp_all_state(MotionModel<T>::Ms::thE) = T(_att_euler_1(1));
            // temp_all_state(MotionModel<T>::Ms::psI) = T(_att_euler_1(0));

            x = temp_all_state.head(MSRowsCount);
            Eigen::Quaterniond _tmp_q;
            _tmp_q.w() = double(x(6));
            _tmp_q.x() = double(x(7));
            _tmp_q.y() = double(x(8));
            _tmp_q.z() = double(x(9));
            _tmp_q.normalized();
            x(6) = T(_tmp_q.w());
            x(7) = T(_tmp_q.x());
            x(8) = T(_tmp_q.y());
            x(9) = T(_tmp_q.z());

        }

        void computeCovarianceFromSigmaPoints() {
            Matrix<T, MSRowsCount, SigmaPointCount> W = sigmaWc.transpose().template replicate<MSRowsCount, 1>();
            Matrix<T, MSRowsCount, SigmaPointCount> tmp_sp = sigmaStatePoints.block(0, 0, MSRowsCount, SigmaPointCount);
            Matrix<T, MSRowsCount, SigmaPointCount> tmp = (tmp_sp.colwise() - x);
            // Vector<T, 3> _x_att = x.block(MotionModel<T>::Ms::phI, 0, 3, 1);
            // Eigen::Matrix3d _x_R;
            // _x_R = Eigen::AngleAxisd(double(_x_att(2)), Eigen::Vector3d::UnitZ())
            //     * Eigen::AngleAxisd(double(_x_att(1)), Eigen::Vector3d::UnitY())
            //     * Eigen::AngleAxisd(double(_x_att(0)), Eigen::Vector3d::UnitX());

            // for (int i = 0; i < SigmaPointCount; ++i) {
            //     Vector<T, 3> _sp_att = sigmaStatePoints.col(i).block(MotionModel<T>::Ms::phI, 0, 3, 1);
            //     Eigen::Matrix3d _sp_R;
            //     _sp_R = Eigen::AngleAxisd(double(_sp_att(2)), Eigen::Vector3d::UnitZ())
            //         * Eigen::AngleAxisd(double(_sp_att(1)), Eigen::Vector3d::UnitY())
            //         * Eigen::AngleAxisd(double(_sp_att(0)), Eigen::Vector3d::UnitX());
                     
            //     Eigen::Matrix3d temp_eR;
            //     temp_eR = (_x_R.transpose()*_sp_R - _sp_R.transpose()*_x_R) / 2.0f;
            //     // Eigen::Matrix3d temp_eR = _x_R.transpose() *_sp_R;

            //     // Eigen::Vector3d temp_ee = temp_eR.eulerAngles(2,1,0);

            //     tmp(MotionModel<T>::Ms::phI, i) = T(temp_eR(2,1));
            //     tmp(MotionModel<T>::Ms::thE, i) = T(temp_eR(0,2));
            //     tmp(MotionModel<T>::Ms::psI, i) = T(temp_eR(1,0));
            //     // tmp(MotionModel<T>::Ms::phI, i) = T(temp_ee(2));
            //     // tmp(MotionModel<T>::Ms::thE, i) = T(temp_ee(1));
            //     // tmp(MotionModel<T>::Ms::psI, i) = T(temp_ee(0));
            //     // std::cout << "tmp[:,"<<i <<"]:" << tmp.col(i).transpose() << std::endl;
            // }
            
            P = tmp.cwiseProduct(W) * tmp.transpose();
        }

        void computeWeights() {
            T L = T(StateRowsCount);
            lambda = alpha * alpha * (L + kappa) - L;
            gamma = std::sqrt(L + lambda);
            T W_m_0 = lambda / ( L + lambda );
            T W_c_0 = W_m_0 + (T(1) - alpha*alpha + beta);
            T W_i   = T(1) / ( T(2) * alpha*alpha * (L + kappa) );

            sigmaWm[0] = W_m_0;
            sigmaWc[0] = W_c_0;

            for (int i = 1; i < SigmaPointCount; ++i) {
                sigmaWm[i] = W_i;
                sigmaWc[i] = W_i;
            }
            // std::cout << "Wm :" << sigmaWm.transpose() << std::endl;
            // std::cout << "Wc :" << sigmaWc.transpose() << std::endl;
        }



    private:
        SigmaWeights sigmaWc;
        SigmaWeights sigmaWm;
        SigmaPoints sigmaStatePoints;
        MotionStates x; 
        // NoiseStates v;
        StateCovariance P;
        NoiseCovariance Q;

        // Weight parameters
        T alpha;    //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
        T beta;     //!< Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
        T kappa;    //!< Secondary scaling parameter (usually 0)
        T gamma;    //!< \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the state dimensionality
        T lambda;   //!< \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$ being the state dimensionality

        MotionModel<T> motion_model;

        bool has_init;

};
}


#endif