#ifndef VIO_UPDATE_HPP_
#define VIO_UPDATE_HPP_

#include "Types.hpp"
#include "imu_predict.hpp"
#include <iostream>

template<typename T>
class Vio_measurement : public Kalman::Vector<T, 9> {
    public:
        KALMAN_VECTOR(Vio_measurement, T, 9)

        static constexpr size_t pX = 0;
        static constexpr size_t pY = 1;
        static constexpr size_t pZ = 2;
        static constexpr size_t vX = 3;
        static constexpr size_t vY = 4;
        static constexpr size_t vZ = 5;
        static constexpr size_t phI = 6;
        static constexpr size_t thE = 7;
        static constexpr size_t psI = 8;

        T px()      const { return (*this)[ pX ]; }
        T py()      const { return (*this)[ pY ]; }
        T pz()      const { return (*this)[ pZ ]; }
        T vx()      const { return (*this)[ vX ]; }
        T vy()      const { return (*this)[ vY ]; }
        T vz()      const { return (*this)[ vZ ]; }
        T phi()      const { return (*this)[ phI ]; }
        T the()      const { return (*this)[ thE ]; }
        T psi()      const { return (*this)[ psI ]; }

        T& px()      { return (*this)[ pX ]; }
        T& py()      { return (*this)[ pY ]; }
        T& pz()      { return (*this)[ pZ ]; }
        T& vx()      { return (*this)[ vX ]; }
        T& vy()      { return (*this)[ vY ]; }
        T& vz()      { return (*this)[ vZ ]; }
        T& phi()     { return (*this)[ phI ]; }
        T& the()     { return (*this)[ thE ]; }
        T& psi()     { return (*this)[ psI ]; }
};

template<typename T>
class Vio_measurement_noise : public Kalman::Vector<T, 9> {
    public:
        KALMAN_VECTOR(Vio_measurement_noise, T, 9)

        static constexpr size_t pX = 0;
        static constexpr size_t pY = 1;
        static constexpr size_t pZ = 2;
        static constexpr size_t vX = 3;
        static constexpr size_t vY = 4;
        static constexpr size_t vZ = 5;
        static constexpr size_t phI = 6;
        static constexpr size_t thE = 7;
        static constexpr size_t psI = 8;

        T px()      const { return (*this)[ pX ]; }
        T py()      const { return (*this)[ pY ]; }
        T pz()      const { return (*this)[ pZ ]; }
        T vx()      const { return (*this)[ vX ]; }
        T vy()      const { return (*this)[ vY ]; }
        T vz()      const { return (*this)[ vZ ]; }
        T phi()      const { return (*this)[ phI ]; }
        T the()      const { return (*this)[ thE ]; }
        T psi()      const { return (*this)[ psI ]; }

        T& px()      { return (*this)[ pX ]; }
        T& py()      { return (*this)[ pY ]; }
        T& pz()      { return (*this)[ pZ ]; }
        T& vx()      { return (*this)[ vX ]; }
        T& vy()      { return (*this)[ vY ]; }
        T& vz()      { return (*this)[ vZ ]; }
        T& phi()     { return (*this)[ phI ]; }
        T& the()     { return (*this)[ thE ]; }
        T& psi()     { return (*this)[ psI ]; }
};

template<typename T>
class VioMeasurementModel {
    public:
        // typedef typename MotionModel<T>::Ms Ms;
        typedef Motion_state<T> Ms;
        typedef Vio_measurement<T> Vm;
        typedef Vio_measurement_noise<T> Vmn;

        typedef Kalman::Matrix<T, Ms::RowsAtCompileTime, Ms::RowsAtCompileTime> MsCov;
        typedef Kalman::Matrix<T, Vmn::RowsAtCompileTime, Vmn::RowsAtCompileTime> VmnCov;
        
        MsCov P;
        VmnCov R;

        VioMeasurementModel() {
            Kalman::Vector<T, Ms::RowsAtCompileTime> P_vector;
            P_vector << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            P = P_vector.asDiagonal();
            Kalman::Vector<T, Vmn::RowsAtCompileTime> R_vector;
            R_vector << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0001, 0.0001, 0.0001;
            R = R_vector.asDiagonal();
        }

        Vm h(const Ms& x_k_1, const Vmn& n) const {
            Vm y_k_1;
            y_k_1.px() = x_k_1.px() + n.px();
            y_k_1.py() = x_k_1.py() + n.py();
            y_k_1.pz() = x_k_1.pz() + n.pz();
            y_k_1.vx() = x_k_1.vx() + n.vx();
            y_k_1.vy() = x_k_1.vy() + n.vy();
            y_k_1.vz() = x_k_1.vz() + n.vz();
            y_k_1.phi() = x_k_1.phi() + n.phi();
            y_k_1.the() = x_k_1.the() + n.the();
            y_k_1.psi() = x_k_1.psi() + n.psi();
            return y_k_1;
        }
};

namespace Kalman {
template<typename T>//, template <T> typename PredictModel>
class Vio_update {
    public:
        Vio_update(){
            reset();
        }

        ~Vio_update() {}

        static constexpr int SigmaPointCount = 2 * (VioMeasurementModel<T>::Ms::RowsAtCompileTime + VioMeasurementModel<T>::Vmn::RowsAtCompileTime) + 1;
        static constexpr int StateRowsCount = VioMeasurementModel<T>::Ms::RowsAtCompileTime + VioMeasurementModel<T>::Vmn::RowsAtCompileTime;
        static constexpr int MSRowsCount = VioMeasurementModel<T>::Ms::RowsAtCompileTime;
        static constexpr int VMSRowsCount = VioMeasurementModel<T>::Vm::RowsAtCompileTime;
        static constexpr int PNSRowsCount = VioMeasurementModel<T>::Vmn::RowsAtCompileTime;

        typedef Vector<T, SigmaPointCount> SigmaWeights;
        typedef Vector<T, StateRowsCount> AllStates;
        typedef Vector<T, VMSRowsCount> MeasureStates;
        typedef Vector<T, PNSRowsCount> NoiseStates;
        typedef Matrix<T, StateRowsCount, SigmaPointCount> SigmaPoints;
        typedef Matrix<T, VMSRowsCount, VMSRowsCount> MeasurementCovariance;
        typedef Matrix<T, PNSRowsCount, PNSRowsCount> NoiseCovariance;
        typedef Matrix<T, VMSRowsCount, SigmaPointCount> VMSigmaPoints;
        typedef Matrix<T, MSRowsCount, VMSRowsCount> FilterGains;
        typedef Matrix<T, MSRowsCount, MSRowsCount> StateCovariance;
        typedef Vector<T, MSRowsCount> MotionStates;

        void reset() {
            // P.setIdentity();
            P = vmmodel.P;
            // P *= T(0.01f);
            // R.setIdentity();
            R = vmmodel.R;
            // R *= T(0.001f);
            z.setZero();
            alpha = T(0.01f);    //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
            beta = T(2.0f);     //!< Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
            kappa = T(0.0f);    //!< Secondary scaling parameter (usually 0)
            computeWeights();
        }

        void update(const typename VioMeasurementModel<T>::Vm& y_k_1,
         MotionStates& x_k_1, 
         StateCovariance& P_k_1,
         const typename VioMeasurementModel<T>::Vmn& n) {
            P = P_k_1;
            z = y_k_1;
            x = x_k_1;
            // Compute sigma point
            if (!computeSigmaPoints(n)) {
                std::cout << "[update]: SigmaPoints cal wrong" << std::endl;
                return;
            }
            // Predict measurement
            computeMeasurementPrediction();
            // Compute innovation covariance
            computeCovarianceFromSigmaPoints();
            // Campute Kalman Gain
            computeKalmanGain();
            // Update state;
            updateState();
            // Update state covariance
            updateStateCovariance();

            x_k_1 = x;
            P_k_1 = P;

        }

        bool computeSigmaPoints(const typename VioMeasurementModel<T>::Vmn& n) {
            CovarianceSquareRoot<AllStates> llt;
            SquareMatrix<T, AllStates::RowsAtCompileTime> P_a;
            P_a.setZero();
            P_a.block(0,0,MSRowsCount, MSRowsCount) = P;
            P_a.block(MSRowsCount, MSRowsCount, PNSRowsCount, PNSRowsCount) = R;
            llt.compute(P_a);
            if (llt.info() != Eigen::Success) {
                return false;
            }

            SquareMatrix<T, AllStates::RowsAtCompileTime> _S = llt.matrixL().toDenseMatrix();
            Vector<T, StateRowsCount> all_x;
            all_x << x, n;
            sigmaStatePoints.template leftCols<1>() = all_x;
            sigmaStatePoints.template block<StateRowsCount, StateRowsCount>(0,1)
                = (gamma * _S).colwise() + all_x;
            sigmaStatePoints.template rightCols<StateRowsCount>()
                = (-gamma * _S).colwise() + all_x;

            return true;
        }

        void computeMeasurementPrediction() {
            // Predict measurements for each sigma point
            computeSigmaPointMeasurements();
            // Predict measurement from sigma measurement
            computePredictionFromSigmaPoints();
        }

        void computeSigmaPointMeasurements() {
            for ( int i = 0; i < SigmaPointCount; ++i) {
                sigmaMeasurePoints.col(i) = vmmodel.h( sigmaStatePoints.col(i).head(MSRowsCount), sigmaStatePoints.col(i).tail(PNSRowsCount));
            }
        }

        void computePredictionFromSigmaPoints() {
            y_predict = sigmaMeasurePoints * sigmaWm;
        }

        void computeCovarianceFromSigmaPoints() {
            decltype(sigmaMeasurePoints) W = sigmaWc.transpose().template replicate<VMSRowsCount,1>();
            decltype(sigmaMeasurePoints) tmp = (sigmaMeasurePoints.colwise() - y_predict);
            P_yy = tmp.cwiseProduct(W) * tmp.transpose();
        }


        void computeKalmanGain() {
            Matrix<T, MSRowsCount, SigmaPointCount> W = sigmaWc.transpose().template replicate<MSRowsCount, 1>();
            Matrix<T, MSRowsCount, VMSRowsCount> P_xy
                = (sigmaStatePoints.block(0,0, MSRowsCount, SigmaPointCount).colwise() - x).cwiseProduct(W).eval()
                * (sigmaMeasurePoints.colwise() - y_predict).transpose();
            K = P_xy * P_yy.inverse();
        }

        void updateState() {
            MeasureStates tmp = z - y_predict;

            Vector<T, 3> _y_att = y_predict.block(VioMeasurementModel<T>::Vm::phI, 0, 3, 1);
            Eigen::Matrix3d _y_R;
            _y_R = Eigen::AngleAxisd(double(_y_att(2)), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(double(_y_att(1)), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(double(_y_att(0)), Eigen::Vector3d::UnitX());
            
            Vector<T, 3> _sp_att = z.block(VioMeasurementModel<T>::Vm::phI, 0, 3, 1);
            Eigen::Matrix3d _sp_R;
            _sp_R = Eigen::AngleAxisd(double(_sp_att(2)), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(double(_sp_att(1)), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(double(_sp_att(0)), Eigen::Vector3d::UnitX());
            
            std::cout << "update: R:" << _sp_R << std::endl;
            std::cout << "update: R1:" << _y_R << std::endl;
            
            // Eigen::Matrix3d temp_eR;
            // temp_eR = (_y_R.transpose()*_sp_R - _sp_R.transpose()*_y_R) / 2.0f;
            Eigen::Matrix3d temp_eR = _y_R.transpose() * _sp_R;

            Eigen::Vector3d temp_ee = temp_eR.eulerAngles(2,1,0);
            
            // tmp(VioMeasurementModel<T>::Vm::phI) = T(temp_eR(2,1));
            // tmp(VioMeasurementModel<T>::Vm::thE) = T(temp_eR(0,2));
            // tmp(VioMeasurementModel<T>::Vm::psI) = T(temp_eR(1,0));
            tmp(VioMeasurementModel<T>::Vm::phI) = T(temp_ee(2));
            tmp(VioMeasurementModel<T>::Vm::thE) = T(temp_ee(1));
            tmp(VioMeasurementModel<T>::Vm::psI) = T(temp_ee(0));

            std::cout << tmp.transpose() << std::endl;

            x += K * tmp;
        }

        void updateStateCovariance() {
            StateCovariance tmp_P = P - K * P_yy * K.transpose();
            P = tmp_P;
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
        VMSigmaPoints sigmaMeasurePoints;
        MeasureStates z; 
        MeasureStates y_predict; 
        MotionStates x;
        // NoiseStates n;
        StateCovariance P;
        MeasurementCovariance P_yy;
        NoiseCovariance R;
        FilterGains K;
        // Weight parameters
        T alpha;    //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
        T beta;     //!< Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
        T kappa;    //!< Secondary scaling parameter (usually 0)
        T gamma;    //!< \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the state dimensionality
        T lambda;   //!< \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$ being the state dimensionality

        VioMeasurementModel<T> vmmodel;
};
}

#endif