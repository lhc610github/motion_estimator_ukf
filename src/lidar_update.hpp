#ifndef LIDAR_UPDATE_HPP_
#define LIDAR_UPDATE_HPP_

#include "Types.hpp"
#include "imu_predict.hpp"
#include <iostream>

template<typename T>
class Lidar_measurement : public Kalman::Vector<T, 1> {
    public:
        KALMAN_VECTOR(Lidar_measurement, T, 1)

        static constexpr size_t z = 0;

        T alt() const { return (*this)[z]; }

        T& alt() { return (*this)[z]; }
};


template<typename T>
class Lidar_measurement_noise : public Kalman::Vector<T, 1> {
    public:
        KALMAN_VECTOR(Lidar_measurement_noise, T, 1)

        static constexpr size_t z = 0;

        T alt() const {return (*this)[z]; }

        T& alt()    {return (*this)[z]; }
};

template<typename T>
class LidarMeasurementModel {
    public:
        typedef Motion_state<T> Ms;
        typedef Lidar_measurement<T> Lm;
        typedef Lidar_measurement_noise<T> Lmn;

        typedef Kalman::Matrix<T, Ms::RowsAtCompileTime, Ms::RowsAtCompileTime> MsCov;
        typedef Kalman::Matrix<T, Lmn::RowsAtCompileTime, Lmn::RowsAtCompileTime> LmnCov;

        MsCov P;
        LmnCov R;

        LidarMeasurementModel() {
            Kalman::Vector<T, Ms::RowsAtCompileTime> P_vector;
            P_vector << 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.1, 0.1, 0.1, 0.1, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001;
            // P_vector << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.00001, 0.00001, 0.000001, 0.000001, 0.000001, 0.000001;
            // P_vector /=10;
            P = P_vector.asDiagonal();
            // P.Identity();
            Kalman::Vector<T, Lmn::RowsAtCompileTime> R_vector;
            R_vector << 0.1;
            R = R_vector.asDiagonal();
        }

        Lm h(const Ms& x_k_1, const Lmn& n) const {
            Lm y_k_1;
            y_k_1.alt() = x_k_1.pz() + n.alt();
            return y_k_1;
        }
};

namespace Kalman {
template<typename T>
class Lidar_update {
    public:
        Lidar_update() {
            reset();
        }

        ~Lidar_update() {}
        static constexpr int SigmaPointCount = 2 * (LidarMeasurementModel<T>::Ms::RowsAtCompileTime + LidarMeasurementModel<T>::Lmn::RowsAtCompileTime) + 1;
        static constexpr int StateRowsCount = LidarMeasurementModel<T>::Ms::RowsAtCompileTime + LidarMeasurementModel<T>::Lmn::RowsAtCompileTime;
        static constexpr int MSRowsCount = LidarMeasurementModel<T>::Ms::RowsAtCompileTime;
        static constexpr int LMSRowsCount = LidarMeasurementModel<T>::Lm::RowsAtCompileTime;
        static constexpr int PNSRowsCount = LidarMeasurementModel<T>::Lmn::RowsAtCompileTime;

        typedef Vector<T, SigmaPointCount> SigmaWeights;
        typedef Vector<T, StateRowsCount> AllStates;
        typedef Vector<T, LMSRowsCount> MeasureStates;
        typedef Vector<T, PNSRowsCount> NoiseStates;
        typedef Matrix<T, StateRowsCount, SigmaPointCount> SigmaPoints;
        typedef Matrix<T, LMSRowsCount, LMSRowsCount> MeasurementCovariance;
        typedef Matrix<T, PNSRowsCount, PNSRowsCount> NoiseCovariance;
        typedef Matrix<T, LMSRowsCount, SigmaPointCount> LMSigmaPoints;
        typedef Matrix<T, MSRowsCount, LMSRowsCount> FilterGains;
        typedef Matrix<T, MSRowsCount, MSRowsCount> StateCovariance;
        typedef Vector<T, MSRowsCount> MotionStates;

        void reset() {
            P = lmmodel.P;
            R = lmmodel.R;
            z.setZero();
            alpha = T(0.6f);
            beta = T(2.0f);
            kappa = T(0.0f);
            computeWeights();
        }

        void update(const typename LidarMeasurementModel<T>::Lm& y_k_1,
         MotionStates& x_k_1, 
         StateCovariance& P_k_1,
         const typename LidarMeasurementModel<T>::Lmn& n) {
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

            // std::cout << "x:" << std::endl;
            // std::cout << x_k_1.transpose() << std::endl;
            // std::cout << "P:" << std::endl;
            // std::cout << P_k_1 << std::endl;
            // std::cout << "x_k_1:" << std::endl;
            // std::cout << x.transpose() << std::endl;
            // std::cout << "P_k_1:" << std::endl;
            // std::cout << P << std::endl;

            // std::cout << "ex:" << std::endl;
            // std::cout << x.transpose() - x_k_1.transpose() << std::endl;
            // std::cout << "eP:" << std::endl;
            // std::cout << P - P_k_1 << std::endl;

            // x_k_1(2) = x(2);
            // P_k_1(2, 2) = P(2, 2);
            x_k_1 = x;
            P_k_1 = P;
        }

        bool computeSigmaPoints(const typename LidarMeasurementModel<T>::Lmn& n) {
            CovarianceSquareRoot<AllStates> llt;
            SquareMatrix<T, AllStates::RowsAtCompileTime> P_a;
            P_a.setZero();
            P_a.block(0,0,MSRowsCount, MSRowsCount) = P;
            P_a.block(MSRowsCount, MSRowsCount, PNSRowsCount, PNSRowsCount) = R;
            llt.compute(P_a);
            if (llt.info() != Eigen::Success) {
                P_a.block(0,0,MSRowsCount, MSRowsCount) = lmmodel.P;
                // std::cout << "[lidar_update]: P matrix is not positive cannot get squareroot" << std::endl;
                std::cout << "\033[33m" << "[lidar_update]: P matrix is not positive cannot get squareroot" << "\033[0m" << std::endl;
                // std::cout << P_a << std::endl;
                // Eigen::EigenSolver<Matrix<T, AllStates::RowsAtCompileTime, AllStates::RowsAtCompileTime>> es(P_a);
                // Matrix<T, AllStates::RowsAtCompileTime, AllStates::RowsAtCompileTime> _D = es.pseudoEigenvalueMatrix();
                // Matrix<T, AllStates::RowsAtCompileTime, AllStates::RowsAtCompileTime> _V = es.pseudoEigenvectors();
                // std::cout << "[lidar_update]: P eigenvalue: " << std::endl;
                // std::cout << _D << std::endl;
                // for (int _i = 0; _i < AllStates::RowsAtCompileTime; _i ++) {
                //     if (_D(_i, _i) < T(0)) {
                //         _D(_i, _i) = T(0);
                //     }
                // }
                // std::cout << "[lidar_update]: change the negative eigenvalue for squareroot: " << std::endl;
                // std::cout << _D << std::endl;
                // P_a = _V * _D * _V.inverse();
                // std::cout << "[lidar_update]: get the new positive conv matrix" << std::endl;
                // std::cout << P_a << std::endl;
                llt.compute(P_a);
                if (llt.info() != Eigen::Success)
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
                sigmaMeasurePoints.col(i) = lmmodel.h( sigmaStatePoints.col(i).head(MSRowsCount), sigmaStatePoints.col(i).tail(PNSRowsCount));
            }
        }

        void computePredictionFromSigmaPoints() {
            y_predict = sigmaMeasurePoints * sigmaWm;
        }

        void computeCovarianceFromSigmaPoints() {
            decltype(sigmaMeasurePoints) W = sigmaWc.transpose().template replicate<LMSRowsCount,1>();
            decltype(sigmaMeasurePoints) tmp = (sigmaMeasurePoints.colwise() - y_predict);
            P_yy = tmp.cwiseProduct(W) * tmp.transpose();
        }


        void computeKalmanGain() {
            Matrix<T, MSRowsCount, SigmaPointCount> W = sigmaWc.transpose().template replicate<MSRowsCount, 1>();
            Matrix<T, MSRowsCount, LMSRowsCount> P_xy
                = (sigmaStatePoints.block(0,0, MSRowsCount, SigmaPointCount).colwise() - x).cwiseProduct(W).eval()
                * (sigmaMeasurePoints.colwise() - y_predict).transpose();
            K = P_xy * P_yy.inverse();
        }

        void updateState() {
            MeasureStates tmp = z - y_predict;
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
        LMSigmaPoints sigmaMeasurePoints;
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

        LidarMeasurementModel<T> lmmodel;
};
}

#endif