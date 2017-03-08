#ifndef UNSCENTED_KALMAN_FILTER_UKF_HPP
#define UNSCENTED_KALMAN_FILTER_UKF_HPP

#include "lib/Eigen/Dense"
#include "measurement_package.hpp"
#include "ground_truth_package.hpp"
#include <vector>

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    ///* state covariance matrix
    Eigen::MatrixXd P_;

    ///* process noise
    Eigen::MatrixXd Q_;

    Eigen::MatrixXd H_laser_;

    Eigen::MatrixXd R_laser_;

    ///* augumented state vector
    Eigen::VectorXd x_aug;

    ///* augumented state covariance matrix
    Eigen::MatrixXd P_aug;

    ///* predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    ///* Sigma points
    Eigen::MatrixXd Xsig_;

    ///* time when the state is true, in us
    long time_us_;

    ///* previous_timestamp
    long previous_timestamp_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

    ///* Weights of sigma points
    Eigen::VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* the current NIS for radar
    double NIS_radar_;

    ///* the current NIS for laser
    double NIS_laser_;

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     * @param gt_package The ground truth of the state x at measurement time
     */
    void ProcessMeasurement(MeasurementPackage measurement_pack);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage measurement_pack);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage measurement_pack);
};

#endif //UNSCENTED_KALMAN_FILTER_UKF_HPP
