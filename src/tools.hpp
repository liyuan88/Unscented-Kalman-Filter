#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include "lib/Eigen/Dense"
#include "measurement_package.hpp"

namespace tools {

    /**
    * A helper method to calculate RMSE.
    */
    Eigen::VectorXd
    CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth);

    float CalculateNISPerformance(const std::vector<float> &nis_values, MeasurementPackage::SensorType sensorType);

};

#endif /* TOOLS_HPP */
