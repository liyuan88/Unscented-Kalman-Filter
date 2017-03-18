#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include "lib/Eigen/Dense"

namespace tools {

    /**
    * A helper method to calculate RMSE.
    */
    Eigen::VectorXd
    CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth);

};

#endif /* TOOLS_HPP */
