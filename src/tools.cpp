#include <iostream>
#include <stdexcept>
#include "tools.hpp"

namespace tools {
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth) {


        if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
            throw std::invalid_argument( "CalculateRMSE () - Error: Invalid input values." );
        }

        Eigen::VectorXd rmse(estimations[0].array().size());
        rmse.fill(0.0d);

        for (int i = 0; i < estimations.size(); ++i) {
            Eigen::VectorXd res = estimations[i] - ground_truth[i];
            res = res.array() * res.array();
            rmse += res;
        }

        rmse /= estimations.size();
        rmse = rmse.array().sqrt();

        return rmse;
    }
}


