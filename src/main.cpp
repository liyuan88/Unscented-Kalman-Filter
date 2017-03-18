#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "lib/Eigen/Dense"
#include "tools.hpp"
#include "ground_truth_package.hpp"
#include "measurement_package.hpp"
#include "lib/cxxopts.hpp"
#include "ukf.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

bool verbose = false;
bool useOnlyRadar = false;
bool useOnlyLidar = false;
string in_file_name_ = "";
string out_file_name_ = "";

void parseOptions(int argc, char *argv[]) {
    try {
        cxxopts::Options options(argv[0], " - Implementation of an Unscented Kalman Filter to"
                " fuse lidar and radar sensor data.\n"
                "Input and Output files are required");

        options.add_options()
                ("h,help", "Print help")
                ("i,input", "Input File", cxxopts::value<std::string>())
                ("o,output", "Output file", cxxopts::value<std::string>())
                ("v,verbose", "verbose flag", cxxopts::value<bool>(verbose))
                ("r,radar", "use only radar data", cxxopts::value<bool>(useOnlyRadar))
                ("l,lidar", "use only lidar data", cxxopts::value<bool>(useOnlyLidar));

        vector<string> optionals = {"input", "output"};
        options.parse_positional(optionals);

        options.parse(argc, argv);

        if (options.count("help")) {
            cout << options.help({"", "Group"}) << endl;
            exit(EXIT_SUCCESS);
        }

        if (options.count("input") == 0) {
            cout << "Please include an input file.\nUse -h to get more information" << std::endl;
            exit(EXIT_FAILURE);
        }

        if (options.count("output") == 0) {
            cout << "Please include an output file.\nUse -h to get more information" << std::endl;
            exit(EXIT_FAILURE);
        }

        in_file_name_ = options["input"].as<string>();
        out_file_name_ = options["output"].as<string>();

    } catch (const cxxopts::OptionException &e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }
}

void check_files(ifstream &in_file, string &in_name,
                 ofstream &out_file, string &out_name) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}


tuple<MeasurementPackage, GroundTruthPackage> parseLine(string line) {
    istringstream iss(line);
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
        // LASER MEASUREMENT
        // read measurements at this timestamp
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        gt_package.sensor_type_ = GroundTruthPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        float x;
        float y;
        iss >> x;
        iss >> y;
        meas_package.raw_measurements_ << x, y;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    } else if (sensor_type.compare("R") == 0) {
        // RADAR MEASUREMENT
        // read measurements at this timestamp
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        gt_package.sensor_type_ = GroundTruthPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        float ro;
        float theta;
        float ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro, theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.timestamp_ = timestamp;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;

    return make_tuple(meas_package, gt_package);
}


void writeLine(ofstream &out_file_, UKF &ukf,
               MeasurementPackage meas_package, GroundTruthPackage gt_package) {
    // output the estimation
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs - est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle - est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate - est

    // output the measurements
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        // output the measurements
        out_file_ << meas_package.raw_measurements_(0) << "\t";
        out_file_ << meas_package.raw_measurements_(1) << "\t";
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        // output the measurements in the cartesian coordinates
        double rho = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        out_file_ << rho * cos(phi) << "\t";
        out_file_ << rho * sin(phi) << "\t";
    }

    // output the ground truth packages
    double x_gt;
    double y_gt;
    double vx_gt;
    double vy_gt;
    double v_gt;
    double yaw_gt;
    double yaw_rate_gt;

    x_gt = gt_package.gt_values_(0);
    y_gt = gt_package.gt_values_(1);
    vx_gt = gt_package.gt_values_(2);
    vy_gt = gt_package.gt_values_(3);
    v_gt = sqrt(vx_gt * vx_gt + vy_gt * vy_gt);
    yaw_gt = fabs(vx_gt) > 0.0001 ? atan(vy_gt / vx_gt) : 0;
    yaw_rate_gt = 0;

    out_file_ << x_gt << "\t";
    out_file_ << y_gt << "\t";
    out_file_ << v_gt << "\t";
    out_file_ << yaw_gt << "\t";
    out_file_ << yaw_rate_gt << "\t";
    out_file_ << vx_gt << "\t";
    out_file_ << vy_gt << "\t";

    // output nis
    out_file_ << ukf.NIS_laser_ << "\t";
    out_file_ << ukf.NIS_radar_ << endl;
}


void processStream(ifstream &in_file_, ofstream &out_file_) {
    UKF ukf;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    string line;
    int cnt = 0;


    while (getline(in_file_, line)) {
        if (line.empty()) {
            continue;
        }

        auto lineParsed = parseLine(line);
        auto meas_package = get<0>(lineParsed);
        auto gt_package = get<1>(lineParsed);

        if (useOnlyRadar & meas_package.sensor_type_ == MeasurementPackage::LASER) {
            continue;
        } else if (useOnlyLidar & meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            continue;
        }

        ukf.ProcessMeasurement(meas_package);

        writeLine(out_file_, ukf, meas_package, gt_package);

        estimations.push_back(ukf.x_.head(2));
        ground_truth.push_back(gt_package.gt_values_.head(2));

        if (verbose) {
            auto sensorType = meas_package.sensor_type_;
            cout << "***** Entry: " << cnt++ << " *****" << endl << endl;
            cout << "SensorType = " << (sensorType == MeasurementPackage::LASER ? "Laser" : "Radar") << endl << endl;
            cout << "x_ = " << ukf.x_ << endl << endl;
            cout << "P_ = " << ukf.P_ << endl << endl;

            if (sensorType == MeasurementPackage::LASER) {
                cout << "NIS Laser = " << ukf.NIS_laser_ << endl << endl;
            } else if (sensorType == MeasurementPackage::RADAR) {
                cout << "NIS Radar = " << ukf.NIS_radar_ << endl << endl;
            }
        }
    }

    // compute the accuracy (RMSE)
    cout << "Accuracy - RMSE:" << endl << tools::CalculateRMSE(estimations, ground_truth) << endl;
}


int main(int argc, char *argv[]) {
    parseOptions(argc, argv);

    ifstream in_file_(in_file_name_.c_str(), ifstream::in);
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    check_files(in_file_, in_file_name_, out_file_, out_file_name_);

    processStream(in_file_, out_file_);

    // close files
    if (out_file_.is_open()) {
        out_file_.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    return EXIT_SUCCESS;
}
