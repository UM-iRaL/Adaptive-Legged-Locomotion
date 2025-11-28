#ifndef QUAD_DATA_RECORDER_H
#define QUAD_DATA_RECORDER_H

#include <Eigen/Dense>
#include <stdexcept>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>

#include <ros/package.h>  // Include this to get the package path

class QuadDataRecorder {
public:
    // Constructor to initialize with the file name
    QuadDataRecorder(const std::string& filename);

    // Method to collect and save the robot body state in each control loop
    void saveData(double time, 
                const Eigen::VectorXd& body_state, 
                std::optional<Eigen::VectorXd> ctrl = std::nullopt, 
                std::optional<Eigen::VectorXd> feet_location = std::nullopt,
                std::optional<Eigen::VectorXd> global_plan = std::nullopt);


    // Destructor to close the file stream
    ~QuadDataRecorder();

private:
    std::ofstream data_file_;
    // std::string data_file_path_;  // Store the full path to the file
};

#endif // QUAD_DATA_RECORDER_H