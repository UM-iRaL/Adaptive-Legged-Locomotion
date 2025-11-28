#include "quad_utils/quad_data_recorder.h"

// Constructor
QuadDataRecorder::QuadDataRecorder(const std::string& filename) {
    // Get the path to the quad_utils package
    std::string pkg_path = ros::package::getPath("quad_utils");
    if (pkg_path.empty()) {
        throw std::runtime_error("quad_utils package not found!");
    }

    // Create the full file path (e.g., <quad_utils>/data/state_data/robot_data.json)
    // data_file_path_ = pkg_path + "/data/state_data/" + filename;
    
    std::filesystem::path pkg_dir(pkg_path);
    std::filesystem::path data_file_path = pkg_dir.parent_path() / "experiment_data" / filename;

    // Open the output file stream in write mode to overwrite the file
    data_file_.open(data_file_path, std::ios::out); 
    if (!data_file_.is_open()) {
        throw std::runtime_error("Unable to open or create file: " + data_file_path.string());
    }
}

// Method to save robot state data into JSON
void QuadDataRecorder::saveData(double time, 
                                const Eigen::VectorXd& body_state, 
                                std::optional<Eigen::VectorXd> ctrl, 
                                std::optional<Eigen::VectorXd> feet_location, 
                                std::optional<Eigen::VectorXd> global_plan) {
    // Create a JSON object
    nlohmann::json json_data;

    // time
    json_data["time"] = time;

    // Add body state to the JSON object
    json_data["body_state"] = std::vector<double>(body_state.data(), body_state.data() + body_state.size());

    // Add joint state if it exists
    if (ctrl.has_value()) {
        json_data["control"] = std::vector<double>(ctrl->data(), ctrl->data() + ctrl->size());
    } else {
        json_data["control"] = nullptr;  
    }

    // Add feet force if it exists
    if (feet_location.has_value()) {
        json_data["feet_location"] = std::vector<double>(feet_location->data(), feet_location->data() + feet_location->size());
    } else {
        json_data["feet_location"] = nullptr;  
    }

    // Add gloabl plan if it exists
    if (global_plan.has_value()) {
        json_data["global_plan"] = std::vector<double>(global_plan->data(), global_plan->data() + global_plan->size());
    } else {
        json_data["global_plan"] = nullptr;  
    }
    // Write the JSON object to the file
    data_file_ << json_data.dump() << std::endl;
}

// Destructor to close the file stream
QuadDataRecorder::~QuadDataRecorder() {
    if (data_file_.is_open()) {
        data_file_.close();
    }
}
