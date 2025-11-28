#include "local_planner/local_planner.h"

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

std::mutex compute_mutex;  // Mutex to protect shared data (if necessary)


LocalPlanner::LocalPlanner(ros::NodeHandle nh)
    : local_body_planner_nonlinear_(), local_footstep_planner_() {
    nh_ = nh;

    // Load rosparams from parameter server
    std::string terrain_map_topic, body_plan_topic, robot_state_topic,
        local_plan_topic, foot_plan_discrete_topic, foot_plan_continuous_topic,
        cmd_vel_topic, control_mode_topic;

    // Load system parameters from launch file (not in config file)
    quad_utils::loadROSParamDefault(nh_, "robot_type", robot_name_,
                                    std::string("go2"));
    quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic);
    quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
    quad_utils::loadROSParam(nh_, "topics/state/ground_truth",
                             robot_state_topic);
    quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
    quad_utils::loadROSParam(nh_, "topics/foot_plan_discrete",
                             foot_plan_discrete_topic);
    quad_utils::loadROSParam(nh_, "topics/foot_plan_continuous",
                             foot_plan_continuous_topic);
    quad_utils::loadROSParam(nh_, "topics/cmd_vel", cmd_vel_topic);
    quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
    quad_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);

    // Setup pubs and subs
    terrain_map_sub_ = nh_.subscribe(terrain_map_topic, 1,
                                     &LocalPlanner::terrainMapCallback, this);
    body_plan_sub_ = nh_.subscribe(body_plan_topic, 1,
                                   &LocalPlanner::robotPlanCallback, this);
    robot_state_sub_ =
        nh_.subscribe(robot_state_topic, 1, &LocalPlanner::robotStateCallback,
                      this, ros::TransportHints().tcpNoDelay(true));
    cmd_vel_sub_ =
        nh_.subscribe(cmd_vel_topic, 1, &LocalPlanner::cmdVelCallback, this);

    local_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(local_plan_topic, 1);
    foot_plan_discrete_pub_ = nh_.advertise<quad_msgs::MultiFootPlanDiscrete>(
        foot_plan_discrete_topic, 1);
    foot_plan_continuous_pub_ =
        nh_.advertise<quad_msgs::MultiFootPlanContinuous>(
            foot_plan_continuous_topic, 1);

    // Load system parameters from parameter server
    quad_utils::loadROSParam(nh_, "/local_planner/update_rate", update_rate_);
    quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);
    quad_utils::loadROSParam(nh_, "/local_planner/horizon_length", N_);
    quad_utils::loadROSParam(nh_, "/local_planner/desired_height", z_des_);
    quad_utils::loadROSParam(nh_, "/local_planner/toe_radius", toe_radius_);
    quad_utils::loadROSParam(nh_, "/local_planner/cmd_vel_x", cmd_vel_x_);
    quad_utils::loadROSParam(nh_, "/local_planner/cmd_vel_scale",
                             cmd_vel_scale_);
    quad_utils::loadROSParam(nh_, "/local_planner/last_cmd_vel_msg_time_max",
                             last_cmd_vel_msg_time_max_);
    quad_utils::loadROSParam(nh_, "/local_planner/cmd_vel_filter_const",
                             cmd_vel_filter_const_);
    quad_utils::loadROSParam(nh_, "/local_planner/stand_vel_threshold",
                             stand_vel_threshold_);
    quad_utils::loadROSParam(nh_, "/local_planner/stand_cmd_vel_threshold",
                             stand_cmd_vel_threshold_);
    quad_utils::loadROSParam(nh_, "/local_planner/stand_pos_error_threshold",
                             stand_pos_error_threshold_);

    // Load system parameters from launch file (not in config file)
    nh.param<bool>("local_planner/use_twist_input", use_twist_input_, false);

    // Convert kinematics
    quadKD_ = std::make_shared<quad_utils::QuadKD>();

    // // Test inertia matrix
    // Eigen::VectorXd state_positions(3 * num_feet_ + 6), state_velocities(3 * num_feet_ + 6), 
    //          ref_foot_acceleration(3 * num_feet_), grf_array(3 * num_feet_), tau_array(3 * num_feet_);
    // std::vector<int> contact_mode(num_feet_);
    // for (int i = 0; i < num_feet_; i++) {
    //   contact_mode[i] = 0;
    // }
    // quadKD_->computeInverseDynamics(state_positions, state_velocities, ref_foot_acceleration, grf_array, contact_mode, tau_array);

    // Load control type: Nominal-MPC, SSI-MPC, L1-MPC
    quad_utils::loadROSParam(nh_, "/control_type/type", control_type_);
    if (robot_name_ == "go2") {
        if (control_type_ == "rf") {
            rf_enable_ = true;
            l1_enable_ = false;
        } else if (control_type_ == "l1") {
            rf_enable_ = false;
            l1_enable_ = true;
        } else {
            rf_enable_ = false;
            l1_enable_ = false;
        }
    } else {
        control_type_ = "nominal";
        rf_enable_ = false;
        l1_enable_ = false;
    }


    // Initialize body and foot position arrays (grf_plan horizon is one index
    // shorter since control after last state is not in the horizon)
    ref_body_plan_ = Eigen::MatrixXd::Zero(N_, Nx_);
    foot_positions_world_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
    foot_velocities_world_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
    foot_accelerations_world_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
    foot_positions_body_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
    current_foot_positions_body_ = Eigen::VectorXd::Zero(num_feet_ * 3);
    current_foot_positions_world_ = Eigen::VectorXd::Zero(num_feet_ * 3);
    current_foot_velocities_world_ = Eigen::VectorXd::Zero(num_feet_ * 3);
    ref_primitive_plan_ = Eigen::VectorXi::Zero(N_);
    ref_ground_height_ = Eigen::VectorXd::Zero(N_);
    grf_plan_ = Eigen::MatrixXd::Zero(N_ - 1, 12);
    for (int i = 0; i < num_feet_; i++) {
        grf_plan_.col(3 * i + 2).fill(13.3 * 9.81 / num_feet_);
    }

    // Initialize body and footstep planners
    initLocalBodyPlanner();
    initLocalFootstepPlanner();

    // Initialize variables for learning unvertainty
    if (control_type_ == "rf") {
        initRfVariables();
        rf_pub_ = nh.advertise<geometry_msgs::Wrench>("/rf", 10, true);
        nominal_pub_ = nh.advertise<geometry_msgs::Wrench>("/nominal", 10, true);
    } else if (control_type_ == "l1") {
        initL1Variables();
        l1_sigma_pub_ = nh.advertise<geometry_msgs::Wrench>("/L1/sigma", 10, true);
        l1_sigma_filt_pub_ = nh.advertise<geometry_msgs::Wrench>("/L1/sigma_filt", 10, true);
        nominal_pub_ = nh.advertise<geometry_msgs::Wrench>("/nominal", 10, true);
    } 

    // Initialize data collector
    quad_utils::loadROSParam(nh_, "/control_type/record_data", record_data_);
    if (record_data_){
        std::string data_file_name;
        if (rf_enable_){
            data_file_name = robot_name_ + "_data_" + std::to_string(rf_n_rf_) + ".json"; 
        } else if (l1_enable_) {
            data_file_name = robot_name_ + "_data_l1.json"; 
        } else {
            data_file_name = robot_name_ + "_data_nominal.json"; 
        }
        data_collector = std::make_shared<QuadDataRecorder>(data_file_name);
    }

    // Initialize twist input variables
    cmd_vel_.resize(6);
    cmd_vel_.setZero();
    initial_timestamp_ = ros::Time::now();
    first_plan_ = true;

    // Initialize stand pose
    stand_pose_.fill(std::numeric_limits<double>::max());
    control_mode_ = STAND;

    // Initialize the time duration to the next plan index
    first_element_duration_ = dt_;

    // Initialize the plan index diff
    plan_index_diff_ = 0;

    // Initialize the plan index
    current_plan_index_ = 0;

    u_last_ = grf_plan_.row(1);
    last_feet_contact_ = {true, true, true, true};   // start with STAND => all feet in contact

    quad_utils::loadROSParam(nh_, "/experiment_params/add_force", add_force_);
    quad_utils::loadROSParam(nh_, "/experiment_params/fx", fx_);
    quad_utils::loadROSParam(nh_, "/experiment_params/fy", fy_);
    quad_utils::loadROSParam(nh_, "/experiment_params/fz", fz_);
    wrench_pub_ = nh.advertise<geometry_msgs::Wrench>("/apply_force/trunk", 10, true);

    if (robot_name_ != "go2") {
        add_force_ = false;
        fx_ = 0.0;
        fy_ = 0.0;
        fz_ = 0.0;        
    } 

    // special_timer_ = nh.createTimer(ros::Duration(5.0), &ControlNode::specialCallback, this);
    rf_alpha_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/robot_1/visualization/alpha", 10);

}

void LocalPlanner::initLocalBodyPlanner() {
  // Create nmpc wrapper class
  SystemID type;
  if (robot_name_ == "spirit" || robot_name_ == "spirit_rotors") {
    type = SPIRIT;
  } else if (robot_name_ == "a1") {
    type = A1;
  } else if (robot_name_ == "go2") {
    type = GO2;
    if (rf_enable_){
        type = GO2_RF;
    } else if (l1_enable_) {
        type = GO2_L1;
    } 
  } else {
    ROS_ERROR("WRONG ROBOT TYPE");
  }
  local_body_planner_nonlinear_ = std::make_shared<NMPCController>(nh_, type);
}

void LocalPlanner::initLocalFootstepPlanner() {
    // Load parameters from server
    double grf_weight, ground_clearance, hip_clearance,
        standing_error_threshold, period_d, foothold_search_radius,
        foothold_obj_threshold;
    std::string obj_fun_layer;
    int period;
    std::vector<double> duty_cycles, phase_offsets;
    quad_utils::loadROSParam(nh_, "/local_footstep_planner/grf_weight",
                             grf_weight);
    quad_utils::loadROSParam(nh_, "/local_footstep_planner/ground_clearance",
                             ground_clearance);
    quad_utils::loadROSParam(nh_, "local_footstep_planner/hip_clearance",
                             hip_clearance);
    quad_utils::loadROSParam(nh_,
                             "/local_footstep_planner/standing_error_threshold",
                             standing_error_threshold);
    quad_utils::loadROSParam(nh_,
                             "local_footstep_planner/foothold_search_radius",
                             foothold_search_radius);
    quad_utils::loadROSParam(nh_,
                             "/local_footstep_planner/foothold_obj_threshold",
                             foothold_obj_threshold);
    quad_utils::loadROSParam(nh_, "/local_footstep_planner/obj_fun_layer",
                             obj_fun_layer);
    quad_utils::loadROSParam(nh_, "/local_footstep_planner/period", period_d);
    quad_utils::loadROSParam(nh_, "/local_footstep_planner/duty_cycles",
                             duty_cycles);
    quad_utils::loadROSParam(nh_, "/local_footstep_planner/phase_offsets",
                             phase_offsets);

    period = period_d / dt_;

    // Confirm grf weight is valid
    if (grf_weight > 1 || grf_weight < 0) {
        grf_weight = std::min(std::max(grf_weight, 0.0), 1.0);
        ROS_WARN("Invalid grf weight, clamping to %4.2f", grf_weight);
    }

    // Create footstep class, make sure we use the same dt as the local planner
    local_footstep_planner_ = std::make_shared<LocalFootstepPlanner>();
    local_footstep_planner_->setTemporalParams(dt_, period, N_, duty_cycles,
                                               phase_offsets);
    local_footstep_planner_->setSpatialParams(
        ground_clearance, hip_clearance, standing_error_threshold, grf_weight,
        quadKD_, foothold_search_radius, foothold_obj_threshold, obj_fun_layer,
        toe_radius_);

    past_footholds_msg_.feet.resize(num_feet_);
}

void LocalPlanner::initRfVariables(){
    quad_utils::loadROSParam(nh_, "/rf_params/rf_size_target_mask", rf_size_target_mask_);
    quad_utils::loadROSParam(nh_, "/rf_params/rf_size_input_mask", rf_size_input_mask_);
    quad_utils::loadROSParam(nh_, "/rf_params/rf_n_rf", rf_n_rf_);
    quad_utils::loadROSParam(nh_, "/rf_params/rf_kernel_type", rf_kernel_type_);
    quad_utils::loadROSParam(nh_, "/rf_params/rf_kernel_std", rf_kernel_std_);
    quad_utils::loadROSParam(nh_, "/rf_params/rf_lr_trans", rf_lr_trans_);
    quad_utils::loadROSParam(nh_, "/rf_params/rf_lr_att", rf_lr_att_);

    std::cout << "************      RF param set up      ****************" << std::endl;
    std::cout << "rf parameters loaded from yaml are:" << std::endl;
    std::cout << "  rf ENABLE:             " << (rf_enable_ ? "True" : "False") << std::endl;
    std::cout << "  size_target_mask:      " << rf_size_target_mask_ << std::endl;
    std::cout << "  size_input_mask:      " << rf_size_input_mask_ << std::endl;
    std::cout << "  num_rff:               " << rf_n_rf_ << std::endl;
    std::cout << "  kernel_type:           " << rf_kernel_type_ << std::endl;
    std::cout << "  kernel std:            " << rf_kernel_std_ << std::endl;
    std::cout << "  learning_rate (trans): " << rf_lr_trans_ << std::endl;
    std::cout << "  learning_rate (att):   " << rf_lr_att_ << std::endl;


    if (rf_size_target_mask_ == 6) {
        rf_target_mask_.assign({6, 7, 8, 9, 10, 11});
    } else {
        ROS_ERROR("Error target state mask, the size should be 6");
    }

    if (rf_size_input_mask_ != 15) {
         ROS_ERROR("Error input state mask, the size should be 15");
    }

    rf_omega_.resize(rf_n_rf_, rf_size_input_mask_);

    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> gaussian_dist(0.0, rf_kernel_std_);
    for (int i = 0; i < rf_n_rf_; ++i) {
        for (int j = 0; j < rf_size_input_mask_; ++j) {
            rf_omega_(i, j) = gaussian_dist(gen);
        }
    }

    std::uniform_real_distribution<> uniform_dist(0.0,  2.0 * M_PI);
    rf_b_.resize(rf_n_rf_, 1);
    for (int i = 0; i < rf_n_rf_; ++i) {
        rf_b_(i, 0) = uniform_dist(gen);
    }

    rf_alpha_.resize(rf_size_target_mask_, rf_n_rf_);
    rf_alpha_.setZero();

    rf_alpha_last_.resize(rf_size_target_mask_, rf_n_rf_);
    rf_alpha_last_.setZero();

    rf_rf_.resize(rf_n_rf_, 1);
    rf_rf_.setZero();

    rf_lr_vec_.resize(Nx_/2, 1);
    rf_lr_vec_ << rf_lr_trans_, rf_lr_trans_, rf_lr_trans_, rf_lr_att_, rf_lr_att_, rf_lr_att_;

    mapEyefromMask(rf_Bh_, Nx_, rf_target_mask_);
    rf_Bh_.transposeInPlace();

    last_state_.resize(Nx_);
    last_state_.setZero();
    last_state_timestamp_ = ros::Time(0);
    is_last_plan_success_ = false;

    // std::cout << "alpha is: " << rf_alpha_.rows() << " x " << rf_alpha_.cols() << std::endl;
    // std::cout << "Z is: " << rf_alpha_.rows() << " x " << rf_alpha_.cols() << std::endl;
    // std::cout << "Omega is: " << rf_omega_.rows() << " x " << rf_omega_.cols() << std::endl;
    // std::cout << "Bh is: " << rf_Bh_.rows() << " x " << rf_Bh_.cols() << std::endl;
    // std::cout << "b is: " << rf_b_.rows() << " x " << rf_b_.cols() << std::endl;
    std::cout << "*******************************************************" << std::endl << std::endl;

    local_body_planner_nonlinear_ -> initParamRff(rf_alpha_, rf_omega_, rf_b_);
}


void LocalPlanner::initL1Variables(){
    quad_utils::loadROSParam(nh_, "/l1_params/l1_size_target_mask", l1_size_target_mask_);
    quad_utils::loadROSParam(nh_, "/l1_params/As_v", As_v_param_);
    quad_utils::loadROSParam(nh_, "/l1_params/As_w", As_w_param_);
    quad_utils::loadROSParam(nh_, "/l1_params/w_f", w_f_);
    quad_utils::loadROSParam(nh_, "/l1_params/w_m1", w_m1_);
    quad_utils::loadROSParam(nh_, "/l1_params/w_m2", w_m2_);

    std::cout << "************      L1 param set up      ****************" << std::endl;
    std::cout << "  l1 ENABLE:             " << (l1_enable_ ? "True" : "False") << std::endl;
    std::cout << "  size_target_mask:      " << l1_size_target_mask_ << std::endl;
    std::cout << "  As_v:                  " << As_v_param_ << std::endl;
    std::cout << "  As_w:                  " << As_w_param_ << std::endl;
    std::cout << "  w_f:                   " << w_f_ << std::endl;
    std::cout << "  w_m1:                  " << w_m1_ << std::endl;
    std::cout << "  w_m2:                  " << w_m2_ << std::endl;

    lpf_f_coef1_ = 0.0;
    lpf_f_coef2_ = 0.0;
    lpf_m1_coef1_ = 0.0;
    lpf_m1_coef2_ = 0.0;
    lpf_m2_coef1_ = 0.0;
    lpf_m2_coef2_ = 0.0;

    l1_size_f_ = l1_size_target_mask_/2;
    l1_size_m_ = l1_size_target_mask_/2;

    l1_f_m_ = Eigen::VectorXd::Zero(l1_size_target_mask_);
    l1_f_ = Eigen::VectorXd::Zero(l1_size_f_);
    l1_m_ = Eigen::VectorXd::Zero(l1_size_m_);
    As_v_ = Eigen::MatrixXd::Identity(l1_size_f_, l1_size_f_) * As_v_param_;
    As_w_ = Eigen::MatrixXd::Identity(l1_size_m_, l1_size_m_) * As_w_param_;
    As_ = Eigen::MatrixXd::Identity(l1_size_target_mask_, l1_size_target_mask_);
    As_.topLeftCorner(As_v_.rows(), As_v_.cols()) = As_v_;
    As_.bottomRightCorner(As_w_.rows(), As_w_.cols()) = As_w_;

    sigma_ = Eigen::VectorXd::Zero(l1_size_target_mask_);
    sigma_f_ = Eigen::VectorXd::Zero(l1_size_f_);
    sigma_m_ = Eigen::VectorXd::Zero(l1_size_m_);

    v_w_tilde_ = Eigen::VectorXd::Zero(l1_size_target_mask_);
    v_tilde_ = Eigen::VectorXd::Zero(l1_size_f_);
    w_tilde_ = Eigen::VectorXd::Zero(l1_size_m_);
    v_w_hat_ = Eigen::VectorXd::Zero(l1_size_target_mask_);
    v_hat_ = Eigen::VectorXd::Zero(l1_size_f_);
    w_hat_ = Eigen::VectorXd::Zero(l1_size_m_);
    v_w_hat_prev_ = Eigen::VectorXd::Zero(l1_size_target_mask_);
    v_hat_prev_ = Eigen::VectorXd::Zero(l1_size_f_);
    w_hat_prev_ = Eigen::VectorXd::Zero(l1_size_m_);

    sigma_filt_ = Eigen::VectorXd::Zero(l1_size_target_mask_);
    sigma_f_filt_ = Eigen::VectorXd::Zero(l1_size_f_);
    sigma_m_filt1_ = Eigen::VectorXd::Zero(l1_size_m_);
    sigma_m_filt2_ = Eigen::VectorXd::Zero(l1_size_m_);
    sigma_f_filt_prev_ = Eigen::VectorXd::Zero(l1_size_f_);
    sigma_m_filt1_prev_ = Eigen::VectorXd::Zero(l1_size_m_);
    sigma_m_filt2_prev_ = Eigen::VectorXd::Zero(l1_size_m_);

    l1_Bh_ = rf_Bh_;

    std::cout << "As_ is: " << std::endl << As_ << std::endl;
    std::cout << "As_v_ is: " << std::endl << As_v_ << std::endl;
    std::cout << "As_w_ is: " << std::endl << As_w_ << std::endl;
    std::cout << "exp(As_v_param_) is: " << exp(As_v_param_) << std::endl;
    std::cout << "exp(As_w_param_) is: " << exp(As_w_param_) << std::endl;
    std::cout << "****************************************************" << std::endl;

    local_body_planner_nonlinear_ -> initParamL1(sigma_filt_);
}

void LocalPlanner::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &msg) {
    grid_map::GridMapRosConverter::fromMessage(*msg, terrain_grid_);

    // Convert to FastTerrainMap structure for faster querying
    terrain_.loadDataFromGridMap(terrain_grid_);
    local_footstep_planner_->updateMap(terrain_);
    local_footstep_planner_->updateMap(terrain_grid_);
}

void LocalPlanner::robotPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr &msg) {
    body_plan_msg_ = msg;
}

void LocalPlanner::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg) {
    // Make sure the data is actually populated
    if (msg->feet.feet.empty() || msg->joints.position.empty()) return;

    robot_state_msg_ = msg;
}

void LocalPlanner::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    // Ignore non-planar components of desired twist
    cmd_vel_[0] = (1 - cmd_vel_filter_const_) * cmd_vel_[0] +
                  cmd_vel_filter_const_ * cmd_vel_scale_ * msg->linear.x;
    cmd_vel_[1] = (1 - cmd_vel_filter_const_) * cmd_vel_[1] +
                  cmd_vel_filter_const_ * cmd_vel_scale_ * msg->linear.y;
    cmd_vel_[2] = 0;
    cmd_vel_[3] = 0;
    cmd_vel_[4] = 0;
    cmd_vel_[5] = (1 - cmd_vel_filter_const_) * cmd_vel_[5] +
                  cmd_vel_filter_const_ * cmd_vel_scale_ * msg->angular.z;

    // Record when this was last reached for safety
    last_cmd_vel_msg_time_ = ros::Time::now();
}

void LocalPlanner::getReference() {
    if (first_plan_) {
        first_plan_ = false;
        past_footholds_msg_ = robot_state_msg_->feet;
        past_footholds_msg_.traj_index = current_plan_index_;
        for (int i = 0; i < num_feet_; i++) {
            past_footholds_msg_.feet[i].header = past_footholds_msg_.header;
            past_footholds_msg_.feet[i].traj_index =
                past_footholds_msg_.traj_index;
        }

        // We want to start from a full period when using twist input
        if (use_twist_input_) {
            initial_timestamp_ = ros::Time::now() - ros::Duration(1e-6);
        }
    }

    // Make sure we use the most recent global plan timestamp for reference
    if (!use_twist_input_) {
        initial_timestamp_ = body_plan_msg_->global_plan_timestamp;
    }

    // Tracking trajectory so enter run mode
    control_mode_ = STEP;

    // Get plan index, compare with the previous one to check if this is a
    // duplicated solve
    int previous_plan_index = current_plan_index_;
    quad_utils::getPlanIndex(initial_timestamp_, dt_, current_plan_index_,
                             first_element_duration_);
    plan_index_diff_ = current_plan_index_ - previous_plan_index;

    // Get the current body and foot positions into Eigen
    current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
    current_state_timestamp_ = robot_state_msg_->header.stamp;
    quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet,
                                         current_foot_positions_world_,
                                         current_foot_velocities_world_);
    local_footstep_planner_->getFootPositionsBodyFrame(
        current_state_, current_foot_positions_world_,
        current_foot_positions_body_);

    // Grab the appropriate states from the body plan and convert to an Eigen
    // matrix
    ref_body_plan_.setZero();
    ref_primitive_plan_.setZero();

    if (use_twist_input_) {
        // Use twist planner
        // Check that we have recent twist data, otherwise set cmd_vel to zero
        ros::Duration time_elapsed_since_msg =
            ros::Time::now() - last_cmd_vel_msg_time_;
        if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_) {
            cmd_vel_.setZero();
            ROS_WARN_THROTTLE(1.0,
                            "No cmd_vel data, setting twist cmd_vel to zero");
        }

        // Set initial ground height
        ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(
            current_state_(0), current_state_(1));

        // If it's not initialized, set to current positions
        if (stand_pose_(0) == std::numeric_limits<double>::max() &&
            stand_pose_(1) == std::numeric_limits<double>::max() &&
            stand_pose_(2) == std::numeric_limits<double>::max()) {
            stand_pose_ << current_state_[0], current_state_[1],
                current_state_[5];
        }

        // Set initial condition for forward integration
        Eigen::Vector2d support_center;
        support_center.setZero();
        for (int i = 0; i < num_feet_; i++) {
            support_center.x() +=
                robot_state_msg_->feet.feet[i].position.x / ((double)num_feet_);
            support_center.y() +=
                robot_state_msg_->feet.feet[i].position.y / ((double)num_feet_);
        }

        // Step if velocity commanded, current velocity exceeds threshold, or
        // too far from center of support
        bool is_stepping =
            (cmd_vel_.norm() > stand_cmd_vel_threshold_ ||
             current_state_.segment(6, 2).norm() > stand_vel_threshold_ ||
             (support_center - current_state_.segment(0, 2)).norm() >
                 stand_pos_error_threshold_);

        if (is_stepping) {
            control_mode_ = STEP;
            stand_pose_ << current_state_[0], current_state_[1],
                current_state_[5];
        } else {
            // If it's standing, try to stablized the waggling
            control_mode_ = STAND;
            Eigen::Vector3d current_stand_pose;
            current_stand_pose << support_center[0], support_center[1],
                current_state_[5];
            stand_pose_ = stand_pose_ * (1 - 1 / update_rate_) +
                          current_stand_pose * 1 / update_rate_;
        }

        ref_body_plan_(0, 0) = stand_pose_[0];  // support_center.x();
        ref_body_plan_(0, 1) = stand_pose_[1];  // support_center.x();
        ref_body_plan_(0, 2) = z_des_ + ref_ground_height_(0);
        ref_body_plan_(0, 3) = 0;
        ref_body_plan_(0, 4) = 0;
        ref_body_plan_(0, 5) = stand_pose_[2];
        ref_body_plan_(0, 6) = cmd_vel_[0] * cos(current_state_[5]) -
                            cmd_vel_[1] * sin(current_state_[5]);
        ref_body_plan_(0, 7) = cmd_vel_[0] * sin(current_state_[5]) +
                            cmd_vel_[1] * cos(current_state_[5]);
        ref_body_plan_(0, 8) = cmd_vel_[2];
        ref_body_plan_(0, 9) = cmd_vel_[3];
        ref_body_plan_(0, 10) = cmd_vel_[4];
        ref_body_plan_(0, 11) = cmd_vel_[5];

        // Alternatively only adaptive pitch
        // ref_body_plan_(0, 4) = local_footstep_planner_->getTerrainSlope(
        //     current_state_(0), current_state_(1), current_state_(6),
        //     current_state_(7));

        // Adaptive roll and pitch
        local_footstep_planner_->getTerrainSlope(
            ref_body_plan_(0, 0), ref_body_plan_(0, 1), ref_body_plan_(0, 5),
            ref_body_plan_(0, 3), ref_body_plan_(0, 4));

        // Integrate to get full body plan (Forward Euler)
        for (int i = 1; i < N_; i++) {
            Eigen::VectorXd current_cmd_vel = cmd_vel_;

            double yaw = ref_body_plan_(i - 1, 5);
            current_cmd_vel[0] =
                cmd_vel_[0] * cos(yaw) - cmd_vel_[1] * sin(yaw);
            current_cmd_vel[1] =
                cmd_vel_[0] * sin(yaw) + cmd_vel_[1] * cos(yaw);

            for (int j = 0; j < 6; j++) {
                if (i == 1) {
                    ref_body_plan_(i, j) =
                        ref_body_plan_(i - 1, j) +
                        current_cmd_vel[j] * first_element_duration_;
                } else {
                    ref_body_plan_(i, j) =
                        ref_body_plan_(i - 1, j) + current_cmd_vel[j] * dt_;
                }
                ref_body_plan_(i, j + 6) = (current_cmd_vel[j]);
            }

            ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(
                ref_body_plan_(i, 0), ref_body_plan_(i, 1));
            ref_body_plan_(i, 2) = z_des_ + ref_ground_height_(i);

            // Alternatively only adaptive pitch
            // ref_body_plan_(i, 4) = local_footstep_planner_->getTerrainSlope(
            //     ref_body_plan_(i, 0), ref_body_plan_(i, 1), ref_body_plan_(i,
            //     6), ref_body_plan_(i, 7));

            // Adaptive roll and pitch
            local_footstep_planner_->getTerrainSlope(
                ref_body_plan_(i, 0), ref_body_plan_(i, 1),
                ref_body_plan_(i, 5), ref_body_plan_(i, 3),
                ref_body_plan_(i, 4));
        }
    } else {
        // Use global plan
        for (int i = 0; i < N_; i++) {
            // If the horizon extends past the reference trajectory, just hold
            // the last state
            if (i + current_plan_index_ > body_plan_msg_->plan_indices.back()) {
                ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(
                    body_plan_msg_->states.back().body);
                if (i < N_) {
                    ref_primitive_plan_(i) =
                        body_plan_msg_->primitive_ids.back();
                }
            } else {
                ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(
                    body_plan_msg_->states[i + current_plan_index_].body);
                if (i < N_) {
                    ref_primitive_plan_(i) =
                        body_plan_msg_->primitive_ids[i + current_plan_index_];
                }
            }
            ref_body_plan_(i, 2) = z_des_ + ref_ground_height_(0);  //INFO: hardcode for now
            ref_body_plan_(i, 4) = 0.0;  //INFO: hardcode for now
            ref_body_plan_(i, 10) = 0.0;  //INFO: hardcode for now
            ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(
                ref_body_plan_(i, 0), ref_body_plan_(i, 1));
        }
        ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(
            current_state_(0), current_state_(1));

        // Stand if the plan has been tracked
        if ((current_state_ - ref_body_plan_.bottomRows(1).transpose())
                .norm() <= stand_pos_error_threshold_) {
            control_mode_ = STAND;
        }
    }

    // Update the body plan to use for foot planning
    int N_current_plan = body_plan_.rows();
    if (N_current_plan < N_) {
        // Cold start with reference plan
        body_plan_.conservativeResize(N_, 12);

        // Initialize with the current foot positions
        for (int i = N_current_plan; i < N_; i++) {
            body_plan_.row(i) = ref_body_plan_.row(i);
            foot_positions_body_.row(i) = current_foot_positions_body_;
            foot_positions_world_.row(i) = current_foot_positions_world_;
        }
    } else {
        // Only shift the foot position if it's a solve for a new plan index
        if (plan_index_diff_ > 0) {
            body_plan_.topRows(N_ - 1) = body_plan_.bottomRows(N_ - 1);
            grf_plan_.topRows(N_ - 2) = grf_plan_.bottomRows(N_ - 2);

            foot_positions_body_.topRows(N_ - 1) =
                foot_positions_body_.bottomRows(N_ - 1);
            foot_positions_world_.topRows(N_ - 1) =
                foot_positions_world_.bottomRows(N_ - 1);
        }
    }

    // Unwrap yaw to avoid discontinuity at -pi to pi (requires NLP constraint
    // bounds beyond [-pi, pi])
    unwrapYawReference();

    // Initialize with current foot and body positions
    body_plan_.row(0) = current_state_;
    foot_positions_body_.row(0) = current_foot_positions_body_;
    foot_positions_world_.row(0) = current_foot_positions_world_;
}

void LocalPlanner::unwrapYawReference() {
    static const int yaw_idx = 5;
    auto &&yaw_ref_traj = ref_body_plan_.col(yaw_idx);

    // Update first element of yaw reference to be within PI of current state
    math_utils::wrapToTarget(yaw_ref_traj(0), current_state_(yaw_idx));

    // Unwrap remainder of yaw reference to remove discontinuities (filtering
    // out differences > pi)
    math_utils::unwrapVector(yaw_ref_traj);
}

void LocalPlanner::mapEyefromMask(Eigen::MatrixXd &res, int dim, const std::vector<int>& mask) {
    // Create the identity matrix of size dim x dim
    Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(dim, dim);

    // Initialize result matrix with size mask.size() x dim
    res = Eigen::MatrixXd::Zero(mask.size(), dim); 

    // Fill res with the rows of the identity matrix indexed by mask
    for (size_t i = 0; i < mask.size(); ++i) {
        res.row(i) = identity_matrix.row(mask[i]);
    }
}

Eigen::VectorXd LocalPlanner::computeRMSE(const std::vector<Eigen::VectorXd>& x,
                            const std::vector<Eigen::VectorXd>& x_ref) {
    // Ensure both vectors have the same size
    if (x.size() != x_ref.size() || x.empty()) {
        throw std::invalid_argument("Vectors must have the same non-zero size");
    }

    Eigen::VectorXd rmse = Eigen::VectorXd::Zero(x[0].size());
    for (size_t i = 0; i < x.size(); ++i) {
        Eigen::VectorXd residual = x[i] - x_ref[i];
        rmse += residual.array().square().matrix();
    }

    // Mean and square root
    rmse = rmse / x.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}

bool LocalPlanner::isContactChanged() {
    bool is_contact_changed = false;
    std::vector<bool> current_feet_contact;

    // Ensure robot_state_msg_ is not null and feet array has enough elements
    if (!robot_state_msg_ || robot_state_msg_->feet.feet.size() < num_feet_) {
        ROS_WARN("Invalid robot_state_msg_ or incorrect number of feet.");
        return false;
    }

    // Populate current_feet_contact with the contact states of the feet
    for (int i = 0; i < num_feet_; ++i) {
        current_feet_contact.push_back(robot_state_msg_->feet.feet[i].contact);
    }

    // Compare current feet contacts with the last feet contacts
    if (current_feet_contact != last_feet_contact_) {
        is_contact_changed = true;
    }

    // Update last_feet_contact_
    last_feet_contact_ = current_feet_contact;

    return is_contact_changed;
}

void LocalPlanner::computeLegDynamic(const Eigen::VectorXd& x, const Eigen::VectorXd& feet_location, 
                                     Eigen::MatrixXd& M, Eigen::VectorXd& h, 
                                     Eigen::MatrixXd& J_u, Eigen::VectorXd& q_dot) {
    // Extracting variables from inputs
    double theta1 = x(3), theta2 = x(4), theta3 = x(5);
    double p_dot1 = x(6), p_dot2 = x(7), p_dot3 = x(8);
    double omega1 = x(9), omega2 = x(10), omega3 = x(11);

    // Precompute trigonometric functions
    double t2 = cos(theta1);
    double t3 = cos(theta2);
    double t4 = cos(theta3);
    double t5 = sin(theta1);
    double t6 = sin(theta2);
    double t7 = sin(theta3);

    if (M.size() > 0) {
        M.resize(6, 6);
        M << 1.33e+1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.33e+1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.33e+1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 7.89688e-2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, t2*4.0276351128e-1, t5*(-4.3173231128e-1),
            0.0, 0.0, 0.0, t6*(-7.89688e-2), t3*t5*4.0276351128e-1, t2*t3*4.3173231128e-1; 
    }
    // Vector h
    if (h.size() > 0) {
        h.resize(6);
        h << 0.0,
             0.0,
             1.30473e+2,
             omega2 * omega3 * 2.89688e-2,
             omega1 * (omega3 * t2 * 6.354822471802063e+15 + omega2 * t5 * 5.832966964260581e+15) * (-5.551115123125783e-17),
             omega2 * omega3 * t6 * (-2.896880000000002e-2) + omega1 * omega2 * t2 * t3 * 3.2379471128e-1 - omega1 * omega3 * t3 * t5 * 3.5276351128e-1;
    }

    // Jacobian J_u
    if (J_u.size() > 0) {
        J_u.resize(6, 12);
        J_u  << 
        1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
        -feet_location(1) * t6 - feet_location(2) * t3 * t7, feet_location(0) * t6 + feet_location(2) * t3 * t4, -t3 * (feet_location(1) * t4 - feet_location(0) * t7),
        -feet_location(4) * t6 - feet_location(5) * t3 * t7, feet_location(3) * t6 + feet_location(5) * t3 * t4, -t3 * (feet_location(4) * t4 - feet_location(3) * t7),
        -feet_location(7) * t6 - feet_location(8) * t3 * t7, feet_location(6) * t6 + feet_location(8) * t3 * t4, -t3 * (feet_location(7) * t4 - feet_location(6) * t7),
        -feet_location(10) * t6 - feet_location(11) * t3 * t7, feet_location(9) * t6 + feet_location(11) * t3 * t4, -t3 * (feet_location(10) * t4 - feet_location(9) * t7),
        -feet_location(2) * t4, -feet_location(2) * t7, feet_location(0) * t4 + feet_location(1) * t7,
        -feet_location(5) * t4, -feet_location(5) * t7, feet_location(3) * t4 + feet_location(4) * t7,
        -feet_location(8) * t4, -feet_location(8) * t7, feet_location(6) * t4 + feet_location(7) * t7,
        -feet_location(11) * t4, -feet_location(11) * t7, feet_location(9) * t4 + feet_location(10) * t7,        
        feet_location(1), -feet_location(0), 0.0, feet_location(4), -feet_location(3), 0.0,
        feet_location(7), -feet_location(6), 0.0, feet_location(10), -feet_location(9), 0.0;
    }

    // Generalized velocity q_dot
    if (q_dot.size() > 0) {
        double t8 = 1.0 / (t3 + 1E-6);
        q_dot.resize(6);
        q_dot << p_dot1,
                 p_dot2,
                 p_dot3,
                 t8 * (omega1 * t3 + omega3 * t2 * t6 + omega2 * t5 * t6),
                 omega2 * t2 - omega3 * t5,
                 t8 * (omega3 * t2 + omega2 * t5);
    }
}

void LocalPlanner::computeLegDynamicGo2(const Eigen::VectorXd& x, const Eigen::VectorXd& feet_location, 
                                     Eigen::MatrixXd& M, Eigen::VectorXd& h, 
                                     Eigen::MatrixXd& J_u, Eigen::VectorXd& q_dot) {
    // Extracting variables from inputs
    double theta1 = x(3), theta2 = x(4), theta3 = x(5);
    double p_dot1 = x(6), p_dot2 = x(7), p_dot3 = x(8);
    double omega1 = x(9), omega2 = x(10), omega3 = x(11);

    // Precompute trigonometric functions
    double t2 = cos(theta1);
    double t3 = cos(theta2);
    double t4 = cos(theta3);
    double t5 = sin(theta1);
    double t6 = sin(theta2);
    double t7 = sin(theta3);
    double t8 = omega1 * omega1;
    double t9 = omega2 * omega2;
    double t10 = omega3 * omega3;

    if (M.size() > 0) {
        M.resize(6, 6);
        // M << 1.6086e+1, 0.0, 0.0, 0.0, 0.0, 0.0,
        //     0.0, 1.6086e+1, 0.0, 0.0, 0.0, 0.0,
        //     0.0, 0.0, 1.6086e+1, 0.0, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 4.4294859e-2, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 0.0, t2*4.4084322384e-1, t5*(-4.695810828399999e-1),
        //     0.0, 0.0, 0.0, t6*(-4.4294859e-2), t3*t5*4.4084322384e-1, t2*t3*4.695810828399999e-1; 
        M << 1.6086e+1, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.6086e+1, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.6086e+1, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 4.4294859e-2, 1.2166e-4, 1.4849e-3,
             0.0, 0.0, 0.0, t2*1.2166e-4-t5*1.4849e-3, t2*4.4084322384e-1+t5*3.12e-5, t2*(-3.12e-5)-t5*4.695810828399999e-1,
             0.0, 0.0, 0.0, t6*(-4.4294859e-2)+t2*t3*1.4849e-3+t3*t5*1.2166e-4, t6*(-1.2166e-4)-t2*t3*3.12e-5+t3*t5*4.4084322384e-1, t6*(-1.4849e-3)+t2*t3*4.695810828399999e-1-t3*t5*3.12e-5; 
    }
    // Vector h
    if (h.size() > 0) {
        h.resize(6);
        // h << 0.0,
        //      0.0,
        //      1.5780366e+2,
        //      omega2 * omega3 * 2.873785899999992e-2,
        //      omega1 * (omega3 * t2 * 6.129020413477212e+16 + omega2 * t5 * 5.714864218008971e+16) * (-6.938893903907228e-18),
        //      omega2 * omega3 * t6 * (-2.873785899999992e-2) + omega1 * omega2 * t2 * t3 * 3.9654836484e-1 - omega1 * omega3 * t3 * t5 * 4.252862238399999e-1;
        h << 0.0,
             0.0,
             1.5780366e+2,
             omega2 * (omega1 * 1.4849e-3 - omega2 * 3.12e-5 + omega3 * 4.695810828399999e-1) -omega3 * (omega1 * 1.2166e-4 + omega2 * 4.4084322384e-1 - omega3 * 3.12e-5),
             t2*t8*(-1.4849e-3)+t2*t10*1.4849e-3-t5*t8*1.2166e-4+t5*t9*1.2166e-4+omega1*omega2*t2*3.12e-5 + omega1*omega3*t2*(-4.252862238399999e-1)+omega2*omega3*t2*1.2166e-4-omega1*omega2*t5*3.9654836484e-1+omega1*omega3*t5*3.12e-5 + omega2*omega3*t5*1.4849e-3,
             t6*t9*3.12e-5-t6*t10*3.12e-5-omega1*omega2*t6*1.4849e-3+omega1*omega3*t6*1.2166e-4-omega2*omega3*t6*2.873785899999992e-2 + t2*t3*t8*1.2166e-4-t2*t3*t9*1.2166e-4-t3*t5*t8*1.4849e-3+t3*t5*t10*1.4849e-3+omega1*omega2*t2*t3*3.9654836484e-1-omega1*omega3*t2*t3*3.12e-5-omega2*omega3*t2*t3*1.4849e-3+omega1*omega2*t3*t5*3.12e-5+omega1*omega3*t3*t5*(-4.252862238399999e-1)+omega2*omega3*t3*t5*1.2166e-4;
    }
       
    // Jacobian J_u
    if (J_u.size() > 0) {
        J_u.resize(6, 12);
        J_u  << 
        1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
        -feet_location(1) * t6 - feet_location(2) * t3 * t7, feet_location(0) * t6 + feet_location(2) * t3 * t4, -t3 * (feet_location(1) * t4 - feet_location(0) * t7),
        -feet_location(4) * t6 - feet_location(5) * t3 * t7, feet_location(3) * t6 + feet_location(5) * t3 * t4, -t3 * (feet_location(4) * t4 - feet_location(3) * t7),
        -feet_location(7) * t6 - feet_location(8) * t3 * t7, feet_location(6) * t6 + feet_location(8) * t3 * t4, -t3 * (feet_location(7) * t4 - feet_location(6) * t7),
        -feet_location(10) * t6 - feet_location(11) * t3 * t7, feet_location(9) * t6 + feet_location(11) * t3 * t4, -t3 * (feet_location(10) * t4 - feet_location(9) * t7),      
        -feet_location(2) * t4, -feet_location(2) * t7, feet_location(0) * t4 + feet_location(1) * t7,
        -feet_location(5) * t4, -feet_location(5) * t7, feet_location(3) * t4 + feet_location(4) * t7,
        -feet_location(8) * t4, -feet_location(8) * t7, feet_location(6) * t4 + feet_location(7) * t7,
        -feet_location(11) * t4, -feet_location(11) * t7, feet_location(9) * t4 + feet_location(10) * t7,       
        feet_location(1), -feet_location(0), 0.0, feet_location(4), -feet_location(3), 0.0,
        feet_location(7), -feet_location(6), 0.0, feet_location(10), -feet_location(9), 0.0;
    }

    // Generalized velocity q_dot
    if (q_dot.size() > 0) {
        double t8 = 1.0 / (t3 + 1E-9);
        q_dot.resize(6);
        q_dot << p_dot1,
                 p_dot2,
                 p_dot3,
                 t8 * (omega1 * t3 + omega3 * t2 * t6 + omega2 * t5 * t6),
                 omega2 * t2 - omega3 * t5,
                 t8 * (omega3 * t2 + omega2 * t5);
    }
}


Eigen::MatrixXd LocalPlanner::computeJu(const Eigen::VectorXd &x0, 
                                        const Eigen::VectorXd &feet_location){
        Eigen::MatrixXd M;;
        
        Eigen::VectorXd h;

        Eigen::VectorXd q_dot;

        Eigen::MatrixXd J_u(6, 12);
        J_u.setZero();

        M.resize(0, 0);
        h.resize(0);
        q_dot.resize(0);

        // Compute leg dynamics
        // computeLegDynamic(x0, feet_location, M, h, J_u, q_dot);
        if(robot_name_ == "go2"){
            computeLegDynamicGo2(x0, feet_location, M, h, J_u, q_dot);
        }else{
            computeLegDynamic(x0, feet_location, M, h, J_u, q_dot);
        }

        return J_u;                        
}

void LocalPlanner::collectData(){
    std::lock_guard<std::mutex> lock(compute_mutex);  // Acquire lock for thread-safe access
    //save data if it is running(trakcing the path or commanded)
    // if (control_mode_ == STEP) {
    if (true) {
        data_collector->saveData(current_state_timestamp_.toSec(), 
                                    last_state_,
                                    grf_plan_.row(0),
                                    current_foot_positions_world_,
                                    ref_body_plan_.row(1)
                                    );
    }

}

void LocalPlanner::applyForce() {
    geometry_msgs::Wrench wrench_msg;
    wrench_msg.force.x = -fx_ * 9.81;
    wrench_msg.force.y = -fy_ * 9.81;
    wrench_msg.force.z = -fz_ * 9.81; 
    wrench_msg.torque.x = 0.0;
    wrench_msg.torque.y = 0.0;
    wrench_msg.torque.z = 0.0;

    // Wait for Gazebo to subscribe, but add a timeout (optional)
    ros::Time start_time = ros::Time::now();
    while (wrench_pub_.getNumSubscribers() == 0 && ros::ok()) {
        ROS_WARN("Waiting for subscribers on /apply_force/trunk...");
        ros::Duration(0.1).sleep();

        // Add a timeout to avoid infinite waiting
        if ((ros::Time::now() - start_time).toSec() > 5.0) {
            ROS_WARN("No subscribers found after 5 seconds, publishing anyway");
            break;
        }
    }

    wrench_pub_.publish(wrench_msg);
    ROS_INFO("Published wrench to /apply_force/trunk");
}

void LocalPlanner::publish_uncertainty_wrench(const Eigen::VectorXd &wrench,
                                              std::string wrench_topic) {
    geometry_msgs::Wrench wrench_msg;
    wrench_msg.force.x = wrench(0);
    wrench_msg.force.y = wrench(1);
    wrench_msg.force.z = wrench(2); 
    wrench_msg.torque.x = wrench(3);
    wrench_msg.torque.y = wrench(4);
    wrench_msg.torque.z = wrench(5);

    if (wrench_topic == "l1_sigma") {
        l1_sigma_pub_.publish(wrench_msg);
    } else if (wrench_topic == "l1_sigma_filt") {
        l1_sigma_filt_pub_.publish(wrench_msg);
    } else if (wrench_topic == "rf") {
        rf_pub_.publish(wrench_msg);
    } else if (wrench_topic == "nominal") {
        nominal_pub_.publish(wrench_msg);
    } else {
        ROS_WARN("Wrong topic for publishing wrench!");
    }
    
}


Eigen::VectorXd LocalPlanner::computeStatePredError(const Eigen::VectorXd &x0, 
                                        const Eigen::VectorXd &u, 
                                        const Eigen::VectorXd &x1, 
                                        const Eigen::VectorXd &feet_location, 
                                        double dt_local,
                                        const Eigen::MatrixXd &alpha,
                                        const Eigen::MatrixXd &rf) {
        Eigen::MatrixXd M(6, 6);;
        
        Eigen::VectorXd h(6);
        Eigen::MatrixXd J_u(6, 12);
        Eigen::VectorXd q_dot(6);
        M.setZero();
        J_u.setZero();
        h.setZero();
        q_dot.setZero();

        // Compute leg dynamics
        if(robot_name_ == "go2"){
            computeLegDynamicGo2(x0, feet_location, M, h, J_u, q_dot);
        }else{
            computeLegDynamic(x0, feet_location, M, h, J_u, q_dot);
        }
        // computeLegDynamic(x0, feet_location, M, h, J_u, q_dot);

        Eigen::VectorXd PredError = x0;
        PredError.setZero();
        PredError << (x1.head(Nx_ / 2) - x0.head(Nx_ / 2)) - q_dot * dt_local,
                        M * (x1.tail(Nx_ / 2) - x0.tail(Nx_ / 2))  + (h - J_u * u) * dt_local;
        publish_uncertainty_wrench(PredError.tail(Nx_ / 2)/dt_local, "nominal");

        // std::cout << "dynamic error: " << PredError.tail(Nx_/2).transpose() << std::endl;

        // std_msgs::Float32MultiArray error_msg;
        // for (int i = 0; i < PredError.size(); ++i)
        // {
        //     error_msg.data.push_back(PredError(i));
        // }
        // dynamic_error_pub_.publish(error_msg);
        // std::cout << "error euler no rf /dt: " << std::endl << PredError/dt_local << std::endl<< std::endl;
        // std::cout << "H is: " << std::endl << h << std::endl<< std::endl;
        // std::cout << "J_u is: " << std::endl << J_u << std::endl<< std::endl;
        // std::cout << "u is: " << std::endl << u << std::endl<< std::endl;
        // std::cout << "J_u * u is: " << std::endl << J_u * u << std::endl;

        Eigen::VectorXd local_residual = alpha * rf;  
        // std::cout << "local_residual is: " << std::endl << local_residual << std::endl;
        publish_uncertainty_wrench(local_residual, "rf");

        std::vector<double> local_residual_vector(local_residual.data(), local_residual.data() + local_residual.size());

        // Convert std::vector<double> to std::vector<float>
        std::vector<float> local_residual_vector_float(local_residual_vector.begin(), local_residual_vector.end());

        // Assign to ROS message
        rf_alpha_msg_.data = local_residual_vector_float;
        PredError.segment(6, rf_size_target_mask_) = PredError.segment(6, rf_size_target_mask_)  - alpha * rf * dt_local;
        
        // std::cout << "dynamic error with rf: " << PredError.tail(Nx_/2).transpose() << std::endl;

        // std::cout << "error with rf: " << alpha * rf * dt_local << std::endl;
        // std::cout << "alpha: " << alpha << std::endl;
        // std::cout << "rff: " << rf  << std::endl;

        // dynamic_error_pub_ = nh.advertise<std_msgs::Int32MultiArray>("rf_dynamic_error", 1);
        // dynamic_residual_pub_ 
        // std::cout << "inner J_u" << std::endl << J_u << std::endl;
        return PredError;
}

void LocalPlanner::computeRFF(){
    std::lock_guard<std::mutex> lock(compute_mutex);  // Acquire lock for thread-safe access
    double dt_computed = (current_state_timestamp_ - last_state_timestamp_).toSec();
    // std::cout << "dt_computed: " << dt_computed << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    if (last_state_.size() != 0 && 
        !isContactChanged() &&
        dt_computed > 1e-5 && 
        is_last_plan_success_){

        if (rf_lr_trans_ == 0.0 && rf_lr_att_ == 0.0){
            rf_alpha_.setZero();
            local_body_planner_nonlinear_ -> updateRffAlpha(rf_alpha_);
        }
        else{
            Eigen::VectorXd u_last_normalized = u_last_/100;
            Eigen::VectorXd rf_Z(rf_size_input_mask_);
            Eigen::VectorXd grf_pos = last_foot_positions_body_;
            Eigen::MatrixXd J_u = computeJu(last_state_, -grf_pos);

            rf_Z << last_state_.tail(9), J_u * u_last_normalized;
            rf_rf_ = (1.0 / std::sqrt(rf_n_rf_)) * (rf_omega_* rf_Z + rf_b_).array().cos();

            // std::cout << "*******************************************************" << std::endl;
            // std::cout << "rf_omega_: " << rf_omega_.rows() << ", " << rf_omega_.cols() << std::endl;
            // std::cout << "rf_Z.size(): " << rf_Z.size() << std::endl;
            // std::cout << "rf_Z: " << rf_Z.transpose() << std::endl;
            // std::cout << "last_state_: " << last_state_.transpose() << std::endl;
            // std::cout << "J_u" << std::endl << J_u << std::endl;
            // std::cout << "J_u * u_last_normalized: " << (J_u * u_last_normalized).transpose() << std::endl;
            // std::cout << "grf in rrf at " << current_state_timestamp_ << std::endl;
            // for (int i = 0; i < 12; ++i){
            //     std::cout << last_foot_positions_body_(i) << " | "<< grf_pos(i) << std::endl;
            // }

            Eigen::VectorXd error = Eigen::VectorXd::Zero(Nx_);
            error = computeStatePredError(last_state_, 
                                        u_last_,
                                        current_state_,
                                        -grf_pos,   //in parameter is negative 
                                        dt_computed, 
                                        rf_alpha_, 
                                        rf_rf_);

            Eigen::VectorXd error_pred = (rf_Bh_.transpose() * (error)) / dt_computed;
            rf_alpha_ += 2.0 * rf_lr_vec_.asDiagonal() * (error_pred * rf_rf_.transpose());
            local_body_planner_nonlinear_ -> updateRffAlpha(rf_alpha_);

            // std::cout << "current state: " << current_state_ << std::endl;
            // std::cout << "u_last: " << u_last_ << std::endl;
            // std::cout << "u_last_normalized: " << u_last_normalized << std::endl;
            // std::cout << "error size: " << error.size() << std::endl;
            // std::cout << "error: " << error << std::endl;
            // std::cout << "error_pred: " << error_pred << std::endl;
            // std::cout << "alpha norm: " << rf_alpha_.norm() << std::endl;
            // std::cout << "alpha: " << rf_alpha_ << std::endl;
            // std::cout << "pred residual is : " << rf_alpha_* rf_rf_<< std::endl;
            // std::cout << "*******************************************************" << std::endl;
        }
    } 
    // End time measurement
    auto end = std::chrono::high_resolution_clock::now();
        
    // Calculate the duration
    std::chrono::duration<double> duration = end - start;
    
    // Output the duration in seconds
    // std::cout << "computeRFF() took " << duration.count() << " seconds to execute." << std::endl;
}


bool LocalPlanner::computeL1Sigma(const Eigen::VectorXd &x0, 
                                        const Eigen::VectorXd &u, 
                                        const Eigen::VectorXd &x1, 
                                        const Eigen::VectorXd &feet_location, 
                                        double dt_local) {
        Eigen::MatrixXd M(6, 6);;
        
        Eigen::VectorXd h(6);
        Eigen::MatrixXd J_u(6, 12);
        Eigen::VectorXd q_dot(6);
        M.setZero();
        J_u.setZero();
        h.setZero();
        q_dot.setZero();

        // Compute leg dynamics
        if(robot_name_ == "go2"){
            computeLegDynamicGo2(x0, feet_location, M, h, J_u, q_dot);
        }else{
            computeLegDynamic(x0, feet_location, M, h, J_u, q_dot);
        }

        // Quadrupe dynamics with L1 
        // (x1.head(Nx_ / 2) - x0.head(Nx_ / 2)) - q_dot * dt_local,
        // M * (x1.tail(Nx_ / 2) - x0.tail(Nx_ / 2))  + (h - J_u * u) * dt_local - sigma * dt_local;

        Eigen::VectorXd v_w_hat_dot = Eigen::VectorXd::Zero(l1_size_target_mask_);
        Eigen::PartialPivLU<Eigen::MatrixXd> lu_M(M);
        Eigen::MatrixXd M_inv = lu_M.inverse();
        // std::cout << "M_inv: " << M_inv << std::endl;
        // std::cout << "M: " << M << std::endl;
        // std::cout << "M_inv * M: " << M_inv * M << std::endl;

        v_w_hat_dot << M_inv * (- h + J_u * u + sigma_) + As_ * v_w_tilde_;
        v_w_hat_ = v_w_hat_prev_ + v_w_hat_dot * dt_local;
        v_w_tilde_ = v_w_hat_ - x1.tail(Nx_ / 2);

        // std::cout << "dt_local: " << dt_local << std::endl;
        // std::cout << "v_w_dot: " << ((x1.tail(Nx_/2)-x0.tail(Nx_/2))/dt_local).transpose() << std::endl;
        // std::cout << "v_w_hat_dot: " << v_w_hat_dot.transpose() << std::endl;
        // std::cout << "v_w_hat_: " << v_w_hat_.transpose() << std::endl;
        // std::cout << "v_w_hat_prev_: " << v_w_hat_prev_.transpose() << std::endl;
        // std::cout << "v_w_tilde_: " << v_w_tilde_.transpose() << std::endl;
        // std::cout << "M_v_w_tilde_: " << (-M*v_w_tilde_).transpose() << std::endl;

        Eigen::VectorXd nominal_residual = M * (x1.tail(Nx_ / 2) - x0.tail(Nx_ / 2)) / dt_local + (h - J_u * u);
        // std::cout << "nominal_residual: " << nominal_residual.transpose() << std::endl;
        publish_uncertainty_wrench(nominal_residual, "nominal");

        Eigen::MatrixXd exp_As_dt_v = Eigen::MatrixXd::Identity(l1_size_f_, l1_size_f_) * exp(As_v_param_ * dt_local);
        Eigen::MatrixXd exp_As_dt_w = Eigen::MatrixXd::Identity(l1_size_m_, l1_size_m_) * exp(As_w_param_ * dt_local);
        
        Eigen::MatrixXd exp_As_dt_v_sub_I = exp_As_dt_v - Eigen::MatrixXd::Identity(l1_size_f_, l1_size_f_);
        Eigen::VectorXd inv_diag_v = exp_As_dt_v_sub_I.diagonal().array().inverse();
        Eigen::MatrixXd exp_As_dt_v_sub_I_inv = inv_diag_v.asDiagonal();

        Eigen::MatrixXd exp_As_dt_w_sub_I = exp_As_dt_w - Eigen::MatrixXd::Identity(l1_size_m_, l1_size_m_);
        Eigen::VectorXd inv_diag_w = exp_As_dt_w_sub_I.diagonal().array().inverse();
        Eigen::MatrixXd exp_As_dt_w_sub_I_inv = inv_diag_w.asDiagonal();

        // std::cout << "exp(As_v_param_ * dt_local): " << exp(As_v_param_ * dt_local) << std::endl;
        // std::cout << "exp_As_dt_v: " << std::endl << exp_As_dt_v << std::endl;
        // std::cout << "exp_As_dt_v_sub_I: " << std::endl << exp_As_dt_v_sub_I << std::endl;
        // std::cout << "inv_diag_v: " << std::endl << inv_diag_v << std::endl;
        // std::cout << "exp_As_dt_v_sub_I_inv: " << std::endl << exp_As_dt_v_sub_I_inv << std::endl;

        // std::cout << "exp(As_w_param_ * dt_local): " << exp(As_w_param_ * dt_local) << std::endl;
        // std::cout << "exp_As_dt_w: " << std::endl << exp_As_dt_w << std::endl;
        // std::cout << "exp_As_dt_w_sub_I: " << std::endl << exp_As_dt_w_sub_I << std::endl;
        // std::cout << "inv_diag_w: " << std::endl << inv_diag_w << std::endl;
        // std::cout << "exp_As_dt_w_sub_I_inv: " << std::endl << exp_As_dt_w_sub_I_inv << std::endl;

        Eigen::VectorXd PhiInvmu_v = - exp_As_dt_v_sub_I_inv * As_v_ * exp_As_dt_v * v_w_tilde_.head(l1_size_f_);
        Eigen::VectorXd PhiInvmu_w = - exp_As_dt_w_sub_I_inv * As_w_ * exp_As_dt_w * v_w_tilde_.tail(l1_size_m_);

        sigma_.head(l1_size_f_) = PhiInvmu_v;
        sigma_.tail(l1_size_m_) = PhiInvmu_w;
        // std::cout << "M_inv * sigma_: " << sigma_.transpose() << std::endl;

        sigma_ = M * sigma_;
        sigma_f_ = sigma_.head(l1_size_f_);
        sigma_m_ = sigma_.tail(l1_size_m_);
        // std::cout << "sigma_: " << sigma_.transpose() << std::endl;
        publish_uncertainty_wrench(sigma_, "l1_sigma");

        lpf_f_coef1_ = exp(-w_f_*dt_local); lpf_f_coef2_ = 1.0 - lpf_f_coef1_;
        lpf_m1_coef1_ = exp(-w_m1_*dt_local); lpf_m1_coef2_ = 1.0 - lpf_m1_coef1_;       
        lpf_m2_coef1_ = exp(-w_m2_*dt_local); lpf_m2_coef2_ = 1.0 - lpf_m2_coef1_;     
        // std::cout << "lpf_f_coef1_, lpf_f_coef2_: " << lpf_f_coef1_ << ", " << lpf_f_coef2_ << std::endl;
        // std::cout << "lpf_m1_coef1_, lpf_m1_coef2_: " << lpf_m1_coef1_ << ", " << lpf_m1_coef2_ << std::endl;
        // std::cout << "lpf_m2_coef1_, lpf_m2_coef2_: " << lpf_m2_coef1_ << ", " << lpf_m2_coef2_ << std::endl;

        sigma_f_filt_ = lpf_f_coef1_ * sigma_f_filt_prev_ + lpf_f_coef2_ * sigma_f_;
        sigma_m_filt1_ = lpf_m1_coef1_ * sigma_m_filt1_prev_ + lpf_m1_coef2_ * sigma_m_;
        sigma_m_filt2_ = lpf_m2_coef1_ * sigma_m_filt2_prev_ + lpf_m2_coef2_ * sigma_m_filt1_;

        sigma_filt_.head(l1_size_f_) = sigma_f_filt_;
        sigma_filt_.tail(l1_size_m_) = sigma_m_filt2_;
        // std::cout << "sigma_filt_: " << sigma_filt_.transpose() << std::endl;
        publish_uncertainty_wrench(sigma_filt_, "l1_sigma_filt");
        
        v_w_hat_prev_ = v_w_hat_;
        sigma_f_filt_prev_ = sigma_f_filt_;
        sigma_m_filt1_prev_ = sigma_m_filt1_;
        sigma_m_filt2_prev_ = sigma_m_filt2_;

        return sigma_filt_.array().isInf().any();
}

void LocalPlanner::computeL1(){
    std::lock_guard<std::mutex> lock(compute_mutex);  // Acquire lock for thread-safe access
    double dt_computed = (current_state_timestamp_ - last_state_timestamp_).toSec();
    // double dt_computed = 0.002;
    auto start = std::chrono::high_resolution_clock::now();

    if (last_state_.size() != 0 && 
        !isContactChanged() &&
        dt_computed > 1e-5 && 
        is_last_plan_success_){
            // std::cout << "************************** compute L1 **************************" << std::endl;
            Eigen::VectorXd grf_pos = last_foot_positions_body_;
            bool l1_fail = computeL1Sigma(last_state_, u_last_, current_state_, -grf_pos, dt_computed);
            if (!l1_fail){
                local_body_planner_nonlinear_ -> updateL1Sigma(sigma_filt_);
            } else {
                ROS_ERROR("L1 update fails!");
            }
    } 

    // End time measurement
    auto end = std::chrono::high_resolution_clock::now();
        
    // Calculate the duration
    std::chrono::duration<double> duration = end - start;
    
    // Output the duration in seconds
    // std::cout << "computeL1() took " << duration.count() << " seconds to execute." << std::endl;
}

bool LocalPlanner::computeLocalPlan() {
    if (terrain_.isEmpty() || body_plan_msg_ == NULL && !use_twist_input_ ||
        robot_state_msg_ == NULL) {
        ROS_WARN_STREAM(
            "ComputeLocalPlan function did not recieve the expected inputs");
        return false;
    }

    // Start the timer
    quad_utils::FunctionTimer timer(__FUNCTION__);

    // Compute the contact schedule
    local_footstep_planner_->computeContactSchedule(
        current_plan_index_, body_plan_, ref_primitive_plan_, control_mode_,
        contact_schedule_);

    // Compute the new footholds if we have a valid existing plan (i.e. if
    // grf_plan is filled)
    local_footstep_planner_->computeFootPlan(
        current_plan_index_, contact_schedule_, body_plan_, grf_plan_,
        ref_body_plan_, current_foot_positions_world_,
        current_foot_velocities_world_, first_element_duration_,
        past_footholds_msg_, foot_positions_world_, foot_velocities_world_,
        foot_accelerations_world_);

    // Transform the new foot positions into the body frame for body planning
    local_footstep_planner_->getFootPositionsBodyFrame(
        body_plan_, foot_positions_world_, foot_positions_body_);

    // Compute grf position considering the toe radius
    Eigen::MatrixXd grf_positions_body = foot_positions_body_;
    Eigen::MatrixXd grf_positions_world = foot_positions_world_;
    for (size_t i = 0; i < 4; i++) {
        grf_positions_body.col(3 * i + 2) =
            foot_positions_body_.col(3 * i + 2).array() - toe_radius_;
        grf_positions_world.col(3 * i + 2) =
            foot_positions_world_.col(3 * i + 2).array() - toe_radius_;
    }

    Eigen::VectorXd current_full_state(36), joint_pos(12), joint_vel(12);
    current_full_state.segment(0, 12) = current_state_;
    quad_utils::vectorToEigen(robot_state_msg_->joints.position, joint_pos);
    quad_utils::vectorToEigen(robot_state_msg_->joints.velocity, joint_vel);
    current_full_state.segment(12, 12) = joint_pos;
    current_full_state.segment(24, 12) = joint_vel;

    last_foot_positions_body_ = grf_positions_body.row(0);

    // Compute leg plan with MPC, return if solve fails
    if (!local_body_planner_nonlinear_->computeLegPlan(
            current_full_state, ref_body_plan_, grf_positions_body,
            grf_positions_world, foot_velocities_world_, contact_schedule_,
            ref_ground_height_, first_element_duration_, plan_index_diff_,
            terrain_grid_, body_plan_, grf_plan_))
        return false;

    N_current_ = body_plan_.rows();
    foot_positions_world_ = grf_positions_world;
    for (size_t i = 0; i < 4; i++) {
        foot_positions_world_.col(3 * i + 2) =
            foot_positions_world_.col(3 * i + 2).array() + toe_radius_;
    }

    // Record computation time and update exponential filter
    compute_time_ = 1000.0 * timer.reportSilent();
    mean_compute_time_ = (filter_smoothing_constant_)*mean_compute_time_ +
                         (1 - filter_smoothing_constant_) * compute_time_;
    ROS_INFO_THROTTLE(0.1, "LocalPlanner took %5.3f ms", compute_time_);

    // Return true if made it this far
    return true;
}

void LocalPlanner::publishLocalPlan() {
    // Create messages to publish
    quad_msgs::RobotPlan local_plan_msg;
    quad_msgs::MultiFootPlanDiscrete future_footholds_msg;
    quad_msgs::MultiFootPlanContinuous foot_plan_msg;

    // Update the headers of all messages
    local_plan_msg.header.stamp = current_state_timestamp_;
    local_plan_msg.header.frame_id = map_frame_;
    local_plan_msg.global_plan_timestamp = initial_timestamp_;
    local_plan_msg.compute_time = compute_time_;
    future_footholds_msg.header = local_plan_msg.header;
    foot_plan_msg.header = local_plan_msg.header;

    // Add NLP diagnostic information
    local_body_planner_nonlinear_->getNLPDiagnostics().loadDiagnosticsMsg(
        local_plan_msg.diagnostics);

    // Compute the discrete and continuous foot plan messages
    local_footstep_planner_->loadFootPlanMsgs(
        contact_schedule_, current_plan_index_, first_element_duration_,
        foot_positions_world_, foot_velocities_world_,
        foot_accelerations_world_, future_footholds_msg, foot_plan_msg);

    // Add body, foot, joint, and grf data to the local plan message
    for (int i = 0; i < N_current_ - 1; i++) {
        // Add the state information
        quad_msgs::RobotState robot_state_msg;
        robot_state_msg.body =
            quad_utils::eigenToBodyStateMsg(body_plan_.row(i));
        robot_state_msg.feet = foot_plan_msg.states[i];
        quad_utils::ikRobotState(*quadKD_, robot_state_msg);

        // Add the GRF information
        quad_msgs::GRFArray grf_array_msg;
        quad_utils::eigenToGRFArrayMsg(grf_plan_.row(i),
                                       foot_plan_msg.states[i], grf_array_msg);
        grf_array_msg.contact_states.resize(num_feet_);
        for (int j = 0; j < num_feet_; j++) {
            grf_array_msg.contact_states[j] = contact_schedule_[i][j];
        }

        // Update the headers and plan indices of the messages
        ros::Time state_timestamp;

        // The first duration will vary
        state_timestamp = (i == 0)
                              ? current_state_timestamp_
                              : current_state_timestamp_ +
                                    ros::Duration(first_element_duration_) +
                                    ros::Duration((i - 1) * dt_);

        quad_utils::updateStateHeaders(robot_state_msg, state_timestamp,
                                       map_frame_, current_plan_index_ + i);
        grf_array_msg.header = robot_state_msg.header;
        grf_array_msg.traj_index = robot_state_msg.traj_index;

        local_plan_msg.states.push_back(robot_state_msg);
        local_plan_msg.grfs.push_back(grf_array_msg);
        local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
        local_plan_msg.primitive_ids.push_back(ref_primitive_plan_(i));
    }

    // Update timestamps to reflect when these messages were published
    local_plan_msg.state_timestamp = current_state_timestamp_;
    auto t_publish = ros::Time::now();
    local_plan_msg.header.stamp = t_publish;
    future_footholds_msg.header.stamp = t_publish;
    foot_plan_msg.header.stamp = t_publish;

    // Publish
    local_plan_pub_.publish(local_plan_msg);
    foot_plan_discrete_pub_.publish(future_footholds_msg);
    foot_plan_continuous_pub_.publish(foot_plan_msg);

    // for visualize rf
    rf_alpha_pub_.publish(rf_alpha_msg_);
}

void LocalPlanner::spin() {
    ros::Rate r(update_rate_);

    while (ros::ok()) {
        ros::spinOnce();
        // Wait until all required data has been received
        if (terrain_.isEmpty() ||
            (body_plan_msg_ == NULL && !use_twist_input_) ||
            robot_state_msg_ == NULL)
            continue;

        if (add_force_) {
            applyForce();
            add_force_ = false;  
        }

        // Get the reference plan and robot state into the desired data
        // structures
        getReference();

        if (rf_enable_) { computeRFF(); }
        if (l1_enable_) { computeL1(); }

        // record the data
        if (record_data_){
            std::thread data_collect_thread(&LocalPlanner::collectData, this);
            data_collect_thread.detach();
        }

        // Compute the local plan and publish if it solved successfully,
        // otherwise just sleep
        is_last_plan_success_ = computeLocalPlan();
        if (is_last_plan_success_) {
            publishLocalPlan(); 
            u_last_ = grf_plan_.row(0);
        }

        last_state_ = current_state_;
        last_state_timestamp_ = current_state_timestamp_;
        r.sleep();
    }
}