#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <gtest/gtest_prod.h>
#include <local_planner/local_footstep_planner.h>
#include <local_planner/local_planner_modes.h>
#include <math.h>
#include <nmpc_controller/nmpc_controller.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/MultiFootPlanDiscrete.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <quad_utils/quad_data_recorder.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Wrench.h>

#include <thread>  // For std::thread
#include <mutex>   // For std::mutex

//! Local Body Planner library
/*!
   Wrapper around Quadrupedal MPC that interfaces with our ROS architecture
*/
class LocalPlanner {
 public:
  /**
   * @brief Constructor for LocalPlanner
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type LocalPlanner
   */
  LocalPlanner(ros::NodeHandle nh);

  /**
   * @brief Primary work function in class, called in node file for this
   * component
   */
  void spin();

 private:
  FRIEND_TEST(LocalPlannerTest, noInputCase);

  /**
   * @brief Initialize the local body planner
   */
  void initLocalBodyPlanner();

  /**
   * @brief Initialize the local footstep planner
   */
  void initLocalFootstepPlanner();

  /**
   * @brief Callback function to handle new terrain map data
   * @param[in] grid_map_msgs::GridMap contining map data
   */
  void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);

  /**
   * @brief Callback function to handle new plans
   * @param[in] msg Robot state trajectory message
   */
  void robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);

  /**
   * @brief Callback function to handle new state estimates
   * @param[in] State estimate message contining position and velocity for each
   * joint and robot body
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  /**
   * @brief Callback function to handle new desired twist data when using twist
   * input
   * @param[in] msg the message contining twist data
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  /**
   * @brief Function to compute reference trajectory from twist command or
   * global plan
   */
  void getReference();

   /**
    * @brief Unwrap the yaw signal in the reference body plan
    */
  void unwrapYawReference();

  /**
   * @brief Function to compute the local plan
   * @return Boolean if local plan was found successfully
   */
  bool computeLocalPlan();

  /**
   * @brief Function to publish the local plan
   */
  void publishLocalPlan();

	/**
	 * @brief Function to record the state 
	 */
	void collectData();

	/**
	 * @brief Initialize rff paramters 
	 */
	void initRfVariables();

	/**
	 * @brief Initialize l1 paramters 
	 */
	void initL1Variables();

	/**
	 * @brief Function to map from mask vector
	 */
	void mapEyefromMask(Eigen::MatrixXd &res, int dim, const std::vector<int> &mask);

  Eigen::VectorXd computeRMSE(const std::vector<Eigen::VectorXd>& x,
                                const std::vector<Eigen::VectorXd>& x_ref);

  /**
  * @brief Function to check if the contact changed
  */
  bool isContactChanged();

  void computeLegDynamic(const Eigen::VectorXd& x, const Eigen::VectorXd& feet_location, 
                                  Eigen::MatrixXd& M, Eigen::VectorXd& h, 
                                  Eigen::MatrixXd& J_u, Eigen::VectorXd& q_dot);

  void computeLegDynamicGo2(const Eigen::VectorXd& x, const Eigen::VectorXd& feet_location, 
                                  Eigen::MatrixXd& M, Eigen::VectorXd& h, 
                                  Eigen::MatrixXd& J_u, Eigen::VectorXd& q_dot);

  Eigen::MatrixXd computeJu(const Eigen::VectorXd &x0, 
                                          const Eigen::VectorXd &feet_location);

  /**
  * @brief Function to compute prediction error using rf
  */
  Eigen::VectorXd computeStatePredError(const Eigen::VectorXd &x0, 
                                        const Eigen::VectorXd &u, 
                                        const Eigen::VectorXd &x1, 
                                        const Eigen::VectorXd &feet_location, 
                                        double dt_local,
                                        const Eigen::MatrixXd &alpha,
                                        const Eigen::MatrixXd &rf);
  /**
  * @brief Function to compute uncertainty using l1
  */
  bool computeL1Sigma(const Eigen::VectorXd &x0, 
                                        const Eigen::VectorXd &u, 
                                        const Eigen::VectorXd &x1, 
                                        const Eigen::VectorXd &feet_location, 
                                        double dt_local);
  /**
   * @brief Function to update random fourier features
   */
  void computeRFF();

  /**
   * @brief Function to update L1
   */
  void computeL1();

  /**
   * @brief Function to publish uncertainty
   */  
  void publish_uncertainty_wrench(const Eigen::VectorXd &wrench, std::string wrench_topic);
  ros::Publisher l1_sigma_pub_;
  ros::Publisher l1_sigma_filt_pub_;
  ros::Publisher nominal_pub_;
  ros::Publisher rf_pub_;

  /**
   * @brief Function to add force
   */ 
  void applyForce();


  /// Robot type: A1 or Spirit
  std::string robot_name_;

  /// ROS subscriber for incoming terrain_map
  ros::Subscriber terrain_map_sub_;

  /// ROS subscriber for incoming body plans
  ros::Subscriber body_plan_sub_;

  /// ROS Subscriber for incoming states
  ros::Subscriber robot_state_sub_;

  /// Subscriber for twist input messages
  ros::Subscriber cmd_vel_sub_;

  /// ROS publisher for local plan output
  ros::Publisher local_plan_pub_;

  /// ROS publisher for discrete foot plan
  ros::Publisher foot_plan_discrete_pub_;

  /// ROS publisher for continuous foot plan
  ros::Publisher foot_plan_continuous_pub_;

  /// Define map frame
  std::string map_frame_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Struct for terrain map data
  FastTerrainMap terrain_;

  /// GridMap for terrain map data
  grid_map::GridMap terrain_grid_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Local Body Planner object
  std::shared_ptr<NMPCController> local_body_planner_nonlinear_;

  /// Local Footstep Planner object
  std::shared_ptr<LocalFootstepPlanner> local_footstep_planner_;

  /// Most recent robot plan
  quad_msgs::RobotPlan::ConstPtr body_plan_msg_;

  /// Most recent robot state
  quad_msgs::RobotState::ConstPtr robot_state_msg_;

  /// Past foothold locations
  quad_msgs::MultiFootState past_footholds_msg_;

  /// Timestamp of the state estimate
  ros::Time current_state_timestamp_;

  /// Current state (ground truth or estimate)
  Eigen::VectorXd current_state_;

  // Current positions of each foot
  Eigen::VectorXd current_foot_positions_world_;

  // Current velocities of each foot
  Eigen::VectorXd current_foot_velocities_world_;

  // Current positions of each foot
  Eigen::VectorXd current_foot_positions_body_;

  /// Current index in the global plan
  int current_plan_index_;

  /// local planner timestep (seconds)
  double dt_;

  /// Computation time in computeLocalPlan
  double compute_time_;

  /// Average computation time in computeLocalPlan
  double mean_compute_time_;

  /// Exponential filter smoothing constant (higher updates slower)
  const double filter_smoothing_constant_ = 0.5;

  /// Standard MPC horizon length
  int N_;

  /// Current MPC horizon length
  int N_current_;

  /// Number of states
  const int Nx_ = 12;

  /// Number of controls
  const int Nu_ = 13;

  /// Number of legs
  const int num_feet_ = 4;

  /// Number of joints per leg
  const int num_joints_per_leg_ = 3;

  /// Matrix of body states (N x Nx: rows correspond to individual states in the
  /// horizon)
  Eigen::MatrixXd body_plan_;

  /// Matrix of body states (N x Nx: rows correspond to individual states in the
  /// horizon)
  Eigen::MatrixXd ref_body_plan_;

  /// Vector of ground height along reference trajectory
  Eigen::VectorXd ref_ground_height_;

  /// Vector of primitive along reference trajectory
  Eigen::VectorXi ref_primitive_plan_;

  /// Matrix of grfs (N x Nu: rows correspond to individual arrays of GRFs in
  /// the horizon)
  Eigen::MatrixXd grf_plan_;

  /// Contact schedule
  std::vector<std::vector<bool>> contact_schedule_;

  /// Matrix of continuous foot positions in world frame
  Eigen::MatrixXd foot_positions_world_;

  /// Matrix of continuous foot velocities in world frame
  Eigen::MatrixXd foot_velocities_world_;

  /// Matrix of continuous foot accelerations in world frame
  Eigen::MatrixXd foot_accelerations_world_;

  /// Matrix of continuous foot positions in body frame
  Eigen::MatrixXd foot_positions_body_;

  /// Matrix of foot contact locations (number of contacts x num_legs_)
  Eigen::MatrixXd foot_plan_discrete_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// Twist input
  Eigen::VectorXd cmd_vel_;

  /// Commanded velocity filter constant
  double cmd_vel_filter_const_;

  /// fix cmd_vel_x for forward twist cmd
  double cmd_vel_x_;

  /// Scale for twist cmd_vel
  double cmd_vel_scale_;

  /// Nominal robot height
  double z_des_;

  /// Time of the most recent cmd_vel data
  ros::Time last_cmd_vel_msg_time_;

  /// Threshold for waiting for twist cmd_vel data
  double last_cmd_vel_msg_time_max_;

  /// Initial timestamp for contact cycling
  ros::Time initial_timestamp_;

  /// Foot initialization flag when using twist input without a global body plan
  bool first_plan_;

  /// Boolean for using twist input instead of a global body plan
  bool use_twist_input_;

  /// Goal state when using a global body plan
  Eigen::VectorXd goal_state_;

  /// Boolean to check if reach goal
  bool reach_goal_;

  /// Vector for stand pose (x, y, yaw)
  Eigen::Vector3d stand_pose_;

  /// Time duration to the next plan index
  double first_element_duration_;

  /// Difference in plan index from last solve
  int plan_index_diff_;

  /// Toe radius
  double toe_radius_;

  /// Control mode
  int control_mode_;

  /// Velocity threshold to enter stand mode
  double stand_vel_threshold_;

  /// Commanded velocity threshold to enter stand mode
  double stand_cmd_vel_threshold_;

  /// Position error threshold (from foot centroid) to enter stand mode
  double stand_pos_error_threshold_;

  /// Nominal-MPC, RFF-MPC, L1-MPC
  std::string control_type_;
  bool record_data_;

  // ****************************************
  // ************    the variables below are for the rff      *********************
  bool rf_enable_;
	int rf_size_target_mask_;
	int rf_size_input_mask_;
	int rf_n_rf_;
	int rf_kernel_type_; // gaussian
	double rf_kernel_std_;
	double rf_lr_trans_;
	double rf_lr_att_;

	std::vector<int> rf_target_mask_;
	std::vector<int> rf_input_mask_;
  std::vector<int> rf_output_mask_;

	Eigen::MatrixXd rf_omega_;
  Eigen::MatrixXd rf_b_;

	Eigen::MatrixXd rf_Bh_; // 6x12: 6 indicate velocity parts
	Eigen::MatrixXd rf_Bz_; // eye(state+u) first
	Eigen::VectorXd rf_residual_;

	Eigen::MatrixXd rf_alpha_;
	Eigen::MatrixXd rf_alpha_last_;
	Eigen::MatrixXd rf_rf_;
	
	Eigen::MatrixXd rf_omega_Bz_;

	Eigen::VectorXd rf_lr_vec_;
  // ****************************************

  // ************    l1      *********************
  bool l1_enable_;
  int l1_size_target_mask_;
  int l1_size_f_;
  int l1_size_m_;

  Eigen::MatrixXd l1_Bh_;
  Eigen::VectorXd l1_f_m_;
  Eigen::VectorXd l1_f_;
  Eigen::VectorXd l1_m_;

  // adaptation law
  double As_v_param_;
  double As_w_param_;
  Eigen::MatrixXd As_;
  Eigen::MatrixXd As_v_;
  Eigen::MatrixXd As_w_;
  Eigen::VectorXd sigma_;
  Eigen::VectorXd sigma_f_;
  Eigen::VectorXd sigma_m_;

  // state predictor
  Eigen::VectorXd v_w_tilde_;
  Eigen::VectorXd v_tilde_;
  Eigen::VectorXd w_tilde_;
  Eigen::VectorXd v_w_hat_;
  Eigen::VectorXd v_hat_;
  Eigen::VectorXd w_hat_;
  Eigen::VectorXd v_w_hat_prev_;
  Eigen::VectorXd v_hat_prev_;
  Eigen::VectorXd w_hat_prev_;

  // low pass filter
  double w_f_;
  double w_m1_;
  double w_m2_;
  double lpf_f_coef1_;
  double lpf_f_coef2_;
  double lpf_m1_coef1_;
  double lpf_m1_coef2_;
  double lpf_m2_coef1_;
  double lpf_m2_coef2_;
  Eigen::VectorXd sigma_filt_;
  Eigen::VectorXd sigma_f_filt_;
  Eigen::VectorXd sigma_m_filt1_;
  Eigen::VectorXd sigma_m_filt2_;
  Eigen::VectorXd sigma_f_filt_prev_;
  Eigen::VectorXd sigma_m_filt1_prev_;
  Eigen::VectorXd sigma_m_filt2_prev_;

  // ****************************************

    
  Eigen::VectorXd last_state_;
	Eigen::VectorXd u_last_;
	std::vector<bool> last_feet_contact_;
  Eigen::VectorXd last_foot_positions_body_;

	ros::Time last_state_timestamp_;
	bool is_last_plan_success_;
      
  int tracking_idx_;
  Eigen::VectorXd tracking_error_;
  Eigen::VectorXd last_ref_state_;

	std::shared_ptr<QuadDataRecorder> data_collector; 
  

  /// ROS publisher for adding wrench
  ros::Publisher wrench_pub_;
  bool add_force_;
  double fx_;
  double fy_;
  double fz_;

  ros::Publisher rf_alpha_pub_;
  std_msgs::Float32MultiArray rf_alpha_msg_;
};

#endif  // LOCAL_PLANNER_H
