#include <rolling/control/rolling_controller.h>


using namespace aerial_robot_model;
using namespace aerial_robot_control;

RollingController::RollingController():
  PoseLinearController(),
  torque_allocation_matrix_inv_pub_stamp_(0)
{
}

void RollingController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rolling_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::RollingNavigator>(navigator_);
  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();

  rotor_tilt_.resize(motor_num_);
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);
  prev_target_gimbal_angles_.resize(motor_num_, 0);

  target_wrench_acc_cog_.resize(6);

  target_acc_cog_.resize(6);
  target_acc_dash_.resize(6);

  full_lambda_trans_.resize(2 * motor_num_);
  full_lambda_rot_.resize(2 * motor_num_);
  full_lambda_all_.resize(2 * motor_num_);
  full_q_mat_.resize(6, 2 * motor_num_);
  full_q_mat_inv_.resize(2 * motor_num_, 6);

  rosParamInit();

  target_roll_ = 0.0;
  target_pitch_ = 0.0;
  target_thrust_z_term_.resize(2 * motor_num_);

  z_limit_ = pid_controllers_.at(Z).getLimitSum();

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  desire_coordinate_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  target_wrench_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_wrench_acc_cog", 1);
  wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
  full_q_mat_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/full_q_mat", 1);
  full_q_mat_inv_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/full_q_mat_inv", 1);
  under_q_mat_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/under_q_mat", 1);
  under_q_mat_inv_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/under_q_mat_inv", 1);
  operability_pub_ = nh_.advertise<std_msgs::Float32>("debug/operability", 1);
  target_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_cog", 1);
  target_acc_dash_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_dash", 1);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &RollingController::jointStateCallback, this);
  z_i_control_flag_sub_ = nh_.subscribe("z_i_control_flag", 1, &RollingController::setZIControlFlagCallback, this);
  z_i_term_sub_ = nh_.subscribe("z_i_term", 1, &RollingController::setZITermCallback, this);
  stay_current_sub_ = nh_.subscribe("stay_current_position", 1, &RollingController::stayCurrentXYPosition, this);
  reset_attitude_gains_sub_ = nh_.subscribe("reset_attitude_gains", 1, &RollingController::resetAttitudeGainsCallback, this);
  control_mode_sub_ = nh_.subscribe("control_mode", 1, &RollingController::setControlModeCallback, this);

  gain_updated_ = false;
  ground_navigation_mode_ = aerial_robot_navigation::FLYING_STATE;

  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  q_mat_.resize(control_dof_, motor_num_);
  q_mat_inv_.resize(motor_num_, control_dof_);

}

void RollingController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
  gain_updated_ = false;
  standing_target_phi_ = 0.0;
}

void RollingController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle base_nh(nh_, "aerial_robot_base_node");
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  getParam<double>(control_nh, "allocation_refine_threshold", allocation_refine_threshold_, 0.01);
  getParam<int>(control_nh, "allocation_refine_max_iteration", allocation_refine_max_iteration_, 1);
  getParam<double>(control_nh, "initial_roll_tilt", initial_roll_tilt_, 0.0);
  getParam<bool>(control_nh, "use_sr_inv", use_sr_inv_, false);
  getParam<double>(control_nh, "sr_inv_weight", sr_inv_weight_, 0.0);
  getParam<bool>(control_nh, "fully_actuated", fully_actuated_, true);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<double>(control_nh, "gimbal_lpf_factor",gimbal_lpf_factor_, 1.0);

  getParam<double>(nh_, "circle_radius", circle_radius_, 0.5);
  rolling_robot_model_->setCircleRadius(circle_radius_);

  getParam<double>(control_nh, "standing_converged_baselink_roll_thresh", standing_converged_baselink_roll_thresh_, 0.0);
  getParam<double>(control_nh, "standing_converged_z_i_term", standing_converged_z_i_term_, 0.0);

  getParam<string>(base_nh, "tf_prefix", tf_prefix_, std::string(""));

  double rotor_tilt1;
  double rotor_tilt2;
  double rotor_tilt3;
  getParam<double>(control_nh, "rotor_tilt1", rotor_tilt1, 0.0);
  rotor_tilt_.at(0) = rotor_tilt1;
  getParam<double>(control_nh, "rotor_tilt2", rotor_tilt2, 0.0);
  rotor_tilt_.at(1) = rotor_tilt2;
  getParam<double>(control_nh, "rotor_tilt3", rotor_tilt3, 0.0);
  rotor_tilt_.at(2) = rotor_tilt3;

  if(!control_nh.getParam("controlled_axis", controlled_axis_))
    {
      controlled_axis_ = std::vector<int>(6, 1);
    }
  ROS_WARN_STREAM("[control] controlled axis: " << controlled_axis_.at(0) << " " << controlled_axis_.at(1) << " " << controlled_axis_.at(2) << " " << controlled_axis_.at(3) << " " << controlled_axis_.at(4) << " " << controlled_axis_.at(5));

}

void RollingController::controlCore()
{
  targetStatePlan();
  calcAccFromCog();
  calcWrenchAllocationMatrix();
  wrenchAllocation();
  calcYawTerm();
}

void RollingController::targetStatePlan()
{
  ground_navigation_mode_ = rolling_navigator_->getGroundNavigationMode();
  if(ground_navigation_mode_ == aerial_robot_navigation::FLYING_STATE)
    {
      ROS_WARN_ONCE("[control] flying state");
    }

  if(ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE)
    {
      ROS_WARN_ONCE("[control] standing state");

      tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      if(std::abs(cog_pos.z() / circle_radius_) < 1.0 && std::asin(cog_pos.z() / circle_radius_) > standing_target_phi_)
        {
          standing_target_phi_ = std::asin(cog_pos.z() / circle_radius_);
          if(standing_target_phi_ > 1.57)
            {
              standing_target_phi_ = 1.57;
            }
        }
      robot_model_->setCogDesireOrientation(standing_target_phi_, 0, 0);
      spinal::DesireCoord desire_coordinate_msg;
      desire_coordinate_msg.roll = standing_target_phi_;
      desire_coordinate_pub_.publish(desire_coordinate_msg);

      tf::Vector3 standing_initial_pos = rolling_navigator_->getStandingInitialPos();
      tf::Vector3 standing_initial_euler = rolling_navigator_->getStandingInitialEuler();

      navigator_->setTargetPosX(standing_initial_pos.x() - circle_radius_ * (1 - cos(standing_target_phi_)) * cos(standing_initial_euler.z() + M_PI / 2.0));
      navigator_->setTargetPosY(standing_initial_pos.y() - circle_radius_ * (1 - cos(standing_target_phi_)) * sin(standing_initial_euler.z() + M_PI / 2.0));
      navigator_->setTargetPosZ(circle_radius_);

      double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
      std::cout << baselink_roll << std::endl;
      if(baselink_roll > 1.3)
        {
          ROS_WARN_STREAM("[control] decrease thrttle bacause baselink roll > " << standing_converged_baselink_roll_thresh_);
          pid_controllers_.at(Z).setITerm(5.0);
          pid_controllers_.at(Z).setErrIUpdateFlag(false);
        }
    }
}

void RollingController::calcAccFromCog()
{
  ground_navigation_mode_ = rolling_navigator_->getGroundNavigationMode();

  if(ground_navigation_mode_ == aerial_robot_navigation::FLYING_STATE || aerial_robot_navigation::STANDING_STATE){
    ROS_WARN_ONCE("[control] calc acc for flying state");
  PoseLinearController::controlCore();
  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  target_acc_cog_.at(0) = target_acc_cog.x();
  target_acc_cog_.at(1) = target_acc_cog.y();
  target_acc_cog_.at(2) = target_acc_cog.z();

  target_acc_dash_.at(0) = target_acc_dash.x();
  target_acc_dash_.at(1) = target_acc_dash.y();
  target_acc_dash_.at(2) = target_acc_dash.z();

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  if(control_dof_ < 6)
    {
      target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
    }
  else
    {
      target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
    }

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  target_wrench_acc_cog_ = target_wrench_acc_cog;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

  if(!controlled_axis_.at(X))
    {
      if(hovering_approximate_)
        {
          target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
        }
      else
        {
          target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
        }
      navigator_->setTargetPitch(target_pitch_);
    }
  if(!controlled_axis_.at(Y))
    {
      if(hovering_approximate_)
        {
          target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
        }
      else
        {
          target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
        }
      navigator_->setTargetRoll(target_roll_);
    }
  }

  else if(ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE)
    {
      ROS_WARN_ONCE("[control] calc acc for standing state");
    }

}

void RollingController::calcWrenchAllocationMatrix()
{
  /* calculate normal allocation */
  Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);

  int last_col = 0;
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();
  for(int i = 0; i < motor_num_; i++)
    {
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
      wrench_matrix.middleCols(last_col, 3) = wrench_map;
      last_col += 3;
    }

  Eigen::Matrix3d inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv = 1 / robot_model_->getMass();
  wrench_matrix.topRows(3) = mass_inv * wrench_matrix.topRows(3);
  wrench_matrix.bottomRows(3) = inertia_inv * wrench_matrix.bottomRows(3);

  /* calculate masked and integrated rotaion matrix */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, 2 * motor_num_);
  const auto links_rotation_from_cog = rolling_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  Eigen::MatrixXd mask(3, 2);
  mask << 0, 0, 1, 0, 0, 1;
  for(int i = 0; i < motor_num_; i++)
    {
      integrated_rot.block(3 * i, 2 * i, 3, 2) = links_rotation_from_cog.at(i) * mask;
    }

  /* calculate integarated allocation */
  full_q_mat_ = wrench_matrix * integrated_rot;

  /* extract controlled axis */
  Eigen::MatrixXd controlled_axis_mask_ = Eigen::MatrixXd::Zero(control_dof_, 6);
  int last_row = 0;
  for(int i = 0; i < controlled_axis_.size(); i++)
    {
      if(controlled_axis_.at(i))
        {
          controlled_axis_mask_(last_row, i) = 1;
          last_row++;
        }
    }
  full_q_mat_ = controlled_axis_mask_ * full_q_mat_;
  full_q_mat_inv_ = aerial_robot_model::pseudoinverse(full_q_mat_);
  target_wrench_acc_cog_ = controlled_axis_mask_ * target_wrench_acc_cog_;

  if(use_sr_inv_)
    {
      // http://www.thothchildren.com/chapter/5bd8d78751d930518903af34
      Eigen::MatrixXd sr_inv = full_q_mat_.transpose() * (full_q_mat_ * full_q_mat_.transpose() + sr_inv_weight_ * Eigen::MatrixXd::Identity(full_q_mat_.rows(), full_q_mat_.rows())).inverse();
      full_q_mat_inv_ = sr_inv;
      ROS_WARN_STREAM_ONCE("[control] use SR-Inverse. weight is " << sr_inv_weight_);
    }
  else
    {
      ROS_WARN_ONCE("[control] use MP-Inverse");
    }

}

void RollingController::wrenchAllocation()
{
  /* actuator mapping */
  full_lambda_trans_ = full_q_mat_inv_.leftCols(control_dof_ - 3) * target_wrench_acc_cog_.head(control_dof_ - 3);
  full_lambda_rot_ = full_q_mat_inv_.rightCols(3) * target_wrench_acc_cog_.tail(3);
  full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;

  int last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd full_lambda_trans_i = full_lambda_trans_.segment(last_col, 2);
      Eigen::VectorXd full_lambda_all_i = full_lambda_all_.segment(last_col, 2);

      double gimbal_angle_i = atan2(-full_lambda_all_i(0), full_lambda_all_i(1));
      ROS_WARN_STREAM_ONCE("[control] gimbal lpf factor: " << gimbal_lpf_factor_);
      prev_target_gimbal_angles_.at(i) = target_gimbal_angles_.at(i);
      target_gimbal_angles_.at(i) = (gimbal_lpf_factor_ - 1.0) / gimbal_lpf_factor_ * prev_target_gimbal_angles_.at(i) + 1.0 / gimbal_lpf_factor_ * gimbal_angle_i;

      target_base_thrust_.at(i) = full_lambda_trans_i.norm() / fabs(cos(rotor_tilt_.at(i)));

      last_col += 2;
    }

  /* update robot model by calculated gimbal angle */
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray gimbal_processed_joint = robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);
  for(int i = 0; i < motor_num_; i++)
    {
      std::string s = std::to_string(i + 1);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = target_gimbal_angles_.at(i);
    }

  /* calculate allocation matrix for realtime control */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
}

void RollingController::calcYawTerm()
{
  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
    {
      if(q_mat_inv_(i, control_dof_ - 1) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, control_dof_ - 1);
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void RollingController::sendCmd()
{
  PoseLinearController::sendCmd();

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < full_lambda_all_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(full_lambda_all_(i));
    }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  std_msgs::Float32MultiArray target_wrench_acc_cog_msg;
  for(int i = 0; i < target_wrench_acc_cog_.size(); i++)
    {
      target_wrench_acc_cog_msg.data.push_back(target_wrench_acc_cog_(i));
    }
  target_wrench_acc_cog_pub_.publish(target_wrench_acc_cog_msg);

  aerial_robot_msgs::WrenchAllocationMatrix wrench_allocation_matrix_msg;
  for(int i = 0; i < q_mat_.cols(); i++)
    {
      wrench_allocation_matrix_msg.f_x.push_back(q_mat_(0, i));
      wrench_allocation_matrix_msg.f_y.push_back(q_mat_(1, i));
      wrench_allocation_matrix_msg.f_z.push_back(q_mat_(2, i));
      wrench_allocation_matrix_msg.t_x.push_back(q_mat_(3, i));
      wrench_allocation_matrix_msg.t_y.push_back(q_mat_(4, i));
      wrench_allocation_matrix_msg.t_z.push_back(q_mat_(5, i));
    }
  wrench_allocation_matrix_pub_.publish(wrench_allocation_matrix_msg);

  std_msgs::Float32MultiArray target_acc_cog_msg;
  for(int i = 0; i < target_acc_cog_.size(); i++)
    {
      target_acc_cog_msg.data.push_back(target_acc_cog_.at(i));
    }
  target_acc_cog_pub_.publish(target_acc_cog_msg);

  std_msgs::Float32MultiArray target_acc_dash_msg;
  for(int i = 0; i < target_acc_dash_.size(); i++)
    {
      target_acc_dash_msg.data.push_back(target_acc_dash_.at(i));
    }
  target_acc_dash_pub_.publish(target_acc_dash_msg);

  // aerial_robot_msgs::WrenchAllocationMatrix full_q_mat_msg;
  // for(int i = 0; i < 2 * motor_num_; i++)
  //   {
  //     full_q_mat_msg.f_x.push_back(full_q_mat_(0, i));
  //     full_q_mat_msg.f_y.push_back(full_q_mat_(1, i));
  //     full_q_mat_msg.f_z.push_back(full_q_mat_(2, i));
  //     full_q_mat_msg.t_x.push_back(full_q_mat_(3, i));
  //     full_q_mat_msg.t_y.push_back(full_q_mat_(4, i));
  //     full_q_mat_msg.t_z.push_back(full_q_mat_(5, i));
  //   }
  // full_q_mat_pub_.publish(full_q_mat_msg);

  // aerial_robot_msgs::WrenchAllocationMatrix full_q_mat_inv_msg;
  // for(int i = 0; i < 2 * motor_num_; i++)
  //   {
  //     full_q_mat_inv_msg.f_x.push_back(full_q_mat_inv_(i, 0));
  //     full_q_mat_inv_msg.f_y.push_back(full_q_mat_inv_(i, 1));
  //     full_q_mat_inv_msg.f_z.push_back(full_q_mat_inv_(i, 2));
  //     full_q_mat_inv_msg.t_x.push_back(full_q_mat_inv_(i, 3));
  //     full_q_mat_inv_msg.t_y.push_back(full_q_mat_inv_(i, 4));
  //     full_q_mat_inv_msg.t_z.push_back(full_q_mat_inv_(i, 5));
  //   }
  // full_q_mat_inv_pub_.publish(full_q_mat_inv_msg);

  // aerial_robot_msgs::WrenchAllocationMatrix under_q_mat_msg;
  // for(int i = 0; i < 2 * motor_num_; i++)
  //   {
  //     under_q_mat_msg.f_z.push_back(under_q_mat_(0, i));
  //     under_q_mat_msg.t_x.push_back(under_q_mat_(1, i));
  //     under_q_mat_msg.t_y.push_back(under_q_mat_(2, i));
  //     under_q_mat_msg.t_z.push_back(under_q_mat_(3, i));
  //   }
  // under_q_mat_pub_.publish(under_q_mat_msg);

  // aerial_robot_msgs::WrenchAllocationMatrix under_q_mat_inv_msg;
  // for(int i = 0; i < 2 * motor_num_; i++)
  //   {
  //     under_q_mat_inv_msg.f_z.push_back(under_q_mat_inv_(i, 0));
  //     under_q_mat_inv_msg.t_x.push_back(under_q_mat_inv_(i, 1));
  //     under_q_mat_inv_msg.t_y.push_back(under_q_mat_inv_(i, 2));
  //     under_q_mat_inv_msg.t_z.push_back(under_q_mat_inv_(i, 3));
  //   }
  // under_q_mat_inv_pub_.publish(under_q_mat_inv_msg);

  std_msgs::Float32 operability_msg;
  Eigen::MatrixXd q_qt;
  if(fully_actuated_) q_qt = full_q_mat_ * full_q_mat_.transpose();
  else q_qt = under_q_mat_ * under_q_mat_.transpose();
  float det = q_qt.determinant();
  operability_msg.data =sqrt(det);
  operability_pub_.publish(operability_msg);

  sendGimbalAngles();

  sendFourAxisCommand();

  sendTorqueAllocationMatrixInv();
}

void RollingController::sendGimbalAngles()
{
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < motor_num_; i++){
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    gimbal_control_msg.name.push_back(std::string("gimbal") + std::to_string(i + 1));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

void RollingController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  if(!controlled_axis_[0])
    {
      flight_command_data.angles[1] = target_pitch_;
    }
  if(!controlled_axis_[1])
    {
      flight_command_data.angles[0] = target_roll_;
    }
  flight_cmd_pub_.publish(flight_command_data);
}

void RollingController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void RollingController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void RollingController::resetAttitudeGains()
{
  ROS_WARN("reset attitude gain in spinal");
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = 0;
  rpy_gain_msg.motors.at(0).roll_i = 0;
  rpy_gain_msg.motors.at(0).roll_d = 0;
  rpy_gain_msg.motors.at(0).pitch_p = 0;
  rpy_gain_msg.motors.at(0).pitch_i = 0;
  rpy_gain_msg.motors.at(0).pitch_d = 0;
  rpy_gain_msg.motors.at(0).yaw_d = 0;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void RollingController::setZIControlFlagCallback(const std_msgs::BoolPtr & msg)
{
  ROS_WARN("z i control update flag is set to %d", msg->data);
  pid_controllers_.at(Z).setErrIUpdateFlag(msg->data);
}

void RollingController::setZITermCallback(const std_msgs::Float32Ptr & msg)
{
  ROS_WARN("set i term of z controller from %lf to %lf", pid_controllers_.at(Z).getITerm(), msg->data);
  pid_controllers_.at(Z).setITerm(msg->data);
}

void RollingController::stayCurrentXYPosition(const std_msgs::Empty & msg)
{
  navigator_->setTargetXyFromCurrentState();
  navigator_->setTargetYawFromCurrentState();
}

void RollingController::resetAttitudeGainsCallback(const std_msgs::Empty & msg)
{
  RollingController::resetAttitudeGains();
}

void RollingController::setControlModeCallback(const std_msgs::Int16Ptr & msg)
{
  if(msg->data == 0)
    {
      fully_actuated_ = true;
      ROS_WARN("set control mode to fully actuated");
    }

  else if(msg->data == 1)
    {
      fully_actuated_ = false;
      pid_msg_.roll.total.at(0) = 0;
      pid_msg_.roll.p_term.at(0) = 0;
      pid_msg_.roll.i_term.at(0) = 0;
      pid_msg_.roll.d_term.at(0) = 0;
      pid_msg_.roll.target_p = 0;
      pid_msg_.roll.err_p = 0;
      pid_msg_.roll.target_d = 0;
      pid_msg_.roll.err_d = 0;
      pid_msg_.pitch.total.at(0) = 0;
      pid_msg_.pitch.p_term.at(0) = 0;
      pid_msg_.pitch.i_term.at(0) = 0;
      pid_msg_.pitch.d_term.at(0) = 0;
      pid_msg_.pitch.target_p = 0;
      pid_msg_.pitch.err_p = 0;
      pid_msg_.pitch.target_d = 0;
      pid_msg_.pitch.err_d = 0;
      ROS_WARN("set control mode to under actuated");
    }

  else
    {
      fully_actuated_ = true;
      ROS_WARN("set control mode to fully actuated");
    }
}

void RollingController::jointStateCallback(const sensor_msgs::JointStateConstPtr & state)
{
  sensor_msgs::JointState joint_state = *state;
  geometry_msgs::TransformStamped tf = rolling_robot_model_->getContactPoint<geometry_msgs::TransformStamped>();
  tf.header = state->header;
  tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  tf.child_frame_id = tf::resolve(tf_prefix_, std::string("cp"));
  br_.sendTransform(tf);
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::RollingController, aerial_robot_control::ControlBase);
