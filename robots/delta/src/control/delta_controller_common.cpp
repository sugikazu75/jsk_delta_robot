#include <delta/control/delta_controller.h>

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
  lambda_trans_.resize(motor_num_);
  lambda_all_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);
  prev_target_gimbal_angles_.resize(motor_num_, 0);
  current_gimbal_angles_.resize(motor_num_, 0);
  current_joint_angles_.resize(2, 0);   // TODO: get from robot model

  target_wrench_acc_cog_.resize(6);
  target_acc_cog_.resize(6);

  full_lambda_all_ = Eigen::VectorXd::Zero(2 * motor_num_);
  full_lambda_trans_ = Eigen::VectorXd::Zero(2 * motor_num_);
  full_lambda_rot_ = Eigen::VectorXd::Zero(2 * motor_num_);

  rosParamInit();

  target_roll_ = 0.0;
  target_pitch_ = 0.0;

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
  gimbal_indices_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("gimbal_indices", 1);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &RollingController::jointStateCallback, this);
  calc_gimbal_in_fc_sub_ = nh_.subscribe("calc_gimbal_in_fc", 1, &RollingController::calcGimbalInFcCallback, this);

  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  target_wrench_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_wrench_acc_cog", 1);
  gravity_compensate_term_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/gravity_compensate_term", 1);
  wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
  full_q_mat_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/full_q_mat", 1);
  operability_pub_ = nh_.advertise<std_msgs::Float32>("debug/operability", 1);
  target_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_cog", 1);
  exerted_wrench_cog_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("debug/exerted_wrench_cog", 1);

  ground_navigation_mode_ = rolling_navigator_->getCurrentGroundNavigationMode();

  q_mat_.resize(6, motor_num_);
  q_mat_inv_.resize(motor_num_, 6);
}

void RollingController::reset()
{
  PoseLinearController::reset();

  setControllerParams("controller");

  setAttitudeGains();

  torque_allocation_matrix_inv_pub_stamp_ = -1;

  calc_gimbal_in_fc_ = false;

  std_msgs::UInt8 gimbal_dof_msg;
  gimbal_dof_msg.data = 0;
  gimbal_dof_pub_.publish(gimbal_dof_msg);

  std_msgs::UInt8MultiArray gimbal_indices_msg;
  gimbal_indices_msg.data.resize(motor_num_);
  gimbal_indices_msg.data.at(0) = 0;
  gimbal_indices_msg.data.at(1) = 2;
  gimbal_indices_msg.data.at(2) = 4;
  gimbal_indices_pub_.publish(gimbal_indices_msg);

  ROS_INFO_STREAM("[control] reset controller\n");
}

void RollingController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");

  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  getParam<bool>(control_nh, "use_sr_inv", use_sr_inv_, false);
  getParam<double>(control_nh, "sr_inv_weight", sr_inv_weight_, 0.0);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "calc_gimbal_in_fc", calc_gimbal_in_fc_, false);
  getParam<double>(control_nh, "gimbal_lpf_factor",gimbal_lpf_factor_, 1.0);
  getParam<double>(nh_, "circle_radius", circle_radius_, 0.5);
  getParam<bool>(control_nh, "realtime_gimbal_allocation", realtime_gimbal_allocation_, false);
  getParam<double>(control_nh, "gravity_compensate_ratio", gravity_compensate_ratio_, 1.0);
  getParam<double>(control_nh, "rolling_minimum_lateral_force", rolling_minimum_lateral_force_, 0.0);
  getParam<double>(control_nh, "steering_mu", steering_mu_, 0.0);

  rolling_robot_model_->setCircleRadius(circle_radius_);

  getParam<string>(nhp_, "tf_prefix", tf_prefix_, std::string(""));

  auto robot_model_xml = robot_model_->getRobotModelXml("robot_description");
  for(int i = 0; i < motor_num_; i++)
    {
      std::string rotor_tilt_name = std::string("rotor_tilt") + std::to_string(i + 1);
      TiXmlElement* rotor_tilt_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement(rotor_tilt_name);
      rotor_tilt_attr->Attribute("value", &rotor_tilt_.at(i));
    }

  rosoutControlParams("controller");
  rosoutControlParams("standing_controller");
  rosoutControlParams("rolling_controller");
}

bool RollingController::update()
{
  ground_navigation_mode_ = rolling_navigator_->getCurrentGroundNavigationMode();

  printDebug();
  if(!PoseLinearController::update()) return false;

  rolling_navigator_->setPrevGroundNavigationMode(ground_navigation_mode_);
  return true;
}

void RollingController::controlCore()
{
  PoseLinearController::controlCore();

  /* set gain if ground navigation mode is updated */
  if(ground_navigation_mode_ != rolling_navigator_->getPrevGroundNavigationMode())
    {
      ROS_WARN_STREAM("[control] change controller gain from "
                      << rolling_navigator_->indexToGroundNavigationModeString(rolling_navigator_->getPrevGroundNavigationMode()) << " to "
                      << rolling_navigator_->indexToGroundNavigationModeString(ground_navigation_mode_));

      switch(ground_navigation_mode_)
        {
        case aerial_robot_navigation::FLYING_STATE:
          {
            setControllerParams("controller");
            break;
          }
        case aerial_robot_navigation::STANDING_STATE:
          {
            setControllerParams("standing_controller");
            ros::NodeHandle standing_nh(nh_, "standing_controller");
            getParam<double>(standing_nh, "gravity_compensate_ratio", gravity_compensate_ratio_, 0.0);
            break;
          }
        case aerial_robot_navigation::ROLLING_STATE:
        case aerial_robot_navigation::DOWN_STATE:
          {
            setControllerParams("rolling_controller");
            ros::NodeHandle rolling_nh(nh_, "rolling_controller");
            getParam<double>(rolling_nh, "gravity_compensate_ratio", gravity_compensate_ratio_, 0.0);
            break;
          }
        default:
          break;
        }
    }

  switch(ground_navigation_mode_)
    {
    case aerial_robot_navigation::FLYING_STATE:
      {
        rolling_robot_model_->setTargetFrame("cog");

        if(rolling_navigator_->getControllersResetFlag())
          {
            for(auto& controller: pid_controllers_)
              {
                controller.reset();
              }
            rolling_navigator_->setControllersResetFlag(false);
          }

        calcAccFromCog();
        calcFlightFullLambda();
        break;
      }

    case aerial_robot_navigation::STANDING_STATE:
    case aerial_robot_navigation::ROLLING_STATE:
    case aerial_robot_navigation::DOWN_STATE:
      {
        /* for stand */
        rolling_robot_model_->setTargetFrame("cp");
        standingPlanning();
        calcStandingFullLambda();
        /* for stand */
        break;
      }

    default:
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[control] ground navigation mode is incorrect");
        break;
      }
    }

  /* common part */
  wrenchAllocation();
  calcYawTerm();
  /* common part */
}

void RollingController::wrenchAllocation()
{
  int last_col = 0;

  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd full_lambda_trans_i = full_lambda_trans_.segment(last_col, 2);
      Eigen::VectorXd full_lambda_all_i = full_lambda_all_.segment(last_col, 2);

      lambda_trans_.at(i) = full_lambda_trans_i.norm() / fabs(cos(rotor_tilt_.at(i)));
      lambda_all_.at(i) = full_lambda_all_i.norm() / fabs(cos(rotor_tilt_.at(i)));

      /* calculate gimbal angles */
      prev_target_gimbal_angles_.at(i) = target_gimbal_angles_.at(i);
      double gimbal_angle_i = atan2(-full_lambda_all_i(0), full_lambda_all_i(1));
      target_gimbal_angles_.at(i) = (gimbal_lpf_factor_ - 1.0) / gimbal_lpf_factor_ * prev_target_gimbal_angles_.at(i) + 1.0 / gimbal_lpf_factor_ * gimbal_angle_i;

      ROS_WARN_STREAM_ONCE("[control] gimbal lpf factor: " << gimbal_lpf_factor_);

      last_col += 2;
    }

  /* solve round offset of gimbal angle */
  for(int i = 0; i < motor_num_; i++)
    {
      if(fabs(target_gimbal_angles_.at(i) - current_gimbal_angles_.at(i)) > M_PI)
        {
          bool converge_flag = false;
          double gimbal_candidate_plus = target_gimbal_angles_.at(i);
          double gimbal_candidate_minus = target_gimbal_angles_.at(i);
          while(!converge_flag)
            {
              gimbal_candidate_plus += 2 * M_PI;
              gimbal_candidate_minus -= 2 * M_PI;
              if(fabs(current_gimbal_angles_.at(i) - gimbal_candidate_plus) < M_PI)
                {
                  ROS_WARN_STREAM_THROTTLE(1.0, "[control] send angle " << gimbal_candidate_plus << " for gimbal" << i << " instead of " << target_gimbal_angles_.at(i));
                  target_gimbal_angles_.at(i) = gimbal_candidate_plus;
                  converge_flag = true;
                }
              else if(fabs(current_gimbal_angles_.at(i) - gimbal_candidate_minus) < M_PI)
                {
                  ROS_WARN_STREAM_THROTTLE(1.0, "[control] send angle " << gimbal_candidate_minus << " for gimbal" << i << " instead of " << target_gimbal_angles_.at(i));
                  target_gimbal_angles_.at(i) = gimbal_candidate_minus;
                  converge_flag = true;
                }
            }
        }
    }

  /* update robot model by calculated gimbal angle */
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray gimbal_processed_joint = robot_model_->getJointPositions();
  for(int i = 0; i < motor_num_; i++)
    {
      std::string s = std::to_string(i + 1);
      if(realtime_gimbal_allocation_)
        {
          ROS_WARN_STREAM_ONCE("[control] use actual gimbal angle for realtime allocation");
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = current_gimbal_angles_.at(i);
        }
      else
        {
          ROS_WARN_STREAM_ONCE("[control] use target gimbal angle for realtime allocation");
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = target_gimbal_angles_.at(i);
        }
    }
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  /* calculate allocation matrix for realtime control */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

  /* set target acc for external wrench estimation */
  Eigen::VectorXd exerted_wrench_acc_cog;
  Eigen::VectorXd thrust = Eigen::VectorXd::Zero(motor_num_);
  for(int i = 0; i < motor_num_; i++)
    {
      thrust(i) = lambda_all_.at(i);
    }

  exerted_wrench_acc_cog = q_mat_ * thrust;
  exerted_wrench_acc_cog.head(3) = 1.0 / robot_model_->getMass() *  exerted_wrench_acc_cog.head(3);
  exerted_wrench_acc_cog.tail(3) = robot_model_->getInertia<Eigen::Matrix3d>().inverse() * exerted_wrench_acc_cog.tail(3);

  setTargetWrenchAccCog(exerted_wrench_acc_cog);
}

void RollingController::calcYawTerm()
{
  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal

  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog"));
  int torque_allocation_matrix_inv_rows;
  if(calc_gimbal_in_fc_) torque_allocation_matrix_inv_rows = 2 * motor_num_;
  else torque_allocation_matrix_inv_rows = motor_num_;

  for (unsigned int i = 0; i < torque_allocation_matrix_inv_rows; i++)
    {
      if(calc_gimbal_in_fc_)
        {
          if(full_q_mat_inv(i, YAW) > max_yaw_scale) max_yaw_scale = full_q_mat_inv(i, YAW);
        }
      else
        {
          if(q_mat_inv_(i, YAW) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW);
        }
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void RollingController::sendCmd()
{
  PoseLinearController::sendCmd();

  /* full lambda  */
  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < full_lambda_all_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(full_lambda_all_(i));
    }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  /*  target wrench in cog frame (flying mode) */
  std_msgs::Float32MultiArray target_wrench_acc_cog_msg;
  for(int i = 0; i < target_wrench_acc_cog_.size(); i++)
    {
      target_wrench_acc_cog_msg.data.push_back(target_wrench_acc_cog_(i));
    }
  target_wrench_acc_cog_pub_.publish(target_wrench_acc_cog_msg);

  /* gravity compensate term (ground mode) */
  std_msgs::Float32MultiArray gravity_compensate_term_msg;
  for(int i = 0; i < 3; i++)
    {
      gravity_compensate_term_msg.data.push_back(gravity_compensate_term_(i));
    }
  gravity_compensate_term_pub_.publish(gravity_compensate_term_msg);

  /* wrench allocation matrix */
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

  /* target acc in world frame */
  // consider rotation in all axis
  std_msgs::Float32MultiArray target_acc_cog_msg;
  for(int i = 0; i < target_acc_cog_.size(); i++)
    {
      target_acc_cog_msg.data.push_back(target_acc_cog_.at(i));
    }
  target_acc_cog_pub_.publish(target_acc_cog_msg);

  /* full wrench allocation matrix from cog */
  aerial_robot_msgs::WrenchAllocationMatrix full_q_mat_msg;
  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog");
  for(int i = 0; i < 2 * motor_num_; i++)
    {
      full_q_mat_msg.f_x.push_back(full_q_mat(0, i));
      full_q_mat_msg.f_y.push_back(full_q_mat(1, i));
      full_q_mat_msg.f_z.push_back(full_q_mat(2, i));
      full_q_mat_msg.t_x.push_back(full_q_mat(3, i));
      full_q_mat_msg.t_y.push_back(full_q_mat(4, i));
      full_q_mat_msg.t_z.push_back(full_q_mat(5, i));
    }
  full_q_mat_pub_.publish(full_q_mat_msg);

  /* exerted wrench in cog frame */
  geometry_msgs::WrenchStamped exerted_wrench_cog_msg;
  Eigen::VectorXd exerted_wrench_cog;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num_);
  lambda.resize(motor_num_);
  for(int i = 0; i < motor_num_; i++) lambda(i) = lambda_all_.at(i);
  exerted_wrench_cog = q_mat_ * lambda;

  exerted_wrench_cog_msg.header.frame_id = tf::resolve(tf_prefix_, std::string("cog"));
  exerted_wrench_cog_msg.wrench.force.x = exerted_wrench_cog(0);
  exerted_wrench_cog_msg.wrench.force.y = exerted_wrench_cog(1);
  exerted_wrench_cog_msg.wrench.force.z = exerted_wrench_cog(2);
  exerted_wrench_cog_msg.wrench.torque.x = exerted_wrench_cog(3);
  exerted_wrench_cog_msg.wrench.torque.y = exerted_wrench_cog(4);
  exerted_wrench_cog_msg.wrench.torque.z = exerted_wrench_cog(5);
  exerted_wrench_cog_pub_.publish(exerted_wrench_cog_msg);

  /* operability of full wrench allocation matrix */
  std_msgs::Float32 operability_msg;
  Eigen::MatrixXd q_qt;
  q_qt = full_q_mat * full_q_mat.transpose();
  float det = q_qt.determinant();
  operability_msg.data =sqrt(det);
  operability_pub_.publish(operability_msg);

  if(!calc_gimbal_in_fc_)
    sendGimbalAngles();

  sendFourAxisCommand();

  setAttitudeGains();

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
  if(calc_gimbal_in_fc_)
    {
      flight_command_data.base_thrust.resize(2 * motor_num_);
      for(int i = 0; i < 2 * motor_num_; i++)
        {
          flight_command_data.base_thrust.at(i) = full_lambda_trans_(i);
        }
    }
  else
    flight_command_data.base_thrust = lambda_trans_;

  flight_cmd_pub_.publish(flight_command_data);
}

void RollingController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      Eigen::MatrixXd torque_allocation_matrix_inv;
      int torque_allocation_matrix_inv_msg_rows;
      if(calc_gimbal_in_fc_)
        {
          torque_allocation_matrix_inv_msg.rows.resize(2 * motor_num_);
          torque_allocation_matrix_inv = aerial_robot_model::pseudoinverse(rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog")).rightCols(3);
          torque_allocation_matrix_inv_msg_rows = 2 * motor_num_;
        }
      else
        {
          torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
          torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
          torque_allocation_matrix_inv_msg_rows = motor_num_;
        }

      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < torque_allocation_matrix_inv_msg_rows; i++)
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

void RollingController::jointStateCallback(const sensor_msgs::JointStateConstPtr & state)
{
  sensor_msgs::JointState joint_state = *state;

  /* get current gimbal angles */
  for(int i = 0; i < joint_state.name.size(); i++)
    {
      if(joint_state.name.at(i).find("gimbal") != string::npos)
        {
          for(int j = 0; j < motor_num_; j++)
            {
              if(joint_state.name.at(i) == std::string("gimbal") + std::to_string(j + 1))
                {
                  std::lock_guard<std::mutex> lock(current_gimbal_angles_mutex_);
                  current_gimbal_angles_.at(j) = joint_state.position.at(i);
                }
            }
        }
      if(joint_state.name.at(i).find("joint") != string::npos)
        {
          for(int j = 0; j < current_joint_angles_.size(); j++)
            {
              if(joint_state.name.at(i) == std::string("joint" + std::to_string(j + 1)))
                {
                  std::lock_guard<std::mutex> lock(current_joint_angles_mutex_);
                  current_joint_angles_.at(j) = joint_state.position.at(i);
                }
            }
        }
    }

  /* tf of contact point */
  geometry_msgs::TransformStamped contact_point_tf = rolling_robot_model_->getContactPoint<geometry_msgs::TransformStamped>();
  contact_point_tf.header = state->header;
  contact_point_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  contact_point_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("contact_point"));
  br_.sendTransform(contact_point_tf);

  /* tf of contact point alined to ground plane */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point = rolling_robot_model_->getContactPoint<KDL::Frame>();
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
  double r, p, y;
  cog_rot.getRPY(r, p, y);

  if(true)
    {
      std::lock_guard<std::mutex> lock(contact_point_alined_mutex_);
      contact_point_alined_.p = contact_point.p;
      contact_point_alined_.M = cog.M;
      contact_point_alined_.M.DoRotX(-r);
      contact_point_alined_.M.DoRotY(-p);
    }
  geometry_msgs::TransformStamped contact_point_alined_tf = kdlToMsg(contact_point_alined_);
  contact_point_alined_tf.header = state->header;
  contact_point_alined_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  contact_point_alined_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("contact_point_alined"));
  br_.sendTransform(contact_point_alined_tf);
}

void RollingController::calcGimbalInFcCallback(const std_msgs::BoolPtr & msg)
{
  if(calc_gimbal_in_fc_ != msg->data)
    {
      std_msgs::UInt8 gimbal_dof_msg;
      if(msg->data)
        {
          gimbal_dof_msg.data = 1;
          ROS_WARN("[control] calc gimbal in fc.");
        }
      else
        {
          gimbal_dof_msg.data = 0;
          ROS_WARN("[control] not calc gimbal in fc.");
        }
      gimbal_dof_pub_.publish(gimbal_dof_msg);
      calc_gimbal_in_fc_ = msg->data ? true : false;
    }
  else
    {
      return;
    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::RollingController, aerial_robot_control::ControlBase);
