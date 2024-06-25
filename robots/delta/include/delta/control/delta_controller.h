// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/pinocchio_robot_model.h>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <delta/mpcConfig.h>
#include <delta/control/osqp_solver.h>
#include <delta/model/delta_robot_model.h>
#include <delta/navigation/delta_navigation.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nlopt.hpp>
#include <numeric>
#include <OsqpEigen/OsqpEigen.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

namespace aerial_robot_control
{
  class RollingController : public PoseLinearController
  {
  public:
    RollingController();
    ~RollingController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

    boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModel() {return robot_model_;}
    boost::shared_ptr<RollingRobotModel> getRollingRobotModel() {return rolling_robot_model_;}

  private:
    ros::Publisher rpy_gain_pub_;                     // for spinal
    ros::Publisher flight_cmd_pub_;                   // for spinal
    ros::Publisher gimbal_control_pub_;               // for servo bridge
    ros::Publisher torque_allocation_matrix_inv_pub_; // for spinal
    ros::Publisher gimbal_dof_pub_;                   // for spinal
    ros::Publisher gimbal_indices_pub_;               // for spinal
    ros::Publisher target_vectoring_force_pub_;       // for debug
    ros::Publisher target_wrench_acc_cog_pub_;        // for debug
    ros::Publisher gravity_compensate_term_pub_;      // for debug
    ros::Publisher wrench_allocation_matrix_pub_;     // for debug
    ros::Publisher full_q_mat_pub_;                   // for debug
    ros::Publisher operability_pub_;                  // for debug
    ros::Publisher target_acc_cog_pub_;               // for debug
    ros::Publisher target_acc_dash_pub_;              // for debug
    ros::Publisher exerted_wrench_cog_pub_;               // for debug
    ros::Subscriber joint_state_sub_;
    ros::Subscriber calc_gimbal_in_fc_sub_;
    dynamic_reconfigure::Server<delta::mpcConfig>::CallbackType dynamic_reconf_func_mpc_;
    boost::shared_ptr<dynamic_reconfigure::Server<delta::mpcConfig>> mpc_reconfigure_server_;

    tf2_ros::TransformBroadcaster br_;
    KDL::Frame contact_point_alined_;

    std::mutex contact_point_alined_mutex_;
    std::mutex current_gimbal_angles_mutex_;
    std::mutex current_joint_angles_mutex_;

    boost::shared_ptr<aerial_robot_navigation::RollingNavigator> rolling_navigator_;
    boost::shared_ptr<aerial_robot_model::PinocchioRobotModel> pinocchio_robot_model_;
    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;

    /* common part */
    std::vector<double> rotor_tilt_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    std::vector<double> prev_target_gimbal_angles_;
    std::vector<double> current_gimbal_angles_;
    std::vector<double> current_joint_angles_;
    Eigen::VectorXd full_lambda_all_;
    Eigen::VectorXd full_lambda_trans_;
    Eigen::VectorXd full_lambda_rot_;
    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;
    double candidate_yaw_term_;
    bool use_sr_inv_;
    double sr_inv_weight_;
    double circle_radius_;
    std::string tf_prefix_;
    bool attitude_control_in_spinal_;
    double gimbal_lpf_factor_;
    int ground_navigation_mode_;
    double rolling_control_timestamp_;
    bool realtime_gimbal_allocation_;
    double torque_allocation_matrix_inv_pub_stamp_;
    double torque_allocation_matrix_inv_pub_interval_;
    bool full_lambda_mode_;

    /* flight mode */
    Eigen::VectorXd target_wrench_acc_cog_;
    std::vector<int> controlled_axis_;
    std::vector<double> target_acc_cog_;
    std::vector<double> target_acc_dash_;
    double target_roll_, target_pitch_; // for under actuated control
    int control_dof_;
    bool calc_gimbal_in_fc_;
    bool hovering_approximate_;

    /* ground mode */
    Eigen::Vector3d gravity_compensate_term_;
    double steering_mu_;
    double gradient_weight_;
    double gravity_compensate_ratio_;
    double rolling_minimum_lateral_force_;
    double gimbal_d_theta_max_;
    double d_lambda_max_;
    double lambda_weight_;
    double d_gimbal_center_weight_;
    double d_gimbal_weight_;
    int ipopt_max_iter_;
    std::vector<double> prev_opt_gimbal_;
    std::vector<double> prev_opt_lambda_;

    /* mpc */
    OsqpEigen::Solver mpc_linear_solver_;
    bool mpc_first_run_;
    double horizon_;
    int n_step_;
    double q_x_p_xy_;
    double q_x_p_z_;
    double q_x_v_xy_;
    double q_x_v_z_;
    double q_x_a_xy_;
    double q_x_a_z_;
    double q_x_w_xy_;
    double q_x_w_z_;
    double r_lambda_;
    Eigen::VectorXd mpc_solution_;

    /* common part */
    bool update() override;
    void reset() override;
    void controlCore() override;
    void rosParamInit();
    void wrenchAllocation();
    void calcYawTerm();

    /* flight mode */
    void calcAccFromCog();
    void calcFlightFullLambda();

    /* rolling mode */
    void calcContactPoint();
    void standingPlanning();
    void calcStandingFullLambda();

    /* mpc */
    void flyingMpc();
    void mpcParamInit();

    /* send command */
    void sendCmd();
    void sendGimbalAngles();
    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void setAttitudeGains();

    /* ros callbacks */
    void jointStateCallback(const sensor_msgs::JointStateConstPtr & msg);
    void calcGimbalInFcCallback(const std_msgs::BoolPtr & msg);
    void mpcCfgCallback(delta::mpcConfig &config, uint32_t level);

    /* utils */
    void setControllerParams(std::string ns);
    void rosoutControlParams(std::string ns);
    void setControlAxisWithNameSpace(std::string ns);
    void rosoutControlAxis(std::string ns);
    void printDebug();

    void slsqpSolve();
    void nonlinearQP();

    void setControlAxis(int axis, int mode)
    {
      int prev_mode = controlled_axis_.at(axis);
      if(axis < 0 || 6 <= axis) return;
      if(mode) controlled_axis_.at(axis) = 1;
      else controlled_axis_.at(axis) = 0;
      if(prev_mode != mode)
        {
          ROS_WARN_STREAM("[control] set control axis about " << axis << " to " << mode << ". The controlled axis is ["
                          << controlled_axis_.at(0) << " "
                          << controlled_axis_.at(1) << " "
                          << controlled_axis_.at(2) << " "
                          << controlled_axis_.at(3) << " "
                          << controlled_axis_.at(4) << " "
                          << controlled_axis_.at(5) << "]"
                          );
        }
    }
  };
};
