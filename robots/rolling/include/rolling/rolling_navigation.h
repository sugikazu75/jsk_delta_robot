// -*- mode: c++ -*-
#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <rolling/model/rolling_robot_model.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Int16.h>

namespace aerial_robot_navigation
{
  enum rolling_mode
    {
     FLYING_STATE,
     STANDING_STATE,
     STEERING_STATE,
     ROLLING_STATE,
     RECOVERING_STATE
    };

  class RollingNavigator : public BaseNavigator
  {
  public:
    RollingNavigator();
    ~RollingNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

    void update() override;
    void reset() override;

    inline tf::Vector3 getCurrTargetBaselinkRot() {return curr_target_baselink_rot_;}
    inline int getCurrentGroundNavigationMode() {return current_ground_navigation_mode_;}
    inline int getPrevGroundNavigationMode() {return prev_ground_navigation_mode_;}
    void setPrevGroundNavigationMode(int mode) {prev_ground_navigation_mode_ = mode;}
    void setGroundNavigationMode(int state);
    inline tf::Vector3 getStandingInitialPos() {return standing_initial_pos_;}
    inline tf::Vector3 getStandingInitialEuler() {return standing_initial_euler_;}
    inline tf::Vector3 getSteeringInitialPos() {return steering_initial_pos_;}
    inline tf::Vector3 getSteeringInitialEuler() {return steering_initial_euler_;}
    inline tf::Vector3 getRollingInitialPos() {return rolling_initial_pos_;}
    inline tf::Vector3 getRollingInitialEuler() {return rolling_initial_euler_;}

  private:
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber ground_navigation_mode_sub_;
    ros::Publisher ground_navigation_mode_pub_;

    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;

    void baselinkRotationProcess();
    void landingProcess();
    void groundModeProcess();

    void rosParamInit() override;
    void setFinalTargetBaselinkRot(tf::Vector3 rot);

    void groundNavigationModeCallback(const std_msgs::Int16Ptr & msg);
    void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg);

    /* navigation mode */
    int current_ground_navigation_mode_;
    int prev_ground_navigation_mode_;

    /* standing mode variable */
    tf::Vector3 standing_initial_pos_;
    tf::Vector3 standing_initial_euler_;

    /* steering mode variable */
    tf::Vector3 steering_initial_pos_;
    tf::Vector3 steering_initial_euler_;
    double steering_joy_stick_deadzone_;

    /* rolling mode variable */
    tf::Vector3 rolling_initial_pos_;
    tf::Vector3 rolling_initial_euler_;
    double rolling_joy_sitck_deadzone_;

    /* target baselink rotation */
    double prev_rotation_stamp_;
    std::vector<double> target_gimbal_angles_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* landing process */
    bool landing_flag_;

    /* rosparam */
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};
