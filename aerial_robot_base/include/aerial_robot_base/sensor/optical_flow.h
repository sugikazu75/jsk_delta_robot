#ifndef OPTICAL_FLOW_MODULE_H
#define OPTICAL_FLOW_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_base/States.h>
#include <px_comm/OpticalFlow.h>
#include <std_msgs/Int32.h>

namespace sensor_plugin
{
  class OpticalFlow :public sensor_base_plugin::SensorBase
  {
  public:
      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator)
      {
        nh_ = ros::NodeHandle(nh, "optical_flow");
        nhp_ = ros::NodeHandle(nhp, "optical_flow");
        estimator_ = estimator;
        baseRosParamInit();
        rosParamInit();

        optical_flow_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);
        optical_flow_sub_ = nh_.subscribe<px_comm::OpticalFlow>("opt_flow", 1, &OpticalFlow::opticalFlowCallback, this, ros::TransportHints().tcpNoDelay());

        raw_pos_z_ = 0;
        pos_z_ = 0;
        raw_vel_z_ = 0;
        vel_z_ = 0;

        raw_vel_x_ = 0;
        filtered_vel_x_ = 0;

        raw_vel_y_ = 0;
        filtered_vel_y_ = 0;

        //debug, but can be constant;
        sensor_fusion_flag_ = false; 
        //ROS_ERROR("kalman_filter_flag_: %s", kalman_filter_flag_?("true"):("false"));
        }

    ~OpticalFlow() {}

    OpticalFlow() {}
  private:
    ros::Publisher optical_flow_pub_;
    ros::Subscriber optical_flow_sub_;
    ros::Time optical_flow_stamp_;

    double start_upper_thre_;
    double start_lower_thre_;
    double start_vel_;

    double x_axis_direction_;
    double y_axis_direction_;

    double sonar_noise_sigma_, level_vel_noise_sigma_;

    float raw_pos_z_;
    float pos_z_;
    float raw_vel_z_;
    float vel_z_;

    float raw_vel_x_;
    float filtered_vel_x_;

    float raw_vel_y_;
    float filtered_vel_y_;

    bool sensor_fusion_flag_;

    void opticalFlowCallback(const px_comm::OpticalFlowConstPtr & optical_flow_msg)
    {
      static int cnt = 0;
      static int CNT = 0;
      static float prev_raw_pos_z;
      static bool stop_flag = true;
      static double previous_secs;
      static double special_increment = 0;
      double current_secs = optical_flow_msg->header.stamp.toSec();

      //**** Global Sensor Fusion Flag Check
      if(!estimator_->getSensorFusionFlag())  return;

      //**** 高さ方向情報の更新
      raw_pos_z_ = optical_flow_msg->ground_distance;

      //  start sensor fusion condition
      if(!sensor_fusion_flag_)
        {
          if(raw_pos_z_ < start_upper_thre_ && raw_pos_z_ > start_lower_thre_ &&
             prev_raw_pos_z < start_lower_thre_ && 
             prev_raw_pos_z > (start_lower_thre_ - 0.1) )
            {//pose init
              //TODO: start flag fresh arm, or use air pressure => refined
              ROS_ERROR("optical flow: start sensor fusion, prev_raw_pos_z : %f", prev_raw_pos_z);

              if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
                {
                  for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                    {
                      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B)))
                        {
                          temp(0,0) = level_vel_noise_sigma_;
                          estimator_->getFuserEgomotion(i)->setInitState(x_axis_direction_ * optical_flow_msg->flow_x /1000.0, 1);
                          estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                          estimator_->getFuserEgomotion(i)->setMeasureFlag();
                        }
                      else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B)))
                        {
                          temp(0,0) = level_vel_noise_sigma_;
                          estimator_->getFuserEgomotion(i)->setInitState(y_axis_direction_ * optical_flow_msg->flow_y /1000.0 ,1);
                          estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                          estimator_->getFuserEgomotion(i)->setMeasureFlag();
                        }
                      else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                        {

                          Eigen::Matrix<double, 2, 1> init_state2 = Eigen::MatrixXd::Zero(2, 1);
                          init_state2(0,0) = optical_flow_msg->ground_distance;
                          init_state2(1,0) = start_vel_;
                          estimator_->getFuserEgomotion(i)->setInitState(init_state2);

                          temp(0,0) = sonar_noise_sigma_;
                          estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                          estimator_->getFuserEgomotion(i)->setMeasureFlag();
                        }
                    }
                }

              if(estimate_mode_ & (1 << EXPERIMENT_MODE))
                {
                  for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                    {
                      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
                      ROS_WARN("this is bias mode");
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B)))
                        {
                          temp(0,0) = level_vel_noise_sigma_;
                          estimator_->getFuserExperiment(i)->setMeasureSigma(temp);

                          estimator_->getFuserExperiment(i)->setInitState(x_axis_direction_ * optical_flow_msg->flow_x /1000.0, 1);

                          estimator_->getFuserExperiment(i)->setMeasureFlag();
                        }
                      else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B)))
                        {
                          temp(0,0) = level_vel_noise_sigma_;
                          estimator_->getFuserExperiment(i)->setMeasureSigma(temp);

                          estimator_->getFuserExperiment(i)->setInitState(y_axis_direction_ * optical_flow_msg->flow_y /1000.0 ,1);
                          estimator_->getFuserExperiment(i)->setMeasureFlag();
                        }
                      else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                        {
                          Eigen::Matrix<double, 2, 1> init_state2 = Eigen::MatrixXd::Zero(2, 1);
                          init_state2(0,0) = optical_flow_msg->ground_distance;
                          init_state2(1,0) = start_vel_;
                          estimator_->getFuserExperiment(i)->setInitState(init_state2);

                          temp(0,0) = sonar_noise_sigma_;
                          estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                          estimator_->getFuserExperiment(i)->setMeasureFlag();
                        }
                    }
                }
              sensor_fusion_flag_ = true;
            }

          //temporariy for end pahse of landing
          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                    {
                      if((estimator_->getFuserEgomotion(i)->getEstimateState())(0,0) <=0 && estimator_->getFuserEgomotion(i)->getFilteringFlag())
                        {
                          ROS_WARN("optical flow: disable z mesaure flag");
                          estimator_->getFuserEgomotion(i)->setMeasureFlag(false);
                          estimator_->getFuserEgomotion(i)->resetState();
                        }
                    }
                }
            }
        }
      else 
        {
          if(prev_raw_pos_z < start_upper_thre_ &&
             prev_raw_pos_z > start_lower_thre_ &&
             raw_pos_z_ < start_lower_thre_ && raw_pos_z_ > (start_lower_thre_ - 0.1))
            {//special process for landing
              sensor_fusion_flag_ = false;
              ROS_ERROR("optical flow sensor: stop sensor fusion");
              return;
            }

          //**** 高さ方向情報の更新
          raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z) / (current_secs - previous_secs);

          //**** 速度情報の更新,ボードの向き
          raw_vel_x_ = x_axis_direction_ * optical_flow_msg->velocity_x; 
          raw_vel_y_ = y_axis_direction_ * optical_flow_msg->velocity_y; 

          filtered_vel_x_ = x_axis_direction_ * optical_flow_msg->flow_x /1000.0;
          filtered_vel_y_ = y_axis_direction_ * optical_flow_msg->flow_y /1000.0;

          Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1); 
          Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 

          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B)))
                    {
                      sigma_temp(0,0) = level_vel_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);
                      if(optical_flow_msg->quality == 0 || raw_vel_x_ == 0 || raw_pos_z_ > 2.5)
                        temp(0, 0) = filtered_vel_x_;
                      else
                        temp(0, 0) = raw_vel_x_;
                      estimator_->getFuserEgomotion(i)->correction(temp);
                    }
                  else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B)))
                    {
                      sigma_temp(0,0) = level_vel_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);

                      if(optical_flow_msg->quality == 0 || raw_vel_y_ == 0 || raw_pos_z_ > 2.5)
                        temp(0, 0) = filtered_vel_y_;
                      else
                        temp(0, 0) = raw_vel_y_;
                      estimator_->getFuserEgomotion(i)->correction(temp);
                    }
                  else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                    {
                      sigma_temp(0,0) = sonar_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);

                      if(raw_pos_z_ != prev_raw_pos_z && raw_pos_z_ < 2.5) temp(0, 0) = raw_pos_z_;
                      estimator_->getFuserEgomotion(i)->correction(temp);
                    }
                }
            }

          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B)))
                    {
                      sigma_temp(0,0) = level_vel_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      if(optical_flow_msg->quality == 0 || raw_vel_x_ == 0 || raw_pos_z_ > 2.5)
                        temp(0, 0) = filtered_vel_x_;
                      else
                        temp(0, 0) = raw_vel_x_;
                      estimator_->getFuserExperiment(i)->correction(temp);
                    }
                  else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B)))
                    {
                      sigma_temp(0,0) = level_vel_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      if(optical_flow_msg->quality == 0 || raw_vel_y_ == 0 || raw_pos_z_ > 2.5)
                        temp(0, 0) = filtered_vel_y_;
                      else
                        temp(0, 0) = raw_vel_y_;
                      estimator_->getFuserExperiment(i)->correction(temp);
                    }
                  else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                    {
                      sigma_temp(0,0) = sonar_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      if(raw_pos_z_ != prev_raw_pos_z && raw_pos_z_ < 2.5) //100Hz
                        temp(0, 0) = raw_pos_z_;
                      estimator_->getFuserExperiment(i)->correction(temp);
                    }
                }
            }

          //publish
          aerial_robot_base::States opt_data;
          opt_data.header.stamp = optical_flow_msg->header.stamp;

          aerial_robot_base::State x_state;
          x_state.id = "x";
          x_state.raw_vel = raw_vel_x_;
          x_state.vel = filtered_vel_x_;

          aerial_robot_base::State y_state;
          y_state.id = "y";
          y_state.raw_vel = raw_vel_y_;
          y_state.vel = filtered_vel_y_;

          aerial_robot_base::State z_state;
          z_state.id = "z";
          z_state.raw_pos = raw_pos_z_;
          z_state.raw_vel = raw_vel_z_;


          Eigen::Matrix<double,2,1> kf_x_state;
          Eigen::Matrix<double,2,1> kf_y_state;
          Eigen::Matrix<double,2,1> kf_z_state;
          Eigen::Matrix<double,3,1> kfb_x_state;
          Eigen::Matrix<double,3,1> kfb_y_state;

          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  if(estimator_->getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                    {//temporary
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B))) 
                        {
                          kfb_x_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::X_B, 1, kfb_x_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W))) 
                        {
                          kfb_x_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::X_W, 1, kfb_x_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B))) 
                        {
                          kfb_y_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_W, 1, kfb_y_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W))) 
                        {
                          kfb_y_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_W, 1, kfb_y_state(1,0));
                        }
                    }
                  if(estimator_->getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc")
                    {//temporary
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B))) 
                        {
                          kf_x_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::X_B, 1, kf_x_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W))) 
                        {
                          kf_x_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::X_W, 1, kf_x_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B))) 
                        {
                          kf_y_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_B, 1, kf_y_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W))) 
                        {
                          kf_y_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_W, 1, kf_y_state(1,0));
                        }
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))) 
                        {
                          kf_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                          estimator_->setEEState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                        }
                    }
                }
            }
          else if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  if(estimator_->getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                    {//temporary
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B))) 
                        {
                          kfb_x_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::X_B, 1, kfb_x_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W))) 
                        {
                          kfb_x_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::X_W, 1, kfb_x_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B))) 
                        {
                          kfb_y_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_B, 1, kfb_y_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W))) 
                        {
                          kfb_y_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_W, 1, kfb_y_state(1,0));
                        }
                    }
                  if(estimator_->getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc")
                    {//temporary
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B))) 
                        {
                          kf_x_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::X_B, 1, kf_x_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W))) 
                        {
                          kf_x_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::X_W, 1, kf_x_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B))) 
                        {
                          kf_y_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::Y_B, 1, kf_y_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W))) 
                        {
                          kf_y_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::Y_W, 1, kf_y_state(1,0));
                        }
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W))) 
                        {
                          kf_z_state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                          estimator_->setEXState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                        }
                    }
                }
            }

          x_state.kf_pos = kf_x_state(0, 0);
          x_state.kf_vel = kf_x_state(1, 0);
          x_state.kfb_pos = kfb_x_state(0, 0);
          x_state.kfb_vel = kfb_x_state(1, 0);
          x_state.kfb_bias = kfb_x_state(2, 0);

          y_state.kf_pos = kf_y_state(0, 0);
          y_state.kf_vel = kf_y_state(1, 0);
          y_state.kfb_pos = kfb_y_state(0, 0);
          y_state.kfb_vel = kfb_y_state(1, 0);
          y_state.kfb_bias = kfb_y_state(2, 0);

          z_state.kf_pos = kf_z_state(0, 0);
          z_state.kf_vel = kf_z_state(1, 0);

          opt_data.states.push_back(x_state);
          opt_data.states.push_back(y_state);
          opt_data.states.push_back(z_state);

          optical_flow_pub_.publish(opt_data);

        }

      //更新
      previous_secs = current_secs;
      prev_raw_pos_z = raw_pos_z_;
    }

    void rosParamInit()
    {

      nhp_.param("level_vel_noise_sigma", level_vel_noise_sigma_, 0.01 );
      printf("level vel noise sigma  is %f\n", level_vel_noise_sigma_);

      nhp_.param("sonar_noise_sigma", sonar_noise_sigma_, 0.01 );
      printf("sonar noise sigma  is %f\n", sonar_noise_sigma_);

      if (!nhp_.getParam ("start_upper_thre", start_upper_thre_))
        start_upper_thre_ = 0;
      printf("%s: start_upper_thre_ is %.3f\n", nhp_.getNamespace().c_str(), start_upper_thre_);
      if (!nhp_.getParam ("start_lower_thre", start_lower_thre_))
        start_lower_thre_ = 0;
      printf("%s: start_lower_thre_ is %.3f\n", nhp_.getNamespace().c_str(), start_lower_thre_);
      if (!nhp_.getParam ("start_vel", start_vel_))
        start_vel_ = 0;
      printf("%s: start_vel_ is %.3f\n", nhp_.getNamespace().c_str(), start_vel_);

      std::string ns = nhp_.getNamespace();
      if (!nhp_.getParam ("x_axis_direction", x_axis_direction_))
        x_axis_direction_ = 1.0;
      printf("%s: x_axis_direction_ is %.3f\n", ns.c_str(), x_axis_direction_);

      if (!nhp_.getParam ("y_axis_direction", y_axis_direction_))
        y_axis_direction_ = -1.0; //-1 is default
      printf("%s: y_axisDirection_ is %.3f\n", ns.c_str(), y_axis_direction_);
    }
  };

};
#endif



