/* 
0. flightNavCallback => implementation
1. the optical flow => indefference

4. flight mode vs navi command => development


 */

#include "aerial_robot_base/flight_navigation.h"



Navigator::Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private, 
                     BasicEstimator* estimator, FlightCtrlInput* flight_ctrl_input,
                     int ctrl_loop_rate)
 : nh_(nh, "navigator"),
   nhp_(nh_private, "navigator"),
   ctrl_loop_rate_(ctrl_loop_rate)
{
  Navigator::rosParamInit(nhp_);

  navi_sub_ = nh_.subscribe<aerial_robot_base::FlightNav>("full_states", 1, &Navigator::naviCallback, this, ros::TransportHints().tcpNoDelay());

  estimator_ = estimator;
  flight_ctrl_input_ = flight_ctrl_input;

  br_ =  new tf::TransformBroadcaster();

  final_target_pos_x_ = 0;
  final_target_vel_x_ = 0;
  final_target_pos_y_ = 0;
  final_target_vel_y_ = 0;
  final_target_pos_z_ = 0;
  final_target_vel_z_ = 0;
  final_target_theta_ = 0;
  final_target_vel_theta_ = 0;
  final_target_phy_ = 0;
  final_target_vel_phy_ = 0;
  final_target_psi_ = 0;
  final_target_vel_psi_ = 0;

  //current target value
  current_target_pos_x_ = 0;
  current_target_vel_x_ = 0;
  current_target_pos_y_ = 0;
  current_target_vel_y_ = 0;
  current_target_pos_z_ = 0;
  current_target_vel_z_ = 0;
  current_target_theta_ = 0;
  current_target_vel_theta_ = 0;
  current_target_phy_ = 0;
  current_target_vel_phy_ = 0;
  current_target_psi_ = 0;
  current_target_vel_psi_ = 0;

  target_pitch_angle_ = 0;
  target_roll_angle_ = 0;


  stopNavigation();
  stopFlight();
  setNaviCommand( IDLE_COMMAND );


  //base navigation mode init
  flight_mode_ = NO_CONTROL_MODE;


  gain_tunning_mode_ = 0; //no tunning mode

}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
  delete br_;
}

void Navigator::naviCallback(const aerial_robot_base::FlightNavConstPtr & msg)
{
  //control mode change (pos/vel)
  if(msg->command_mode == aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND)
    {
      //only change mode in world based control (only optical flow in forbidden)
      if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
        {
          ROS_INFO("change to vel pos-based control");
          xy_control_mode_ = VEL_WORLD_BASED_CONTROL_MODE;
        }
    }
  if(msg->command_mode == aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND)
    {
      //only change mode in world based control (only optical flow in forbidden)
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
        {
          ROS_INFO("change to pos control");
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
        }
    }

  //for x & y
  if(msg->command_mode == aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND)
    {
      setTargetVelX(msg->target_vel_x);
      setTargetVelY(msg->target_vel_y);
    }
  else if(msg->command_mode == aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND)
    {
      setTargetPosX(msg->target_pos_x);
      setTargetPosY(msg->target_pos_y);
    }

  //for z
  if(msg->pos_z_navi_mode == aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND)
    {
      addTargetPosZ(msg->target_pos_diff_z);
    }
  else if(msg->pos_z_navi_mode == aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND)
    {
      setTargetPosZ(msg->target_pos_z);
    }

  //for psi
  //not good, be careful
  if(msg->command_mode != aerial_robot_base::FlightNav::NO_NAVIGATION)
    {
      setTargetPsi(msg->target_psi);
    }
}


void Navigator::tfPublish()
{
  //TODO mutex
  tf::Transform map_to_target;
  tf::Quaternion tmp;
  
  map_to_target.setOrigin(tf::Vector3(current_target_pos_x_, current_target_pos_y_, current_target_pos_z_));
  tmp.setRPY(0.0, 0.0, current_target_psi_); 
  map_to_target.setRotation(tmp);
  ros::Time tm = ros::Time::now();
  br_->sendTransform(tf::StampedTransform(map_to_target, tm, map_frame_, target_frame_));

}

void Navigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  //*** teleop navigation

  if (!nh.getParam ("map_frame", map_frame_))
    map_frame_ = "unknown";
  printf("%s: map_frame_ is %s\n", ns.c_str(), map_frame_.c_str());

  if (!nh.getParam ("target_frame", target_frame_))
    target_frame_ = "unknown";
  printf("%s: target_frame_ is %s\n", ns.c_str(), target_frame_.c_str());

  if (!nh.getParam ("xy_control_mode", xy_control_mode_))
    xy_control_mode_ = 0;
  printf("%s: xy_control_mode_ is %d\n", ns.c_str(), xy_control_mode_);

  //hidden variable
  if (!nh.getParam ("xy_vel_mode_pos_ctrl_takeoff", xy_vel_mode_pos_ctrl_takeoff_))
    xy_vel_mode_pos_ctrl_takeoff_ = true;
  printf("%s: xy_vel_mode_pos_ctrl_takeoff is %s\n", ns.c_str(), xy_vel_mode_pos_ctrl_takeoff_ ? ("true") : ("false"));

}

TeleopNavigator::TeleopNavigator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 BasicEstimator* estimator, 
                                 FlightCtrlInput* flight_ctrl_input,
                                 int ctrl_loop_rate)
  :Navigator(nh, nh_private, estimator, flight_ctrl_input, ctrl_loop_rate)
{
  TeleopNavigator::rosParamInit(nhp_);


  //joystick init
  vel_control_flag_ = false;
  pos_control_flag_ = false;
  alt_control_flag_ = false;
  yaw_control_flag_ = false;

  arming_ack_sub_ = nh_.subscribe<std_msgs::Int8>("/arming_ack", 1, &TeleopNavigator::armingAckCallback, this, ros::TransportHints().tcpNoDelay());
  takeoff_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/takeoff", 1, &TeleopNavigator::takeoffCallback, this, ros::TransportHints().tcpNoDelay());
  halt_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/halt", 1, &TeleopNavigator::haltCallback, this, ros::TransportHints().tcpNoDelay());
  land_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/land", 1, &TeleopNavigator::landCallback, this, ros::TransportHints().tcpNoDelay());
  start_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/start", 1,&TeleopNavigator::startCallback, this, ros::TransportHints().tcpNoDelay());
  roll_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/roll", 1, &TeleopNavigator::rollCallback, this, ros::TransportHints().tcpNoDelay());
  pitch_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/pitch", 1, &TeleopNavigator::pitchCallback, this, ros::TransportHints().tcpNoDelay());
  yaw_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/yaw", 1, &TeleopNavigator::yawCallback, this, ros::TransportHints().tcpNoDelay());
  throttle_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/throttle", 1, &TeleopNavigator::throttleCallback, this, ros::TransportHints().tcpNoDelay());
  ctrl_mode_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/ctrl_mode", 1, &TeleopNavigator::xyControlModeCallback, this, ros::TransportHints().tcpNoDelay());

  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy_stick_command", 1, &TeleopNavigator::joyStickControl, this, ros::TransportHints().udp());

  if(flight_ctrl_input_->getMotorNumber() > 1)
    rc_cmd_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisCommand>("/aerial_robot_control", 10); 
  else
    {
    rc_cmd_pub_ = nh_.advertise<aerial_robot_msgs::RcData>("/kduino/rc_cmd", 10);
    rc_cmd_pub2_ = nh_.advertise<aerial_robot_msgs::RcData2>("/aerial_robot_control", 10); 

    } 
 
  stop_teleop_sub_ = nh_.subscribe<std_msgs::UInt8>("stop_teleop", 1, &TeleopNavigator::stopTeleopCallback, this, ros::TransportHints().tcpNoDelay());
  teleop_flag_ = true;

  config_cmd_pub_ = nh_.advertise<std_msgs::UInt16>("/config_cmd", 10);

  //temporarily
  joints_ctrl_pub_= nh_.advertise<std_msgs::Int8>("/teleop_command/joints_ctrl", 2);


}

TeleopNavigator::~TeleopNavigator()
{
  printf(" deleted teleop navigator input!\n");
}

void TeleopNavigator::armingAckCallback(const std_msgs::Int8ConstPtr& ack_msg)
{
  if(ack_msg->data == 0)
    {//  arming off
      ROS_INFO("STOP RES From AERIAL ROBOT");
      stopNavigation(); 
      setNaviCommand(IDLE_COMMAND);
    }
 
 if(ack_msg->data == 1)
    {//  arming on
      ROS_INFO("START RES From AERIAL ROBOT");
      startNavigation(); 
      setNaviCommand(IDLE_COMMAND);
    }
}


void TeleopNavigator::takeoffCallback(const std_msgs::EmptyConstPtr & msg){
  if(getStartAble())  
    {
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
      setNaviCommand(TAKEOFF_COMMAND);
      ROS_INFO("Takeoff command");
    }
  else  stopFlight();
}

void TeleopNavigator::startCallback(const std_msgs::EmptyConstPtr & msg)
  {//すべて軸に対して、初期化
    setNaviCommand(START_COMMAND);
    final_target_pos_x_ = getStatePosX();
    final_target_pos_y_ = getStatePosY();
    final_target_psi_   = getStatePsiBoard();
    final_target_pos_z_ = takeoff_height_;
    ROS_INFO("Start command");
  }

void TeleopNavigator::landCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) 
      xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
    setNaviCommand(LAND_COMMAND);
    //更新
    final_target_pos_x_ = getStatePosX();
    final_target_pos_y_ = getStatePosY();
    final_target_psi_   = getStatePsiBoard();
    final_target_pos_z_ = 0;
    ROS_INFO("Land command");
  }

void TeleopNavigator::haltCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
    setNaviCommand(STOP_COMMAND);
    flight_mode_ = RESET_MODE;
    setTargetPosX(getStatePosX());
    setTargetPosY(getStatePosY());
    setTargetPsi(getStatePsiBoard());
    setTargetPosZ(0);

    ROS_INFO("Halt command");
  }

void TeleopNavigator::rollCallback(const std_msgs::Int8ConstPtr & msg)
{// + : right ; - : left
  if(getStartAble() && getFlightAble())
    {
      setNaviCommand(HOVER_COMMAND);
      if(msg->data == 1){
        ROS_INFO("RIGHT");
        if(navi_frame_ == MAP_FRAME)
          final_target_pos_y_ -= even_move_distance_; //0.2m on now state
        else if(navi_frame_ == BODY_FRAME)
          {
	    if(fabs(cos(getStatePsiBoard())) > 
	       fabs(sin(getStatePsiBoard())))
	      {
		final_target_pos_y_-= left_right_distance_ *
		  (fabs(cos(getStatePsiBoard()))/cos(getStatePsiBoard()));
		final_target_pos_x_ = getStatePosX();
	      }
	    else if(fabs(cos(getStatePsiBoard())) <
		    fabs(sin(getStatePsiBoard())))
	      {
		final_target_pos_x_ += left_right_distance_ *
		  (fabs(sin(getStatePsiBoard()))/sin(getStatePsiBoard()));
		final_target_pos_y_= getStatePosY();
	      
	      }
          }
      }else{
        ROS_INFO("LEFT");
        if(navi_frame_ == MAP_FRAME)
          final_target_pos_y_+= even_move_distance_; //0.2m on now state
        else if(navi_frame_ == BODY_FRAME)
          {
            if(fabs(cos(getStatePsiBoard())) >
               fabs(sin(getStatePsiBoard())))
              {
                final_target_pos_y_+= left_right_distance_ * 
                  (fabs(cos(getStatePsiBoard())) / cos(getStatePsiBoard()));
                final_target_pos_x_= getStatePosX();
              }
            else if(fabs(cos(getStatePsiBoard())) <
                    fabs(sin(getStatePsiBoard())))
              {
                final_target_pos_x_-= left_right_distance_ * 
                  (fabs(sin(getStatePsiBoard())) / sin(getStatePsiBoard()));
                final_target_pos_y_= getStatePosY();
              }
	    }
	}
      }
  }

void TeleopNavigator::pitchCallback(const std_msgs::Int8ConstPtr & msg)
{// + : backward ; - : forward
  if(getStartAble() && getFlightAble())  
    {
      setNaviCommand(HOVER_COMMAND);
      if(msg->data == 1)
	{
	  ROS_INFO("FORWARD");
	  if(navi_frame_ == MAP_FRAME) final_target_pos_x_+= even_move_distance_; //0.2m
	  else if(navi_frame_ == BODY_FRAME)
	    {
	      if(fabs(cos(getStatePsiBoard())) >
		 fabs(sin(getStatePsiBoard()))){
		final_target_pos_x_ += forward_backward_distance_ * 
		  (fabs(cos(getStatePsiBoard())) / cos(getStatePsiBoard()));
		//final_target_pos_y_ = getStatePosy_();
	      }
	      else if(fabs(cos(getStatePsiBoard())) <
		      fabs(sin(getStatePsiBoard()))){
		//final_target_pos_x_= getStatePosX();
		final_target_pos_y_ += forward_backward_distance_ * 
		  (fabs(sin(getStatePsiBoard())) / sin(getStatePsiBoard()));
	      }
	    }
	}
      else
	{
	  ROS_INFO("BACKFORWARD");
	  if(navi_frame_ == MAP_FRAME) final_target_pos_x_-= even_move_distance_; //0.2m
	  else if(navi_frame_ == BODY_FRAME)
	    {
              //bad
	      final_target_pos_x_= getStatePosX();
	      final_target_pos_y_ = getStatePosY();
	    }
	}
    }
}


void TeleopNavigator::yawCallback(const std_msgs::Int8ConstPtr & msg)
{
  if(getStartAble() && getFlightAble())  
    { 
      setNaviCommand(HOVER_COMMAND);

      if(navi_frame_ == BODY_FRAME)
	{
	  final_target_pos_x_ = getStatePosX();
	  final_target_pos_y_ = getStatePosY();
	}


      if(msg->data == -1){
        final_target_psi_ = getStatePsiBoard() - M_PI/2.0; 
	ROS_INFO("CW");
      }else if(msg->data == 1){
        final_target_psi_ = getStatePsiBoard() + M_PI/2.0; 
	ROS_INFO("CCW");
      }

      if(msg->data == -2){
	//final_target_psi_ = getStatePsi() - 0.1; //5 degree 
        final_target_psi_ -= 0.1; //bad method
	ROS_INFO("delta:CW");
      }else if(msg->data == 2){
	//final_target_psi_ = getStatePsi() + 0.1;  //5 degree
        final_target_psi_ += 0.1; //bad method
	ROS_INFO("delta:CCW");
      }
      else
        {
          //nothing
        }
      //yaw_finalTargetの補正
      if(final_target_psi_ > M_PI) final_target_psi_ -= 2 * M_PI;
      else if(final_target_psi_ < -M_PI) final_target_psi_ += 2 *M_PI;
    }

}

void TeleopNavigator::throttleCallback(const std_msgs::Int8ConstPtr & msg)
{
  if(getStartAble() && getFlightAble())  
    { 
      setNaviCommand(HOVER_COMMAND);

      if(msg->data == 1){
	final_target_pos_z_ += up_down_distance_; 
	ROS_INFO("incremental UP");
      }else if (msg->data == -1){
	final_target_pos_z_ -= up_down_distance_; 
	ROS_INFO("incremental DOWN");
      }else if (msg->data == 0){
        final_target_pos_z_ -= up_down_distance_; 
        ROS_INFO("absolute throttle modification");
      }
      
    }
  //ROS_INFO("Thrust command");
}

void TeleopNavigator::xyControlModeCallback(const std_msgs::Int8ConstPtr & msg)
{
    if(getStartAble())
    {
      if(msg->data == 0)
        {
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("x/y position control mode");
        }
      if(msg->data == 1)
        {
          xy_control_mode_ = VEL_LOCAL_BASED_CONTROL_MODE;
          ROS_INFO("x/y velocity control mode");
        }
    }
}

void TeleopNavigator::stopTeleopCallback(const std_msgs::UInt8ConstPtr & stop_msg)
{
  if(stop_msg->data == 1) 
    {
      ROS_WARN("stop teleop control");
      teleop_flag_ = false;
    }
  else if(stop_msg->data == 0) 
    {
      ROS_WARN("start teleop control");
      teleop_flag_ = true;
    }
}


void TeleopNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{ //botton assignment: http://wiki.ros.org/ps3joy
  //temporary
  static bool joint_ctrl_flag   = false;


  if(gain_tunning_mode_ == ATTITUDE_GAIN_TUNNING_MODE)
    {
      static bool gain_tunning_flag_  = false;
      //ROS_INFO("ATTITUDE_GAIN_TUNNING_MODE");

      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          final_target_pos_z_ = takeoff_height_;  // 0.55m
          final_target_psi_  = getStatePsiBoard();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flight_mode_= RESET_MODE;
          setTargetPsi(getStatePsiBoard());

          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {
          if(getStartAble())  
            {
              setNaviCommand(TAKEOFF_COMMAND);
              ROS_INFO("Takeoff command");
            }
          else  stopFlight();

          return;
        }
      //landing
      if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
        {
          setNaviCommand(LAND_COMMAND);
          //更新
          final_target_pos_z_= 0; 
          final_target_psi_  = getStatePsiBoard();
          ROS_INFO("Land command");

          return;
        }

      //pitch && roll angle command
      if(joy_msg->buttons[1] == 0)
	{//no push the left joysitck
	  target_pitch_angle_ = joy_msg->axes[1] * target_angle_rate_;
	  target_roll_angle_ = - joy_msg->axes[0]  * target_angle_rate_;
	}
      if(joy_msg->buttons[1] == 1)
	{//push the left joysitck
	  ROS_INFO("large angle");
	  target_pitch_angle_
	    = joy_msg->axes[1] * target_angle_rate_ * cmd_angle_lev2_gain_;
	  target_roll_angle_
	    = - joy_msg->axes[0] * target_angle_rate_ * cmd_angle_lev2_gain_;
	}

      //throttle, TODO: not good
      if(fabs(joy_msg->axes[3]) > 0.2)
        {
          if(joy_msg->buttons[2] == 0 && getNaviCommand() == HOVER_COMMAND)
            {//push the right joysitck
              alt_control_flag_ = true;
              if(joy_msg->axes[3] >= 0) 
                final_target_pos_z_+= target_alt_interval_;
              else 
                final_target_pos_z_-= target_alt_interval_;
              ROS_INFO("Thrust command");
            }
        }
      else
        {
          if(alt_control_flag_)
            {
              alt_control_flag_= false;
              final_target_pos_z_= getStatePosZ();
              ROS_INFO("Fixed Alt command, targetPosz_is %f",final_target_pos_z_);
            }
        }

      //yaw
      if(joy_msg->buttons[2] == 1 && getNaviCommand() == HOVER_COMMAND)
        {
          if(fabs(joy_msg->axes[2]) > 0.05)
            {
              final_target_psi_ = getStatePsiBoard() + joy_msg->axes[2] * target_yaw_rate_;
              ROS_INFO("yaw control");
            }
          else
            final_target_psi_ = getStatePsiBoard();
        }

      //gain tunning
      if(joy_msg->buttons[10] == 1 && !gain_tunning_flag_) //left up trigger
        {
          std_msgs::UInt16 att_p_gain_cmd;
          att_p_gain_cmd.data = 161;
          config_cmd_pub_.publish(att_p_gain_cmd); 
          gain_tunning_flag_ = true;
        }
      if(joy_msg->buttons[8] == 1 && !gain_tunning_flag_) //left down trigger
        {
          std_msgs::UInt16 att_p_gain_cmd;
          att_p_gain_cmd.data = 162;
          config_cmd_pub_.publish(att_p_gain_cmd); 
          gain_tunning_flag_ = true;
        }
      if(joy_msg->buttons[11] == 1 && !gain_tunning_flag_) //right up trigger
        {
          std_msgs::UInt16 att_p_gain_cmd;
          att_p_gain_cmd.data = 167;
          config_cmd_pub_.publish(att_p_gain_cmd); 
          att_p_gain_cmd.data = 163;
          config_cmd_pub_.publish(att_p_gain_cmd); 
          gain_tunning_flag_ = true;
        }
      if(joy_msg->buttons[9] == 1 && !gain_tunning_flag_) //right down trigger
        {
          std_msgs::UInt16 att_p_gain_cmd;
          att_p_gain_cmd.data = 168;
          config_cmd_pub_.publish(att_p_gain_cmd); 
          att_p_gain_cmd.data = 164;
          config_cmd_pub_.publish(att_p_gain_cmd); 
          gain_tunning_flag_ = true;
        }
      if(joy_msg->buttons[8] == 0 &&  joy_msg->buttons[9] == 0 &&
         joy_msg->buttons[10] == 0 &&joy_msg->buttons[11] == 0 && gain_tunning_flag_)
        gain_tunning_flag_  = false;



    }
  else if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE || xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
   {
      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          final_target_pos_x_= getStatePosX();
          final_target_pos_y_= getStatePosY();
          final_target_pos_z_ = takeoff_height_;  // 0.55m
          final_target_psi_  = getStatePsiBoard();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flight_mode_= RESET_MODE;

          setTargetPosX(getStatePosX());
          setTargetPosY(getStatePosY());
          setTargetPsi(getStatePsiBoard());


          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {

          if(getStartAble())  
            {
              if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
              setNaviCommand(TAKEOFF_COMMAND);
              ROS_INFO("Takeoff command");
            }
          else  stopFlight();

          return;
        }
      //landing
      if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
        {
          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

          setNaviCommand(LAND_COMMAND);
          //更新
          final_target_pos_x_= getStatePosX();
          final_target_pos_y_= getStatePosY();
          final_target_pos_z_= 0; 
          final_target_psi_  = getStatePsiBoard();
          ROS_INFO("Land command");

          return;
        }

      //change to vel control mode
      if(joy_msg->buttons[12] == 1 && !vel_control_flag_)
        {
          ROS_INFO("change to vel pos-based control");
          vel_control_flag_ = true;
          xy_control_mode_ = VEL_WORLD_BASED_CONTROL_MODE;
          final_target_vel_x_= 0; current_target_vel_x_= 0;
          final_target_vel_y_= 0; current_target_vel_y_= 0;
        }
      if(joy_msg->buttons[12] == 0 && vel_control_flag_)
        vel_control_flag_ = false;

      //change to pos control mode
      if(joy_msg->buttons[14] == 1 && !pos_control_flag_)
        {
          ROS_INFO("change to pos control");
          pos_control_flag_ = true;
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          final_target_pos_x_= getStatePosX();
          final_target_pos_y_= getStatePosY();
        }
      if(joy_msg->buttons[14] == 0 && pos_control_flag_)
        pos_control_flag_ = false;

      //pitch && roll vel command for vel_mode
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE && getNaviCommand() == HOVER_COMMAND)
        {
          if(joy_msg->buttons[1] == 0)
            {//no push the left joysitck
              final_target_vel_x_= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_;
              final_target_vel_y_= joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_;
            }
          if(joy_msg->buttons[1] == 1)
            {//push the left joysitck
              ROS_INFO("strong vel control");
              final_target_vel_x_
                = joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_ * cmd_vel_lev2_gain_;
              final_target_vel_y_
                = joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_ * cmd_vel_lev2_gain_;
            }
        }

      //throttle, TODO: not good
      if(fabs(joy_msg->axes[3]) > 0.2)
        {
          if(joy_msg->buttons[2] == 0 && getNaviCommand() == HOVER_COMMAND)
            {//push the right joysitck
              alt_control_flag_ = true;
              if(joy_msg->axes[3] >= 0) 
                final_target_pos_z_+= target_alt_interval_;
              else 
                final_target_pos_z_-= target_alt_interval_;
              ROS_INFO("Thrust command");
            }
        }
      else
        {
          if(alt_control_flag_)
            {
              alt_control_flag_= false;
              final_target_pos_z_= getStatePosZ();
              ROS_INFO("Fixed Alt command, targetPosz_is %f",final_target_pos_z_);
            }
        }

#if 0 //no yaw 90 control
      //yaw
      if(!yaw_control_flag_)
        {
          if(joy_msg->buttons[10] == 1 && getNaviCommand() == HOVER_COMMAND)
            {//CCW
              ROS_INFO("CCW, change to pos control mode");
              yaw_control_flag_ = true;
              xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
              //final_target_psi_ += 3.1415/2.0; 
              final_target_psi_ = getStatePsiBoard() + M_PI/2.0; 
              if(final_target_psi_ > M_PI) final_target_psi_ -= 2 * M_PI;
              else if(final_target_psi_ < -M_PI) final_target_psi_ += 2 *M_PI;

              final_target_pos_x_= getStatePosX();
              final_target_pos_y_= getStatePosY();
            }
          if(joy_msg->buttons[11] == 1 && getNaviCommand() == HOVER_COMMAND)
            {//CW
              ROS_INFO("CW,, change to pos control mode");
              yaw_control_flag_ = true;
              xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

              final_target_psi_ = getStatePsiBoard() - 3.1415/2.0; 
              if(final_target_psi_ > M_PI) final_target_psi_ -= 2 * M_PI;
              else if(final_target_psi_ < -M_PI) final_target_psi_ += 2 *M_PI;

              final_target_pos_x_= getStatePosX();
              final_target_pos_y_= getStatePosY();
            }
        }
      if(yaw_control_flag_ && joy_msg->buttons[10] == 0 && joy_msg->buttons[11] == 0)
        {
          yaw_control_flag_ = false;
          ROS_INFO("stop yaw control");
        }
#endif 
      if(joy_msg->buttons[2] == 1 && getNaviCommand() == HOVER_COMMAND)
        {
          if(fabs(joy_msg->axes[2]) > 0.05)
            {
              final_target_psi_ = getStatePsiBoard() + joy_msg->axes[2] * target_yaw_rate_;
              ROS_INFO("yaw control");
            }
          else
            final_target_psi_ = getStatePsiBoard();
        }
    }
  else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE || xy_control_mode_ == POS_LOCAL_BASED_CONTROL_MODE)
    {
      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          final_target_pos_x_= getStatePosX();
          final_target_pos_y_= getStatePosY();
          final_target_pos_z_= takeoff_height_;  // 0.55m
          final_target_psi_  = getStatePsiBoard();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flight_mode_= RESET_MODE;

          setTargetPosX(getStatePosX());
          setTargetPosY(getStatePosY());
          setTargetPsi(getStatePsiBoard());

          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {
          if(getStartAble())  
            {
              setNaviCommand(TAKEOFF_COMMAND);
              ROS_INFO("Takeoff command");
            }
          else  stopFlight();

          return;
        }
      //landing
      if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
        {
          xy_control_mode_ = VEL_LOCAL_BASED_CONTROL_MODE;
          setNaviCommand(LAND_COMMAND);
          //更新
          final_target_pos_x_= getStatePosX();
          final_target_pos_y_= getStatePosY();
          final_target_pos_z_= 0; 
          final_target_psi_  = getStatePsiBoard();
          ROS_INFO("Land command");

          return;
        }

      if(getStartAble() && getFlightAble() && getNaviCommand() != LAND_COMMAND 
         && teleop_flag_)  
        {//start &  takeoff & !land
          setNaviCommand(HOVER_COMMAND);

          //pitch
          final_target_vel_x_= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_;
          //roll
          final_target_vel_y_= joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_;
          //throttle
          if(fabs(joy_msg->axes[3]) > 0.2)
            {
              alt_control_flag_ = true;
              if(joy_msg->axes[3] >= 0) 
                final_target_pos_z_+= target_alt_interval_;
              else 
                final_target_pos_z_-= target_alt_interval_;

              ROS_INFO("Thrust command");
            }
          else
            {
              if(alt_control_flag_)
                {
                  alt_control_flag_= false;
                  final_target_pos_z_= getStatePosZ();
                  ROS_INFO("Fixed Alt command, targetPosz_is %f",final_target_pos_z_);
                }
            }
          if(final_target_pos_z_< 0.35) final_target_pos_z_= 0.35; // shuisei 
          if(final_target_pos_z_> 3) final_target_pos_z_= 3;
          //yaw 1
          final_target_psi_ = joy_msg->axes[2] * target_yaw_rate_;
          //yaw 2
          if(joy_msg->buttons[10] == 1)
            {
              ROS_INFO("CCW");
              final_target_psi_ = target_yaw_rate_;
            }
          if(joy_msg->buttons[11] == 1)
            {
              ROS_INFO("CW");
              final_target_psi_ = - target_yaw_rate_;
            }

        }
    }

  if(!joint_ctrl_flag)
    {//joints angle ctrl
      if(joy_msg->buttons[8] == 1)
        {
          std_msgs::Int8 joints_ctrl_cmd;
          joints_ctrl_cmd.data = 7;
          joints_ctrl_pub_.publish(joints_ctrl_cmd);
          joint_ctrl_flag = true;
          ROS_INFO("to ku model");
        }
      if(joy_msg->buttons[10] == 1)
        {
          std_msgs::Int8 joints_ctrl_cmd;
          joints_ctrl_cmd.data = 8;
          joints_ctrl_pub_.publish(joints_ctrl_cmd);
          joint_ctrl_flag = true;
          ROS_INFO("to normal model");
        }
    }
  if(joy_msg->buttons[8] == 0 && joy_msg->buttons[10] == 0 && joint_ctrl_flag)
    joint_ctrl_flag = false;
}


void TeleopNavigator::targetValueCorrection()
{

  //no keystone correction
  current_target_pos_x_  = final_target_pos_x_;
  current_target_pos_y_= final_target_pos_y_;
  current_target_pos_z_  = final_target_pos_z_;
  current_target_vel_z_  = final_target_vel_z_;
  current_target_theta_ = final_target_theta_;
  current_target_vel_theta_ = final_target_vel_theta_;

  current_target_phy_ = final_target_phy_;
  current_target_vel_phy_ = final_target_vel_phy_;
  current_target_psi_ = final_target_psi_;
  current_target_vel_psi_ = final_target_vel_psi_;

#if 1 //keystone
  // for pitch and roll velocity control
  //pitch
  if(final_target_vel_x_ - current_target_vel_x_ > target_pitch_roll_interval_)
    current_target_vel_x_ += target_pitch_roll_interval_;
  else if (final_target_vel_x_ - current_target_vel_x_ < - target_pitch_roll_interval_)
    current_target_vel_x_ -= target_pitch_roll_interval_;
  else
    current_target_vel_x_ = final_target_vel_x_;
  //roll
  if(final_target_vel_y_ - current_target_vel_y_ > target_pitch_roll_interval_)
    current_target_vel_y_ += target_pitch_roll_interval_;
  else if (final_target_vel_y_ - current_target_vel_y_ < - target_pitch_roll_interval_)
    current_target_vel_y_ -= target_pitch_roll_interval_;
  else
    current_target_vel_y_ = final_target_vel_y_;
#else
  current_target_vel_x_ = final_target_vel_x_;
  current_target_vel_y_ = final_target_vel_y_;
#endif 

}


void TeleopNavigator::sendRcCmd()
{
  if(getNaviCommand() == START_COMMAND)
    { 
     ROS_INFO("START_COMMAND");
     std_msgs::UInt16 start_cmd;
     start_cmd.data = ARM_ON_CMD;
     config_cmd_pub_.publish(start_cmd); 
    }
  else if(getNaviCommand() == STOP_COMMAND)
    { 
      std_msgs::UInt16 stop_cmd;
      stop_cmd.data = ARM_OFF_CMD;
      config_cmd_pub_.publish(stop_cmd);
    }
  else if(getNaviCommand() == TAKEOFF_COMMAND ||
          getNaviCommand() == LAND_COMMAND ||
          getNaviCommand() == HOVER_COMMAND)
    {
      if(flight_ctrl_input_->getMotorNumber() > 1)
        {
          aerial_robot_msgs::FourAxisCommand four_axis_command_data;
          four_axis_command_data.angles[0]  =  flight_ctrl_input_->getRollValue();
          four_axis_command_data.angles[1] =  flight_ctrl_input_->getPitchValue();
          for(int i =0; i < flight_ctrl_input_->getMotorNumber(); i++)
            {
              four_axis_command_data.yaw_pi_term[i]   =  (flight_ctrl_input_->getYawValue())[i];
              four_axis_command_data.throttle_pid_term[i] = (flight_ctrl_input_->getThrottleValue())[i] ;
            }
          rc_cmd_pub_.publish(four_axis_command_data);
        }
      else
        {
          aerial_robot_msgs::RcData rc_data;
          rc_data.roll  =  flight_ctrl_input_->getRollValue();
          rc_data.pitch =  flight_ctrl_input_->getPitchValue();
          rc_data.yaw   =  (flight_ctrl_input_->getYawValue())[0];
          rc_data.throttle = (flight_ctrl_input_->getThrottleValue())[0];
          rc_cmd_pub_.publish(rc_data);


          aerial_robot_msgs::RcData2 rc_data2;
          rc_data2.roll  =  flight_ctrl_input_->getRollValue();
          rc_data2.pitch =  flight_ctrl_input_->getPitchValue();
          rc_data2.yaw   =  (flight_ctrl_input_->getYawValue())[0];
          rc_data2.throttle = (flight_ctrl_input_->getThrottleValue())[0];
          rc_cmd_pub2_.publish(rc_data2);

        }
    }
  else
    {
      //ROS_ERROR("ERROR PISITION COMMAND, CAN NOT BE SEND TO Quadcopter");
    }
}

void TeleopNavigator::teleopNavigation()
{
  static int convergence_cnt = 0; 
  static int clock_cnt = 0; //mainly for velocity control takeoff

  //keystron correction / target value process
  targetValueCorrection();

  if(getNaviCommand() == START_COMMAND)
    { //takeoff phase
      flight_mode_= NO_CONTROL_MODE;
      estimator_->setSensorFusionFlag(true);
      force_landing_flag_ = false; //is here good?
    }
  else if(getNaviCommand() == TAKEOFF_COMMAND)
    { //Take OFF Phase

      flight_mode_= TAKEOFF_MODE;

      //TODO convergenceFunction();
      if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
        {
          if (fabs(getTargetPosZ() - getStatePosZ()) < POS_Z_THRE &&
              fabs(getTargetPosX() - getStatePosX()) < POS_X_THRE &&
              fabs(getTargetPosY() - getStatePosY()) < POS_Y_THRE)
            convergence_cnt++;
        }
      else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE)
        {
          //TODO => check same as pos_world_based_control_mode
          if (fabs(getTargetPosZ() - getStatePosZ()) < POS_Z_THRE)
              convergence_cnt++;
        }

      if (convergence_cnt > ctrl_loop_rate_) 
	{ //*** 安定収束した 20 ~ 40
          if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
            {
              convergence_cnt = 0;
              setNaviCommand(HOVER_COMMAND); 
              startFlight();
              ROS_WARN(" x/y pos stable hovering POS_WORLD_BASED_CONTROL_MODE");
            }
          else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(xy_vel_mode_pos_ctrl_takeoff_)
                {
                  clock_cnt++;
                  ROS_WARN(" stable, clock_cnt: %d ", clock_cnt);
                  if(clock_cnt > (ctrl_loop_rate_ * TAKEOFF_COUNT))
                    {
                      clock_cnt = 0;
                      convergence_cnt = 0;
                      setNaviCommand(HOVER_COMMAND); 
                      startFlight();
                      ROS_WARN(" x/y pos stable hovering VEL_LOCAL_BASED_CONTROL_MODE");
                    }
                }
              else
                {
                  convergence_cnt = 0;
                  setNaviCommand(HOVER_COMMAND); 
                  startFlight();
                  ROS_WARN(" no x/y pos stable hovering ");
                }
            }
        }
    }
  else if(getNaviCommand() == LAND_COMMAND)
    {
      ROS_WARN(" land command");

      //state correct flag(Kalman Filter, especially for px4flow
      estimator_->setSensorFusionFlag(false);

      if (!getFlightAble()  && getStartAble()) 
	{
          ROS_ERROR(" land land mode mode");
	  setNaviCommand(STOP_COMMAND);
	  flight_mode_= RESET_MODE;

          setTargetPosX(getStatePosX());
          setTargetPosY(getStatePosY());
          setTargetPsi(getStatePsiBoard());


          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
	}

      else if (getFlightAble()  && getStartAble())
	{
	  flight_mode_= LAND_MODE; //--> for control

              if (fabs(getTargetPosZ() - getStatePosZ()) < POS_Z_THRE)
                convergence_cnt++;
              if (convergence_cnt > ctrl_loop_rate_) 
                { 
                  convergence_cnt = 0;
                  stopFlight();
                }
	} 
      else if (!getStartAble())
	{
	  flight_mode_= NO_CONTROL_MODE;
	  setNaviCommand(IDLE_COMMAND);

	}
    }
  else if(getNaviCommand() == HOVER_COMMAND)
    {
      flight_mode_= FLIGHT_MODE;
    }
  else if(getNaviCommand() == IDLE_COMMAND)
    {
      flight_mode_= NO_CONTROL_MODE;
    }
  else if(getNaviCommand() == STOP_COMMAND)
    {
      estimator_->setSensorFusionFlag(false);

      if(flight_mode_ != RESET_MODE)
        flight_mode_= NO_CONTROL_MODE;
    }
  else
    {
      flight_mode_= NO_CONTROL_MODE;
      ROS_WARN("ERROR COMMAND, CAN NOT BE SEND TO AERIAL ROBOT");
    }
}


void TeleopNavigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  if (!nh.getParam ("state_mode", state_mode_))
    state_mode_ = EGOMOTION_ESTIMATE;
  printf("state_mode_ is %f\n", state_mode_);

  //*** teleop navigation
  if (!nh.getParam ("takeoff_height", takeoff_height_))
    takeoff_height_ = 0;
  printf("%s: takeoff_height_ is %.3f\n", ns.c_str(), takeoff_height_);

  if (!nh.getParam ("even_move_distance", even_move_distance_))
    even_move_distance_ = 0;
  printf("%s: even_move_distance_ is %.3f\n", ns.c_str(), even_move_distance_);

  if (!nh.getParam ("up_down_distance", up_down_distance_))
    up_down_distance_ = 0;
  printf("%s: up_down_distance_ is %.3f\n", ns.c_str(), up_down_distance_);

  if (!nh.getParam ("forward_backward_distance", forward_backward_distance_))
    forward_backward_distance_ = 0;
  printf("%s: forward_backward_distance_ is %.3f\n", ns.c_str(), forward_backward_distance_);

  if (!nh.getParam ("left_right_distance", left_right_distance_))
    left_right_distance_ = 0;
  printf("%s: left_right_distance_ is %.3f\n", ns.c_str(), left_right_distance_);

  if (!nh.getParam ("target_vel_rate", target_vel_rate_))
    target_vel_rate_ = 0;
  printf("%s: target_vel_rate_ is %.3f\n", ns.c_str(), target_vel_rate_);

  if (!nh.getParam ("target_pitch_roll_interval", target_pitch_roll_interval_))
    target_pitch_roll_interval_ = 0;
  printf("%s: target_pitch_roll_interval_ is %.3f\n", ns.c_str(), target_pitch_roll_interval_);

  if (!nh.getParam ("target_alt_interval", target_alt_interval_))
    target_alt_interval_ = 0;
  printf("%s: target_alt_interval_ is %.3f\n", ns.c_str(), target_alt_interval_);

  if (!nh.getParam ("target_yaw_rate", target_yaw_rate_))
    target_yaw_rate_ = 0;
  printf("%s: target_yaw_rate_ is %.3f\n", ns.c_str(), target_yaw_rate_);

  if (!nh.getParam ("navi_frame_int", navi_frame_int_))
    navi_frame_int_ = 0;
  printf("%s: navi_frame_int_ is %d\n", ns.c_str(), navi_frame_int_);
  navi_frame_ = navi_frame_int_;

  //hidden variable
  if (!nh.getParam ("cmd_vel_lev2_gain", cmd_vel_lev2_gain_))
    cmd_vel_lev2_gain_ = 1.0;
  printf("%s: cmd_vel_lev2_gain_ is %.3f\n", ns.c_str(), cmd_vel_lev2_gain_);

  if (!nh.getParam ("gain_tunning_mode", gain_tunning_mode_))
    gain_tunning_mode_ = 0;
  printf("%s: gain_tunning_mode_ is %d\n", ns.c_str(), gain_tunning_mode_);
  if (!nh.getParam ("target_angle_rate", target_angle_rate_))
    target_angle_rate_ = 1.0;
  printf("%s: target_angle_rate_ is %f\n", ns.c_str(), target_angle_rate_);
  if (!nh.getParam ("cmd_angle_lev2_gain", cmd_angle_lev2_gain_))
    cmd_angle_lev2_gain_ = 1.0;
  printf("%s: cmd_angle_lev2_gain_ is %f\n", ns.c_str(), cmd_angle_lev2_gain_);

}
