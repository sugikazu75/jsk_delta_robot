// **** the dynamicxel motor number is opposite to the joint num, should be fixed

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <string>

/*
  control_node => dynamicxel_bridge => real robot => dynamixel_bridge => robot model
 */

typedef struct{
  ros::Subscriber joint_state_sub;
  std::string joint_state_sub_name;
  ros::Publisher joint_ctrl_pub;
  std::string joint_ctrl_pub_name;
  std::string joint_name;
  float current_angle;
  float target_angle;
  int angle_sgn;
  double angle_offset;
  double angle_max;
  double angle_min;
}JointInfo;


class HydraJoints
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;


  int joint_num_;
  std::vector<JointInfo> joints_info_;

  std::string start_sub_name_, stop_sub_name_;
  ros::Subscriber start_sub_, stop_sub_;
  ros::Publisher joints_state_pub_;
  ros::Subscriber joints_ctrl_sub_ ; //for manual transform
  std::string joints_ctrl_sub_name_;

  bool start_flag;
  
  double bridge_rate_;
  ros::Timer  bridge_timer_;

  //temporary control
  ros::Publisher joint1_pub_ ,joint2_pub_, joint3_pub_;
  ros::Subscriber joints_ctrl_temp_sub_ ; //for manual transform

public:

  HydraJoints(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh),nhp_(nhp)
  {

    nhp_.param("joint_num", joint_num_, 3); 
    
    joints_info_.resize(joint_num_);    
    //for(std::vector<JointInfo>::iterator joint_info = joints_info_.begin(); joint_info != joints_info_.end(); joint_info++)
    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        //std::stringstream reverse_joint_no;
        //reverse_joint_no << joint_num_ - i; // reverse
        //joints_info_[i].joint_ctrl_pub_name = std::string("/j") + reverse_joint_no.str()  + std::string("_controller/command");

        joints_info_[i].joint_ctrl_pub_name = std::string("/j") + joint_no.str()  + std::string("_controller/command");
        joints_info_[i].joint_ctrl_pub = nh_.advertise<std_msgs::Float64>(joints_info_[i].joint_ctrl_pub_name, 1); 

        joints_info_[i].joint_state_sub_name = std::string("/j") + joint_no.str()  + std::string("_controller/state");
        //joints_info_[i].joint_state_sub = nh_.subscribe<dynamixel_msgs::JointState>(joints_info_[i].joint_state_sub_name, 1, boost::bind(&HydraJoints::jointCallback, this, _1, joint_num_ - i - 1)); //reverse
        joints_info_[i].joint_state_sub = nh_.subscribe<dynamixel_msgs::JointState>(joints_info_[i].joint_state_sub_name, 1, boost::bind(&HydraJoints::jointCallback, this, _1, i)); //reverse

        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_max"), joints_info_[i].angle_max, 1.57); //real angle
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_min"), joints_info_[i].angle_min, -1.57); //real angle
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_sgn"), joints_info_[i].angle_sgn, 1);
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_offset"), joints_info_[i].angle_offset, 0.0);

        std::stringstream joint_no2;
        joint_no2 << i + 1;
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_name"), joints_info_[i].joint_name, std::string("joint") + joint_no2.str());

        ROS_INFO("joint%d attribute: angle_max: %f, angle_min: %f, angle_sng: %d, angle_offset: %f", i + 1, joints_info_[i].angle_max, joints_info_[i].angle_min, joints_info_[i].angle_sgn, joints_info_[i].angle_offset);
      }

    nhp_.param("start_sub_name", start_sub_name_, std::string("hydra/joints_start"));
    nhp_.param("stop_sub_name", stop_sub_name_, std::string("hydra/joints_stop"));
    nhp_.param("joints_ctrl_sub_name", joints_ctrl_sub_name_, std::string("hydra/joints_ctrl"));
    joints_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    joints_ctrl_sub_ = nh_.subscribe<sensor_msgs::JointState>(joints_ctrl_sub_name_, 1, &HydraJoints::jointsCtrlCallback, this, ros::TransportHints().tcpNoDelay());
    start_sub_ = nh_.subscribe<std_msgs::Empty>(start_sub_name_, 1, &HydraJoints::startCallback, this, ros::TransportHints().tcpNoDelay());
    stop_sub_ = nh_.subscribe<std_msgs::Empty>(stop_sub_name_, 1, &HydraJoints::stopCallback, this, ros::TransportHints().tcpNoDelay());

    nhp_.param("bridge_rate", bridge_rate_, 40.0);
    bridge_timer_ = nhp_.createTimer(ros::Duration(1.0 / bridge_rate_), &HydraJoints::bridgeFunc, this);

    //temporary control
    joints_ctrl_temp_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/joints_ctrl",  1, &HydraJoints::jointsTempCtrlCallback, this, ros::TransportHints().tcpNoDelay());
    joint1_pub_ = nh_.advertise<std_msgs::Float64>("/j1_controller/command", 1);
    joint2_pub_ = nh_.advertise<std_msgs::Float64>("/j2_controller/command", 1);
    joint3_pub_ = nh_.advertise<std_msgs::Float64>("/j3_controller/command", 1);

  }



  ~HydraJoints()
  {
  }

  void jointCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i)
  {
    
    joints_info_[i].current_angle = joints_info_[i].angle_sgn * (msg->current_pos - joints_info_[i].angle_offset);
    //ROS_INFO("i:%d, current_angle: %f, current_pose:%f, angle_sgn: %d, angle_offset: %f", i, joints_info_[i].current_angle, msg->current_pos, joints_info_[i].angle_sgn, joints_info_[i].angle_offset);
    
    //bad!!!
#if 1 //the singular position(pi/2)
    if(fabs(joints_info_[i].current_angle - M_PI/2) < 0.09) // 0.18 = 10 / 180 * pi ; 0.09 = 5 / 180 * pi
      joints_info_[i].current_angle= M_PI/2;

    if(fabs(joints_info_[i].current_angle + M_PI/2) < 0.09) // 0.18 = 10 / 180 * pi ; 0.09 = 5 / 180 * pi
      joints_info_[i].current_angle= -M_PI/2;

#endif
  }


  void jointsCtrlCallback(const sensor_msgs::JointStateConstPtr& joints_ctrl_msg)
  {
    for(int i = 0; i < joint_num_; i ++)
      {
        //if(joints_ctrl_msg->name[i] == joints_info_[i].joint_name)
        if(1)
          {
            //ROS_INFO("joint %d position %f", i, joints_ctrl_msg->position[i]);
            joints_info_[i].target_angle = joints_ctrl_msg->position[i];
            double dynamicxel_postion = joints_info_[i].target_angle * joints_info_[i].angle_sgn  + joints_info_[i].angle_offset;

            std_msgs::Float64 command;
            command.data = dynamicxel_postion;
            joints_info_[i].joint_ctrl_pub.publish(command);
          }
      }
  }


  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    ROS_INFO("start joints rotation");
    start_flag = true;
  }

  void stopCallback(const std_msgs::EmptyConstPtr & msg)
  {
    ROS_INFO("stop joints rotation");
    start_flag = false;
    
  }

  void bridgeFunc(const ros::TimerEvent & e)
  {
    sensor_msgs::JointState hydra_joints_state;
    hydra_joints_state.header.stamp = ros::Time::now();
    hydra_joints_state.name.resize(joint_num_);
    hydra_joints_state.position.resize(joint_num_);
    
    for(int i = 0; i < joint_num_; i ++)
      {
        hydra_joints_state.name[i] = joints_info_[i].joint_name;
        hydra_joints_state.position[i] = joints_info_[i].current_angle;
        //hydra_joints_state.position[i] = 1.57;
      }

    joints_state_pub_.publish(hydra_joints_state);
  }


  //temporary control
  const static  double transform_delta_angle_ = 0.1;
void jointsTempCtrlCallback(const std_msgs::Int8ConstPtr& msg)
  {
    static bool init_flag = true;
    if(init_flag)
      {
        joints_info_[0].target_angle = joints_info_[0].current_angle;
        joints_info_[1].target_angle = joints_info_[1].current_angle;
        joints_info_[2].target_angle = joints_info_[2].current_angle;
        init_flag = false;
      }
    
    //joint1 
    if(msg->data == 1)
      { // joint1 --
        joints_info_[0].target_angle -= transform_delta_angle_;
        std_msgs::Float64 command;
        command.data = joints_info_[0].target_angle * joints_info_[0].angle_sgn + joints_info_[0].angle_offset;
        joint1_pub_.publish(command);
        ROS_INFO("1: command.data: %f", command.data);
      }
    if(msg->data == 2)
      { // joint1 ++
        joints_info_[0].target_angle += transform_delta_angle_;
        std_msgs::Float64 command;
        command.data = joints_info_[0].target_angle * joints_info_[0].angle_sgn + joints_info_[0].angle_offset;
        joint1_pub_.publish(command);
        ROS_INFO("1: command.data: %f", command.data);
      }
    if(msg->data == 3)
      { // joint3 --
        joints_info_[1].target_angle -= transform_delta_angle_;
        std_msgs::Float64 command;
        command.data = joints_info_[1].target_angle * joints_info_[1].angle_sgn  + joints_info_[1].angle_offset;
        ROS_INFO("2: command.data: %f", command.data);
        joint2_pub_.publish(command);
      }
    if(msg->data == 4)
      { // joint3 ++
        joints_info_[1].target_angle += transform_delta_angle_;
        std_msgs::Float64 command;
        command.data = joints_info_[1].target_angle * joints_info_[1].angle_sgn + joints_info_[1].angle_offset;
        joint2_pub_.publish(command);
        ROS_INFO("2: command.data: %f", command.data);
      }
    if(msg->data == 5)
      { // joint2 --
        joints_info_[2].target_angle -= transform_delta_angle_;
        std_msgs::Float64 command;
        command.data = joints_info_[2].target_angle * joints_info_[2].angle_sgn + joints_info_[2].angle_offset;
        ROS_INFO("3: command.data: %f", command.data);
        joint3_pub_.publish(command);
      }
    if(msg->data == 6)
      { // joint2 ++
        joints_info_[2].target_angle += transform_delta_angle_;
        std_msgs::Float64 command;
        command.data = joints_info_[2].target_angle * joints_info_[2].angle_sgn + joints_info_[2].angle_offset;
        joint3_pub_.publish(command);
        ROS_INFO("3: command.data: %f", command.data);
      }
    if(msg->data == 7)
      { // joint1&3 --
        joints_info_[2].target_angle -= transform_delta_angle_;
        std_msgs::Float64 command1;
        command1.data = joints_info_[2].target_angle * joints_info_[2].angle_sgn + joints_info_[2].angle_offset;
        joint1_pub_.publish(command1);
        ROS_INFO("1&3: command: %f", command1.data);

        joints_info_[0].target_angle -= transform_delta_angle_;
        std_msgs::Float64 command3;
        command3.data = joints_info_[0].target_angle * joints_info_[0].angle_sgn + joints_info_[0].angle_offset;
        joint3_pub_.publish(command3);

      }
    if(msg->data == 8)
      { // joint1&3 ++
        joints_info_[2].target_angle += transform_delta_angle_;
        std_msgs::Float64 command1;
        command1.data = joints_info_[2].target_angle * joints_info_[2].angle_sgn + joints_info_[2].angle_offset;
        joint1_pub_.publish(command1);
        ROS_INFO("1&3: command: %f", command1.data);

        joints_info_[0].target_angle += transform_delta_angle_;
        std_msgs::Float64 command3;
        command3.data = joints_info_[0].target_angle * joints_info_[0].angle_sgn + joints_info_[0].angle_offset;
        joint3_pub_.publish(command3);

      }


  }



};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hydra_joitns");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  HydraJoints *hydra_joints = new HydraJoints(n,np);
  ros::spin();
  delete hydra_joints;

  return 0;
}
