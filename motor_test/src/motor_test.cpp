#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <takasako_sps/PowerInfo.h>

#define PWM_DU 2000.0f
#define STOP_PWM 0.5f // 1000 / 2000

class MotorTest
{
public:
  MotorTest(ros::NodeHandle nh, ros::NodeHandle nh_private)
   : nh_(nh), nhp_(nh_private)
  {
    nhp_.param("force_sensor_sub_name", force_sensor_sub_name_, std::string("forces"));
    nhp_.param("power_info_sub_name", power_info_sub_name_, std::string("power_info"));
    //nhp_.param("motor_pwm_sub_name", motor_pwm_sub_name_, std::string("/kduino/motor4_cmd"));
    nhp_.param("motor_pwm_sub_name", motor_pwm_sub_name_, std::string("power_pwm"));

    nhp_.param("auto_flag", auto_flag_, true);
    nhp_.param("duration", duration_, 4.0);
    nhp_.param("pwm_incremental_value", pwm_incremental_value_, 50);
    nhp_.param("min_pwm_value", min_pwm_value_, 1100);
    nhp_.param("max_pwm_value", max_pwm_value_, 1950);

    fp_ = fopen("motor_test.txt", "a+");
    if( fp_ == NULL ) {
      printf( "Cant not open file\n" );
    }

    log_flag_ = 0;
    pwm_value_ = 0;

    force_snesor_sub_ = nh_.subscribe<std_msgs::Int32MultiArray>(force_sensor_sub_name_, 1, &MotorTest::forceSensorCallback, this, ros::TransportHints().tcpNoDelay());
    power_info_sub_ = nh_.subscribe<takasako_sps::PowerInfo>(power_info_sub_name_, 1, &MotorTest::powerInfoCallback, this, ros::TransportHints().tcpNoDelay());
    start_cmd_sub_ =  nh_.subscribe<std_msgs::Empty>("start_log_cmd", 1,  &MotorTest::startCallback, this, ros::TransportHints().tcpNoDelay());
    sps_on_pub_ = nh.advertise<std_msgs::Empty>("/power_on_cmd", 1);

    if(!auto_flag_)
      {
        motor_pwm_sub_ = nh_.subscribe<std_msgs::UInt16>(motor_pwm_sub_name_, 1, &MotorTest::motorPwmCallback, this, ros::TransportHints().tcpNoDelay());
        log_flag_ = 2;
      }
    else
      {
        motor_pwm_pub_ = nh_.advertise<std_msgs::Float32>(motor_pwm_sub_name_,1);
      }

    ros::ServiceClient calib_client = nh_.serviceClient<std_srvs::Empty>("/cfs_sensor_calib");
    std_srvs::Empty srv;
    if (calib_client.call(srv))
      {
        ROS_INFO("done calib");
      }
    else
      {
        ROS_ERROR("Failed to call service add_two_ints");
      }

    sps_on_pub_.publish(std_msgs::Empty());

    if(auto_flag_)
      pwm_timer_ = nhp_.createTimer(ros::Duration(1.0 / 50), &MotorTest::pwmFunc,this);


  }

  ~MotorTest()
  {
    fprintf(fp_, "done\n"); 
    fclose(fp_);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber  force_snesor_sub_;
  ros::Subscriber motor_pwm_sub_;
  ros::Subscriber power_info_sub_;
  ros::Subscriber start_cmd_sub_;
  ros::Publisher motor_pwm_pub_;
  ros::Publisher sps_on_pub_;

  ros::Timer  pwm_timer_;

  std::string motor_pwm_sub_name_;
  std::string power_info_sub_name_;
  std::string force_sensor_sub_name_;
  uint16_t pwm_value_;

  bool auto_flag_;
  double duration_;
  int pwm_incremental_value_;
  int min_pwm_value_, max_pwm_value_;
  float currency_;

  ros::Time init_time_;

  FILE * fp_; // for log

  int log_flag_;

  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    log_flag_ = 1;

    pwm_value_ = min_pwm_value_;
    std_msgs::Float32 cmd_msg;
    cmd_msg.data = pwm_value_  / PWM_DU;
    motor_pwm_pub_.publish(cmd_msg);
    init_time_ = ros::Time::now();
    ROS_INFO("first time");

  }


  void powerInfoCallback(const takasako_sps::PowerInfoConstPtr & msg)
  {
    currency_ = msg->currency;
  }

  void forceSensorCallback(const std_msgs::Int32MultiArrayConstPtr & msg)
  {
    if(pwm_value_ >= 1150 && log_flag_ == 2) //1000 initialize input, must be set in the first step
      {
        float force_norm = sqrt(msg->data[0]*msg->data[0] + 
                                msg->data[1]*msg->data[1] +
                                msg->data[2]*msg->data[2]);
        
        fprintf(fp_, "%d %d %d %d %f %d %d %d %f\n", 
                pwm_value_, msg->data[0], msg->data[1], msg->data[2], force_norm, msg->data[3], msg->data[4], msg->data[5], currency_);
      }
  }

  void motorPwmCallback(const std_msgs::UInt16ConstPtr & msg)
  {
    pwm_value_ = msg->data;
  }

  void pwmFunc(const ros::TimerEvent & e)
  {
   
    if(!log_flag_) return;


    if(ros::Time::now().toSec() - init_time_.toSec() > duration_)
      {
        ROS_INFO("incremnetal");
        pwm_value_ += pwm_incremental_value_;

        if(pwm_value_ > max_pwm_value_)
          {
            std_msgs::Float32 cmd_msg;
            cmd_msg.data = STOP_PWM;
            motor_pwm_pub_.publish(cmd_msg);

            ROS_WARN("stop incremental");
            ros::shutdown();
          }

        std_msgs::Float32 cmd_msg;
        cmd_msg.data = pwm_value_ / PWM_DU;
        motor_pwm_pub_.publish(cmd_msg);
        log_flag_ = 1;
        init_time_ = ros::Time::now();

      }

    if(ros::Time::now().toSec() - init_time_.toSec() > 1.0)
      {
        if(log_flag_ == 1)  log_flag_ = 2;
      }

  }

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "force_sensor_log");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  MotorTest*  logNode = new MotorTest(nh, nh_private);
  ros::spin ();
  delete logNode;
  return 0;
}
