#include <rolling/util/pwm_republisher.h>

MotorPWMRepublisher::MotorPWMRepublisher(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  nhp_.param("motor_num", motor_num_, 0);

  /* motor pwms */
  motor_pwm_sub_ = nh_.subscribe("motor_pwms", 1, &MotorPWMRepublisher::motorPwmCallback, this);
  motor_pwm_pubs_.resize(0);
  for(int i = 0; i < motor_num_; i++)
    {
      motor_pwm_pubs_.push_back(nh_.advertise<std_msgs::Float32>("debug/motor_pwm/motor" + std::to_string(i + 1), 1));
    }

  /* desire coordinate */
  desire_coordinate_sub_ = nh_.subscribe("desire_coordinate", 1, &MotorPWMRepublisher::desireCoordinateCallback, this);
  desire_coordinate_roll_pub_ = nh_.advertise<std_msgs::Float32>("debug/desire_coordinate/roll", 1);
  desire_coordinate_pitch_pub_ = nh_.advertise<std_msgs::Float32>("debug/desire_coordinate/pitch", 1);
  desire_coordinate_roll_ = 0.0;
  desire_coordinate_pitch_ = 0.0;

  /* timer */
  timer_ = nh_.createTimer(ros::Duration(0.1), &MotorPWMRepublisher::timerCallback, this);
}

void MotorPWMRepublisher::motorPwmCallback(const spinal::PwmsPtr & pwm_msg)
{
  if(pwm_msg->motor_value.size() != motor_num_) return;
  for(int i = 0; i < motor_num_; i++)
    {
      std_msgs::Float32 msg;
      msg.data = pwm_msg->motor_value.at(i);
      motor_pwm_pubs_.at(i).publish(msg);
    }
}

void MotorPWMRepublisher::desireCoordinateCallback(const spinal::DesireCoordPtr & desire_coordinate_msg)
{
  desire_coordinate_roll_ =  desire_coordinate_msg->roll;
  desire_coordinate_pitch_ =  desire_coordinate_msg->pitch;
}

void MotorPWMRepublisher::timerCallback(const ros::TimerEvent& e)
{
  std_msgs::Float32 msg;
  msg.data = desire_coordinate_roll_;
  desire_coordinate_roll_pub_.publish(msg);
  msg.data = desire_coordinate_pitch_;
  desire_coordinate_pitch_pub_.publish(msg);
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "motor_pwm_republisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MotorPWMRepublisher *motor_pwm_republisher = new MotorPWMRepublisher(nh, nh_private);
  ros::spin();
  delete motor_pwm_republisher;
  return 0;
}
