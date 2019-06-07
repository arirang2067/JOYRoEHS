#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  
  vel_pub_ = nh_.advertise<std_msgs::String>("/heroehs/test_joystic", 10);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::String string;
  if(joy->axes[6] == 1)
    string.data = "left";
  else if(joy->axes[6] == -1)
    string.data = "right";
  else if(joy->axes[7] == 1)
    string.data = "forward";
  else if(joy->axes[7] == -1)
    string.data = "backward";
  else if(joy->buttons[4] == 1)
    string.data = "turn left";
  else if(joy->buttons[5] == 1)
    string.data = "turn right";
  else if(joy->buttons[2] == 1)
    string.data = "stop";
  else
    string.data = "Default";
  vel_pub_.publish(string.data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

