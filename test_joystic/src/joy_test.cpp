#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <time.h>

class TeleopTurtle
{
public:
  TeleopTurtle();
  std_msgs::String string;
  ros::Publisher vel_pub_;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  
  int linear_, angular_;
  
  ros::Subscriber joy_sub_;
};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{
  ROS_INFO("test_joystic_start");
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<std_msgs::String>("/heroehs/test_joystic", 10);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  ROS_INFO("time");
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
  else if(joy->buttons[0] == 1)
    string.data = "expanded left";
  else if(joy->buttons[1] == 1)
    string.data = "expanded right";
  else if(joy->buttons[2] == 1)
    string.data = "stop";
  else
    string.data = "stop";
  //vel_pub_.publish(this->string.data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;
  clock_t start, current;
  start = clock();
  
  while(ros::ok())
  {
    if((float)(current-start)/(CLOCKS_PER_SEC)>0.04)
    {
      ROS_INFO("time1");
      teleop_turtle.vel_pub_.publish(teleop_turtle.string.data);
      start = clock();   
    }
    else current = clock();
    usleep(1000);
    ros::spinOnce();
  }
}

