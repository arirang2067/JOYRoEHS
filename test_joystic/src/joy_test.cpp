#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <fstream>

class Joystic
{
public:
  Joystic();
  std_msgs::String string;
  ros::Publisher vel_pub_;
  int command_switch;
  //Text_Input//
  float Command_Period;
  /////////////
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void Text_Input(void);
  ros::NodeHandle nh_;
  
  int linear_, angular_;
  
  ros::Subscriber joy_sub_;
};


Joystic::Joystic():
  linear_(1),
  angular_(2)
{
  Text_Input();
  command_switch = 0;
  ROS_INFO("test_joystic_start");
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<std_msgs::String>("/heroehs/test_joystic", 10);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joystic::joyCallback, this);
}

void Joystic::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->axes[6] == 1)
  {
    string.data = "left";
    command_switch = 2;
  }
  else if(joy->axes[6] == -1)
  {
    string.data = "right";
    command_switch = 2;
  }
  else if(joy->axes[7] == 1)
  {
    string.data = "forward";
    command_switch = 2;
  }
  else if(joy->axes[7] == -1)
  {
    string.data = "backward";
    command_switch = 2;
  }
  else if(joy->buttons[4] == 1)
  {
    string.data = "turn left";
    command_switch = 2;
  }
  else if(joy->buttons[5] == 1)
  {
    string.data = "turn right";
    command_switch = 2;
  }
  else if(joy->buttons[0] == 1)
  {
    string.data = "expanded left";
    command_switch = 2;
  }
  else if(joy->buttons[1] == 1)
  {
    string.data = "expanded right";
    command_switch = 2;
  }
  else if(joy->buttons[2] == 1)
  {
    string.data = "stop";
    command_switch = 2;
  }
  else
  {
    if(command_switch == 2)
    {
      string.data = "stop";
      command_switch = 0;
    }
    else if(command_switch == 1)
    {
      string.data = "stop";
    }
    else if(command_switch == 0)command_switch = 0;
  }
  //ROS_INFO("%d",command_switch);
}

void Joystic::Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::string init_pose_path;
  std::ifstream inFile;
  init_pose_path = ros::package::getPath("test_joystic") + "/joystic_input.txt";
  inFile.open(init_pose_path.c_str());
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: Command_Period = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  Joystic joystic_controller;
  clock_t start, current;
  start = clock();
  float count;
  count = 0;
  ROS_INFO("%f",joystic_controller.Command_Period);

  while(ros::ok())
  {
    
    if(count > 1000*1.6)//joystic_controller.Command_Period
    {
      if(joystic_controller.command_switch > 0)
      joystic_controller.vel_pub_.publish(joystic_controller.string);
      count = 0;
    }
    else count += 1;
    usleep(1000);
    ros::spinOnce();
  }
}

