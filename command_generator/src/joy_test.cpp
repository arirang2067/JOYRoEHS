#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <fstream>
#include <command_generator/FootStepCommand.h>

class Command_generator 
{
public:
  Command_generator();
  std_msgs::String motion_command;
  std_msgs::String speed_command;
  command_generator::FootStepCommand FootParam;
  ros::Publisher vel_pub_;
  void Set_FootParam(void);
  int command_switch;
  int speed_switch;
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

Command_generator::Command_generator():
  linear_(1),
  angular_(2)
{
  //Default_Setting//
  Text_Input();
  command_switch = 0;
  speed_switch = 2;
  FootParam.step_num = 4;
  FootParam.step_time = 1;
  FootParam.step_length = 0.2;
  FootParam.side_step_length = 0.05;
  FootParam.step_angle_rad = 0.3;
  //////////////////
  ROS_INFO("command_generator_start");
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<command_generator::FootStepCommand>("/heroehs/command_generator", 10);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Command_generator::joyCallback, this);
}

void Command_generator::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->axes[6] == 1)
  {
    FootParam.command = "left";
    command_switch = 2;
  }
  else if(joy->axes[6] == -1)
  {
    FootParam.command = "right";
    command_switch = 2;
  }
  else if(joy->axes[7] == 1)
  {
    FootParam.command = "forward";
    command_switch = 2;
  }
  else if(joy->axes[7] == -1)
  {
    FootParam.command = "backward";
    command_switch = 2;
  }
  else if(joy->buttons[4] == 1)
  {
    FootParam.command = "turn left";
    command_switch = 2;
  }
  else if(joy->buttons[5] == 1)
  {
    FootParam.command = "turn right";
    command_switch = 2;
  }
  else if(joy->buttons[0] == 1)
  {
    FootParam.command = "expanded left";
    command_switch = 2;
  }
  else if(joy->buttons[1] == 1)
  {
    FootParam.command = "expanded right";
    command_switch = 2;
  }
  else if(joy->buttons[2] == 1)
  {
    FootParam.command = "stop";
    command_switch = 2;
  }
  else if(joy->buttons[6] == 1)
  {
    speed_switch = 1;
  }
  else if(joy->buttons[7] == 1)
  {
    speed_switch = 3;
  }
  else if(joy->buttons[8] == 1)
  {
    speed_switch = 2;
  }
  else
  {
    if(command_switch == 2)
    {
      FootParam.command = "stop";
      command_switch = 0;
    }
    else if(command_switch == 1)
    {
      FootParam.command = "stop";
    }
    else if(command_switch == 0)command_switch = 0;
  }
  //ROS_INFO("%d",command_switch);
}

void Command_generator::Set_FootParam(void)
{
  if(speed_switch == 1)
  {
    FootParam.step_num = 4;
    FootParam.step_time = 3;
    FootParam.step_length = 0.2;
    FootParam.side_step_length = 0.05;
    FootParam.step_angle_rad = 0.3;
  }
  else if(speed_switch == 2)
  {
    FootParam.step_num = 4;
    FootParam.step_time = 2;
    FootParam.step_length = 0.2;
    FootParam.side_step_length = 0.05;
    FootParam.step_angle_rad = 0.3;
  }
  else if(speed_switch == 3)
  {
    FootParam.step_num = 4;
    FootParam.step_time = 1;
    FootParam.step_length = 0.2;
    FootParam.side_step_length = 0.05;
    FootParam.step_angle_rad = 0.3;
  }
}

void Command_generator::Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::string init_pose_path;
  std::ifstream inFile;
  init_pose_path = ros::package::getPath("command_generator") + "/command_input.txt";
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
  ros::init(argc, argv, "command_generator");
  Command_generator command_controller;
  clock_t start, current;
  start = clock();
  float count;
  count = 0;
  ROS_INFO("%f",command_controller.Command_Period);

  while(ros::ok())
  {
    
    if(count > 1000*command_controller.Command_Period)
    {
      command_controller.Set_FootParam();
      if(command_controller.command_switch > 0)
      command_controller.vel_pub_.publish(command_controller.FootParam);
      count = 0;
    }
    else count += 1;
    usleep(1000);
    ros::spinOnce();
  }
}

