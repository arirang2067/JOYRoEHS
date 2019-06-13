#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <ctime>
#include <diagnostic_msgs/KeyValue.h>
#include <alice_foot_step_generator/FootStepCommand.h>

using namespace std;

class Command_generator 
{
public:
  Command_generator();
  std_msgs::String motion_command;
  std_msgs::String speed_command;
  alice_foot_step_generator::FootStepCommand FootParam;
  ros::Publisher vel_pub_;
  void Set_FootParam(void);
  void Write_Log(void);
  int command_switch;
  string speed_switch;
  string init_log_path;
  clock_t start_time;
  int init_hour, init_min, init_sec;
  ofstream out;
  //Text_Input//
  float Command_Period;
  /////////////

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void decisionCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command);
  void Input_Text(void);
  void Make_Log(void);


  ros::NodeHandle nh_;
  
  int linear_, angular_;
  
  ros::Subscriber joy_sub_;
  ros::Subscriber dec_sub_;
};

Command_generator::Command_generator():
  linear_(1),
  angular_(2)
{
  //Default_Setting//
  Input_Text();
  Make_Log();
  command_switch = 0;
  speed_switch = 2;
  FootParam.step_num = 4;
  FootParam.step_time = 1;
  FootParam.step_length = 0.2;
  FootParam.side_step_length = 0.05;
  FootParam.step_angle_rad = 0.3;
  start_time = clock();
  //////////////////
  
  ROS_INFO("command_generator_start");
  ROS_INFO("%f", (float)CLOCKS_PER_SEC);
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/command_generator", 10);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Command_generator::joyCallback, this);
  dec_sub_ = nh_.subscribe<diagnostic_msgs::KeyValue>("/heroehs/move_command", 10, &Command_generator::decisionCallback, this);
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
    speed_switch = "1";
  }
  else if(joy->buttons[7] == 1)
  {
    speed_switch = "3";
  }
  else if(joy->buttons[8] == 1)
  {
    speed_switch = "2";
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

void Command_generator::decisionCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command)
{
  if(move_command->key == "left")
  {
    FootParam.command = "left";
  }
  else if(move_command->key == "right")
  {
    FootParam.command = "right";
  }
  else if(move_command->key == "forward")
  {
    FootParam.command = "forward";
  }
  else if(move_command->key == "backward")
  {
    FootParam.command = "backward";
  }
  else if(move_command->key == "turn left")
  {
    FootParam.command = "turn left";
  }
  else if(move_command->key == "turn right")
  {
    FootParam.command = "turn right";
  }
  else if(move_command->key == "expanded left")
  {
    FootParam.command = "expanded left";
  }
  else if(move_command->key == "expanded right")
  {
    FootParam.command = "expanded right";
  }
  else if(move_command->key == "stop")
  {
    FootParam.command = "stop";
  }

  if(move_command->value == "1")
  {
    speed_switch = "1";
  }
  else if(move_command->value == "3")
  {
    speed_switch = "3";
  }
  else if(move_command->value == "2")
  {
    speed_switch = "2";
  }
  command_switch = 2;
}

void Command_generator::Set_FootParam(void)
{
  if(speed_switch == "1")
  {
    FootParam.step_num = 4;
    FootParam.step_time = 3;
    FootParam.step_length = 0.15;
    FootParam.side_step_length = 0.05;
    FootParam.step_angle_rad = 0.3;
  }
  else if(speed_switch == "2")
  {
    FootParam.step_num = 4;
    FootParam.step_time = 2;
    FootParam.step_length = 0.15;
    FootParam.side_step_length = 0.05;
    FootParam.step_angle_rad = 0.3;
  }
  else if(speed_switch == "3")
  {
    FootParam.step_num = 4;
    FootParam.step_time = 1;
    FootParam.step_length = 0.15;
    FootParam.side_step_length = 0.05;
    FootParam.step_angle_rad = 0.3;
  }
}

void Command_generator::Input_Text(void)
{
  int i = 0;
  size_t found;
  string init_pose_path;
  ifstream inFile;
  init_pose_path = ros::package::getPath("command_generator") + "/command_input.txt";
  inFile.open(init_pose_path.c_str());
  for(string line; std::getline(inFile,line);)
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

void Command_generator::Make_Log(void)
{
    time_t curr_time;
    struct tm *curr_tm;
    int year, month, day;
    curr_time = time(NULL);
    curr_tm = localtime(&curr_time);
    year = curr_tm->tm_year + 1900;
    month = curr_tm->tm_mon + 1;
    day = curr_tm->tm_mday;
    init_hour = curr_tm->tm_hour;
    init_min = curr_tm->tm_min;
    init_sec = curr_tm->tm_sec;
    char Logname[256];
    sprintf(Logname,"%d-%d-%d-%d-%d-%d",year,month,day,init_hour,init_min,init_sec);
    init_log_path = ros::package::getPath("command_generator") + "/log/" + Logname + ".txt";
    out.open(init_log_path.c_str());
    out<<"command|";
    out<<"step_num|";
    out<<"step_time|";
    out<<"step_length|";
    out<<"side_step_length|";
    out<<"step_angle_rad|";
    out<<"logtime|"<<'\n';
}

void Command_generator::Write_Log(void)
{
    clock_t curr_t;
    curr_t = clock();
    float result_time;
    result_time = (float)(curr_t-start_time)/(CLOCKS_PER_SEC);
    time_t curr_time;
    struct tm *curr_tm;
    int year, month, day, hour, min, sec;
    curr_time = time(NULL);
    curr_tm = localtime(&curr_time);
    year = curr_tm->tm_year + 1900;
    month = curr_tm->tm_mon + 1;
    day = curr_tm->tm_mday;
    hour = curr_tm->tm_hour - init_hour;
    min = curr_tm->tm_min - init_min;
    sec = curr_tm->tm_sec - init_sec;
    if(sec < 0) 
    {
      sec = sec+60;
      min = min - 1;
    }
    if(min < 0)
    {
      min = min+60;
      hour = hour - 1;
    }
    if(hour < 0)
    {
      hour = hour+24;
    }
    char Logname[256];
    sprintf(Logname,"%d:%d:%d",hour,min,sec);
    out<<FootParam.command<<"|";
    out<<FootParam.step_num<<"|";
    out<<FootParam.step_time<<"|";
    out<<FootParam.step_length<<"|";
    out<<FootParam.side_step_length<<"|";
    out<<FootParam.step_angle_rad<<"|";
    out<<Logname<<"|"<<'\n';
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
      {
        command_controller.vel_pub_.publish(command_controller.FootParam);
        command_controller.Write_Log();
      }
      count = 0;
    }
    else count += 1;
    usleep(1000);
    ros::spinOnce();
  }
  command_controller.out.close();
}

