#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
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
  int joy_alice_id;
  double default_step_num;
  double default_step_length;
  double default_side_step_length;
  double default_step_angle_rad;
  double default_step_time;
  double expanded_step_num;
  double expanded_step_length;
  double expanded_side_step_length;
  double expanded_step_angle_rad;
  double expanded_step_time;
  double centered_step_num;
  double centered_step_length;
  double centered_side_step_length;
  double centered_step_angle_rad;
  double centered_step_time;

  ros::Publisher vel_pub_;
  string step_type;
  void Set_FootParam(void);
  void Write_Log(void);
  int command_switch;
  string speed_switch;
  string init_log_path;
  string current_status;
  string accept_or_ignore;
  clock_t start_time;
  int init_hour, init_min, init_sec;
  ofstream out;
  //Text_Input//
  float Command_Period;
  /////////////

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void decisionCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command);
  void alice_id_Callback(const std_msgs::String::ConstPtr& alice_id);
  void current_status_Callback(const std_msgs::String::ConstPtr& log_moving_status);
  void parse_step_param_data(std::string path);
  void Input_Text(void);
  void Make_Log(void);


  ros::NodeHandle nh_;
  
  int linear_, angular_;
  
  ros::Subscriber joy_sub_;
  ros::Subscriber dec_sub_;
  ros::Subscriber curr_status_sub_;
  ros::Subscriber alice_id_sub_;
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
  FootParam.step_num = 0;
  FootParam.step_length = 0;
  FootParam.side_step_length = 0;
  FootParam.step_angle_rad = 0;
  FootParam.step_time = 0;
  start_time = clock();
  //////////////////
  
  ROS_INFO("command_generator_start");
  ROS_INFO("%f", (float)CLOCKS_PER_SEC);
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/command_generator", 10);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Command_generator::joyCallback, this);
  dec_sub_ = nh_.subscribe<diagnostic_msgs::KeyValue>("/heroehs/move_command", 10, &Command_generator::decisionCallback, this);
  curr_status_sub_ = nh_.subscribe<std_msgs::String>("/heroehs/log_moving_status", 10, &Command_generator::current_status_Callback, this);
  alice_id_sub_ = nh_.subscribe<std_msgs::String>("/heroehs/alice_id", 10, &Command_generator::alice_id_Callback, this);
}

void Command_generator::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->axes[6] == 1)
  {
    FootParam.command = "left";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[6] == -1)
  {
    FootParam.command = "right";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[7] == 1)
  {
    FootParam.command = "forward";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[7] == -1)
  {
    FootParam.command = "backward";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[2] == -1)
  {
    FootParam.command = "centered left";
    step_type = "centered";
    command_switch = 2;
  }
  else if(joy->axes[5] == -1)
  {
    FootParam.command = "centered right";
    step_type = "centered";
    command_switch = 2;
  }
  else if(joy->buttons[4] == 1)
  {
    FootParam.command = "turn left";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->buttons[5] == 1)
  {
    FootParam.command = "turn right";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->buttons[0] == 1)
  {
    FootParam.command = "expanded left";
    step_type = "expanded";
    command_switch = 2;
  }
  else if(joy->buttons[1] == 1)
  {
    FootParam.command = "expanded right";
    step_type = "expanded";
    command_switch = 2;
  }
  else if(joy->buttons[2] == 1)
  {
    FootParam.command = "stop";
    step_type = "default";
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
      step_type = "default";
      command_switch = 0;
    }
    else if(command_switch == 1)
    {
      FootParam.command = "stop";
      step_type = "default";
    }
    else if(command_switch == 0)command_switch = 0;
  }
  Set_FootParam();
  //ROS_INFO("%d",command_switch);
}

void Command_generator::decisionCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command)
{
  if(move_command->key == "left")
  {
    FootParam.command = "left";
    step_type = "default";
  }
  else if(move_command->key == "right")
  {
    FootParam.command = "right";
    step_type = "default";
  }
  else if(move_command->key == "forward")
  {
    FootParam.command = "forward";
    step_type = "default";
  }
  else if(move_command->key == "backward")
  {
    FootParam.command = "backward";
    step_type = "default";
  }
  else if(move_command->key == "turn left")
  {
    FootParam.command = "turn left";
    step_type = "default";
  }
  else if(move_command->key == "turn right")
  {
    FootParam.command = "turn right";
    step_type = "default";
  }
  else if(move_command->key == "expanded left")
  {
    FootParam.command = "expanded left";
    step_type = "expanded";
  }
  else if(move_command->key == "expanded right")
  {
    FootParam.command = "expanded right";
    step_type = "expanded";
  }
  else if(move_command->key == "centered left")
  {
    FootParam.command = "centered left";
    step_type = "centered";
  }
  else if(move_command->key == "centered right")
  {
    FootParam.command = "centered right";
    step_type = "centered";
  }
  else if(move_command->key == "stop")
  {
    FootParam.command = "stop";
    step_type = "default";
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
  Set_FootParam();
}

void Command_generator::alice_id_Callback(const std_msgs::String::ConstPtr& alice_id)
{
  std::string step_path_;
  if(alice_id->data == "1")
  {
    joy_alice_id = 1;
    step_path_ = ros::package::getPath("alice_gui") + "/config/step_parameter1.yaml";
  }
  else if(alice_id->data == "2")
  {
    joy_alice_id = 2;
    step_path_ = ros::package::getPath("alice_gui") + "/config/step_parameter2.yaml";
  }
  parse_step_param_data(step_path_);
}

void Command_generator::current_status_Callback(const std_msgs::String::ConstPtr& log_moving_status)
{
  current_status = log_moving_status->data;
}

void Command_generator::Set_FootParam(void)
{
  if(step_type == "default")
  {
    FootParam.step_num = default_step_num;
    FootParam.step_length = default_step_length;
    FootParam.side_step_length = default_side_step_length;
    FootParam.step_angle_rad = default_step_angle_rad;
    FootParam.step_time = default_step_time;
  }
  else if(step_type == "expanded")
  {
    FootParam.step_num = expanded_step_num;
    FootParam.step_length = expanded_step_length;
    FootParam.side_step_length = expanded_side_step_length;
    FootParam.step_angle_rad = expanded_step_angle_rad;
    FootParam.step_time = expanded_step_time;
  }
  else if(step_type == "centered")
  {
    FootParam.step_num = centered_step_num;
    FootParam.step_length = centered_step_length;
    FootParam.side_step_length = centered_side_step_length;
    FootParam.step_angle_rad = centered_step_angle_rad;
    FootParam.step_time = centered_step_time;
  }
  if(joy_alice_id == 1)
  {
    if(speed_switch == "1")
    {
      FootParam.step_time = FootParam.step_time*1.5;
    }
    else if(speed_switch == "2")
    {
      FootParam.step_time = FootParam.step_time*1;
    }
    else if(speed_switch == "3")
    {
      FootParam.step_time = FootParam.step_time*0.5;
    }
  }
}
void Command_generator::parse_step_param_data(std::string path)
{
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	default_step_num = doc["default_step_num"].as<double>();
	default_step_length = doc["default_step_length"].as<double>();
	default_side_step_length = doc["default_side_step_length"].as<double>();
	default_step_angle_rad = doc["default_step_angle_radian"].as<double>();
	default_step_time = doc["default_step_time"].as<double>();

	expanded_step_num = doc["expanded_step_num"].as<double>();
	expanded_step_length = doc["expanded_step_length"].as<double>();
	expanded_side_step_length = doc["expanded_side_step_length"].as<double>();
	expanded_step_angle_rad = doc["expanded_step_angle_radian"].as<double>();
	expanded_step_time = doc["expanded_step_time"].as<double>();

	centered_step_num = doc["centered_step_num"].as<double>();
	centered_step_length = doc["centered_step_length"].as<double>();
	centered_side_step_length = doc["centered_side_step_length"].as<double>();
	centered_step_angle_rad = doc["centered_step_angle_radian"].as<double>();
	centered_step_time = doc["centered_step_time"].as<double>();
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
    out<<"status|";
    out<<"accept/ignore|";
    out<<"step_time|";
    out<<"step_num|";
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
    if(FootParam.command == current_status)accept_or_ignore = "accept";
    else accept_or_ignore = "ignore";
    char Logname[256];
    sprintf(Logname,"%d:%d:%d",hour,min,sec);
    out<<FootParam.command<<"|";
    out<<current_status<<"|";
    out<<accept_or_ignore<<"|";
    out<<FootParam.step_time<<"|";
    out<<FootParam.step_num<<"|";
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

