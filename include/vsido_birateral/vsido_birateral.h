#ifndef VSIDO_BIRATERAL_H_
#define VSIDO_BIRATERAL_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <interbotix_xs_msgs/TorqueEnable.h>
#include <interbotix_xs_msgs/Reboot.h>
#include <interbotix_xs_msgs/RobotInfo.h>
#include <interbotix_xs_msgs/MotorGains.h>
#include <interbotix_xs_msgs/TorqueEnable.h>
#include <interbotix_xs_msgs/OperatingModes.h>
#include <interbotix_xs_msgs/RegisterValues.h>

#include <interbotix_xs_sdk/xs_sdk_obj.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <yaml-cpp/yaml.h>

/*
#include <ros/ros.h>
#include <urdf/model.h>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "interbotix_xs_msgs/Reboot.h"
#include "interbotix_xs_msgs/RobotInfo.h"
#include "interbotix_xs_msgs/MotorGains.h"
#include "interbotix_xs_msgs/TorqueEnable.h"
#include "interbotix_xs_msgs/OperatingModes.h"
#include "interbotix_xs_msgs/RegisterValues.h"
#include "interbotix_xs_msgs/JointGroupCommand.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include "interbotix_xs_msgs/JointTrajectoryCommand.h"
*/

#ifndef B1000000
#define B1000000 0010010
#endif

#define SERIAL_BAUDRATE 1000000      // V-Sido Birateral Board is preset to 1M baud
#define SERIAL_DEVICE "/dev/ttyACM0" // Default Serial Port Name

struct YAMLConfigs
{
  std::vector<MotorInfo> motor_info_vec;
  std::vector<std::string> gripper_order;
  std::unordered_map<std::string, const ControlItem*> control_items;
  std::unordered_map<std::string, float> sleep_map;
  std::unordered_map<std::string, JointGroup> group_map;
  std::unordered_map<std::string, MotorState> motor_map;
  std::unordered_map<std::string, std::vector<std::string>> shadow_map;
  std::unordered_map<std::string, std::vector<std::string>> sister_map;
  std::unordered_map<std::string, Gripper> gripper_map;
  std::unordered_map<std::string, size_t> js_index_map;
};

// VSido Biratera Class
class VSidoBirateral
{
public:
  /// @brief Constructor for the VSidoBirateralROS
  /// @param node_handle - ROS NodeHandle
  /// @param success [out] - bool indicating if the node launched successfully
  //  explicit VSidoBirateral(ros::NodeHandle *node_handle, bool &success);
  explicit VSidoBirateral(ros::NodeHandle *node_handle, boost::asio::io_service &io_service, bool &success);

  /// @brief Destructor for the VSidoBirateralROS
  ~VSidoBirateral();

  void robot_get_joint_state(std::string const &name, float *position = NULL, float *velocity = NULL, float *effort = NULL);

  YAML::Node motor_configs;                                                     // Holds all the information in the motor_configs YAML file
  YAML::Node mode_configs;                                                      // Holds all the information in the mode_configs YAML file

private:
  int timer_hz = 100;     // Frequency at which the ROS Timer publishing joint states should run
  bool pub_states = true; // Boolean that determines if joint states should be published;

  bool once_received_flag = false;

  /*
  // pubに必要なデータ類
  std::vector<std::string> joint_names; // Names of all joints
  std::vector<std::string> joint_ids;   // Names of all joints
  uint8_t joint_num;                    // Number of all Joints
  size_t js_index;                      // Index in the published JointState message 'name' list belonging to the gripper motor
  float horn_radius;                    // Distance [m] from the motor horn's center to its edge
  float arm_length;                     // Distance [m] from the edge of the motor horn to a finger
  std::string left_finger;              // Name of the 'left_finger' joint as defined in the URDF (if present)
  std::string right_finger;             // Name of the 'right_finger' joint as defined in the URDF (if present)
  */

  ros::NodeHandle node;                   // ROS Node handle
  ros::NodeHandle master_node;                   // ROS Node handle
  ros::NodeHandle puppet_node;                   // ROS Node handle

  YAMLConfigs master_yaml_configs; // Holds all the information in the master_configs YAML file
  YAMLConfigs puppet_yaml_configs; // Holds all the information in the puppet_configs YAML file 

  ros::Publisher pub_master_joint_states; // ROS Publisher responsible for publishing joint states
  ros::Publisher pub_puppet_joint_states; // ROS Publisher responsible for publishing joint states

  ros::ServiceServer srv_master_torque_enable; // ROS Service Server used to torque on/off any motor
  ros::ServiceServer srv_puppet_torque_enable; // ROS Service Server used to torque on/off any motor

  ros::ServiceServer srv_master_set_operating_modes; // ROS Service Server used to set the operating mode of any motor
  ros::ServiceServer srv_puppet_set_operating_modes; // ROS Service Server used to set the operating mode of any motor

  ros::ServiceServer srv_master_reboot_motors; // ROS Service Server used to reboot any motor
  ros::ServiceServer srv_puppet_reboot_motors; // ROS Service Server used to reboot any motor

  ros::ServiceServer srv_master_get_robot_info; // ROS Service Server used to get information about the robot
  ros::ServiceServer srv_puppet_get_robot_info; // ROS Service Server used to get information about the robot

  ros::ServiceServer srv_master_set_motor_pid_gains; // ROS Service Server used to set the PID gains of any motor
  ros::ServiceServer srv_puppet_set_motor_pid_gains; // ROS Service Server used to set the PID gains of any motor

  ros::ServiceServer srv_master_set_motor_registers; // ROS Service Server used to set the registers of any motor
  ros::ServiceServer srv_puppet_set_motor_registers; // ROS Service Server used to set the registers of any motor

  ros::ServiceServer srv_master_get_motor_registers; // ROS Service Server used to get the registers of any motor
  ros::ServiceServer srv_puppet_get_motor_registers; // ROS Service Server used to get the registers of any motor

  ros::Timer tmr_joint_states;                 // ROS Timer used to continuously publish joint states
  sensor_msgs::JointState master_joint_states; // Holds the most recent JointState message
  sensor_msgs::JointState puppet_joint_states; // Holds the most recent JointState message

  // for rosparam
  std::string port_name = SERIAL_DEVICE; // USB port Name
  std::string side_name = "left";
  std::string master_robot_name = "master_" + side_name;
  std::string puppet_robot_name = "puppet_" + side_name;
  int baudrate = SERIAL_BAUDRATE;

  std::string js_topic="joint_states";

  std::string js_master_topic = master_robot_name + "/joint_state"; // Desired JointState topic name
  std::string js_puppet_topic = puppet_robot_name + "/joint_state"; // Desired JointState topic name

  int16_t diff_time;
  std::vector<int16_t> position_vec;
  std::vector<int16_t> data_vec;
  std::mutex position_vec_mutex;

  boost::asio::io_service &io;
  boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

  // unsigned char serial_read_buffer[1024];
  boost::asio::streambuf response_data;

  /// @brief Initializes the Robot Mode;
  /// @param <bool> [out] - True if the port was successfully; False otherwise
  bool init_model(void);

  bool master_get_motor_configs(void);
  bool puppet_get_motor_configs(void);  

  /// @brief Initializes the port to communicate with V-Sido Birateral Board
  /// @param <bool> [out] - True if the port was successfully opened; False otherwise
  bool init_port(void);

  /// @brief Initialize ROS Publishers
  void init_publishers(void);

  /// @brief Initialize ROS Subscribers
  void init_subscribers(void);

  /// @brief Initialize ROS Services
  void init_services(void);

  /// @brief Initialize ROS Timers
  void init_timers(void);

  void publish_joint_states(void);
  void publish_joint_states2(void);  

  /// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
  /// @param e - TimerEvent message
  void robot_update_joint_states(const ros::TimerEvent &e);

  float robot_convert_load(int const &value)
  {
       float load = 0;
   const float CURRENT_UNIT = 2.69f;// 2.69mA per unit
   load = (float)(value * CURRENT_UNIT);
   return load;
    /*
   float load = 0;
   const float LOAD_UNIT = 0.1f;
   if (value == 1023 || value == 0) load = 0.0f;
   else {
    load = (float)(value * LOAD_UNIT);
   } 
   return load;
   */
  };

  float robot_convert_angular_position_to_radius(int const &int_angular_position)
  {
    float f_deg = int_angular_position * 0.1;
    float rad = ((f_deg) / 180.0 * M_PI);

    return rad;
  };

  float robot_convert_linear_position_to_radian(Gripper const &gripper,float const &linear_position)
  {
  float half_dist = linear_position / 2.0;
  float arm_length = gripper.arm_length;
  float horn_radius = gripper.horn_radius;

    float result = 3.14159 / 2.0 - acos((pow(horn_radius, 2) + pow(half_dist, 2) - pow(arm_length, 2)) / (2 * horn_radius * half_dist));
    return result;
  };

  float robot_convert_angular_position_to_linear(Gripper const &gripper,float const &angular_position)
  {
    float arm_length = gripper.arm_length;
    float horn_radius = gripper.horn_radius;  

    float a1 = horn_radius * sin(angular_position);
    float c = sqrt(pow(horn_radius, 2) - pow(a1, 2));
    float a2 = sqrt(pow(arm_length, 2) - pow(c, 2));
    return a1 + a2;
  };

  bool sendSerialCmd(unsigned char ch);

  void doReceive(void);
  void data_received(const boost::system::error_code &error, size_t bytes_transffered);

  bool master_srv_torque_enable(interbotix_xs_msgs::TorqueEnable::Request &req, interbotix_xs_msgs::TorqueEnable::Response &res);

  bool master_srv_get_robot_info(interbotix_xs_msgs::RobotInfo::Request &req, interbotix_xs_msgs::RobotInfo::Response &res);
  bool puppet_srv_get_robot_info(interbotix_xs_msgs::RobotInfo::Request &req, interbotix_xs_msgs::RobotInfo::Response &res);

  bool robot_srv_torque_enable(interbotix_xs_msgs::TorqueEnable::Request &req, interbotix_xs_msgs::TorqueEnable::Response &res)
  {
    return true;
  }

  bool robot_srv_set_operating_modes(interbotix_xs_msgs::OperatingModes::Request &req, interbotix_xs_msgs::OperatingModes::Response &res)
  {
    return true;
  };

  bool robot_srv_reboot_motors(interbotix_xs_msgs::Reboot::Request &req, interbotix_xs_msgs::Reboot::Response &res)
  {
    return true;
  };

  bool robot_srv_get_robot_info(interbotix_xs_msgs::RobotInfo::Request &req, interbotix_xs_msgs::RobotInfo::Response &res)
  {

    ROS_INFO_STREAM("robot_srv_get_robot_info called." << req);
    return true;
  };

  bool robot_srv_set_motor_pid_gains(interbotix_xs_msgs::MotorGains::Request &req, interbotix_xs_msgs::MotorGains::Response &res)
  {
    return true;
  };

  bool robot_srv_set_motor_registers(interbotix_xs_msgs::RegisterValues::Request &req, interbotix_xs_msgs::RegisterValues::Response &res)
  {
    return true;
  };

  bool robot_srv_get_motor_registers(interbotix_xs_msgs::RegisterValues::Request &req, interbotix_xs_msgs::RegisterValues::Response &res)
  {
    return true;
  };

};

#endif
