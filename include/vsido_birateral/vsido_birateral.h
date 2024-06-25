#ifndef VSIDO_BIRATERAL_H_
#define VSIDO_BIRATERAL_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <interbotix_xs_msgs/TorqueEnable.h>

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

#define SERIAL_BAUDRATE 1000000 // V-Sido Birateral Board is preset to 1M baud
#define SERIAL_DEVICE "/dev/ttyACM0"  // Default Serial Port Name

// VSido Biratera Class
class VSidoBirateral
{
public:

  /// @brief Constructor for the VSidoBirateralROS
  /// @param node_handle - ROS NodeHandle
  /// @param success [out] - bool indicating if the node launched successfully
//  explicit VSidoBirateral(ros::NodeHandle *node_handle, bool &success);

  explicit VSidoBirateral(ros::NodeHandle *node_handle,boost::asio::io_service &io_service, bool &success);

  /// @brief Destructor for the VSidoBirateralROS
  ~VSidoBirateral();

  /// @brief Get states for a single joint
  /// @param name - desired joint name
  /// @param position [out] - current joint position [rad]
  /// @param velocity [out] - current joint velocity [rad/s]
  /// @param effort [out] - current joint effort [mA]
  void robot_get_joint_state(std::string const& name, float *position=NULL, float *velocity=NULL, float *effort=NULL);

private:

  int timer_hz=100;                                                                 // Frequency at which the ROS Timer publishing joint states should run
  bool pub_states=true;                                                              // Boolean that determines if joint states should be published;

  bool once_received_flag = false;
  //pubに必要なデータ類
  std::vector<std::string> joint_names;// Names of all joints
  std::vector<std::string> joint_ids;// Names of all joints  
  uint8_t joint_num;// Number of all Joints
  size_t js_index;                                                      // Index in the published JointState message 'name' list belonging to the gripper motor
  float horn_radius;                                                    // Distance [m] from the motor horn's center to its edge
  float arm_length;                                                     // Distance [m] from the edge of the motor horn to a finger
  std::string left_finger;                                              // Name of the 'left_finger' joint as defined in the URDF (if present)
  std::string right_finger;                                             // Name of the 'right_finger' joint as defined in the URDF (if present)

  ros::NodeHandle node;                                                         // ROS Node handle
  ros::Publisher pub_master_joint_states;                                              // ROS Publisher responsible for publishing joint states
  ros::Publisher pub_puppet_joint_states;                                              // ROS Publisher responsible for publishing joint states  
  ros::ServiceServer srv_torque_enable;                                         // ROS Service Server used to torque on/off any motor

  ros::Timer tmr_joint_states;                                                  // ROS Timer used to continuously publish joint states
  sensor_msgs::JointState master_joint_states;                                         // Holds the most recent JointState message
  sensor_msgs::JointState puppet_joint_states;                                         // Holds the most recent JointState message  

  //for rosparam
  std::string port_name=SERIAL_DEVICE;                                                   //USB port Name
  std::string side_name="left";
  std::string master_robot_name="master_"+side_name;
  std::string puppet_robot_name="puppet_"+side_name;
  int baudrate=SERIAL_BAUDRATE;
  
  std::string js_master_topic=master_robot_name+"/joint_state";                                                         // Desired JointState topic name
  std::string js_puppet_topic=puppet_robot_name+"/joint_state";                                                         // Desired JointState topic name

  int16_t diff_time;
  std::vector<int16_t> position_vec;                                       
  std::mutex position_vec_mutex;

  boost::asio::io_service &io;
  boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

  //unsigned char serial_read_buffer[1024];
  boost::asio::streambuf response_data;

  /// @brief Initializes the Robot Mode;
  /// @param <bool> [out] - True if the port was successfully; False otherwise
  bool init_model(void);

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

  /// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
  /// @param e - TimerEvent message
  void robot_update_joint_states(const ros::TimerEvent &e);

  float robot_convert_angular_position_to_radius(int const& int_angular_position)
  {
    float f_deg=int_angular_position*0.1;
    float rad=( (f_deg)/180.0 * M_PI);

    return rad;
  };

  float robot_convert_linear_position_to_radian(float const& linear_position)
  {
  float half_dist = linear_position / 2.0;
  float result = 3.14159/2.0 - acos((pow(horn_radius, 2) + pow(half_dist,2) - pow(arm_length, 2)) / (2 * horn_radius * half_dist));
  return result;
  };

  float robot_convert_angular_position_to_linear(float const& angular_position)
  {
    float a1 = horn_radius * sin(angular_position);
    float c = sqrt(pow(horn_radius,2) - pow(a1,2));
    float a2 = sqrt(pow(arm_length,2) - pow(c,2));
    return a1 + a2;
  };

  bool sendSerialCmd(unsigned char ch);

  void doReceive(void);
  void data_received(const boost::system::error_code& error, size_t bytes_transffered);

  bool robot_srv_torque_enable(interbotix_xs_msgs::TorqueEnable::Request &req, interbotix_xs_msgs::TorqueEnable::Response &res);
};

#endif
