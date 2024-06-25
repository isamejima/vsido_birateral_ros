#include "vsido_birateral/vsido_birateral.h"

/// @brief Constructor for the VSidoBirateral
/// @param node_handle - ROS NodeHandle
/// @param io_service - boost::asio::io_service
/// @param success [out] - bool indicating if the node launched successfully
VSidoBirateral::VSidoBirateral(ros::NodeHandle *node_handle, boost::asio::io_service &io_service, bool &success)
    : node(*node_handle), io(io_service)
{
  if (!init_model())
  {
    success = false;
    return;
  }

  if (!init_port())
  {
    success = false;
    return;
  }

  init_publishers();
  init_subscribers();
  init_services();
  init_timers();

  doReceive();
  // sendSerialCmd('3');
  sendSerialCmd('s');

  //  robot_wait_for_joint_states();
  ROS_INFO("V-Sido Birateral node is up!");
}

/// @brief Destructor for the VSidoBirateral
VSidoBirateral::~VSidoBirateral()
{
  sendSerialCmd('0');
  std::cout << "sendSerialCmd :" << "0" << std::endl;
  sleep(1);
  serial_port_ptr->close();
  std::cout << "serial_port closed" << std::endl;
}

bool VSidoBirateral::init_model(void)
{
  ros::NodeHandle pnh("~");
  pnh.getParam("port", port_name);
  pnh.getParam("baudrate", baudrate);
  pnh.getParam("side_name", side_name);

  ROS_INFO_STREAM("side_name: "<< side_name);

  master_robot_name="master_"+side_name;
  puppet_robot_name="puppet_"+side_name;  

  js_master_topic=master_robot_name+"/joint_state";
  js_puppet_topic=puppet_robot_name+"/joint_state";

  ROS_INFO_STREAM("js_master_topic : "<< js_master_topic);
  ROS_INFO_STREAM("js_puppet_topic : "<< js_puppet_topic);  

  joint_names = std::vector<std::string>{
      "waist",
      "shoulder",
      "elbow",
      "forearm_roll",
      "wrist_angle",
      "wrist_rotate",
      "gripper"};
  joint_ids = std::vector<std::string>{
      "1",
      "2",
      "3",
      "4",
      "5",
      "6",
      "7"};
  joint_num = 7;

  js_index = 6;
  horn_radius = 0.0275;
  arm_length = 0.035;
  left_finger = "left finger";
  right_finger = "right finger";

  position_vec.resize(joint_num, 0);

  return true;
}

/// @brief Initializes the port to communicate with the Dynamixel servos
/// @param <bool> [out] - True if the port was successfully opened; False otherwise
bool VSidoBirateral::init_port(void)
{

  /*
    if (serial_port_ptr)
    {
      ROS_INFO("error : port is not opened...");
      ROS_INFO_STREAM(serial_port_ptr);
      return false;
    }
  */
  serial_port_ptr = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io));

  try
  {
    ROS_INFO_STREAM("serial port open:" << port_name);
    serial_port_ptr->open(port_name);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM("Unable to open port :"<< port_name);
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  try
  {
    ROS_INFO_STREAM("set baudrate:"<<baudrate);
    serial_port_ptr->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM("Unable to set baud rate :");
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  if (serial_port_ptr->is_open())
  {
    ROS_INFO_STREAM("Serial Port initialized");
    return true;
  }

  else
  {
    return false;
  }
}

/// @brief Initialize ROS Publishers
void VSidoBirateral::init_publishers(void)
{
  // masterとpuppetのtopicをpublishする
  pub_master_joint_states = node.advertise<sensor_msgs::JointState>(js_master_topic, 1);
  pub_puppet_joint_states = node.advertise<sensor_msgs::JointState>(js_puppet_topic, 1);
}

/// @brief Initialize ROS Subscribers
void VSidoBirateral::init_subscribers(void)
{
  // sub_command_group = node.subscribe("commands/joint_group", 5, &VSidoBirateral::robot_sub_command_group, this);
}

/// @brief Initialize ROS Services
void VSidoBirateral::init_services(void)
{
  // record_episode.pyのopening_ceremony内にて、エンドエフェクタのトルクOFFをしているため、バイラテラル開始のトリガーとして利用。
  srv_torque_enable = node.advertiseService("torque_enable", &VSidoBirateral::robot_srv_torque_enable, this);
}

/// @brief Initialize ROS Timers
void VSidoBirateral::init_timers(void)
{
  // 定期送信用だが、シリアル受信割り込み毎にpublishする仕様に変更したのでコメントアウト
  // tmr_joint_states = node.createTimer(ros::Duration(1.0 / timer_hz), &VSidoBirateral::robot_update_joint_states, this);
}

/// @brief ROS Timer publishes them to the joint_states topic
/// @param e - TimerEvent message [unused]
void VSidoBirateral::robot_update_joint_states(const ros::TimerEvent &e)
{
  publish_joint_states();
}

/// @brief publishes to the joint_states topic
void VSidoBirateral::publish_joint_states()
{
  const char *log;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  std::vector<int16_t> p_vec;
  bool flag = false;

  {
    std::lock_guard<std::mutex> lock(position_vec_mutex); // 念の為ロックする
    p_vec = position_vec;
    flag = once_received_flag;
  }

  // 一度でもシリアル受信できていないならreturn
  if (flag == false)
  {
    return;
  }

  // データ長異常でreturn
  if (p_vec.size() != joint_num)
    return;

  // 7軸分
  for (int id = 0; id < p_vec.size(); id++)
  {
    float position = 0;
    float velocity = 0;
    float effort = 0;

    position = robot_convert_angular_position_to_radius(p_vec.at(id));
    joint_state_msg.position.push_back(position);
    joint_state_msg.velocity.push_back(0); // 速度は使っていないので0埋め
    joint_state_msg.effort.push_back(0);   // 電流はつかっていないので0埋め
  }

  // グリッパー周りを追加js_indexを参照する
  joint_state_msg.name.push_back(left_finger.c_str());
  joint_state_msg.name.push_back(right_finger.c_str());
  float pos = robot_convert_angular_position_to_linear(joint_state_msg.position.at(js_index));

  joint_state_msg.position.push_back(pos);
  joint_state_msg.position.push_back(-pos);
  joint_state_msg.velocity.push_back(0); // 速度は使っていないので0埋め
  joint_state_msg.velocity.push_back(0); // 速度は使っていないので0埋め
  joint_state_msg.effort.push_back(0);   // 電流は使っていないので0埋め
  joint_state_msg.effort.push_back(0);   // 電流は使っていないので0埋め

  // Publish the message to the joint_states topic

  joint_state_msg.header.stamp = ros::Time::now();

  master_joint_states = joint_state_msg;
  puppet_joint_states = joint_state_msg;

  pub_master_joint_states.publish(joint_state_msg);
  pub_puppet_joint_states.publish(joint_state_msg);
}

/// @brief Waits until first JointState message is received
bool VSidoBirateral::sendSerialCmd(unsigned char ch)
{
  if (!serial_port_ptr)
  {
    ROS_INFO_STREAM("serial_port_ptr is null");
    return false;
  }

  if (!serial_port_ptr->is_open())
  {
    ROS_INFO("error : port is not opened...");
    return false;
  }

  try
  {
    serial_port_ptr->write_some(boost::asio::buffer(&ch, 1));
  }
  catch (const std::exception &ex)
  {
    ROS_INFO_STREAM(ex.what());
    return false;
  }

  ROS_INFO_STREAM("sendSerialCmd :" << ch);

  return true;
}

void VSidoBirateral::doReceive(void)
{
  if (!serial_port_ptr->is_open())
  {
    ROS_WARN("doReceive::serial port is not open");
    return;
  }

  // 非同期受信
  // 改行コードまで
  // callbackでdata_received関数を呼ぶ
  boost::asio::async_read_until(*serial_port_ptr, response_data, "\n", boost::bind(&VSidoBirateral::data_received, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  // ctrl+c受付用
  boost::this_thread::interruption_point();
}

void VSidoBirateral::data_received(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (bytes_transferred > 0)
  {

    std::string buf_str;
    std::istream is(&response_data);
    is >> buf_str;

    if (buf_str.size() != 0)
    {
      // stoiで処理失敗するので、改行を消す
      buf_str.erase(std::remove(buf_str.begin(), buf_str.end(), '\n'), buf_str.end());
      // ROS_INFO_STREAM("buf_str:" << buf_str);

      bool cast_success = true;
      std::vector<int16_t> int_vec;
      //tokenizerでデータ分割
      boost::tokenizer<boost::char_separator<char>> tokens(buf_str, boost::char_separator<char>(",", "\n"));

      for (auto item : tokens)
      {
        try
        {
          int_vec.push_back(stoi(item));
        }
        catch (std::exception &e)
        {
          //stoiに失敗したらfalseに
          cast_success = false;
          ROS_WARN_STREAM("error:" << e.what());
        }
      }

      //適切にデータ処理できたら
      if (cast_success == true && int_vec.size() == position_vec.size() + 1)
      {
        {
          std::lock_guard<std::mutex> lock(position_vec_mutex); //念のためロックする
          once_received_flag = true;
          diff_time=int_vec.at(0);
          for (int i = 0; i < position_vec.size(); i++)
          {
            position_vec.at(i) = int_vec.at(i + 1);
          }        
        }
        publish_joint_states();
      }
      else
      {
        ROS_WARN_STREAM("cast failed.buf_str:" << buf_str);
      }
    }
  }
  doReceive();
}

bool VSidoBirateral::robot_srv_torque_enable(interbotix_xs_msgs::TorqueEnable::Request &req, interbotix_xs_msgs::TorqueEnable::Response &res)
{
  // responseは使っていない

  // single joint commandのときだけcmd発行
  // gripperのtorque offのときだけcmd発行
  if (req.cmd_type == "single" && req.name == "gripper" && req.enable == false)
  {
    sendSerialCmd('3');
  }

  return true;
}