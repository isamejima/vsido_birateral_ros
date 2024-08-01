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
  sendSerialCmd('3');
  sendSerialCmd('s');

  //  robot_wait_for_joint_states();
  ROS_INFO("V-Sido Birateral node is up!");
  success=true;
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
  node.getParam("port_name", port_name);
  node.getParam("baudrate", baudrate);
  node.getParam("side_name", side_name);

  ROS_INFO_STREAM("port_name: "<< port_name);
  ROS_INFO_STREAM("side_name: "<< side_name);

  master_robot_name="master_"+side_name;
  puppet_robot_name="puppet_"+side_name;  
  
  master_get_motor_configs();
  puppet_get_motor_configs();  

  if(master_yaml_configs.group_map["all"].joint_names.size()!=puppet_yaml_configs.group_map["all"].joint_names.size()){
    ROS_ERROR("master and puppet joint size is not equal");
    return false;
  }

  //position_vecをリサイズし、ゼロで初期化
  position_vec.resize(master_yaml_configs.group_map["all"].joint_names.size(), 0);
  data_vec.resize(master_yaml_configs.group_map["all"].joint_names.size()*3, 0);

  return true;
}

/// @brief Initializes the port to communicate with the Dynamixel servos
/// @param <bool> [out] - True if the port was successfully opened; False otherwise
bool VSidoBirateral::init_port(void)
{
  serial_port_ptr = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io));

  try
  {
    ROS_INFO_STREAM("try serial port open:" << port_name);
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
    return false;
  }

  if (serial_port_ptr->is_open())
  {
    ROS_INFO_STREAM("Serial Port initialized");
    return true;
  }
  
  return false;
}

/// @brief Initialize ROS Publishers
void VSidoBirateral::init_publishers(void)
{
  std::string puppet_node_name="/puppet_"+side_name;
  std::string master_node_name="/master_"+side_name;  

  ros::NodeHandle master_nh(master_node_name);
  ros::NodeHandle puppet_nh(puppet_node_name);

  // masterとpuppetのtopicをpublishする
  pub_master_joint_states = master_nh.advertise<sensor_msgs::JointState>(js_topic, 1);
  pub_puppet_joint_states = puppet_nh.advertise<sensor_msgs::JointState>(js_topic, 1);
}

/// @brief Initialize ROS Subscribers
void VSidoBirateral::init_subscribers(void)
{
  // sub_command_group = node.subscribe("commands/joint_group", 5, &VSidoBirateral::robot_sub_command_group, this);
}

/// @brief Initialize ROS Services
void VSidoBirateral::init_services(void)
{
  std::string puppet_node_name="/puppet_"+side_name;
  std::string master_node_name="/master_"+side_name;  

  ros::NodeHandle master_nh(master_node_name);
  ros::NodeHandle puppet_nh(puppet_node_name);

  srv_master_set_operating_modes = master_nh.advertiseService("set_operating_modes", &VSidoBirateral::robot_srv_set_operating_modes, this);
  srv_puppet_set_operating_modes = puppet_nh.advertiseService("set_operating_modes", &VSidoBirateral::robot_srv_set_operating_modes, this);

  srv_master_set_motor_pid_gains = master_nh.advertiseService("set_motor_pid_gains", &VSidoBirateral::robot_srv_set_motor_pid_gains, this);
  srv_puppet_set_motor_pid_gains = puppet_nh.advertiseService("set_motor_pid_gains", &VSidoBirateral::robot_srv_set_motor_pid_gains, this);

  srv_master_set_motor_registers = master_nh.advertiseService("set_motor_registers", &VSidoBirateral::robot_srv_set_motor_registers, this);
  srv_puppet_set_motor_registers = puppet_nh.advertiseService("set_motor_registers", &VSidoBirateral::robot_srv_set_motor_registers, this);

  srv_master_get_motor_registers = master_nh.advertiseService("get_motor_registers", &VSidoBirateral::robot_srv_get_motor_registers, this);
  srv_puppet_get_motor_registers = puppet_nh.advertiseService("get_motor_registers", &VSidoBirateral::robot_srv_get_motor_registers, this);

  // alohaのcodeでモデル情報が必要
  srv_master_get_robot_info = master_nh.advertiseService("get_robot_info", &VSidoBirateral::master_srv_get_robot_info, this);
  srv_puppet_get_robot_info = puppet_nh.advertiseService("get_robot_info", &VSidoBirateral::puppet_srv_get_robot_info, this);

  // record_episode.pyのopening_ceremony内にて、エンドエフェクタのトルクOFFをしているため、バイラテラル開始のトリガーとして利用。
  srv_master_torque_enable = master_nh.advertiseService("torque_enable", &VSidoBirateral::master_srv_torque_enable, this);//ここだけ例外
  srv_puppet_torque_enable = puppet_nh.advertiseService("torque_enable", &VSidoBirateral::robot_srv_torque_enable, this);  
 
  srv_master_reboot_motors = master_nh.advertiseService("reboot_motors", &VSidoBirateral::robot_srv_reboot_motors, this);
  srv_puppet_reboot_motors = puppet_nh.advertiseService("reboot_motors", &VSidoBirateral::robot_srv_reboot_motors, this);  
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

  sensor_msgs::JointState master_joint_state_msg;
  sensor_msgs::JointState puppet_joint_state_msg;  

  //masterまわり
  master_joint_state_msg.name = master_yaml_configs.group_map["all"].joint_names;  
  //arm
  for (auto const& name : master_yaml_configs.group_map["all"].joint_names)
  {
    float position = 0;
    float velocity = 0;
    float effort = 0;

    position = robot_convert_angular_position_to_radius(p_vec.at(master_yaml_configs.js_index_map[name]));
    master_joint_state_msg.position.push_back(position);
    master_joint_state_msg.velocity.push_back(velocity); // 速度は使っていないので0
    master_joint_state_msg.effort.push_back(effort);   // 電流はつかっていないので0
  }

  //取得するのはpuppetの角度なので、masterのgripperの角度に変換
  //下記、mobile alohaエンドエフェクタ向けパラメータ
  float MASTER_GRIPPER_JOINT_OPEN = -0.8;
  float MASTER_GRIPPER_JOINT_CLOSE = -1.65;
  float PUPPET_GRIPPER_JOINT_OPEN = 1.4910;
  float PUPPET_GRIPPER_JOINT_CLOSE = -0.6213;
  float raw_pos=master_joint_state_msg.position.at(master_yaml_configs.js_index_map["gripper"]);
  float conv_pos=(raw_pos - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE;
  master_joint_state_msg.position.at(master_yaml_configs.js_index_map["gripper"])=conv_pos;
  //ROS_INFO_STREAM("raw:"<<raw_pos<<",conv:"<<conv_pos);

  //gripper length
  for (auto const& name : master_yaml_configs.gripper_order)
  {
    master_joint_state_msg.name.push_back(master_yaml_configs.gripper_map[name].left_finger.c_str());
    master_joint_state_msg.name.push_back(master_yaml_configs.gripper_map[name].right_finger.c_str());
    float pos = robot_convert_angular_position_to_linear(master_yaml_configs.gripper_map[name], master_joint_state_msg.position.at(master_yaml_configs.gripper_map[name].js_index));
    master_joint_state_msg.position.push_back(pos);
    master_joint_state_msg.position.push_back(-pos);
    master_joint_state_msg.velocity.push_back(0);
    master_joint_state_msg.velocity.push_back(0);
    master_joint_state_msg.effort.push_back(0);
    master_joint_state_msg.effort.push_back(0);
  }

  //puppetまわり
  puppet_joint_state_msg.name = puppet_yaml_configs.group_map["all"].joint_names;  
  //arm
  for (auto const& name : master_yaml_configs.group_map["all"].joint_names)
  {
    float position = 0;
    float velocity = 0;
    float effort = 0;

    position = robot_convert_angular_position_to_radius(p_vec.at(master_yaml_configs.js_index_map[name]));
    puppet_joint_state_msg.position.push_back(position);
    puppet_joint_state_msg.velocity.push_back(velocity); // 速度は使っていないので0埋め
    puppet_joint_state_msg.effort.push_back(effort);   // 電流はつかっていないので0埋め
  }

  for (auto const& name : puppet_yaml_configs.gripper_order)
  {
    puppet_joint_state_msg.name.push_back(puppet_yaml_configs.gripper_map[name].left_finger.c_str());
    puppet_joint_state_msg.name.push_back(puppet_yaml_configs.gripper_map[name].right_finger.c_str());
    float pos = robot_convert_angular_position_to_linear(puppet_yaml_configs.gripper_map[name], puppet_joint_state_msg.position.at(puppet_yaml_configs.gripper_map[name].js_index));
    puppet_joint_state_msg.position.push_back(pos);
    puppet_joint_state_msg.position.push_back(-pos);
    puppet_joint_state_msg.velocity.push_back(0);
    puppet_joint_state_msg.velocity.push_back(0);
    puppet_joint_state_msg.effort.push_back(0);
    puppet_joint_state_msg.effort.push_back(0);
  }

  ros::Time current_time = ros::Time::now();
  master_joint_state_msg.header.stamp = current_time;
  puppet_joint_state_msg.header.stamp = current_time;

  pub_master_joint_states.publish(master_joint_state_msg);
  pub_puppet_joint_states.publish(puppet_joint_state_msg);
}

/// @brief publishes to the joint_states topic
void VSidoBirateral::publish_joint_states2()
{
  std::vector<int16_t> d_vec;
  bool flag = false;
  {
    std::lock_guard<std::mutex> lock(position_vec_mutex); // 念の為ロックする
    d_vec = data_vec;
    flag = once_received_flag;
  }

  // 一度でもシリアル受信できていないならreturn
  if (flag == false)
  {
    return;
  }

  sensor_msgs::JointState master_joint_state_msg;
  sensor_msgs::JointState puppet_joint_state_msg;  

  //masterまわり
  master_joint_state_msg.name = master_yaml_configs.group_map["all"].joint_names;  
  //arm
  for (auto const& name : master_yaml_configs.group_map["all"].joint_names)
  {
    float position = 0;
    float velocity = 0;
    float effort = 0;

//    ROS_INFO_STREAM("name:"<<name);
    int eff_index=master_yaml_configs.js_index_map[name]*3+0;    
    int vel_index=master_yaml_configs.js_index_map[name]*3+1;
    int pos_index=master_yaml_configs.js_index_map[name]*3+2;
//    ROS_INFO_STREAM("eff_index:"<<eff_index<<",vel_index:"<<vel_index<<",pos_index:"<<pos_index);
    effort = robot_convert_load(data_vec.at(eff_index));
    velocity = robot_convert_angular_position_to_radius(data_vec.at(vel_index));//convert 0.1[deg/sec]/unit to 1.0[deg/sec]
    position = robot_convert_angular_position_to_radius(data_vec.at(pos_index));//convert 0.1[deg]/unit to 1.0[deg]
//    ROS_INFO_STREAM("data_vec:"<<data_vec.at(eff_index)<<","<<data_vec.at(vel_index)<<","<<data_vec.at(pos_index));    
//    ROS_INFO_STREAM("effort:"<<effort<<",velocity:"<<velocity<<",position:"<<position);    
    master_joint_state_msg.position.push_back(position);
    master_joint_state_msg.velocity.push_back(velocity); // 速度は使っていないので0
    master_joint_state_msg.effort.push_back(effort);   // 電流はつかっていないので0
  }

  //取得するのはpuppetの角度なので、masterのgripperの角度に変換
  //下記、mobile alohaエンドエフェクタ向けパラメータ
  float MASTER_GRIPPER_JOINT_OPEN = -0.8;
  float MASTER_GRIPPER_JOINT_CLOSE = -1.65;
  float PUPPET_GRIPPER_JOINT_OPEN = 1.4910;
  float PUPPET_GRIPPER_JOINT_CLOSE = -0.6213;
  float raw_pos=master_joint_state_msg.position.at(master_yaml_configs.js_index_map["gripper"]);
  float conv_pos=(raw_pos - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE;
  master_joint_state_msg.position.at(master_yaml_configs.js_index_map["gripper"])=conv_pos;
  //ROS_INFO_STREAM("raw:"<<raw_pos<<",conv:"<<conv_pos);

  //gripper length
  for (auto const& name : master_yaml_configs.gripper_order)
  {
    master_joint_state_msg.name.push_back(master_yaml_configs.gripper_map[name].left_finger.c_str());
    master_joint_state_msg.name.push_back(master_yaml_configs.gripper_map[name].right_finger.c_str());
    float pos = robot_convert_angular_position_to_linear(master_yaml_configs.gripper_map[name], master_joint_state_msg.position.at(master_yaml_configs.gripper_map[name].js_index));
    master_joint_state_msg.position.push_back(pos);
    master_joint_state_msg.position.push_back(-pos);
    master_joint_state_msg.velocity.push_back(0);
    master_joint_state_msg.velocity.push_back(0);
    master_joint_state_msg.effort.push_back(0);
    master_joint_state_msg.effort.push_back(0);
  }

  //puppetまわり
  puppet_joint_state_msg.name = puppet_yaml_configs.group_map["all"].joint_names;  
  //arm
  for (auto const& name : master_yaml_configs.group_map["all"].joint_names)
  {
    float position = 0;
    float velocity = 0;
    float effort = 0;

    int eff_index=master_yaml_configs.js_index_map[name]*3+0;    
    int vel_index=master_yaml_configs.js_index_map[name]*3+1;
    int pos_index=master_yaml_configs.js_index_map[name]*3+2;

    position = robot_convert_angular_position_to_radius(data_vec.at(pos_index));//convert 0.1[deg]/unit to 1.0[deg]
    velocity = robot_convert_angular_position_to_radius(data_vec.at(vel_index));//convert 0.1[deg/sec]/unit to 1.0[deg/sec]
    effort = robot_convert_load(data_vec.at(eff_index));

    puppet_joint_state_msg.position.push_back(position);
    puppet_joint_state_msg.velocity.push_back(velocity); // 速度は使っていないので0埋め
    puppet_joint_state_msg.effort.push_back(effort);   // 電流はつかっていないので0埋め
  }

  for (auto const& name : puppet_yaml_configs.gripper_order)
  {
    puppet_joint_state_msg.name.push_back(puppet_yaml_configs.gripper_map[name].left_finger.c_str());
    puppet_joint_state_msg.name.push_back(puppet_yaml_configs.gripper_map[name].right_finger.c_str());
    float pos = robot_convert_angular_position_to_linear(puppet_yaml_configs.gripper_map[name], puppet_joint_state_msg.position.at(puppet_yaml_configs.gripper_map[name].js_index));
    puppet_joint_state_msg.position.push_back(pos);
    puppet_joint_state_msg.position.push_back(-pos);
    puppet_joint_state_msg.velocity.push_back(0);
    puppet_joint_state_msg.velocity.push_back(0);
    puppet_joint_state_msg.effort.push_back(0);
    puppet_joint_state_msg.effort.push_back(0);
  }

  ros::Time current_time = ros::Time::now();
  master_joint_state_msg.header.stamp = current_time;
  puppet_joint_state_msg.header.stamp = current_time;

  pub_master_joint_states.publish(master_joint_state_msg);
  pub_puppet_joint_states.publish(puppet_joint_state_msg);
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
    ROS_INFO("error : port is not open...");
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
  // 改行まで
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
//      ROS_INFO_STREAM("buf_str:" << buf_str);

      bool cast_success = true;
      std::vector<int16_t> int_vec;
      //tokenizerでデータ分割
      //",",,"\n"で分割
      boost::tokenizer<boost::char_separator<char>> tokens(buf_str, boost::char_separator<char>(","));

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
      else if(cast_success == true && int_vec.size() == data_vec.size()+1)
      {
        {
          std::lock_guard<std::mutex> lock(position_vec_mutex); //念のためロックする
          once_received_flag = true;
          diff_time=int_vec.at(0);
          for (int i = 0; i < data_vec.size(); i++)
          {
            data_vec.at(i) = int_vec.at(i + 1);
          }        
        }
        publish_joint_states2();        
      }
      else
      {
        ROS_WARN_STREAM("cast failed.buf_str:" << buf_str);
      }
    }
  }
  doReceive();
}

bool VSidoBirateral::master_srv_torque_enable(interbotix_xs_msgs::TorqueEnable::Request &req, interbotix_xs_msgs::TorqueEnable::Response &res)
{
  // responseは使っていない

  // single joint commandのときだけcmd発行
  // gripperのtorque offのときだけcmd発行

  static bool once_flag=false;

  if (req.cmd_type == "single" && req.name == "gripper" && req.enable == false)
  {
    //torque offを受信1回目
    if(once_flag==false){
    sendSerialCmd('3');
    once_flag=true;
    return true;
    }
    else {
    //birateral startした直後の受信は無視      
      once_flag=false;
    return true;      
    }
  }

  return true;
}

bool VSidoBirateral::master_get_motor_configs(void)
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

  std::string motor_configs_file, mode_configs_file;
  node.getParam("master_motor_configs", motor_configs_file);

  ROS_INFO_STREAM("motor_configs_file: "<< motor_configs_file);

  try
  {
    motor_configs = YAML::LoadFile(motor_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    ROS_FATAL(" Motor Config file at '%s' was not found or has a bad format. Shutting down...", motor_configs_file.c_str());
    ROS_FATAL(" YAML Error: '%s'", error.what());
    return false;
  }
  if (motor_configs.IsNull())
  {
    ROS_FATAL(" Motor Config file at '%s' was not found. Shutting down...", motor_configs_file.c_str());
    return false;
  }

  node.getParam("master_mode_configs", mode_configs_file);
  try
  {
    mode_configs = YAML::LoadFile(mode_configs_file.c_str());
    ROS_INFO(" Loaded mode configs from '%s'.", mode_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    ROS_ERROR(" Motor Config file at '%s' was not found or has a bad format. Shutting down...", mode_configs_file.c_str());
    ROS_ERROR(" YAML Error: '%s'", error.what());
    return false;
  }
  if (mode_configs.IsNull())
    ROS_INFO(" Mode Config file is empty. Will use defaults.");

/*
  port = motor_configs["port"].as<std::string>(PORT);
  if (mode_configs["port"])
    port = mode_configs["port"].as<std::string>(PORT);
    */

  YAML::Node all_motors = motor_configs["motors"];
  for (YAML::const_iterator motor_itr = all_motors.begin(); motor_itr != all_motors.end(); motor_itr++)
  {
    std::string motor_name = motor_itr->first.as<std::string>();
    YAML::Node single_motor = all_motors[motor_name];
    uint8_t id = (uint8_t)single_motor["ID"].as<int32_t>();
    motor_map.insert({motor_name, {id, "position", "velocity"}});
    for (YAML::const_iterator info_itr = single_motor.begin(); info_itr != single_motor.end(); info_itr++)
    {
      std::string reg = info_itr->first.as<std::string>();
      if (reg != "ID" && reg != "Baud_Rate")
      {
        int32_t value = info_itr->second.as<int32_t>();
        MotorInfo motor_info = {id, reg, value};
        motor_info_vec.push_back(motor_info);
      }
    }
  }

  YAML::Node all_grippers = motor_configs["grippers"];
  for (YAML::const_iterator gripper_itr = all_grippers.begin(); gripper_itr != all_grippers.end(); gripper_itr++)
  {
    std::string gripper_name = gripper_itr->first.as<std::string>();
    YAML::Node single_gripper = all_grippers[gripper_name];
    Gripper gripper;
    gripper.horn_radius = single_gripper["horn_radius"].as<float>(0.014);
    gripper.arm_length = single_gripper["arm_length"].as<float>(0.024);
    gripper.left_finger = single_gripper["left_finger"].as<std::string>("left_finger");
    gripper.right_finger = single_gripper["right_finger"].as<std::string>("right_finger");
    gripper_map.insert({gripper_name, gripper});
  }

  YAML::Node joint_order = motor_configs["joint_order"];
  YAML::Node sleep_positions = motor_configs["sleep_positions"];

  if (joint_order.size() != sleep_positions.size())
  {
    ROS_FATAL(
      " Error when parsing Motor Config file: Length of joint_order list (%ld) does not match length of sleep_positions list (%ld).",
      joint_order.size(), sleep_positions.size());
    return false;
  }

  JointGroup all_joints;
  all_joints.joint_num = (uint8_t) joint_order.size();
  all_joints.mode = "position";
  all_joints.profile_type = "velocity";
  for (size_t i{0}; i < joint_order.size(); i++)
  {
    std::string joint_name = joint_order[i].as<std::string>();
    all_joints.joint_names.push_back(joint_name);
    all_joints.joint_ids.push_back(motor_map[joint_name].motor_id);
    js_index_map.insert({joint_name, i});
    shadow_map.insert({joint_name, {joint_name}});
    sister_map.insert({joint_name, {joint_name}});
    sleep_map.insert({joint_name, sleep_positions[i].as<float>(0)});
    if (gripper_map.count(joint_name) > 0)
    {
      gripper_map[joint_name].js_index = i;
      gripper_order.push_back(joint_name);
    }
  }

  for (auto const& name : gripper_order)
  {
    js_index_map.insert({gripper_map[name].left_finger, js_index_map.size()});
    js_index_map.insert({gripper_map[name].right_finger, js_index_map.size()});
  }

  group_map.insert({"all", all_joints});
  //all_ptr = &group_map["all"];

  YAML::Node all_shadows = motor_configs["shadows"];
  for (YAML::const_iterator master_itr = all_shadows.begin(); master_itr != all_shadows.end(); master_itr++)
  {
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    YAML::Node shadow_list = master["shadow_list"];
    for (size_t i{0}; i < shadow_list.size(); i++)
      shadow_map[master_name].push_back(shadow_list[i].as<std::string>());
  }

  YAML::Node all_sisters = motor_configs["sisters"];
  for (YAML::const_iterator sister_itr = all_sisters.begin(); sister_itr != all_sisters.end(); sister_itr++)
  {
    std::string sister_one = sister_itr->first.as<std::string>();
    std::string sister_two = sister_itr->second.as<std::string>();
    sister_map[sister_one].push_back(sister_two);
    sister_map[sister_two].push_back(sister_one);
  }

  YAML::Node all_groups = motor_configs["groups"];
  for (YAML::const_iterator group_itr = all_groups.begin(); group_itr != all_groups.end(); group_itr++)
  {
    std::string name = group_itr->first.as<std::string>();
    YAML::Node joint_list = all_groups[name];
    JointGroup group;
    group.joint_num = (uint8_t) joint_list.size();
    for (size_t i{0}; i < joint_list.size(); i++)
    {
      std::string joint_name = joint_list[i].as<std::string>();
      group.joint_names.push_back(joint_name);
      group.joint_ids.push_back(motor_map[joint_name].motor_id);
    }
    group_map.insert({name, group});
  }

  /*
  YAML::Node pub_configs = motor_configs["joint_state_publisher"];
  timer_hz = pub_configs["update_rate"].as<int>(100);
  pub_states = pub_configs["publish_states"].as<bool>(true);
  js_topic = pub_configs["topic_name"].as<std::string>("joint_states");
  */

  master_yaml_configs.motor_info_vec=motor_info_vec;
  master_yaml_configs.gripper_order=gripper_order;
  master_yaml_configs.control_items=control_items;
  master_yaml_configs.sleep_map=sleep_map;
  master_yaml_configs.group_map=group_map;
  master_yaml_configs.motor_map=motor_map;
  master_yaml_configs.shadow_map=shadow_map;
  master_yaml_configs.sister_map=sister_map;
  master_yaml_configs.gripper_map=gripper_map;
  master_yaml_configs.js_index_map=js_index_map;
  
  ROS_INFO("Loaded motor configs from '%s'.", motor_configs_file.c_str());
  return true;
}

bool VSidoBirateral::puppet_get_motor_configs(void)
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

  std::string motor_configs_file, mode_configs_file;
  node.getParam("puppet_motor_configs", motor_configs_file);

  try
  {
    motor_configs = YAML::LoadFile(motor_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    ROS_FATAL(" Motor Config file at '%s' was not found or has a bad format. Shutting down...", motor_configs_file.c_str());
    ROS_FATAL(" YAML Error: '%s'", error.what());
    return false;
  }
  if (motor_configs.IsNull())
  {
    ROS_FATAL(" Motor Config file at '%s' was not found. Shutting down...", motor_configs_file.c_str());
    return false;
  }

  node.getParam("puppet_mode_configs", mode_configs_file);
  try
  {
    mode_configs = YAML::LoadFile(mode_configs_file.c_str());
    ROS_INFO(" Loaded mode configs from '%s'.", mode_configs_file.c_str());
  }
  catch (YAML::BadFile &error)
  {
    ROS_ERROR(" Motor Config file at '%s' was not found or has a bad format. Shutting down...", mode_configs_file.c_str());
    ROS_ERROR(" YAML Error: '%s'", error.what());
    return false;
  }
  if (mode_configs.IsNull())
    ROS_INFO(" Mode Config file is empty. Will use defaults.");

/*
  port = motor_configs["port"].as<std::string>(PORT);
  if (mode_configs["port"])
    port = mode_configs["port"].as<std::string>(PORT);
    */

  YAML::Node all_motors = motor_configs["motors"];
  for (YAML::const_iterator motor_itr = all_motors.begin(); motor_itr != all_motors.end(); motor_itr++)
  {
    std::string motor_name = motor_itr->first.as<std::string>();
    YAML::Node single_motor = all_motors[motor_name];
    uint8_t id = (uint8_t)single_motor["ID"].as<int32_t>();
    motor_map.insert({motor_name, {id, "position", "velocity"}});
    for (YAML::const_iterator info_itr = single_motor.begin(); info_itr != single_motor.end(); info_itr++)
    {
      std::string reg = info_itr->first.as<std::string>();
      if (reg != "ID" && reg != "Baud_Rate")
      {
        int32_t value = info_itr->second.as<int32_t>();
        MotorInfo motor_info = {id, reg, value};
        motor_info_vec.push_back(motor_info);
      }
    }
  }

  YAML::Node all_grippers = motor_configs["grippers"];
  for (YAML::const_iterator gripper_itr = all_grippers.begin(); gripper_itr != all_grippers.end(); gripper_itr++)
  {
    std::string gripper_name = gripper_itr->first.as<std::string>();
    YAML::Node single_gripper = all_grippers[gripper_name];
    Gripper gripper;
    gripper.horn_radius = single_gripper["horn_radius"].as<float>(0.014);
    gripper.arm_length = single_gripper["arm_length"].as<float>(0.024);
    gripper.left_finger = single_gripper["left_finger"].as<std::string>("left_finger");
    gripper.right_finger = single_gripper["right_finger"].as<std::string>("right_finger");
    gripper_map.insert({gripper_name, gripper});
  }

  YAML::Node joint_order = motor_configs["joint_order"];
  YAML::Node sleep_positions = motor_configs["sleep_positions"];

  if (joint_order.size() != sleep_positions.size())
  {
    ROS_FATAL(
      " Error when parsing Motor Config file: Length of joint_order list (%ld) does not match length of sleep_positions list (%ld).",
      joint_order.size(), sleep_positions.size());
    return false;
  }

  JointGroup all_joints;
  all_joints.joint_num = (uint8_t) joint_order.size();
  all_joints.mode = "position";
  all_joints.profile_type = "velocity";
  for (size_t i{0}; i < joint_order.size(); i++)
  {
    std::string joint_name = joint_order[i].as<std::string>();
    all_joints.joint_names.push_back(joint_name);
    all_joints.joint_ids.push_back(motor_map[joint_name].motor_id);
    js_index_map.insert({joint_name, i});
    shadow_map.insert({joint_name, {joint_name}});
    sister_map.insert({joint_name, {joint_name}});
    sleep_map.insert({joint_name, sleep_positions[i].as<float>(0)});
    if (gripper_map.count(joint_name) > 0)
    {
      gripper_map[joint_name].js_index = i;
      gripper_order.push_back(joint_name);
    }
  }

  for (auto const& name : gripper_order)
  {
    js_index_map.insert({gripper_map[name].left_finger, js_index_map.size()});
    js_index_map.insert({gripper_map[name].right_finger, js_index_map.size()});
  }

  group_map.insert({"all", all_joints});
  //all_ptr = &group_map["all"];

  YAML::Node all_shadows = motor_configs["shadows"];
  for (YAML::const_iterator master_itr = all_shadows.begin(); master_itr != all_shadows.end(); master_itr++)
  {
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    YAML::Node shadow_list = master["shadow_list"];
    for (size_t i{0}; i < shadow_list.size(); i++)
      shadow_map[master_name].push_back(shadow_list[i].as<std::string>());
  }

  YAML::Node all_sisters = motor_configs["sisters"];
  for (YAML::const_iterator sister_itr = all_sisters.begin(); sister_itr != all_sisters.end(); sister_itr++)
  {
    std::string sister_one = sister_itr->first.as<std::string>();
    std::string sister_two = sister_itr->second.as<std::string>();
    sister_map[sister_one].push_back(sister_two);
    sister_map[sister_two].push_back(sister_one);
  }

  YAML::Node all_groups = motor_configs["groups"];
  for (YAML::const_iterator group_itr = all_groups.begin(); group_itr != all_groups.end(); group_itr++)
  {
    std::string name = group_itr->first.as<std::string>();
    YAML::Node joint_list = all_groups[name];
    JointGroup group;
    group.joint_num = (uint8_t) joint_list.size();
    for (size_t i{0}; i < joint_list.size(); i++)
    {
      std::string joint_name = joint_list[i].as<std::string>();
      group.joint_names.push_back(joint_name);
      group.joint_ids.push_back(motor_map[joint_name].motor_id);
    }
    group_map.insert({name, group});
  }

  /*
  YAML::Node pub_configs = motor_configs["joint_state_publisher"];
  timer_hz = pub_configs["update_rate"].as<int>(100);
  pub_states = pub_configs["publish_states"].as<bool>(true);
  js_topic = pub_configs["topic_name"].as<std::string>("joint_states");
  */

  puppet_yaml_configs.motor_info_vec=motor_info_vec;
  puppet_yaml_configs.gripper_order=gripper_order;
  puppet_yaml_configs.control_items=control_items;
  puppet_yaml_configs.sleep_map=sleep_map;
  puppet_yaml_configs.group_map=group_map;
  puppet_yaml_configs.motor_map=motor_map;
  puppet_yaml_configs.shadow_map=shadow_map;
  puppet_yaml_configs.sister_map=sister_map;
  puppet_yaml_configs.gripper_map=gripper_map;
  puppet_yaml_configs.js_index_map=js_index_map;
  
  ROS_INFO("Loaded motor configs from '%s'.", motor_configs_file.c_str());
  return true;
}

bool VSidoBirateral::master_srv_get_robot_info(interbotix_xs_msgs::RobotInfo::Request &req, interbotix_xs_msgs::RobotInfo::Response &res)
{
  std::vector<MotorInfo> motor_info_vec=master_yaml_configs.motor_info_vec;
  std::vector<std::string> gripper_order=master_yaml_configs.gripper_order; 
  std::unordered_map<std::string, const ControlItem*> control_items=master_yaml_configs.control_items;
  
  std::unordered_map<std::string, float> sleep_map=master_yaml_configs.sleep_map;
  std::unordered_map<std::string, JointGroup> group_map=master_yaml_configs.group_map;
  std::unordered_map<std::string, MotorState> motor_map=master_yaml_configs.motor_map;
  std::unordered_map<std::string, std::vector<std::string>> shadow_map=master_yaml_configs.shadow_map;
  std::unordered_map<std::string, std::vector<std::string>> sister_map=master_yaml_configs.sister_map;
  std::unordered_map<std::string, Gripper> gripper_map=master_yaml_configs.gripper_map;
  std::unordered_map<std::string, size_t> js_index_map=master_yaml_configs.js_index_map;                        

  bool urdf_exists = false;
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;
  // Parse the urdf model to get joint limit info
  
  std::string robot_name = master_robot_name;
//  if (ros::param::has("robot_description"))
  {
    model.initParam(robot_name + "/robot_description");
    urdf_exists = true;
  }  

  if (req.cmd_type == "group")
  {
    res.joint_names = group_map[req.name].joint_names;
    res.profile_type = group_map[req.name].profile_type;
    res.mode = group_map[req.name].mode;
  }
  else if (req.cmd_type == "single")
  {
    res.joint_names.push_back(req.name);
    res.profile_type = motor_map[req.name].profile_type;
    res.mode = motor_map[req.name].mode;
  }

  res.num_joints = res.joint_names.size();

  for (auto &name : res.joint_names)
  {
    res.joint_ids.push_back(motor_map[name].motor_id);
    if (gripper_map.count(name) > 0)
    {
      res.joint_sleep_positions.push_back(robot_convert_angular_position_to_linear(gripper_map[name],0));
      name = gripper_map[name].left_finger;
    }
    else
      res.joint_sleep_positions.push_back(sleep_map[name]);
    res.joint_state_indices.push_back(js_index_map[name]);
    if (urdf_exists)
    {
      ptr = model.getJoint(name);
      res.joint_lower_limits.push_back(ptr->limits->lower);
      res.joint_upper_limits.push_back(ptr->limits->upper);
      res.joint_velocity_limits.push_back(ptr->limits->velocity);
    }
  }

  if (req.name != "all")
  {
    res.name.push_back(req.name);
  }
  else
  {
    for (auto key : group_map)
    {
      res.name.push_back(key.first);
    }
  }

  return true;
}

bool VSidoBirateral::puppet_srv_get_robot_info(interbotix_xs_msgs::RobotInfo::Request &req, interbotix_xs_msgs::RobotInfo::Response &res)
{
  std::vector<MotorInfo> motor_info_vec=puppet_yaml_configs.motor_info_vec;
  std::vector<std::string> gripper_order=puppet_yaml_configs.gripper_order; 
  std::unordered_map<std::string, const ControlItem*> control_items=puppet_yaml_configs.control_items;
  
  std::unordered_map<std::string, float> sleep_map=puppet_yaml_configs.sleep_map;
  std::unordered_map<std::string, JointGroup> group_map=puppet_yaml_configs.group_map;
  std::unordered_map<std::string, MotorState> motor_map=puppet_yaml_configs.motor_map;
  std::unordered_map<std::string, std::vector<std::string>> shadow_map=puppet_yaml_configs.shadow_map;
  std::unordered_map<std::string, std::vector<std::string>> sister_map=puppet_yaml_configs.sister_map;
  std::unordered_map<std::string, Gripper> gripper_map=puppet_yaml_configs.gripper_map;
  std::unordered_map<std::string, size_t> js_index_map=puppet_yaml_configs.js_index_map;                        

  bool urdf_exists = false;
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;

  ROS_INFO_STREAM("puppet_srv_get_robot_info:"<<req);

  // Parse the urdf model to get joint limit info
  std::string robot_name = puppet_robot_name;
  //if (ros::param::has("robot_description"))
  {
    model.initParam(robot_name + "/robot_description");
    urdf_exists = true;
  }

  if (req.cmd_type == "group")
  {
    res.joint_names = group_map[req.name].joint_names;
    res.profile_type = group_map[req.name].profile_type;
    res.mode = group_map[req.name].mode;
  }
  else if (req.cmd_type == "single")
  {
    res.joint_names.push_back(req.name);
    res.profile_type = motor_map[req.name].profile_type;
    res.mode = motor_map[req.name].mode;
  }

  res.num_joints = res.joint_names.size();

  for (auto &name : res.joint_names)
  {
    res.joint_ids.push_back(motor_map[name].motor_id);
    if (gripper_map.count(name) > 0)
    {
      res.joint_sleep_positions.push_back(robot_convert_angular_position_to_linear(gripper_map[name],0));
      name = gripper_map[name].left_finger;
    }
    else
      res.joint_sleep_positions.push_back(sleep_map[name]);
    res.joint_state_indices.push_back(js_index_map[name]);
    if (urdf_exists)
    {
      ptr = model.getJoint(name);
      res.joint_lower_limits.push_back(ptr->limits->lower);
      res.joint_upper_limits.push_back(ptr->limits->upper);
      res.joint_velocity_limits.push_back(ptr->limits->velocity);
    }
  }

  if (req.name != "all")
  {
    res.name.push_back(req.name);
  }
  else
  {
    for (auto key : group_map)
    {
      res.name.push_back(key.first);
    }
  }

  return true;
}