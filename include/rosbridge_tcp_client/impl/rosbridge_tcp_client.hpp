#ifndef ROSBRIDGE_TCP_CLIENT_HPP_
#define ROSBRIDGE_TCP_CLIENT_HPP_

ROSBridgeTCPClient::ROSBridgeTCPClient()
{
  std::signal(SIGINT, [](int sig){
    ROS_ERROR("Ctrl+C pressed. Exiting...");
    ros::shutdown();
    exit(1);
  });

  rosbridge_address_ = nh_.param<std::string>("rosbridge_address", "127.0.0.1");
  rosbridge_port_ = nh_.param<int>("rosbridge_port", 9090);
  socket_.reset(new tcp::socket(io_service_));
  recv_buffer_.resize(PACKET_SIZE);
}

ROSBridgeTCPClient::~ROSBridgeTCPClient(){}

bool ROSBridgeTCPClient::connect()
{
  boost::asio::ip::address rosbridge_address = boost::asio::ip::address::from_string(rosbridge_address_);

  tcp::endpoint rosbridge_endpoint(rosbridge_address, rosbridge_port_);

  try{
    socket_->connect(rosbridge_endpoint);
    bridge_connected_ = true;
    socket_->async_read_some(boost::asio::buffer(recv_buffer_),
    [&](const boost::system::error_code& ec, std::size_t recv_byte){
      receiveCallback(ec, recv_byte);
    });

    recv_thread_ = std::thread([this](){ io_service_.run(); });
  }
  catch (std::exception & e){
    ROS_WARN_STREAM("[ROSBridgeTCPClient] " << e.what());
    return false;
  }

  return true;
}

void ROSBridgeTCPClient::receiveCallback(const boost::system::error_code& ec, std::size_t recv_byte)
{
  if(ec){
    ROS_WARN_STREAM("Error in receive: " << ec.message());
    bridge_connected_ = false;
    return;
  }

  std::string recv_str(recv_buffer_.begin(), recv_buffer_.begin() + recv_byte);
  json recv_json = json::parse(recv_str);

  // Parse and do something
  ROS_INFO_STREAM(recv_json.dump());

  socket_->async_read_some(boost::asio::buffer(recv_buffer_),
  [&](const boost::system::error_code& ec, std::size_t recv_byte){
    receiveCallback(ec, recv_byte);
  });
}

void ROSBridgeTCPClient::advertise(const std::string &topic_name, const std::string &topic_type)
{
  if(!bridge_connected_){
    while(!connect()){
      rate_.sleep();
    }
  }

  json adv_json;
  adv_json["op"] = "advertise";
  adv_json["topic"] = topic_name;
  adv_json["type"] = topic_type;

  socket_->async_write_some(boost::asio::buffer(adv_json.dump()),
    [&](const boost::system::error_code& ec, std::size_t transfer_byte) {
      if(ec){
        ROS_WARN_STREAM("Error in advertise: " << ec.message());
        bridge_connected_ = false;
      }
      else{
        ROS_INFO_STREAM("Advertise " << topic_name);
      }
  });

  adv_topic_set_.insert(topic_name);
}

void ROSBridgeTCPClient::subscribe(const std::string &topic_name, const std::string &topic_type, int throttle_rate = 0, int queue_length = 10)
{
  if(!bridge_connected_){
    while(!connect()){
      rate_.sleep();
    }
  }

  if(!adv_topic_set_.count(topic_name)){
    advertise(topic_name, topic_type);
  }

  json sub_json;
  sub_json["op"] = "subscribe";
  sub_json["topic"] = topic_name;
  sub_json["msgs_data"] = topic_type;
  sub_json["throttle_rate"] = throttle_rate;
  sub_json["queue_length"] = queue_length;

  socket_->async_write_some(boost::asio::buffer(sub_json.dump()),
    [&](const boost::system::error_code& ec, std::size_t transfer_byte) {
      if(ec){
        ROS_WARN_STREAM("Error in subscribe: " << ec.message());
        bridge_connected_ = false;
      }
      else{
        ROS_INFO_STREAM("Subscribe " << topic_name);
      }
  });
}

void ROSBridgeTCPClient::publish(const std::string &topic_name, const std::string &topic_type, const json &msg_json)
{
  if(!bridge_connected_){
    while(!connect()){
      rate_.sleep();
    }
  }

  if(!adv_topic_set_.count(topic_name)){
    advertise(topic_name, topic_type);
  }

  json pub_json;
  pub_json["op"] = "publish";
  pub_json["topic"] = topic_name;
  pub_json["msg"] = msg_json;

  socket_->async_write_some(boost::asio::buffer(pub_json.dump()),
    [&](const boost::system::error_code& ec, std::size_t transfer_byte) {
      if(ec){
        ROS_WARN_STREAM("Error in publish: " << ec.message());
        bridge_connected_ = false;
      }
      else{
        ROS_INFO_STREAM("Publish " << topic_name);
      }
  });
}

#endif