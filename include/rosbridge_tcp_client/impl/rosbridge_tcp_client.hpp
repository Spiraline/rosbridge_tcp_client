#ifndef ROSBRIDGE_TCP_CLIENT_HPP_
#define ROSBRIDGE_TCP_CLIENT_HPP_

ROSBridgeTCPClient::ROSBridgeTCPClient():
  nh_("~")
{
  std::signal(SIGINT, [](int sig){
    ROS_ERROR("Ctrl+C pressed. Exiting...");
    exit(1);
  });

  rosbridge_ip_ = nh_.param<std::string>("rosbridge_ip", "127.0.0.1");
  rosbridge_port_ = nh_.param<int>("rosbridge_port", 9090);
  socket_.reset(new tcp::socket(io_service_));
  recv_buffer_.resize(PACKET_SIZE);
}

ROSBridgeTCPClient::~ROSBridgeTCPClient(){}

bool ROSBridgeTCPClient::connect()
{
  boost::asio::ip::address rosbridge_address = boost::asio::ip::address::from_string(rosbridge_ip_);

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

  // TCP는 메시지 경계가 없으므로 여러 메시지가 한번에 들어올 수 있다.
  // 따라서 op로 시작하는 모든 메시지를 처리한다.
  std::vector<std::string> substr_list;
  std::size_t start_idx = 0;
  std::size_t end_idx = recv_str.find(DELIMITER, 1);
  while(end_idx != std::string::npos){
    substr_list.push_back(recv_str.substr(start_idx, end_idx - start_idx));
    start_idx = end_idx;
    end_idx = recv_str.find(DELIMITER, 1);
  }
  substr_list.push_back(recv_str.substr(start_idx));

  for(const std::string& recv_substr: substr_list){
    json recv_json = json::parse(recv_substr);

    // Check validity
    if(recv_json.find("op") != recv_json.end() && 
      recv_json.find("topic") != recv_json.end() &&
      recv_json.find("msg") != recv_json.end())
    {
      const std::string &op = recv_json["op"].get<std::string>();
      const std::string &topic_name = recv_json["topic"].get<std::string>();
      json &msg = recv_json["msg"];

      // Insert to mailbox
      if(op == "publish" && mailbox_.count(topic_name)){
        mailbox_[topic_name] = msg;
      }
    }
  }

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
    mailbox_[topic_name] = json({});
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