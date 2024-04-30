#ifndef ROSBRIDGE_WEBSOCKET_CLIENT_HPP_
#define ROSBRIDGE_WEBSOCKET_CLIENT_HPP_

class MyException : public std::exception {
public:
  MyException(std::string error_str){
    error_str_ = error_str;
  }
  virtual const char* what() const throw(){
    return error_str_.c_str();
  }
private:
  std::string error_str_;
};

ROSBridgeWebSocketClient::ROSBridgeWebSocketClient(boost::asio::io_service& io_service):
  nh_("~"),
  ws_(io_service),
  resolver_(io_service)
{
  std::signal(SIGINT, [](int sig){
    ROS_ERROR("Ctrl+C pressed. Exiting...");
    exit(1);
  });

  rosbridge_ip_ = nh_.param<std::string>("rosbridge_ip", "127.0.0.1");
  rosbridge_port_ = nh_.param<int>("rosbridge_port", 9090);
}

ROSBridgeWebSocketClient::~ROSBridgeWebSocketClient(){}

bool ROSBridgeWebSocketClient::connect()
{
  boost::asio::ip::address rosbridge_address = boost::asio::ip::address::from_string(rosbridge_ip_);
  tcp::endpoint bridge_endpoint(rosbridge_address, rosbridge_port_);
  tcp::resolver::iterator results = resolver_.resolve(bridge_endpoint);

  try{
    boost::asio::async_connect(ws_.next_layer(), results,
    [&](const boost::system::error_code &ec, auto e){
      if(ec){
        throw MyException(ec.message());
      }
    });

    ws_.async_handshake(rosbridge_ip_ + ":" + std::to_string(rosbridge_port_), "/",
    [&](const boost::system::error_code &ec){
      if(ec){
        throw MyException(ec.message());
      }
    });

    ws_.async_read(recv_buffer_,
    [&](const boost::system::error_code& ec, std::size_t recv_byte){
      receiveCallback(ec, recv_byte);
    });

    bridge_connected_ = true;
  }
  catch (std::exception & e){
    ROS_WARN_STREAM("[ROSBridgeWebSocketClient] " << e.what());
    return false;
  }

  return true;
}

void ROSBridgeWebSocketClient::receiveCallback(const boost::system::error_code& ec, std::size_t recv_byte)
{
  if(ec){
    ROS_WARN_STREAM("Error in receive: " << ec.message());
    bridge_connected_ = false;
    return;
  }

  std::string recv_str = boost::beast::buffers_to_string(recv_buffer_.data()).substr(0, recv_byte);

  json recv_json = json::parse(recv_str);

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

  ws_.async_read(recv_buffer_,
  [&](const boost::system::error_code& ec, std::size_t recv_byte){
    receiveCallback(ec, recv_byte);
  });
}

void ROSBridgeWebSocketClient::advertise(const std::string &topic_name, const std::string &topic_type)
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

  ws_.async_write(boost::asio::buffer(adv_json.dump()),
    [&](const boost::system::error_code& ec, std::size_t) {
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

void ROSBridgeWebSocketClient::subscribe(const std::string &topic_name, const std::string &topic_type, int throttle_rate = 0, int queue_length = 10)
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

  ws_.async_write(boost::asio::buffer(sub_json.dump()),
    [&](const boost::system::error_code& ec, std::size_t) {
      if(ec){
        ROS_WARN_STREAM("Error in subscribe: " << ec.message());
        bridge_connected_ = false;
      }
      else{
        ROS_INFO_STREAM("Subscribe " << topic_name);
      }
  });
}

void ROSBridgeWebSocketClient::publish(const std::string &topic_name, const std::string &topic_type, const json &msg_json)
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

  ws_.async_write(boost::asio::buffer(pub_json.dump()),
    [&](const boost::system::error_code& ec, std::size_t) {
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