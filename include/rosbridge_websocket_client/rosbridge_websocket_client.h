#ifndef ROSBRIDGE_WEBSOCKET_CLIENT_H_
#define ROSBRIDGE_WEBSOCKET_CLIENT_H_

#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <string>
#include <exception>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <signal.h>

using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;
using json = nlohmann::json;

class ROSBridgeWebSocketClient
{
private:
  ros::NodeHandle nh_;
  websocket::stream<tcp::socket> ws_;
  tcp::resolver resolver_;
  boost::beast::flat_buffer recv_buffer_;

  std::string rosbridge_ip_;
  int rosbridge_port_;

  bool bridge_connected_{false};

  std::unordered_set<std::string> adv_topic_set_;

  ros::Rate rate_{ros::Rate(1)};

  // key: topic name
  // value: last msg에 대한 json
  std::unordered_map<std::string, json> mailbox_;

public:
  ROSBridgeWebSocketClient(boost::asio::io_service& io_service);
  ~ROSBridgeWebSocketClient();
  void advertise(const std::string &topic_name, const std::string &topic_type);
  void subscribe(const std::string &topic_name, const std::string &topic_type, int throttle_rate, int queue_length);
  void publish(const std::string &topic_name, const std::string &topic_type, const json &msg_json);

  inline std::unordered_map<std::string, json>& getMailBox(){
    return mailbox_;
  };

private:
  bool connect();
  void receiveCallback(const boost::system::error_code& ec, std::size_t recv_byte);
};

#include <rosbridge_websocket_client/impl/rosbridge_websocket_client.hpp>

#endif