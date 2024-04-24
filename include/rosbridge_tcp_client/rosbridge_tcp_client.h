#ifndef ROSBRIDGE_TCP_CLIENT_H_
#define ROSBRIDGE_TCP_CLIENT_H_

#include <unordered_set>
#include <thread>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <signal.h>

using boost::asio::ip::tcp;
using json = nlohmann::json;

class TCPClient
{
private:
  ros::NodeHandle nh_;
  ros::Timer connect_timer_;

  boost::asio::io_service io_service_;
  boost::shared_ptr<tcp::socket> socket_;
  static constexpr int PACKET_SIZE = 2048;
  std::vector<char> recv_buffer_;
  std::thread recv_thread_;

  std::string rosbridge_address_;
  int rosbridge_port_;

  bool bridge_connected_{false};

  std::unordered_set<std::string> adv_topic_set_;

  ros::Rate rate_{ros::Rate(1)};

public:
  TCPClient();
  ~TCPClient();
  void advertise(const std::string &topic_name, const std::string &topic_type);
  void subscribe(const std::string &topic_name, const std::string &topic_type, int throttle_rate, int queue_length);
  void publish(const std::string &topic_name, const std::string &topic_type, const json &msg_json);

private:
  bool connect();
  void receiveCallback(const boost::system::error_code& ec, std::size_t recv_byte);
};

#include <rosbridge_tcp_client/impl/rosbridge_tcp_client.hpp>

#endif