#ifndef ROSCPP_TCP_CLIENT_H_
#define ROSCPP_TCP_CLIENT_H_

#include <unordered_set>
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
  void connectTimerCallback(const ros::TimerEvent e);
};

#include <roscpp_tcp_client/impl/roscpp_tcp_client.hpp>

#endif