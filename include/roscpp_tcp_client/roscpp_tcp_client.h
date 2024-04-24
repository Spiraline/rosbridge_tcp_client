#ifndef ROSCPP_TCP_CLIENT_H_
#define ROSCPP_TCP_CLIENT_H_

#include <unordered_map>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <ros/ros.h>

using boost::asio::ip::tcp;
using json = nlohmann::json;

class TCPClient
{
public:
  TCPClient();
  ~TCPClient();

private:
  boost::asio::io_service io_service_;
  boost::shared_ptr<tcp::socket> tx_client_, rx_client_;

  void connect();
  void subscribe();
  void advertise();
};

#include <roscpp_tcp_client/impl/roscpp_tcp_client.hpp>

#endif