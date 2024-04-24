#include <roscpp_tcp_client/roscpp_tcp_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roscpp_tcp_talker");

  TCPClient tcp_client;

  ros::spin();

  return 0;
}
