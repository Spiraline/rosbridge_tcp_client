#include <roscpp_tcp_client/roscpp_tcp_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roscpp_tcp_listener");

  TCPClient tcp_client;

  tcp_client.subscribe("/test_msg", "std_msgs/Float32");

  ros::spin();

  return 0;
}
