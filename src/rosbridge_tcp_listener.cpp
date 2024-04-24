#include <rosbridge_tcp_client/rosbridge_tcp_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbridge_tcp_listener");

  TCPClient tcp_client;

  tcp_client.subscribe("/test_msg", "std_msgs/Float32");

  ros::spin();

  return 0;
}
