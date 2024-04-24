#include <roscpp_tcp_client/roscpp_tcp_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roscpp_tcp_talker");

  TCPClient tcp_client;

  json float32_msg;
  int cnt = 0;
  ros::Rate rate(10);

  while(ros::ok()){
    float32_msg["data"] = (float)cnt;
    tcp_client.publish("/test_msg", "std_msgs/Float32", float32_msg);
    cnt++;
    rate.sleep();
  }

  return 0;
}
