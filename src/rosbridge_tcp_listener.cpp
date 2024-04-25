#include <rosbridge_tcp_client/rosbridge_tcp_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbridge_tcp_listener");

  ROSBridgeTCPClient tcp_client;

  tcp_client.subscribe("/test_msg", "std_msgs/Float32");

  ros::Rate rate(10);

  while(ros::ok()){
    auto& mailbox = tcp_client.getMailBox();

    for(const auto& [topic_name, msg]: mailbox){
      ROS_INFO_STREAM(topic_name);
      ROS_INFO_STREAM(msg.dump());
    }

    rate.sleep();
  }

  return 0;
}
