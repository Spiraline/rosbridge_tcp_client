#include <rosbridge_websocket_client/rosbridge_websocket_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbridge_websocket_listener");

  boost::asio::io_service io_service;

  ROSBridgeWebSocketClient ws_client(io_service);

  std::thread recv_thread = std::thread([&io_service](){ io_service.run(); });

  ws_client.subscribe("/test_msg", "std_msgs/Float32");

  ros::Rate rate(10);

  while(ros::ok()){
    auto& mailbox = ws_client.getMailBox();

    for(const auto& [topic_name, msg]: mailbox){
      ROS_INFO_STREAM(topic_name);
      ROS_INFO_STREAM(msg.dump());
    }

    rate.sleep();
  }

  return 0;
}
