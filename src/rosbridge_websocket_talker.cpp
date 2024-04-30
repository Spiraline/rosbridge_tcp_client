#include <rosbridge_websocket_client/rosbridge_websocket_client.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbridge_websocket_talker");

  boost::asio::io_service io_service;

  ROSBridgeWebSocketClient ws_client(io_service);

  std::thread recv_thread = std::thread([&io_service](){ io_service.run(); });

  json float32_msg;
  int cnt = 0;
  ros::Rate rate(10);

  while(ros::ok()){
    float32_msg["data"] = (float)cnt;
    ws_client.publish("/test_msg", "std_msgs/Float32", float32_msg);
    cnt++;
    rate.sleep();
  }

  return 0;
}
