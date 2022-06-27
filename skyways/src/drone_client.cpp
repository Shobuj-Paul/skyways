#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if(argc!=2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: drone_client <vehicle_id>");
    return 1;
  }
  std::string id = argv[1]; 
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("drone_client");
  rclcpp::Client<skyways::srv::DataPacket>::SharedPtr client = node->create_client<skyways::srv::DataPacket>("drone_service");
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher = node->create_publisher<geometry_msgs::msg::PoseArray>(id + "/waypoints_publisher", 10);
  rclcpp::WallRate loop_rate(500ms);
  auto request = std::make_shared<skyways::srv::DataPacket::Request>();
  request->vehicle_id = id;
  RCLCPP_INFO(rclcpp::get_logger("sending"), "Vehicle ID: %s", request->vehicle_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("sending"), "Start Position: %f %f %f", request->start_position.x, request->start_position.y, request->start_position.z);
  RCLCPP_INFO(rclcpp::get_logger("sending"), "End Position: %f %f %f", request->end_position.x, request->end_position.y, request->end_position.z);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("client"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  geometry_msgs::msg::PoseArray waypoints;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
      waypoints = result.get()->waypoints;
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Lane ID: %d", result.get()->lane_id);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Velocity Magnitude: %f", result.get()->vel_mag);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Geofence Radius: %f", result.get()->geofence_radius);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Corridor Radius: %f", result.get()->corridor_radius);
      auto n = waypoints.poses.size();
      for(int i=0; i<(int)n; i++){
        RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoint %d: %f %f %f", i, waypoints.poses[i].position.x, waypoints.poses[i].position.y, waypoints.poses[i].position.z);
      }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("client"), "Failed to call service drone_service.");
  }

  for(int i=0; i<100 && rclcpp::ok(); i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("publisher"), "Publishing waypoints on topic waypoints_publisher");
    try {
      publisher->publish(waypoints);
      rclcpp::spin_some(node);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("publisher"), "Exception: %s", e.what());
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}