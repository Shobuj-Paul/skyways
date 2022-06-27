#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>

void data(const std::shared_ptr<skyways::srv::DataPacket::Request> request, std::shared_ptr<skyways::srv::DataPacket::Response> response)
{
  if (request->vehicle_id == "drone1") {
    response->lane_id = 1;
    response->vel_mag = 1.0;
    response->geofence_radius = 2.5;
    response->corridor_radius = 5.0;
    response->waypoints.poses.resize(5);
    response->waypoints.poses[0].position.x = 0;
    response->waypoints.poses[0].position.y = 0;
    response->waypoints.poses[0].position.z = 5;
    response->waypoints.poses[1].position.x = 0;
    response->waypoints.poses[1].position.y = 5;
    response->waypoints.poses[1].position.z = 5;
    response->waypoints.poses[2].position.x = 5;
    response->waypoints.poses[2].position.y = 5;
    response->waypoints.poses[2].position.z = 5;
    response->waypoints.poses[3].position.x = 5;
    response->waypoints.poses[3].position.y = 0;
    response->waypoints.poses[3].position.z = 5;
    response->waypoints.poses[4].position.x = 0;
    response->waypoints.poses[4].position.y = 0;
    response->waypoints.poses[4].position.z = 5;
  }
  else if (request->vehicle_id == "drone2") {
    response->lane_id = 1;
    response->vel_mag = 1.0;
    response->geofence_radius = 2.5;
    response->corridor_radius = 5.0;
    response->waypoints.poses.resize(5);
    response->waypoints.poses[0].position.x = 0;
    response->waypoints.poses[0].position.y = 0;
    response->waypoints.poses[0].position.z = 5;
    response->waypoints.poses[1].position.x = 5;
    response->waypoints.poses[1].position.y = 0;
    response->waypoints.poses[1].position.z = 5;
    response->waypoints.poses[2].position.x = 5;
    response->waypoints.poses[2].position.y = 5;
    response->waypoints.poses[2].position.z = 5;
    response->waypoints.poses[3].position.x = 0;
    response->waypoints.poses[3].position.y = 5;
    response->waypoints.poses[3].position.z = 5;
    response->waypoints.poses[4].position.x = 0;
    response->waypoints.poses[4].position.y = 0;
    response->waypoints.poses[4].position.z = 5;
  }
  else {
      RCLCPP_ERROR(rclcpp::get_logger("client"), "Vehicle ID not found.");
  }

  RCLCPP_INFO(rclcpp::get_logger("received"), "Vehicle ID: %s", request->vehicle_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("received"), "Start Position: %f %f %f", request->start_position.x, request->start_position.y, request->start_position.z);
  RCLCPP_INFO(rclcpp::get_logger("received"), "End Position: %f %f %f", request->end_position.x, request->end_position.y, request->end_position.z);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("drone_server");

  rclcpp::Service<skyways::srv::DataPacket>::SharedPtr service =             
    node->create_service<skyways::srv::DataPacket>("drone_service",  &data);

  RCLCPP_INFO(rclcpp::get_logger("drone_server"), "Ready to send data packet.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}