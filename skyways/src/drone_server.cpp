#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>

//Callback function for processing the request data packet and generating the response data packet.
void data(const std::shared_ptr<skyways::srv::DataPacket::Request> request, std::shared_ptr<skyways::srv::DataPacket::Response> response)
{
  if (request->vehicle_id == "drone1") {
    response->vel_mag = 1.0;
    response->geofence_radius = 1.0;
    response->corridor_radius = 5.0;
    response->altitude = 5.0;
  }
  else if (request->vehicle_id == "drone2") {
    response->vel_mag = 1.0;
    response->geofence_radius = 1.0;
    response->corridor_radius = 5.0;
    response->altitude = 5.0;
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
  rclcpp::init(argc, argv); //Initialize the ROS2 node.
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("drone_server"); //Create node object.
  rclcpp::Service<skyways::srv::DataPacket>::SharedPtr service = node->create_service<skyways::srv::DataPacket>("drone_service",  &data); // Create service object with callback function.
  RCLCPP_INFO(rclcpp::get_logger("drone_server"), "Ready to send data packet.");

  rclcpp::spin(node); //Spin the node once.
  rclcpp::shutdown(); //Shutdown the node.
}