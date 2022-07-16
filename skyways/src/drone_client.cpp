#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); //Initialize the node
  if(argc!=2) //Check condition for minimum number of arguments
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: drone_client <vehicle_id>");
    return 1;
  }
  // Storing the command line arguments into temporary variables
  std::string id = argv[1]; //Vehicle ID
  
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("drone_client"); //Create a node object.
  rclcpp::Client<skyways::srv::DataPacket>::SharedPtr client = node->create_client<skyways::srv::DataPacket>("drone_service"); //Create the client object.
  rclcpp::Publisher<skyways::srv::DataPacket_Response>::SharedPtr publisher = node->create_publisher<skyways::srv::DataPacket_Response>(id + "/data_packet", 10); //Create a publisher object with specific vehicle ID.
  rclcpp::WallRate loop_rate(500ms); //Rate object for sleep time in the loop.
  
  auto request = std::make_shared<skyways::srv::DataPacket::Request>(); //Request object for accessing request data packet.
  
  //Assign request data packet values.
  request->vehicle_id = id;
  request->start_position.x = 0;
  request->start_position.y = 0;
  request->start_position.z = 0;
  request->end_position.x = 0;
  request->end_position.y = 0;
  request->end_position.z = 0;
  
  //Log the values in terminal.
  RCLCPP_INFO(rclcpp::get_logger("sending"), "Vehicle ID: %s", request->vehicle_id.c_str());
  //RCLCPP_INFO(rclcpp::get_logger("sending"), "Start Position: %f %f %f", request->start_position.x, request->start_position.y, request->start_position.z);
  //RCLCPP_INFO(rclcpp::get_logger("sending"), "End Position: %f %f %f", request->end_position.x, request->end_position.y, request->end_position.z);

  //Wait for the service to be available.
  while (!client->wait_for_service(1s)) 
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("client"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  skyways::srv::DataPacket_Response packet;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
      packet.waypoints = result.get()->waypoints; //Get the waypoints from the response.
      packet.vel_mag = result.get()->vel_mag; //Get the velocity magnitude from the response.
      packet.geofence_radius = result.get()->geofence_radius; //Get the geofence radius from the response.
      packet.corridor_radius = result.get()->corridor_radius; //Get the corridor radius from the response.
      packet.lane_id = result.get()->lane_id; //Get the lane ID from the response.
      packet.altitude = result.get()->altitude; //Get the altitude from the response.
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Velocity Magnitude: %f", packet.vel_mag);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Geofence Radius: %f", packet.geofence_radius);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Corridor Radius: %f", packet.corridor_radius);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Altitude: %f", packet.altitude);
      auto n = packet.waypoints.poses.size(); //Get the number of waypoints.
      for(int i=0; i<(int)n; i++) //Log the waypoints in the terminal
      {
        RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoint %d: %f %f %f", i, packet.waypoints.poses[i].position.x, packet.waypoints.poses[i].position.y, packet.waypoints.poses[i].position.z);
      }
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("client"), "Failed to call service drone_service.");
  }

  while(rclcpp::ok()) //Start publisher to publish waypoints for ROS 1 control node.
  {
    RCLCPP_INFO(rclcpp::get_logger("publisher"), "Publishing waypoints on topic waypoints_publisher");
    try {
      publisher->publish(packet);
      rclcpp::spin_some(node);
    } 
    catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("publisher"), "Exception: %s", e.what());
    }
    loop_rate.sleep(); //Sleep for some time.
  }
  rclcpp::shutdown(); //Shutdown the node.
  return 0;
}