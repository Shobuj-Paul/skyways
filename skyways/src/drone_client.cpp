#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); //Initialize the node
  if(argc!=8) //Check condition for minimum number of arguments
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: drone_client <vehicle_id> <start x> <start y> <start z> <end x> <end y> <end z>");
    return 1;
  }
  // Storing the command line arguments into temporary variables
  std::string id = argv[1]; 
  double start_x = atof(argv[2]);
  double start_y = atof(argv[3]);
  double start_z = atof(argv[4]);
  double end_x = atof(argv[5]);
  double end_y = atof(argv[6]);
  double end_z = atof(argv[7]);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("drone_client"); //Create a node object.
  rclcpp::Client<skyways::srv::DataPacket>::SharedPtr client = node->create_client<skyways::srv::DataPacket>("drone_service"); //Create the client object.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher = node->create_publisher<geometry_msgs::msg::PoseArray>(id + "/waypoints_publisher", 10); //Create a publisher object with specific vehicle ID.
  rclcpp::WallRate loop_rate(500ms); //Rate object for sleep time in the loop.
  auto request = std::make_shared<skyways::srv::DataPacket::Request>(); //Request object for accessing request data packet.
  //Assign request data packet values.
  request->vehicle_id = id;
  request->start_position.x = start_x;
  request->start_position.y = start_y;
  request->start_position.z = start_z;
  request->end_position.x = end_x;
  request->end_position.y = end_y;
  request->end_position.z = end_z;
  //Log the values in terminal.
  RCLCPP_INFO(rclcpp::get_logger("sending"), "Vehicle ID: %s", request->vehicle_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("sending"), "Start Position: %f %f %f", request->start_position.x, request->start_position.y, request->start_position.z);
  RCLCPP_INFO(rclcpp::get_logger("sending"), "End Position: %f %f %f", request->end_position.x, request->end_position.y, request->end_position.z);

  //Wait for the service to be available.
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
      waypoints = result.get()->waypoints; //Get the waypoints from the response.
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Lane ID: %d", result.get()->lane_id);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Velocity Magnitude: %f", result.get()->vel_mag);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Geofence Radius: %f", result.get()->geofence_radius);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Corridor Radius: %f", result.get()->corridor_radius);
      auto n = waypoints.poses.size(); //Get the number of waypoints.
      for(int i=0; i<(int)n; i++) //Log the waypoints in the terminal
      {
        RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoint %d: %f %f %f", i, waypoints.poses[i].position.x, waypoints.poses[i].position.y, waypoints.poses[i].position.z);
      }
  } else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("client"), "Failed to call service drone_service.");
  }

  for(int i=0; i<100 && rclcpp::ok(); i++) //Start publisher to publish waypoints for ROS 1 control node.
  {
    RCLCPP_INFO(rclcpp::get_logger("publisher"), "Publishing waypoints on topic waypoints_publisher");
    try {
      publisher->publish(waypoints);
      rclcpp::spin_some(node);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("publisher"), "Exception: %s", e.what());
    }
    loop_rate.sleep(); //Sleep for some time.
  }

  rclcpp::shutdown(); //Shutdown the node.
  return 0;
}