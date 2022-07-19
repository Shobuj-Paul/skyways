#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
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
  rclcpp::WallRate loop_rate(500ms); //Rate object for sleep time in the loop.
  
  //Topic Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub = node->create_publisher<std_msgs::msg::Float64>(id + "/data_packet/velocity", 10);  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr geofence_pub = node->create_publisher<std_msgs::msg::Float64>(id + "/data_packet/geofence_radius", 10);  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub = node->create_publisher<std_msgs::msg::Float64>(id + "/data_packet/altitude", 10);
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_pub = node->create_publisher<geometry_msgs::msg::PoseArray>(id + "/data_packet/waypoints", 10);

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
  geometry_msgs::msg::PoseArray waypoints;
  std_msgs::msg::Float64 vel_mag;
  std_msgs::msg::Float64 altitude;
  std_msgs::msg::Float64 geofence_radius;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
      waypoints = result.get()->waypoints; //Get the waypoints from the response.
      vel_mag.data = result.get()->vel_mag; //Get the velocity magnitude from the response.
      geofence_radius.data = result.get()->geofence_radius; //Get the geofence radius from the response.
      altitude.data = result.get()->altitude; //Get the altitude from the response.
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Velocity Magnitude: %f", vel_mag.data);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Geofence Radius: %f", geofence_radius.data);
      RCLCPP_INFO(rclcpp::get_logger("recieved"), "Altitude: %f", altitude.data);

      auto n = waypoints.poses.size(); //Get the number of waypoints.
      for(int i=0; i<(int)n; i++) //Log the waypoints in the terminal
      {
        RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoint %d: %f %f %f", i+1, waypoints.poses[i].position.x, waypoints.poses[i].position.y, waypoints.poses[i].position.z);
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
      vel_pub->publish(vel_mag);
      geofence_pub->publish(geofence_radius);
      altitude_pub->publish(altitude);
      waypoint_pub->publish(waypoints);
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