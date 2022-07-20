#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>
#include <fstream>

using namespace std;

//Callback function for processing the request data packet and generating the response data packet.
void data(const std::shared_ptr<skyways::srv::DataPacket::Request> request, std::shared_ptr<skyways::srv::DataPacket::Response> response)
{
    if (request->vehicle_id == "drone1") 
    {
        RCLCPP_INFO(rclcpp::get_logger("received"), "Vehicle ID: %s", request->vehicle_id.c_str());
        // Read waypoints from text file and log them in terminal
        string home = getenv("HOME");
        ifstream WaypointReader(home + "/colcon_ws/src/skyways/files/WaypointLog.txt");
        string line;
        int lines;
        for(lines = 0; getline(WaypointReader,line); lines++);
        RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoints Extracted: %d", lines);
        WaypointReader.clear();
        WaypointReader.seekg(0);
        response->waypoints.poses.resize(lines);
        for(int i=0; getline(WaypointReader,line); i++)
        {
            istringstream iss(line);
            double a, b;
            if(!(iss >> a >> b))
                break;
            response->waypoints.poses[i].position.x = a;
            response->waypoints.poses[i].position.y = b;
            RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoint GPS Coordinates %d: (%lf, %lf)", i+1, response->waypoints.poses[i].position.x, response->waypoints.poses[i].position.y);
        }
        WaypointReader.close();
        response->vel_mag = 1.0;
        response->geofence_radius = 1.0;
        response->corridor_radius = 5.0;
        response->altitude = 5.0;
    }
    else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("client"), "Vehicle ID not found.");
    }
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