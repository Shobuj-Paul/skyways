#include <rclcpp/rclcpp.hpp>
#include <skyways/srv/data_packet.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>

using namespace std;

//Callback function for processing the request data packet and generating the response data packet.
void data(const std::shared_ptr<skyways::srv::DataPacket::Request> request, std::shared_ptr<skyways::srv::DataPacket::Response> response)
{
    if (request->vehicle_id == "drone1") 
    {
        ifstream WaypointReader("../text_files/WaypointLog.txt");
        string line;
        int lines;
        for(lines = 0; std::getline(WaypointReader,line); lines++);
        response->waypoints.poses.resize(lines);
        for(int i = 0; std::getline(WaypointReader,line); i++)
        {
            stringstream ss(line);
            string waypoint;
            ss>>waypoint;
            response->waypoints.poses[i].position.x = stod(waypoint);
            ss>>waypoint;
            response->waypoints.poses[i].position.y = stod(waypoint);
        }
        response->vel_mag = 1.0;
        response->geofence_radius = 1.0;
        response->corridor_radius = 5.0;
        response->altitude = 5.0;

        RCLCPP_INFO(rclcpp::get_logger("received"), "Vehicle ID: %s", request->vehicle_id.c_str());
        for(int i=0; i<lines; i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("received"), "Waypoint X: %lf Waypoint Y: %lf", response->waypoints.poses[i].position.x);
        }
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