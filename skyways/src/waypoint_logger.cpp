#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <fstream>

using namespace std;

class NavSatSubscriber : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
        void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
            {
                lat = msg->latitude;
                lon = msg->longitude;
                rclcpp::shutdown();
            }
    public:
        double lat, lon;
        NavSatSubscriber(string id) : Node("gps_subscriber")
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(id + "/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&NavSatSubscriber::gps_callback, this, std::placeholders::_1));
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if(argc!=2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: waypoint_logger <vehicle_id>");
        return 1;
    }
    string id = string(argv[1]);
    auto node = std::make_shared<NavSatSubscriber>(id);
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("check"), "Latitude: %lf Longitude: %lf", node.get()->lat, node.get()->lon);
    
    ofstream WaypointLogger("colcon_ws/src/skyways/files/WaypointLog.txt", ios_base::app);
    WaypointLogger << node.get()->lat << " " << node.get()->lon << endl;
    WaypointLogger.close();

    return 0;
}