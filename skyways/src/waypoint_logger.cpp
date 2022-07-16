#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <iostream>
#include <fstream>

using namespace std;

class NavSatSubscriber : public rclcpp::Node
{
    public:
        NavSatSubscriber(string id) : Node("gps_subscriber")
        {
            subscription_ =  this->create_subscription<sensor_msgs::msg::NavSatFix>(id + "/mavros/global_position/global", 10, [this](sensor_msgs::msg::NavSatFix::UniquePtr msg){
                    ofstream WaypointLogger("../text_files/WaypointLog.txt");
                    char ch = 'y';
                    while(ch!='n')
                    {
                        WaypointLogger << msg->latitude << " " << msg->longitude << endl;
                        cout<<"Log another waypoint (y/n)? ";
                        cin>>ch;
                    }
                    WaypointLogger.close();
                });
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if(argc!=2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: waypoint_logger <vehicle_id>");
        return 1;
    }
    string id;
    rclcpp::spin(std::make_shared<NavSatSubscriber>(id));
    rclcpp::shutdown();
    return 0;
}