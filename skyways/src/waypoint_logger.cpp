#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <fstream>

using namespace std;
using std::placeholders::_1;

class NavSatSubscriber : public rclcpp::Node
{
    public:
        NavSatSubscriber(string id) : Node("gps_subscriber")
        {
            subscription_ =  this->create_subscription<sensor_msgs::msg::NavSatFix>(id + "/mavros/global_position/global", 10, [this](const sensor_msgs::msg::NavSatFix::UniquePtr msg)
            {
                ofstream WaypointLogger("src/skyways/text_files/WaypointLog.txt");
                    char ch = 'y';
                    while(ch!='n')
                    {
                        WaypointLogger << msg->latitude << endl << msg->longitude << endl;
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
    string id = argv[1];
    rclcpp::spin(std::make_shared<NavSatSubscriber>(id));
    rclcpp::shutdown();
    return 0;
}