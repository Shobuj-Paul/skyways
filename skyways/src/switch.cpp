#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class SwitchPublisher : public rclcpp::Node
{
    private:
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        SwitchPublisher() : Node("switch_publisher")
        {
            publisher_ = this->create_publisher<std_msgs::msg::Bool>("/gfswitch", 10);
            auto timer_callback = [this]()-> void {
                auto message = std_msgs::msg::Bool();
                message.data = true;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data ? "true" : "false");
                this->publisher_->publish(message);
            };
            timer_ = this->create_wall_timer(500ms, timer_callback);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwitchPublisher>());
    rclcpp::shutdown();
    return 0;
}