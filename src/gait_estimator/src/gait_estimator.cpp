#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>   // <-- REQUIRED!

class GaitEstimatorNode : public rclcpp::Node
{
public:
    GaitEstimatorNode()
    : Node("gait_estimator")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data",
            10,
            std::bind(&GaitEstimatorNode::imuCallback, this, std::placeholders::_1)
        );

        gait_pub_ = this->create_publisher<std_msgs::msg::String>("gait_phase", 10);

        RCLCPP_INFO(this->get_logger(), "Gait Estimator node started.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double pitch = msg->orientation.y;

        std::string phase;

        if (pitch > 0.5)
            phase = "SWING";
        else if (pitch < -0.5)
            phase = "STANCE";
        else
            phase = "MID";

        std_msgs::msg::String out;
        out.data = phase;

        gait_pub_->publish(out);

        RCLCPP_INFO(this->get_logger(), "Pitch: %.2f → Phase: %s", pitch, phase.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GaitEstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
