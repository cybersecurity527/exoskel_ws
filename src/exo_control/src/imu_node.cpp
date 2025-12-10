#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <chrono>

using namespace std::chrono_literals;

class BNO055Node : public rclcpp::Node {
public:
    BNO055Node(int mux_channel)
        : Node("imu_node"), mux_channel_(mux_channel)
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        fd_ = open("/dev/i2c-1", O_RDWR);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open I2C bus");
            return;
        }

        selectMuxChannel(mux_channel_);

        if (ioctl(fd_, I2C_SLAVE, 0x28) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not set BNO055 address");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "IMU initialized on mux channel %d", mux_channel_);

        timer_ = this->create_wall_timer(
            20ms, std::bind(&BNO055Node::readImu, this)
        );
    }

private:
    void selectMuxChannel(int channel)
    {
        ioctl(fd_, I2C_SLAVE, 0x70);     // TCA9548A address
        uint8_t cmd = 1 << channel;      // enable channel X
        write(fd_, &cmd, 1);
        usleep(5000);
    }

    void readImu()
    {
        selectMuxChannel(mux_channel_);

        uint8_t reg = 0x1A;   // Start of Euler angle registers
        write(fd_, &reg, 1);

        uint8_t buf[6];
        read(fd_, buf, 6);

        int16_t heading = buf[0] | (buf[1] << 8);
        int16_t roll    = buf[2] | (buf[3] << 8);
        int16_t pitch   = buf[4] | (buf[5] << 8);

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = now();
        msg.header.frame_id = "imu";

        // convert to radians
        msg.orientation.z = heading / 16.0 * (M_PI / 180);
        msg.orientation.x = roll    / 16.0 * (M_PI / 180);
        msg.orientation.y = pitch   / 16.0 * (M_PI / 180);

        pub_->publish(msg);
    }

    int fd_;
    int mux_channel_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int channel = 0;
    if (argc > 1) {
        channel = std::stoi(argv[1]);
    }

    auto node = std::make_shared<BNO055Node>(channel);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
