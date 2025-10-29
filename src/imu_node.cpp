#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_ros2/imu_parser.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>


class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), parser_(), stop_flag_(false), fd_(-1) {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baud", 115200);
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        serial_thread_ = std::thread(&ImuNode::serial_read_loop, this);
    }

    ~ImuNode() {
        stop_flag_ = true;
        if (serial_thread_.joinable()) serial_thread_.join();
        if (fd_ > 0) close(fd_);
    }

private:
    void serial_read_loop() {
        std::string port = get_parameter("port").as_string();
        int baudrate = get_parameter("baud").as_int();
        int baud = B115200;
        if (baudrate == 9600) baud = B9600;
        else if (baudrate == 19200) baud = B19200;
        else if (baudrate == 38400) baud = B38400;
        else if (baudrate == 57600) baud = B57600;
        else if (baudrate == 115200) baud = B115200;
        // 可扩展更多波特率

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s", port.c_str());
            return;
        }
        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tcsetattr(fd_, TCSANOW, &tty);

        uint8_t buf[256];
        while (rclcpp::ok() && !stop_flag_) {
            int n = read(fd_, buf, sizeof(buf));
            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    parser_.feed(buf[i]);
                }
                publish_imu_data();
            }
            usleep(2000); // 2ms
        }
    }

    void publish_imu_data() {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_frame";

        imu_msg.linear_acceleration.x = parser_.acc[0];
        imu_msg.linear_acceleration.y = parser_.acc[1];
        imu_msg.linear_acceleration.z = parser_.acc[2];

        imu_msg.angular_velocity.x = parser_.gyr[0];
        imu_msg.angular_velocity.y = parser_.gyr[1];
        imu_msg.angular_velocity.z = parser_.gyr[2];

        // Assuming the orientation is not available, set to zero
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        publisher_->publish(imu_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    ImuParser parser_;
    std::thread serial_thread_;
    std::atomic<bool> stop_flag_;
    int fd_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}