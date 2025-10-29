#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_ros2/imu_parser.hpp"

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), parser_() {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "imu/raw_data", 10,
            std::bind(&ImuNode::data_callback, this, std::placeholders::_1));
    }

private:
    void data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        for (auto byte : msg->data) {
            parser_.feed(byte);
        }
        publish_imu_data();
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
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    ImuParser parser_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}