#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
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
        // 判断欧拉角是否合法（可根据实际需求调整范围）
        double roll_val = parser_.eul[1];
        double pitch_val = parser_.eul[0];
        double yaw_val = parser_.eul[2];
        bool valid = true;
        // 例如 roll/pitch 一般在 -90~90，yaw 在 -180~180
        if (std::isnan(roll_val) || std::isnan(pitch_val) || std::isnan(yaw_val)) valid = false;
        if (fabs(roll_val) > 90.0 || fabs(pitch_val) > 90.0 || fabs(yaw_val) > 180.0) valid = false;
        if (fabs(roll_val) < 1e-6 && fabs(pitch_val) < 1e-6 && fabs(yaw_val) < 1e-6) valid = false;
        if (!valid) {
            RCLCPP_WARN(this->get_logger(), "eul非法或全为0: roll=%.2f pitch=%.2f yaw=%.2f，数据未发布", roll_val, pitch_val, yaw_val);
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_frame";

        imu_msg.linear_acceleration.x = parser_.acc[0];
        imu_msg.linear_acceleration.y = parser_.acc[1];
        imu_msg.linear_acceleration.z = parser_.acc[2];

        imu_msg.angular_velocity.x = parser_.gyr[0];
        imu_msg.angular_velocity.y = parser_.gyr[1];
        imu_msg.angular_velocity.z = parser_.gyr[2];

        // 欧拉角转四元数
        double roll = roll_val * M_PI / 180.0;
        double pitch = pitch_val * M_PI / 180.0;
        double yaw = yaw_val * M_PI / 180.0;
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

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