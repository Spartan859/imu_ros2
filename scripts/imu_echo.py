#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import math

class ImuSimpleEcho(Node):
    def __init__(self):
        super().__init__('imu_simple_echo')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10)
        self.last_print = time.time()

    def listener_callback(self, msg):
        now = time.time()
        # 只每0.5秒打印一次
        if now - self.last_print > 0.5:
            # eul: orientation (四元数，需转换为欧拉角)
            # gyr: angular_velocity
            # 四元数转欧拉角
            qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            # roll (x轴旋转)
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            # pitch (y轴旋转)
            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
            # yaw (z轴旋转)
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            print(f"gyr: x={msg.angular_velocity.x:.3f}, y={msg.angular_velocity.y:.3f}, z={msg.angular_velocity.z:.3f}")
            print(f"eul: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
            self.last_print = now


def main(args=None):
    rclpy.init(args=args)
    node = ImuSimpleEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
