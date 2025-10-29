import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial
import threading
import time

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.publisher = self.create_publisher(UInt8MultiArray, 'imu/raw_data', 10)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._imu_read_loop)
        self.thread.start()

    def _imu_read_loop(self):
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=0.5)
        except Exception as e:
            self.get_logger().error(f"打开串口失败 {port}@{baud}: {e}")
            self.stop_event.set()
            return
        self.get_logger().info(f"串口已打开: {port} @ {baud}, 开始读取 IMU 数据")
        try:
            while rclpy.ok() and not self.stop_event.is_set():
                data = ser.read(ser.in_waiting or 1)
                if data:
                    msg = UInt8MultiArray()
                    msg.data = list(data)
                    self.publisher.publish(msg)
                time.sleep(0.002)
        except Exception as e:
            self.get_logger().error(f"IMU 读取线程异常: {e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self.stop_event.set()

    def destroy_node(self):
        self.stop_event.set()
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
