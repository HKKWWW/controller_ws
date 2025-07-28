import threading
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ultrasonic_sensor_pkg.ultrasonic import Ultrasonic


class UltrasonicNode(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_node')

        # ------------- 加载 .yaml 文件 -------------
        self.usb_port = self.declare_parameter("usb_port", "/dev/ttyUSB0").value

        self.sensor_front_frame = self.declare_parameter('sensor_front_frame', 'ultrasonic_front').value
        self.sensor_left_frame = self.declare_parameter('sensor_left_frame', 'ultrasonic_left').value
        self.sensor_right_frame = self.declare_parameter('sensor_right_frame', 'ultrasonic_right').value
        self.sensor_behind_frame = self.declare_parameter('sensor_behind_frame', 'ultrasonic_behind').value
        
        self.min_range = self.declare_parameter('min_range', 0.02).value
        self.max_range = self.declare_parameter('max_range', 4.0).value
        self.is_fliter = self.declare_parameter('is_fliter', False).value
        self.field_of_view = self.declare_parameter('field_of_view', 0.5236).value
        self.pub_rate = 1.0 / self.declare_parameter('update_rate', 10.0).value
        self.get_logger().info(f"pub_rate: {self.pub_rate}")

        # ------------ 初始化超声波传感器 ------------
        self.ultrasonic = Ultrasonic(usb_port=self.usb_port, 
                                     max_range=self.max_range, 
                                     min_range=self.min_range, 
                                     is_fliter=self.is_fliter)  # 连接超声波传感器
        self.ultrasonic_queue = queue.Queue(maxsize=10)  # 创建读取超声波传感器队列
        self.ultrasonic_thread = threading.Thread(target=self.ultrasonic.run, args=(self.ultrasonic_queue, ), daemon=True)  # 创建读取超声波传感器线程
        self.ultrasonic_thread.start()

        # --------------- 发布超声波话题 ---------------
        self.publisher_ = self.create_publisher(Float32MultiArray, '/ultrasonic_topic', 10)
        
        # ------------- 定时发布超声波数据 ----------
        self.timer = self.create_timer(self.pub_rate, self.ultrasonic_pub_callback)

    def ultrasonic_pub_callback(self) -> None:
        try:
            data = self.ultrasonic_queue.get_nowait()
        except queue.Empty:
            return  # 无数据直接返回 不发布

        msg = Float32MultiArray()
        msg.data = data
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Publishing: {data}")

    def __del__(self) -> None:
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    ultrasonic_node = UltrasonicNode()
    try:
        rclpy.spin(ultrasonic_node)
    except KeyboardInterrupt:
        ultrasonic_node.get_logger().warn('Keyboard interrupt, shutting down...')
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
