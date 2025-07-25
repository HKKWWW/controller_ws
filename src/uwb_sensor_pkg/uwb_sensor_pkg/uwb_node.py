import threading
import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from synapath_interfaces.msg import UWBMsg
from uwb_sensor_pkg.uwb import UWB

class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_node')

        # ------------- 加载 .yaml 文件 -------------
        self.usb_port = self.declare_parameter("usb_port", "/dev/ttyUSB0").value    # 指定端口

        self.uwb_T0_frame = self.declare_parameter('uwb_T0_frame', 'uwb_T0').value  # frame_id
        self.uwb_A0_frame = self.declare_parameter('uwb_A0_frame', 'uwb_A0').value
        self.uwb_A1_frame = self.declare_parameter('uwb_A1_frame', 'uwb_A1').value
        self.uwb_A2_frame = self.declare_parameter('uwb_A2_frame', 'uwb_A2').value
        self.uwb_A3_frame = self.declare_parameter('uwb_A3_frame', 'uwb_A3').value

        self.uwb_T0_coord = [self.declare_parameter('uwb_T0_x', 0.0).value,  # T0 坐标
                           self.declare_parameter('uwb_T0_y', 0.0).value,
                           self.declare_parameter('uwb_T0_z', 0.0).value]
        self.uwb_A0_coord = [self.declare_parameter('uwb_A0_x', 0.0).value,  # A0 坐标
                           self.declare_parameter('uwb_A0_y', 0.0).value,
                           self.declare_parameter('uwb_A0_z', 0.0).value]
        self.uwb_A1_coord = [self.declare_parameter('uwb_A1_x', 0.0).value,  # A1 坐标
                           self.declare_parameter('uwb_A1_y', 0.0).value,
                           self.declare_parameter('uwb_A1_z', 0.0).value]
        self.uwb_A2_coord = [self.declare_parameter('uwb_A2_x', 0.0).value,  # A2 坐标
                           self.declare_parameter('uwb_A2_y', 0.0).value,
                           self.declare_parameter('uwb_A2_z', 0.0).value]
        self.uwb_A3_coord = [self.declare_parameter('uwb_A3_x', 0.0).value,  # A3 坐标
                           self.declare_parameter('uwb_A3_y', 0.0).value,
                           self.declare_parameter('uwb_A3_z', 0.0).value]
        self.get_logger().info(
            f"\nT0: {self.uwb_T0_coord}\n"
            f"A0: {self.uwb_A0_coord}\n"
            f"A1: {self.uwb_A1_coord}\n"
            f"A2: {self.uwb_A2_coord}\n"
            f"A3: {self.uwb_A3_coord}"
        )

        self.min_distance = self.declare_parameter('min_distance', 0.0).value
        self.max_distance = self.declare_parameter('max_distance', 149.0).value

        self.pub_rate = 1.0 / self.declare_parameter('update_rate', 10.0).value
        self.get_logger().info(f"pub_rate: {self.pub_rate}")

        # ------------ 初始化 UWB ------------
        self.uwb = UWB(usb_port=self.usb_port,                                            # 连接 UWB
                                max_distance=self.max_distance, 
                                min_distance=self.min_distance)                           
        self.uwb_queue = queue.Queue(maxsize=10)                                          # 创建读取 UWB 队列
        self.uwb_thread = threading.Thread(target=self.uwb.run, args=(self.uwb_queue, ), daemon=True)  # 创建读取 UWB 线程
        self.uwb_thread.start()
        
        # ------------- 定时发布 UWB 数据 ----------
        self.publisher_ = self.create_publisher(UWBMsg, '/uwb_coord', 10)
        self.publisher_ = self.create_publisher(UWBMsg, '/uwb_imu', 10)
        
        # ------------- 定时发布 UWB 数据 ----------
        self.timer = self.create_timer(self.pub_rate, self.uwb_pub_callback)
        
    def uwb_pub_callback(self):
        self.get_logger().info("Hello UWB")

        uwb_msg = UWBMsg()
        '''
        std_msgs/Header header

        float32 t0_x
        float32 t0_y
        float32 t0_z

        float32 a0_distance
        float32 a1_distancetry_shutdown
        float32 a2_distance
        float32 a3_distance
        '''
        uwb_msg.header.frame_id = self.uwb_T0_frame
        uwb_msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(UWBMsg())

    def __del__(self):
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    uwb_node = UWBNode()
    try:
        rclpy.spin(uwb_node)
    except KeyboardInterrupt:
        uwb_node.get_logger().warn('Keyboard interrupt, shutting down...')
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
