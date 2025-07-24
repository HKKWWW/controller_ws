import threading
import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from synapath_interfaces.msg import UWBMsg
from UWB_sensor_pkg.UWB import UWB

class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_node')

        # ------------- 加载 .yaml 文件 -------------
        self.usb_port = self.declare_parameter("usb_port", "/dev/ttyUSB0").value  # 指定端口

        self.UWB_T0_frame = self.declare_parameter('UWB_T0_frame', 'UWB_T0').value  # frame_id
        self.UWB_A0_frame = self.declare_parameter('UWB_A0_frame', 'UWB_A0').value
        self.UWB_A1_frame = self.declare_parameter('UWB_A1_frame', 'UWB_A1').value
        self.UWB_A2_frame = self.declare_parameter('UWB_A2_frame', 'UWB_A2').value
        self.UWB_A3_frame = self.declare_parameter('UWB_A3_frame', 'UWB_A3').value

        self.UWB_T0_cor = [self.declare_parameter('UWB_T0_x', 0.0).value,  # T0 坐标
                           self.declare_parameter('UWB_T0_y', 0.0).value,
                           self.declare_parameter('UWB_T0_z', 0.0).value]
        self.UWB_A0_cor = [self.declare_parameter('UWB_A0_x', 0.0).value,  # A0 坐标
                           self.declare_parameter('UWB_A0_y', 0.0).value,
                           self.declare_parameter('UWB_A0_z', 0.0).value]
        self.UWB_A1_cor = [self.declare_parameter('UWB_A1_x', 0.0).value,  # A1 坐标
                           self.declare_parameter('UWB_A1_y', 0.0).value,
                           self.declare_parameter('UWB_A1_z', 0.0).value]
        self.UWB_A2_cor = [self.declare_parameter('UWB_A2_x', 0.0).value,  # A2 坐标
                           self.declare_parameter('UWB_A2_y', 0.0).value,
                           self.declare_parameter('UWB_A2_z', 0.0).value]
        self.UWB_A3_cor = [self.declare_parameter('UWB_A3_x', 0.0).value,  # A3 坐标
                           self.declare_parameter('UWB_A3_y', 0.0).value,
                           self.declare_parameter('UWB_A3_z', 0.0).value]
        # self.get_logger().info(
        #     f"T0: {self.UWB_T0_cor}\n"
        #     f"A0: {self.UWB_A0_cor}\n"
        #     f"A1: {self.UWB_A1_cor}\n"
        #     f"A2: {self.UWB_A2_cor}\n"
        #     f"A3: {self.UWB_A3_cor}"
        # )

        self.min_distance = self.declare_parameter('min_distance', 0.0).value
        self.max_distance = self.declare_parameter('max_distance', 149.0).value

        self.pub_rate = 1.0 / self.declare_parameter('update_rate', 10.0).value
        self.get_logger().info(f"pub_rate: {self.pub_rate}")

        # ------------ 初始化 UWB ------------
        # self.uwb = UWB(usb_port=self.usb_port, 
        #                         max_distance=self.max_distance, 
        #                         min_distance=self.min_distance)  # 连接超声波传感器
        # self.UWB_queue = queue.Queue(maxsize=10)  # 创建读取超声波传感器队列
        # self.UWB_thread = threading.Thread(target=self.uwb.run, args=(self.UWB_queue, ))  # 创建读取超声波传感器线程
        # self.UWB_thread.start()
        
        # ------------- 定时发布 UWB 数据 ----------
        self.publisher_ = self.create_publisher(UWBMsg, '/UWB_cor', 10)
        self.publisher_ = self.create_publisher(UWBMsg, '/UWB_imu', 10)
        
        # ------------- 定时发布 UWB 数据 ----------
        self.timer = self.create_timer(self.pub_rate, self.UWB_pub_callback)
        
    def UWB_pub_callback(self):
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
        uwb_msg.header.frame_id = self.UWB_T0_frame
        uwb_msg.header.stamp = self.get_clock().now().to_msg()

        
        self.publisher_.publish(UWBMsg())

def main(args=None):
    rclpy.init(args=args)

    UWB_node = UWBNode()
    try:
        rclpy.spin(UWB_node)
    except KeyboardInterrupt:
        UWB_node.get_logger().warn('Keyboard interrupt, shutting down...')
    finally:
        # UWB_node.UWB_thread.stop()
        UWB_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
