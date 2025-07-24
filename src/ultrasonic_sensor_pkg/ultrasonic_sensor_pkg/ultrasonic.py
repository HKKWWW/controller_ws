import serial
import threading
import queue
import time
from loguru import logger
from collections import deque

class Ultrasonic:
    def __init__(self, usb_port='/dev/ttyUSB0', max_range=1.6, min_range=0.014, is_fliter=False):
        # -------------- 连接超声波传感器 ----------------
        logger.info("正在连接超声传感器 ...")
        for _ in range(5):
            try:
                self.ser = serial.Serial(usb_port, 115200, timeout=0.1)
                logger.info("成功连接超声波")
                # --------------- 超声波参数 ----------------
                self.max_range = max_range
                self.min_range = min_range

                if not is_fliter:  # 是否开启滤波
                    self.get_ultrasonic_function = self.get_ultrasonic_distances
                    return
                logger.info("--- 开启滤波 ---")
                self.get_ultrasonic_function = self.get_ultrasonic_distances_fliter
                self.windows = [deque(maxlen=5) for _ in range(4)]  # 四个平滑窗口
                
                return
            except serial.SerialException as e:
                logger.error(f"连接失败: {e}")
                time.sleep(0.5)
        raise serial.SerialException("无法连接超声波传感器")

    def run(self, read_ultrasonic_queue:queue.Queue):

        while True:
            try:
                ultrasonic_distances = self.get_ultrasonic_distances()  # 获取数据
                read_ultrasonic_queue.put_nowait(ultrasonic_distances)  # 将超声波传感器数据压入队列
            except Exception as e:
                print(e)

            time.sleep(0.02)

    def get_ultrasonic_distances_fliter(self):  # 修该此读取数据
        data = self.read_frame()
        d0 = data
        d1 = -1
        d2 = -1
        d3 = -1
        # print("read frames:", d0, d1, d2, d3)  # 调试输出

        ultrasonic_distances = [float(d0), float(d1), float(d2), float(d3)]

        filtered = []
        for i, v in enumerate(ultrasonic_distances):
            # 范围过滤 超出阈值范围外的数据 置为 -1
            if v is None or not (self.min_range * 1000 <= v <= self.max_range * 1000):
                val = float(-1.0)
            else:
                val = float(v / 1000.0)      # 转为米
                self.windows[i].append(val)  # 添加到滑动窗口

            # 滑动平均
            avg = sum(self.windows[i]) / len(self.windows[i])
            filtered.append(avg)

        return ultrasonic_distances
    
    def get_ultrasonic_distances(self):  # 修该此读取数据
        data = self.read_frame()
        d0 = data
        d1 = -1
        d2 = -1
        d3 = -1
        # print("read frames:", d0, d1, d2, d3)  # 调试输出

        ultrasonic_distances = [float(d0), float(d1), float(d2), float(d3)]

        for index in range(4):
                ultrasonic_distances[index] = float(ultrasonic_distances[index] / 1000) # 转换为米制
                if ultrasonic_distances[index] > self.max_range or ultrasonic_distances[index] < self.min_range:
                    ultrasonic_distances[index] = -1.0

        return ultrasonic_distances

    def read_frame(self) -> float | None:
        # 按 FF + 数据位 (高位 + 低位) + FD 协议读取一帧
        # 同步帧头 0xFF
        for _ in range(3):
            first = self.ser.read(1)
            if first:
                break
            time.sleep(0.005)  # 等待 5 ms

        if first[0] == 0xFF:
            time.sleep(0.01)  # 等待 10 ms，以避免数据丢失
            data = self.ser.read(3)
            if len(data) != 3 or data[2] != 0xFD:
                return float('nan')
            # print(data)
            return (data[0] << 8) | data[1]
        
        return float('nan')


if __name__ == '__main__':
    ultrasonic = Ultrasonic(usb_port="/dev/ttyUSB0")

    ultrasonic_queue = queue.Queue(maxsize=8)  # 创建读取超声波传感器队列

    ultrasonic_thread = threading.Thread(target=ultrasonic.run, args=(ultrasonic_queue, ), daemon=True)  # 创建读取超声波传感器线程
    ultrasonic_thread.start()

    while True:
        try:
            ultrasonic_data = ultrasonic_queue.get_nowait()

            print(ultrasonic_data)

            time.sleep(0.1)
        except queue.Empty:
            pass
        
    ultrasonic_thread.stop()
    