import serial
import threading
import queue
import time
from loguru import logger
from collections import deque

class UWB():
    def __init__(self, usb_port='/dev/ttyUSB0', max_distance=149.0, min_distance=0.0):
        # -------------- 连接超声波传感器 ----------------
        logger.info("正在连接 UWB ...")
        for _ in range(5):
            try:
                self.ser = serial.Serial(usb_port, 115200, timeout=0.1)
                logger.info("成功连接 UWB")
                # --------------- UWB 参数 ----------------
                self.max_distance = max_distance
                self.min_distance = min_distance
                self.get_uwb_function = self.get_uwb_distances

                return
            except serial.SerialException as e:
                logger.error(f"连接失败: {e}")
                time.sleep(0.5)
        raise serial.SerialException("无法连接 UWB")

    def run(self, read_uwb_queue:queue.Queue):

        while True:
            try:
                uwb_data = self.get_uwb_function()  # 获取数据
                
                # ...
                read_uwb_queue.put_nowait(uwb_data)  # 将超声波传感器数据压入队列

            except Exception as e:
                print(e)

            time.sleep(0.02)

    def get_uwb_distances(self):  # 修该此读取数据
        data = self.read_frame()
        d0 = data
        d1 = -1
        d2 = -1
        d3 = -1
        # print("read frames:", d0, d1, d2, d3)  # 调试输出

        uwb_distances = [float(d0), float(d1), float(d2), float(d3)]

        for index in range(4):
                uwb_distances[index] = float(uwb_distances[index] / 1000) # 转换为米制
                if uwb_distances[index] > self.max_distance or uwb_distances[index] < self.min_distance:
                    uwb_distances[index] = -1.0

        return uwb_distances

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
    uwb = UWB(usb_port="/dev/ttyUSB0")

    uwb_queue = queue.Queue(maxsize=8)  # 创建读取超声波传感器队列

    uwb_thread = threading.Thread(target=uwb.run, args=(uwb_queue, ), daemon=True)  # 创建读取超声波传感器线程
    uwb_thread.start()

    while True:
        try:
            uwb_data = uwb_queue.get_nowait()

            print(uwb_data)

            time.sleep(0.1)
        except queue.Empty:
            pass
        
    uwb_thread.stop()
    