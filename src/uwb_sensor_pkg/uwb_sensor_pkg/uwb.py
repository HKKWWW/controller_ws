import serial
import threading
import queue
import time
from loguru import logger
from collections import deque

class UWB():
    def __init__(self, usb_port='/dev/ttyUSB0', max_distance=149.0, min_distance=0.0) -> None:
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

    def run(self, read_uwb_queue: queue.Queue) -> None:
        while True:
            try:
                uwb_data = self.get_uwb_function()
                if uwb_data:
                    read_uwb_queue.put_nowait(uwb_data)
            except Exception as e:
                read_uwb_queue.get_nowait()
                read_uwb_queue.put_nowait(uwb_data)
            time.sleep(0.02)

    def read_frame(self) -> str | None:
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("mi,"):
                return line
        except Exception as e:
            logger.error(f"读取串口失败: {e}")
        return None

    def get_uwb_distances(self) -> dict | None:
        line = self.read_frame()
        if line is None:
            return None

        fields = line.split(',')
        if len(fields) < 23:
            logger.warning(f"字段数量不足: {len(fields)} => {line}")
            return None

        try:
            time_sec = float(fields[1])
            distances = []
            for i in range(2, 6):  # A0~A3 一共接受 3 个基站距离
                d = fields[i]
                distances.append(float(d) if d != 'null' else None)

            acc = list(map(float, fields[10:13]))
            gyro = list(map(float, fields[13:16]))
            mag = list(map(float, fields[16:19]))
            pitch, roll, yaw = map(float, fields[19:22])
            tag_id = fields[22]

            return {
                "timestamp": time_sec,
                "distances": distances,
                "acc": acc,
                "gyro": gyro,
                "mag": mag,
                "angle": [pitch, roll, yaw],
                "tag_id": tag_id
            }

        except ValueError as e:
            logger.error(f"解析失败: {e}, line: {line}")
            return None


if __name__ == '__main__':
    uwb = UWB(usb_port="/dev/ttyUSB0")

    uwb_queue = queue.Queue(maxsize=8)  # 创建读取超声波传感器队列

    uwb_thread = threading.Thread(target=uwb.run, args=(uwb_queue, ), daemon=True)  # 创建读取超声波传感器线程
    uwb_thread.start()

    while True:
        try:
            uwb_data = uwb_queue.get_nowait()

            logger.info(uwb_data)

            time.sleep(1)
        except queue.Empty:
            pass
    