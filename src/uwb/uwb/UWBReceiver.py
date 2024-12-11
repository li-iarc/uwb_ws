import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import Float32
import numpy as np

# 定義 ID 與 serial port 對應關係
ID_TO_SERIAL_PORT = {
    0: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_02619997-if00-port0',
    1: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_0111621A-if00-port0',
    2: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199A8-if00-port0',
    3: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_026199F2-if00-port0',
    4: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF44C-if00-port0',
    5: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF47D-if00-port0',
    6: '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_025EF471-if00-port0'
}

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_estimate=0.0, initial_estimate_error=1.0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate
        self.estimate_error = initial_estimate_error

    def update(self, measurement):
        kalman_gain = self.estimate_error / (self.estimate_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * self.estimate_error + self.process_variance
        return self.estimate

class UWBReceiver(Node):
    def __init__(self, id, serial_port):
        super().__init__(f'uwb_receiver_node_{id}')
        self.id = id
        self.ser = serial.Serial(serial_port, 115200, timeout=1)
        self.distance = None
        self.previous_distance = None
        self.outlier_threshold = 500
        self.lock = threading.Lock()
        self.publisher_ = self.create_publisher(Float32, f'uwb_data_{id}', 1)
        self.kalman_filter = None
        self.initial_samples = []
        self.samples_needed = 99
        self.median_initial_value = None

        threading.Thread(target=self.read_from_serial, daemon=True).start()

    def initialize_filter(self, distance):
        self.initial_samples.append(distance)
        if len(self.initial_samples) >= self.samples_needed:
            self.median_initial_value = np.median(self.initial_samples)
            self.kalman_filter = KalmanFilter(process_variance=1.0, measurement_variance=4.0,
                                              initial_estimate=self.median_initial_value, initial_estimate_error=1.0)
            self.distance = self.median_initial_value
            self.previous_distance = self.distance
            self.get_logger().info(f"Kalman filter initialized with median value: {self.median_initial_value}")

    def is_outlier(self, current_value, previous_value):
        if previous_value is None:
            return False
        return abs(current_value - previous_value) > self.outlier_threshold

    def read_from_serial(self):
        buffer = ""
        try:
            while True:
                if self.ser.in_waiting > 0:
                    # 使用 errors='ignore' 忽略解碼錯誤
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    lines = buffer.split('\r\n')

                    for line in lines[:-1]:
                        try:
                            # 嘗試解析數據
                            distance_str = line.split(":")[1].strip().replace(' cm', '')
                            distance = float(distance_str)
                        except (ValueError, IndexError):
                            # 無效數據，報錯時附帶原始訊息
                            self.get_logger().warning(f"Received invalid data format: {line}")
                            continue

                        with self.lock:
                            if self.kalman_filter is None:
                                self.initialize_filter(distance)
                            else:
                                if not self.is_outlier(distance, self.previous_distance):
                                    self.distance = self.kalman_filter.update(distance)
                                    self.previous_distance = self.distance
                                    self.publish_uwb_data()
                                else:
                                    self.get_logger().warning(f"Outlier detected and ignored: {distance}")

                    buffer = lines[-1]
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")


    def publish_uwb_data(self):
        msg = Float32()
        msg.data = round(self.distance, 2)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published UWB data from ID {self.id}: Distance = {msg.data}")

    def print_initial_samples(self):
        if len(self.initial_samples) >= self.samples_needed:
            print(f"[UWB Receiver {self.get_name()}] Initial samples collected: {self.initial_samples}")
            print(f"[UWB Receiver {self.get_name()}] Median initial value: {self.median_initial_value}")
        else:
            print(f"[UWB Receiver {self.get_name()}] Not enough samples collected for initialization.")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("請指定要使用的UWB ID，例如：python3 your_script.py 0 1 2")
        sys.exit(1)

    ids_to_use = [int(id) for id in sys.argv[1:]]
    
    for id in ids_to_use:
        if id not in ID_TO_SERIAL_PORT:
            print(f"無效的ID：{id}")
            sys.exit(1)

    nodes = [UWBReceiver(id, ID_TO_SERIAL_PORT[id]) for id in ids_to_use]

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Printing initial samples and median values for each node...")
        for node in nodes:
            node.print_initial_samples()
    finally:
        for node in nodes:
            node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()