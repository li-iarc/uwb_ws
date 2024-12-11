'''
三個數值都更新後才輸出
'''
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

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

class UWBDataAggregator(Node):
    def __init__(self, ids_to_use):
        super().__init__('uwb_data_aggregator')
        self.data = {id: None for id in ids_to_use}
        self.ids_to_use = ids_to_use

        # 動態創建訂閱者，訂閱指定的 UWB ID 主題
        for id in ids_to_use:
            self.create_subscription(Float32, f'uwb_data_{id}', lambda msg, id=id: self.callback(msg, id), 1)
            self.get_logger().info(f"Subscribed to uwb_data_{id}")

        # 整合後的數據發布至 'uwb_data' 主題
        self.publisher_ = self.create_publisher(Float32MultiArray, 'uwb_data', 1)

    def callback(self, msg, id):
        # 更新接收的數據
        self.data[id] = msg.data
        self.get_logger().info(f"Received distance from ID {id}: {msg.data}")

        # 檢查是否所有數據都已接收，並發布
        if all(value is not None for value in self.data.values()):
            self.publish_combined_data()

    def publish_combined_data(self):
        # 將數據打包成 Float32MultiArray 並發布
        msg = Float32MultiArray(data=[self.data[id] for id in self.ids_to_use])
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published combined UWB data: {msg.data}")

        # 重置資料，等待新的數據
        for id in self.ids_to_use:
            self.data[id] = None

def main(args=None):
    rclpy.init(args=args)

    # 從命令列參數獲取要使用的 ID 列表
    if len(sys.argv) < 2:
        print("請指定要使用的 UWB ID，例如：ros2 run uwb UWBDataAggregator 0 2 3")
        sys.exit(1)

    ids_to_use = [int(id) for id in sys.argv[1:]]
    
    # 檢查 ID 是否有效
    for id in ids_to_use:
        if id not in ID_TO_SERIAL_PORT:
            print(f"無效的 ID：{id}")
            sys.exit(1)

    aggregator = UWBDataAggregator(ids_to_use)
    rclpy.spin(aggregator)
    aggregator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
