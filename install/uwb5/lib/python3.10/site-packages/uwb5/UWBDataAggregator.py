import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class UWBDataAggregator(Node):
    def __init__(self, ids_to_use):
        super().__init__('uwb_data_aggregator')
        self.ids_to_use = ids_to_use
        self.data = {id: None for id in ids_to_use}
        self.last_update_time = {id: None for id in ids_to_use}
        self.first_publish_done = False
        self.timeout = 5.0  # 超時時間（秒）

        # 訂閱每個指定的 UWBReceiver 節點
        for id in ids_to_use:
            self.create_subscription(Float32, f'uwb_data_{id}', lambda msg, id=id: self.callback(msg, id), 10)
            self.get_logger().info(f"Subscribed to uwb_data_{id}")

        # 整合後的數據發布至 'uwb_data' 主題
        self.publisher_ = self.create_publisher(Float32MultiArray, 'uwb_data', 1)

        # 定時檢查超時
        self.timer = self.create_timer(1.0, self.check_timeout)

    def callback(self, msg, id):
        # 更新接收的數據與時間戳
        self.data[id] = msg.data
        self.last_update_time[id] = time.time()
        self.get_logger().info(f"Received distance from ID {id}: {msg.data}")

        # 發布數據
        if not self.first_publish_done:
            if all(value is not None for value in self.data.values()):
                self.publish_combined_data()
                self.first_publish_done = True
        else:
            self.publish_combined_data()

    def publish_combined_data(self):
        # 發布數據
        if all(value is not None for value in self.data.values()):
            msg = Float32MultiArray(data=[self.data[id] for id in self.ids_to_use])
        else:
            # 若未完成首次更新，發布 [0, 0, 0, 0]
            msg = Float32MultiArray(data=[0.0] * len(self.ids_to_use))
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published combined UWB data: {msg.data}")

    def check_timeout(self):
        # 檢查是否有數據超時
        current_time = time.time()
        for id, last_time in self.last_update_time.items():
            if last_time is None or (current_time - last_time > self.timeout):
                self.get_logger().warning(f"Data from ID {id} has not been updated for {self.timeout} seconds.")
                self.reset_data()
                break

    def reset_data(self):
        # 重置所有數據為 0 並發布
        self.data = {id: 0.0 for id in self.ids_to_use}
        self.publisher_.publish(Float32MultiArray(data=[0.0] * len(self.ids_to_use)))
        self.first_publish_done = False

def main(args=None):
    rclpy.init(args=args)

    # 從命令列參數獲取要使用的 ID 列表
    if len(sys.argv) < 2:
        print("請指定要使用的 UWB ID，例如：ros2 run uwb UWBDataAggregator 0 1 2 3")
        sys.exit(1)

    ids_to_use = [int(id) for id in sys.argv[1:]]
    aggregator = UWBDataAggregator(ids_to_use)

    try:
        rclpy.spin(aggregator)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        aggregator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
