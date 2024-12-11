'''
計算後無濾波,將數據接收後計算出目標點與角度
'''
import rclpy
from rclpy.node import Node
import math
from scipy.optimize import fsolve
from std_msgs.msg import Float32MultiArray

class UWBCalculator(Node):
    def __init__(self):
        super().__init__('uwb_calculator_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'uwb_data',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'H_and_Degree', 1)

    def listener_callback(self, msg):
        AH, BH, CH = msg.data
        self.calculate_and_output(AH, BH, CH)

    def calculate_and_output(self, AH, BH, CH):
        # 檢查是否所有距離都是 0
        if AH == 0 and BH == 0 and CH == 0:
            H_x, H_y = 0.0, 0.0
            theta_degrees = 0.0
            self.publish_result(H_x, H_y, theta_degrees)
            self.get_logger().info(f"H: ({H_x:.2f}, {H_y:.2f}), θ: {theta_degrees:.2f}° (All distances are zero)")
            return

        # 定義固定點座標
        A = (0, 51)
        B = (-32, -51)
        C = (32, -51)
        F = (0, 0)

        # 定義方程組以解算 H 點位置
        def equations(p):
            x, y, z = p
            eq1 = math.sqrt((x - B[0])**2 + (y - B[1])**2 + z**2) - BH
            eq2 = math.sqrt((x - C[0])**2 + (y - C[1])**2 + z**2) - CH
            eq3 = math.sqrt((x - A[0])**2 + (y - A[1])**2 + z**2) - AH
            return [eq1, eq2, eq3]

        try:
            # 使用 fsolve 解方程組
            H_x, H_y, H_z = fsolve(equations, (1, 1, 1))

            # 確保 Z 軸值為正
            H_z = abs(H_z)

            # 計算 H 點相對於 F 點的方向角
            theta = math.atan2(H_y - F[1], H_x - F[0])
            theta_degrees = math.degrees(theta)

            # 確保角度為正
            if theta_degrees < 0:
                theta_degrees += 360

            self.publish_result(H_x, H_y, theta_degrees)
            self.get_logger().info(f"H: ({H_x:.2f}, {H_y:.2f}), θ: {theta_degrees:.2f}°")

        except Exception as e:
            self.get_logger().error(f"Failed to calculate H point: {e}")

    def publish_result(self, H_x, H_y, theta_degrees):
        # 發布結果
        msg = Float32MultiArray()
        msg.data = [H_x, H_y, theta_degrees]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UWBCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
