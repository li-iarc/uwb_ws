'''
計算後濾波,將數據接收後計算出目標點與角度
'''
import rclpy
from rclpy.node import Node
import math
from scipy.optimize import fsolve
from std_msgs.msg import Float32MultiArray
import numpy as np

class UWBCalculator(Node):
    def __init__(self):
        super().__init__('uwb_calculator_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'uwb_data',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'H_and_Degree', 1)

        # 卡爾曼濾波初始化
        self.state = np.array([0.0, 0.0, 0.0])  # 狀態: [H_x, H_y, theta]
        self.covariance = np.eye(3) * 0.1      # 狀態協方差矩陣
        self.process_noise = np.eye(3) * 0.01  # 過程噪聲
        self.measurement_noise = np.eye(3) * 0.5  # 測量噪聲

    def listener_callback(self, msg):
        try:
            data = msg.data  # 接收數組數據
            if len(data) < 3:
                self.get_logger().error("Insufficient data received. At least 3 distances are required.")
                return

            # 取前三個距離值
            AH, BH, CH = data[:3]

            # 檢查距離是否有效
            if any(d <= 0 for d in [AH, BH, CH]):
                self.get_logger().error("Received invalid distances. All distances must be positive.")
                return

            self.calculate_and_output(AH, BH, CH)
        except Exception as e:
            self.get_logger().error(f"Error in listener callback: {e}")

    def calculate_and_output(self, AH, BH, CH):
        if AH == 0 and BH == 0 and CH == 0:
            self.publish_result(0.0, 0.0, 0.0)
            self.get_logger().info("All distances are zero, outputting default values.")
            return

        # 固定點座標
        A = (  0, 32)
        B = (-32,  0)
        C = ( 32,  0)
        F = (  0,  0)

        def equations(p):
            x, y, z = p
            eq1 = math.sqrt((x - A[0])**2 + (y - A[1])**2 + z**2) - AH
            eq2 = math.sqrt((x - B[0])**2 + (y - B[1])**2 + z**2) - BH
            eq3 = math.sqrt((x - C[0])**2 + (y - C[1])**2 + z**2) - CH
            return [eq1, eq2, eq3]

        try:
            init_guess = (
                (A[0] + B[0] + C[0]) / 3,
                (A[1] + B[1] + C[1]) / 3,
                1
            )
            H_x, H_y, H_z = fsolve(equations, init_guess)

            H_z = abs(H_z)

            theta = math.atan2(H_y - F[1], H_x - F[0])
            theta_degrees = math.degrees(theta)
            if theta_degrees < 0:
                theta_degrees += 360

            # 使用卡爾曼濾波更新
            self.kalman_filter(np.array([H_x, H_y, theta_degrees]))

        except Exception as e:
            self.get_logger().error(f"Failed to calculate H point: {e}")
            self.publish_result(0.0, 0.0, 0.0)

    def kalman_filter(self, measurement):
        """
        使用卡爾曼濾波平滑計算結果
        """
        # 預測階段
        predicted_state = self.state  # 假設靜止模型
        predicted_covariance = self.covariance + self.process_noise

        # 更新階段
        K = predicted_covariance @ np.linalg.inv(predicted_covariance + self.measurement_noise)  # 卡爾曼增益
        self.state = predicted_state + K @ (measurement - predicted_state)  # 更新狀態
        self.covariance = (np.eye(3) - K) @ predicted_covariance  # 更新協方差

        # 發布平滑後的結果
        H_x, H_y, theta_degrees = self.state
        self.publish_result(H_x, H_y, theta_degrees)

    def publish_result(self, H_x, H_y, theta_degrees):
        """
        將平滑後的 H 點和角度發布
        """
        msg = Float32MultiArray()
        msg.data = [H_x, H_y, theta_degrees]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Filtered H: ({H_x:.2f}, {H_y:.2f}), θ: {theta_degrees:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = UWBCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
