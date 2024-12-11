'''
三個一組分別計算出四個點後進行平均 
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
        self.publisher_ = self.create_publisher(Float32MultiArray, 'smoothed_point_and_angle', 1)

        # 卡爾曼濾波初始化
        self.state = np.array([0.0, 0.0])  # 狀態: [avg_x, avg_y]
        self.covariance = np.eye(2) * 0.1
        self.process_noise = np.eye(2) * 0.01
        self.measurement_noise = np.eye(2) * 1.0

    def listener_callback(self, msg):
        try:
            AH, BH, CH, DH = msg.data

            # 檢查距離是否有效
            if any(d <= 0 for d in [AH, BH, CH, DH]):
                self.get_logger().error("Received invalid distances. All distances must be positive.")
                return

            self.calculate_and_output(AH, BH, CH, DH)
        except Exception as e:
            self.get_logger().error(f"Error in listener callback: {e}")

    def calculate_and_output(self, AH, BH, CH, DH):
        # 固定點座標
        A = (-32,  51)
        B = ( 32,  51)
        C = (-32, -51)
        D = ( 32, -51)
        F = (0, 0)  # 新增點 F

        # 通用方程組求解函數
        def solve_point(p1, p2, p3, d1, d2, d3):
            def equations(p):
                x, y, z = p
                eq1 = math.sqrt((x - p1[0])**2 + (y - p1[1])**2 + z**2) - d1
                eq2 = math.sqrt((x - p2[0])**2 + (y - p2[1])**2 + z**2) - d2
                eq3 = math.sqrt((x - p3[0])**2 + (y - p3[1])**2 + z**2) - d3
                return [eq1, eq2, eq3]

            try:
                init_guess = (
                    (p1[0] + p2[0] + p3[0]) / 3,
                    (p1[1] + p2[1] + p3[1]) / 3,
                    1
                )
                x, y, z = fsolve(equations, init_guess)
                z = abs(z)  # 確保 z 為正
                return x, y
            except Exception as e:
                self.get_logger().error(f"Failed to solve point: {e}")
                return 0.0, 0.0

        # 計算四個點
        H_x, H_y = solve_point(A, B, C, AH, BH, CH)
        I_x, I_y = solve_point(A, B, D, AH, BH, DH)
        J_x, J_y = solve_point(A, C, D, AH, CH, DH)
        K_x, K_y = solve_point(B, C, D, BH, CH, DH)

        # 將 H, I, J, K 平均為一個點
        avg_x = np.mean([H_x, I_x, J_x, K_x])
        avg_y = np.mean([H_y, I_y, J_y, K_y])

        # 使用卡爾曼濾波平滑
        smoothed_x, smoothed_y = self.kalman_filter(np.array([avg_x, avg_y]))

        # 計算角度
        angle = math.atan2(smoothed_y - F[1], smoothed_x - F[0])
        angle_degrees = math.degrees(angle)
        if angle_degrees < 0:
            angle_degrees += 360

        # 發布平滑點和角度
        self.publish_result(smoothed_x, smoothed_y, angle_degrees)

    def kalman_filter(self, measurement):
        # 預測階段
        predicted_state = self.state  # 假設靜止模型
        predicted_covariance = self.covariance + self.process_noise

        # 更新階段
        K = predicted_covariance @ np.linalg.inv(predicted_covariance + self.measurement_noise)  # 卡爾曼增益
        self.state = predicted_state + K @ (measurement - predicted_state)  # 更新狀態
        self.covariance = (np.eye(2) - K) @ predicted_covariance  # 更新協方差

        return self.state

    def publish_result(self, x, y, angle):
        msg = Float32MultiArray()
        msg.data = [x, y, angle]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Smoothed Point: ({x:.2f}, {y:.2f}), Angle: {angle:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = UWBCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
