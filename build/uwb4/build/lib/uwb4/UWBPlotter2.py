import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class UWBPlotter(Node):
    def __init__(self):
        super().__init__('uwb_plotter')

        # 固定點座標
        self.points = {
            'A': (-32, 51),
            'B': (32, 51),
            'C': (-32, -51),
            'D': (32, -51),
            'F': (0, 0)  # 新增點 F
        }
        self.smoothed_point = None  # 動態平滑點
        self.angle = None  # 方向角
        self.fh_line = None  # F-H 連線

        # 訂閱 UWBCalculator 發布的平滑點和角度
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'smoothed_point_and_angle',
            self.listener_callback,
            10
        )

        # 初始化繪圖
        self.fig, self.ax = plt.subplots()
        self.point_plots = {}
        self.smoothed_plot = None
        self.init_plot()

    def listener_callback(self, msg):
        """處理來自 UWBCalculator 的平滑點和角度數據"""
        smoothed_x, smoothed_y, angle = msg.data
        self.smoothed_point = (smoothed_x, smoothed_y)
        self.angle = angle
        self.get_logger().info(f"Updated Smoothed Point: ({smoothed_x:.2f}, {smoothed_y:.2f}), Angle: {angle:.2f}°")

        # 更新繪圖
        self.update_plot()

    def init_plot(self):
        """初始化繪圖"""
        self.ax.set_title("UWB Points and Smoothed Point with Angle")
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-500, 500)
        self.ax.set_aspect('equal')

        # 繪製固定點 A, B, C, D, F
        for label, (x, y) in self.points.items():
            self.point_plots[label] = self.ax.plot(x, y, 'o', label=f"{label} ({x}, {y})", color='black')[0]

        # 初始化平滑動態點
        self.smoothed_plot, = self.ax.plot([], [], 'ro', label='Smoothed Point (--, --)')

        # 初始化 F-H 連線
        self.fh_line, = self.ax.plot([], [], 'b-', linewidth=1.5, label='F-H Line (Angle: --°)')

        # 添加圖例
        self.ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), borderaxespad=0.)

        # 調整布局以防止圖例遮擋內容
        self.fig.tight_layout(rect=[0, 0, 0.8, 1])

    def update_plot(self):
        """更新繪圖"""
        if self.smoothed_point is None or self.angle is None:
            return

        # 更新平滑動態點位置
        smoothed_x, smoothed_y = self.smoothed_point
        self.smoothed_plot.set_data(smoothed_x, smoothed_y)
        self.smoothed_plot.set_label(f"H ({smoothed_x:.2f}, {smoothed_y:.2f})")

        # 更新 F-H 連線
        f_x, f_y = self.points['F']
        self.fh_line.set_data([f_x, smoothed_x], [f_y, smoothed_y])
        self.fh_line.set_label(f"Angle: {self.angle:.2f}°")

        # 刷新圖例
        self.ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), borderaxespad=0.)

        # 刷新繪圖
        self.fig.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = UWBPlotter()

    # 使用 matplotlib 動態顯示
    def run_plotter():
        plt.show(block=False)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.1)

    try:
        run_plotter()
    except KeyboardInterrupt:
        print("\nPlotting interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
