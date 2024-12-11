import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class UWBPlotter(Node):
    def __init__(self):
        super().__init__('uwb_plotter')

        # 固定點座標
        self.points = {
            'A': (  0, 32),
            'B': (-32,  0),
            'C': ( 32,  0),
            'F': (  0,  0)
        }
        self.h_point = None
        self.angle = None

        # 訂閱 UWBCalculator 發布的 H 點與角度
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'H_and_Degree',
            self.listener_callback,
            10
        )

        # 初始化繪圖
        self.fig, self.ax = plt.subplots()
        self.point_plots = {}
        self.fh_line = None
        self.init_plot()

    def listener_callback(self, msg):
        """處理來自 UWBCalculator 的 H 點與角度數據"""
        h_x, h_y, theta = msg.data
        self.h_point = (h_x, h_y)
        self.angle = theta
        self.get_logger().info(f"Updated H: {self.h_point}, θ: {self.angle:.2f}°")

        # 更新繪圖
        self.update_plot()

    def init_plot(self):
        """初始化繪圖"""
        self.ax.set_title("UWB Points and Angle")
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-500, 500)
        self.ax.set_aspect('equal')

        # 繪製 A, B, C, F 固定點
        for label, (x, y) in self.points.items():
            self.point_plots[label] = self.ax.plot(x, y, 'o', label=f"{label} ({x}, {y})")[0]

        # 初始化 H 點與 F-H 連線
        self.point_plots['H'] = self.ax.plot([], [], 'go', label='H (--, --)')[0]
        self.fh_line, = self.ax.plot([], [], 'r-', linewidth=2, label='Angle: --°')

        # 將圖例放置於圖外
        self.ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), borderaxespad=0.)

        # 調整布局以防止圖例遮擋內容
        self.fig.tight_layout(rect=[0, 0, 0.8, 1])

    def update_plot(self):
        """更新繪圖"""
        if self.h_point is None:
            return

        # 更新 H 點
        h_x, h_y = self.h_point
        self.point_plots['H'].set_data(h_x, h_y)

        # 更新 F-H 連線
        f_x, f_y = self.points['F']
        self.fh_line.set_data([f_x, h_x], [f_y, h_y])

        # 更新角度與 H 點圖例
        self.point_plots['H'].set_label(f"H ({h_x:.2f}, {h_y:.2f})")
        self.fh_line.set_label(f"Angle: {self.angle:.2f}°")
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
