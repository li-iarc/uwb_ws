'''
繪製四個點
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class UWBPlotter(Node):
    def __init__(self):
        super().__init__('uwb_plotter')

        # 固定點座標
        self.points = {
            # 'A': (0, 50),
            # 'B': (-30, 0),
            # 'C': (30, 0),
            # 'D': (0, -30)
            'A': (0, 100),
            'B': (-50, 0),
            'C': (50, 0),
            'D': (0, -50)
        }
        self.dynamic_points = {
            'H': {'coords': None, 'color': 'g'},  # 綠色
            'I': {'coords': None, 'color': 'b'},  # 藍色
            'J': {'coords': None, 'color': 'm'},  # 紫色
            'K': {'coords': None, 'color': 'c'}   # 青色
        }

        # 訂閱 UWBCalculator 發布的八個點的數據
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'points_output',
            self.listener_callback,
            10
        )

        # 初始化繪圖
        self.fig, self.ax = plt.subplots()
        self.point_plots = {}
        self.init_plot()

    def listener_callback(self, msg):
        """處理來自 UWBCalculator 的八個點數據"""
        H_x, H_y, I_x, I_y, J_x, J_y, K_x, K_y = msg.data
        self.dynamic_points['H']['coords'] = (H_x, H_y)
        self.dynamic_points['I']['coords'] = (I_x, I_y)
        self.dynamic_points['J']['coords'] = (J_x, J_y)
        self.dynamic_points['K']['coords'] = (K_x, K_y)
        self.get_logger().info(
            f"Updated Points: H({H_x:.2f}, {H_y:.2f}), I({I_x:.2f}, {I_y:.2f}), J({J_x:.2f}, {J_y:.2f}), K({K_x:.2f}, {K_y:.2f})"
        )

        # 更新繪圖
        self.update_plot()

    def init_plot(self):
        """初始化繪圖"""
        self.ax.set_title("UWB Points")
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-500, 500)
        self.ax.set_aspect('equal')

        # 繪製 A, B, C, D 固定點
        for label, (x, y) in self.points.items():
            self.point_plots[label] = self.ax.plot(x, y, 'o', label=f"{label} ({x}, {y})", color='black')[0]

        # 初始化 H, I, J, K 動態點（不同顏色）
        for label, info in self.dynamic_points.items():
            self.point_plots[label] = self.ax.plot([], [], 'o', label=f"{label} (--, --)", color=info['color'])[0]

        # 添加圖例
        self.ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), borderaxespad=0.)

        # 調整布局以防止圖例遮擋內容
        self.fig.tight_layout(rect=[0, 0, 0.8, 1])

    def update_plot(self):
        """更新繪圖"""
        # 更新動態點位置
        for label, info in self.dynamic_points.items():
            coords = info['coords']
            if coords is not None:
                x, y = coords
                self.point_plots[label].set_data(x, y)
                self.point_plots[label].set_label(f"{label} ({x:.2f}, {y:.2f})")

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
