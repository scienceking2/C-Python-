import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time


class CmdVisualizer(Node):
    def __init__(self):
        super().__init__('cmd_visualizer')

        self.subscription = self.create_subscription(
            Twist,
            '/smooth_cmd',
            self.listener_callback,
            10
        )

        # 최근 N개만 저장해서 실시간 그래프처럼 보이게
        self.window_size = 300
        self.times = deque(maxlen=self.window_size)
        self.linear_x = deque(maxlen=self.window_size)
        self.angular_z = deque(maxlen=self.window_size)

        self.start_time = time.time()

    def listener_callback(self, msg: Twist):
        t = time.time() - self.start_time
        self.times.append(t)
        self.linear_x.append(msg.linear.x)
        self.angular_z.append(msg.angular.z)


def main():
    rclpy.init()

    node = CmdVisualizer()

    # matplotlib 초기 플롯 설정
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    line1, = ax1.plot([], [], label='linear.x')
    line2, = ax2.plot([], [], label='angular.z')

    ax1.set_ylabel('linear.x [m/s]')
    ax2.set_ylabel('angular.z [rad/s]')
    ax2.set_xlabel('time [s]')

    ax1.legend()
    ax2.legend()

    # 애니메이션 업데이트 함수
    def update(frame):
        rclpy.spin_once(node, timeout_sec=0.01)

        if len(node.times) == 0:
            return line1, line2

        line1.set_data(node.times, node.linear_x)
        line2.set_data(node.times, node.angular_z)

        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()

        return line1, line2

    ani = animation.FuncAnimation(fig, update, interval=20, blit=False)

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
