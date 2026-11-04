import rclpy
import math
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
import tf2_ros
from tf_transformations import euler_from_quaternion
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class Robot(Node):
    def __init__(self):
        super().__init__('practice03_node')
        self.declare_parameter('duration', 90.0)
        self.duration = self.get_parameter('duration').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.sub2 = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.gt_data = []
        self.odom_data = []
        self.cmd_data = []
        self.cmd_data_eval = []

        self.start = False
        self.finish = False
        self.t0 = None
        self.last_clock = None

        self.last_cmd = (0.0, 0.0)
        self.cmd_pose = None
        self.prev = None

    def clock_cb(self, msg):
        self.last_clock = msg.clock.sec + msg.clock.nanosec / 10**9

    def cmd_cb(self, msg):
        if self.finish:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 10**9
        v = msg.twist.linear.x
        w = msg.twist.angular.z
        self.cmd_data.append((t, v, w))
        self.last_cmd = (v, w)

    def pose(self, target, source):
        try:
            tr = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        a, b, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (t.x, t.y, yaw)

    def timer_cb(self):
        if self.finish:
            return
        t_now = self.last_clock if self.last_clock is not None else 0.0

        if not self.start:
            if not self.cmd_data:
                return
            gt = self.pose('map', 'base_link_gt')
            odom = self.pose('map', 'base_link')
            if gt is None or odom is None:
                return
            self.t0 = t_now
            self.cmd_pose = list(gt)
            self.prev = t_now
            self.start = True
            self.get_logger().info(f'Start t0={self.t0:.3f}, gt={gt}')
            return
        
        allTime = t_now - self.t0
        if allTime >= self.duration:
            self.finish = True
            self.get_logger().info(f'Finish, allTime={t_now - self.t0:.1f}s')
            self.save_graphic()
            rclpy.shutdown()
            return

        gt = self.pose('map', 'base_link_gt')
        odom = self.pose('map', 'base_link')
        if gt is None or odom is None:
            return

        dt = t_now - self.prev
        self.prev = t_now
        v, w = self.last_cmd
        x, y, yaw = self.cmd_pose
        if abs(w) < 10**(-6):
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
        else:
            x += (v / w) * (math.sin(yaw + w * dt) - math.sin(yaw))
            y += (v / w) * (-math.cos(yaw + w * dt) + math.cos(yaw))
            yaw += w * dt
        self.cmd_pose = [x, y, yaw]

        self.gt_data.append((allTime, *gt))
        self.odom_data.append((allTime, *odom))
        self.cmd_data_eval.append((allTime, x, y, yaw))

    def norm(self, x):
        return (x + np.pi) % (2 * np.pi) - np.pi

    def save_graphic(self):
        if not self.gt_data:
            return
        
        gt = np.array(self.gt_data)
        od = np.array(self.odom_data)
        cm = np.array(self.cmd_data_eval)

        err_od = np.sqrt((od[:, 1] - gt[:, 1])**2 + (od[:, 2] - gt[:, 2])**2)
        err_cm = np.sqrt((cm[:, 1] - gt[:, 1])**2 + (cm[:, 2] - gt[:, 2])**2)
        ang_od = np.abs(self.norm(od[:, 3] - gt[:, 3]))
        ang_cm = np.abs(self.norm(cm[:, 3] - gt[:, 3]))

        fig, ax = plt.subplots(1, 3, figsize=(18, 5))

        ax[0].plot(gt[:, 1], gt[:, 2], label='GT')
        ax[0].plot(od[:, 1], od[:, 2], label='Encoders')
        ax[0].plot(cm[:, 1], cm[:, 2], label='Cmd')
        ax[0].set_aspect('equal')
        ax[0].set_xlabel('x, m'); ax[0].set_ylabel('y, m')
        ax[0].legend(); ax[0].grid(True)
        ax[0].set_title('Trajectories')

        ax[1].plot(gt[:, 0], err_od, label='Encoders')
        ax[1].plot(gt[:, 0], err_cm, label='Cmd')
        ax[1].set_xlabel('t, s'); ax[1].set_ylabel('position error, m')
        ax[1].legend(); ax[1].grid(True)
        ax[1].set_title('Position error')

        ax[2].plot(gt[:, 0], ang_od, label='Encoders')
        ax[2].plot(gt[:, 0], ang_cm, label='Cmd')
        ax[2].set_xlabel('t, s'); ax[2].set_ylabel('yaw error, rad')
        ax[2].legend(); ax[2].grid(True)
        ax[2].set_title('Yaw error')

        fig.tight_layout()
        fig.savefig('noise_01.png', dpi=120)
        self.get_logger().info('Saved: noise_0.png')


def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
