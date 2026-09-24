import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class Robot(Node):

    def __init__(self):
        super().__init__('practice_node')

        self.declare_parameter('topic', '/turtle2')
        self.declare_parameter('num', 0)

        self.topic = self.get_parameter('topic').value
        self.num = int(self.get_parameter('num').value)

        self.publisher = self.create_publisher(Twist, f'{self.topic}/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, f'{self.topic}/pose', self.odom_cb, 10)
        self.cli = self.create_client(SetPen, f'{self.topic}/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        self.req = SetPen.Request()


        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.x = 0
        self.y = 0
        self.z = 0

        self.init = 1

        self.start_x = 0
        self.start_y = 0
        self.start_z = 0

        self.target_dist = 0
        self.target_z = 0

        self.stateInd = 0

        self.pen_init = 0

        self.numbers = [
            [  # 0
                'start',
                'left',
                'forward',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'forward',
                'left',
                'forward',
                'stop'
            ],
            [  # 1
                'start',
                'left',
                'forward',
                'forward',
                'stop'
            ],
            [  # 2
                'start',
                'left',
                'left',
                'forward',
                'right',
                'forward',
                'right',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'stop'
            ],
            [  # 3
                'start',
                'penup',
                'backward',
                'pendown',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'right',
                'penup',
                'forward',
                'right',
                'pendown',
                'forward',
                'right',
                'forward',
                'stop'
            ],
            [  # 4
                'start',
                'left',
                'forward',
                'forward',
                'penup',
                'left',
                'forward',
                'left',
                'pendown',
                'forward',
                'left',
                'forward',
                'stop'
            ],
            [  # 5
                'start',
                'penup',
                'backward',
                'pendown',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'right',
                'forward',
                'right',
                'forward',
                'stop'
            ],
            [  # 6
                'start',
                'penup',
                'left',
                'forward',
                'forward',
                'left',
                'pendown',
                'forward',
                'left',
                'forward',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'stop'
            ],
            [  # 7
                'start',
                'left',
                'forward',
                'forward',
                'left',
                'forward',
                'stop'
            ],
            [  # 8
                'start',
                'left',
                'forward',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'penup',
                'backward',
                'right',
                'pendown',
                'forward',
                'left',
                'forward',
                'stop'
            ],
            [  # 9
                'start',
                'penup',
                'backward',
                'pendown',
                'pendown',
                'forward',
                'left',
                'forward',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'left',
                'forward',
                'stop'
            ]
        ]

    
    def odom_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.theta


    def pen_cb(self, resp):
        self.get_logger().info(f"pen set: {resp.result()}")
        self.pen_init = 1

    def setPen(self, off):
        if self.init:
            self.req.r = 255
            self.req.g = 255
            self.req.b = 255
            self.req.width = 3
            self.req.off = not off
            resp = self.cli.call_async(self.req)
            resp.add_done_callback(self.pen_cb)
            self.init = 0
        if self.pen_init:
            self.init = 1
            self.stateInd += 1
            self.pen_init = 0
            


    def normAngle(self, angle):
        if angle > math.pi: angle -= math.pi * 2
        if angle < -math.pi: angle += math.pi * 2
        return angle


    def turn(self, minus=1):
        minus = -minus
        if self.init:
            self.start_z = self.z
            self.target_z = self.normAngle(self.start_z + minus * math.pi / 2)
            self.init = 0

        # self.get_logger().info(f'{self.target_z}')
        msg = Twist()
        if abs(self.normAngle(self.target_z - self.z)) > 0.15:
            # self.get_logger().info(f'{abs(self.normAngle(self.target_z - self.z))}')
            msg.angular.z = minus * 0.8
            self.publisher.publish(msg)
        elif abs(self.normAngle(self.target_z - self.z)) > 0.01:
            # self.get_logger().info(f'{abs(self.normAngle(self.target_z - self.z))}')
            msg.angular.z = minus * 0.1
            self.publisher.publish(msg)
        else: 
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.init = 1
            self.stateInd += 1


    def move(self, minus=1):
        if self.init:
            self.start_x = self.x
            self.start_y = self.y
            self.target_dist = 3
            self.init = 0
        msg = Twist()
        if abs(self.target_dist - ((self.x - self.start_x) ** 2 + (self.y - self.start_y) ** 2) ** 0.5) > 0.1:
            #self.get_logger().info(f'{abs(self.target_dist - ((self.x - self.start_x) ** 2 + (self.y - self.start_y) ** 2) ** 0.5)}')
            msg.linear.x = minus * 1.0
            self.publisher.publish(msg)
        else:
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.init = 1
            self.stateInd += 1
        

    def stop(self):
        if self.init:
            self.get_logger().info("finish " + self.topic)
            self.init = 0
        if True:
            msg = Twist()
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            

    def timer_callback(self):
        if self.numbers[self.num][self.stateInd] == 'forward':
            self.move() 
        elif self.numbers[self.num][self.stateInd] == 'backward':
            self.move(-1) 
        elif self.numbers[self.num][self.stateInd] == 'left':
            self.turn(-1)
        elif self.numbers[self.num][self.stateInd] == 'right':
            self.turn(1)
        elif self.numbers[self.num][self.stateInd] == 'penup':
            self.setPen(0)
        elif self.numbers[self.num][self.stateInd] == 'pendown':
            self.setPen(1)
        elif self.numbers[self.num][self.stateInd] == 'start':
            self.setPen(1)
        elif self.numbers[self.num][self.stateInd] == 'stop':
            self.stop()
            


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    
    try:
        rclpy.spin(robot)
    except (KeyboardInterrupt):
        pass
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
