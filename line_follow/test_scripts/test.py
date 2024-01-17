import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from RobotHandler import RobotHandler


class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('twist_subscriber')
        self.robothandler_ = RobotHandler()
        self.robot_ip = "192.168.192.5"              #"192.168.0.40"
        self.control_port = 19205
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.robothandler_.connect_robot(self.robot_ip, self.control_port)
    
    def __del__(self):
        self.robothandler_.disconnect_robot()
        # print(f"{self.name} is about to be destroyed")

    def listener_callback(self, msg):
        self.robothandler_.cmd_vel_pub_api(self.robot_ip, self.control_port, msg.linear.x, msg.angular.z)
        self.get_logger().info('I heard: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = TwistSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
