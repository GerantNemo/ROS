import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class Etat(Node):

    def __init__(self):
        super().__init__('etat')

        self.pos = [0, 0, 0]
        self.vit = [0, 0, 0]
        self.acc = [0, 0, 0]

        self.angles = [0, 0, 0]
        self.vit_ang = [0, 0, 0]

        self.depth_sub = self.create_subscription(String,"/SousMarin/measured/Depth",self.depth_callback,10)

        self.odom_pub_ = self.create_publisher(Odometry, "/model/SousMarin/odometry", 10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        self.get_logger().info('Emission etat')
        
        odom_msg = Odometry()

        odom_msg.pose.pose.position.x = float(self.pos[0])
        odom_msg.pose.pose.position.y = float(self.pos[1])
        odom_msg.pose.pose.position.z = float(self.pos[2])

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 0.0

        odom_msg.twist.twist.linear.x = float(self.vit[0])
        odom_msg.twist.twist.linear.y = float(self.vit[1])
        odom_msg.twist.twist.linear.z = float(self.vit[2])

        odom_msg.twist.twist.angular.x = float(self.vit_ang[0])
        odom_msg.twist.twist.angular.y = float(self.vit_ang[1])
        odom_msg.twist.twist.angular.z = float(self.vit_ang[2])

        self.odom_pub_.publish(odom_msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def depth_callback(self, msg_depth):

        self.pos[2] = msg_depth.data

def main(args=None):
    rclpy.init(args=args)

    etat = Etat()

    rclpy.spin(etat)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    etat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()