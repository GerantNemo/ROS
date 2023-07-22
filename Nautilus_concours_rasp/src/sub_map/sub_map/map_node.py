import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from std_msgs import Float32MultiArray


class Map_Node(Node):

    def __init__(self):

        super().__init__('map_node')

        self.depth = 0
        self.obstacles = []
        self.points = []
        
        #Subscribers
        self.obstacles_sub_ = self.create_subscription(Pose,"/SousMarin/Sonar/Obstacles",self.obstacles_callback,10)
        self.points_sub_ = self.create_subscription(Pose,"/SousMarin/Sonar/Points",self.enregistrement,10)
        self.depth_sub_ = self.create_subscription(Float64,"/SousMarin/measured/Depth",self.depth_callback,10)
        
        #Publishers
        self.publisher_ = self.create_publisher(Pose, "/SousMarin/Pose", 10)
        

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0

    def timer_callback(self):

        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
    def enregistrement(self, msg):
        self.points = msg

        


    def depth_callback(self, msg):
        self.depth = msg
    
    def obstacles_callback(self, msg):
        self.obstacles = msg


def main(args=None):
    rclpy.init(args=args)

    ekf = Map_Node()

    rclpy.spin(ekf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()