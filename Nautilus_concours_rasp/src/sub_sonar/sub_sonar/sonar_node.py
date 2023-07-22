import rclpy
from rclpy.node import Node

from sub_sonar.submodules.Sonar_ROS import Sonar

from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class SonarMain(Node):

    def __init__(self):
        super().__init__('sonar_main')

        self.pos_init_pub_ = self.create_publisher(Vector3, '/SousMarin/Sonar/Pos_Init', 10)
        self.pos_cible_pub_ = self.create_publisher(Vector3, '/SousMarin/Sonar/Pos_Cible', 10)
        self.angle_pub_ = self.create_publisher(Float64, '/SousMarin/Sonar/Angle', 10)

        self.requete_sub = self.create_subscription(Int32, '/SousMarin/Sonar/Requete', self.scan_callback,10)

    def scan_callback(self, msg):

        self.get_logger().info('Scan commence')

        req = msg
        posInit,posCible,Angle = Sonar("Center")

        #Angle de -pi à pi 
        pos_init = Vector3()
        pos_cible = Vector3()
        diff_angle = Float64()

        pos_init.x =posInit[0]
        pos_init.y = posInit[1]

        pos_cible.x = posCible[0]
        pos_cible.y = posCible[1]

        diff_angle.data = Angle

        self.get_logger().info('Scan termine')

        self.pos_init_pub_.publish(pos_init)
        self.pos_cible_pub_.publish(pos_cible)
        self.pos_cible_pub_.publish(diff_angle)

def main(args=None):
    rclpy.init(args=args)

    sonar_main = SonarMain()

    rclpy.spin(sonar_main)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sonar_main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
