import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from synapse_msgs.msg import PolynomialTrajectory
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
# ros2 run joy joy_node
# ros2 run ros_gz_bridge parameter_bridge /poly@synapse_msgs/msg/PolynomialTrajectory@gz.msgs.PolynomialTrajectory

class PolyTraj(Node):

    def __init__(self):
        super().__init__('poly_traj_node')
        self.counter = 0
        self.joyTopic = "/joy"
        self.joySub = self.create_subscription(Joy, '{:s}'.format(self.joyTopic), 
                                               self.joyCallback, qos_profile_sensor_data)
        self.Pub = self.create_publisher(PolynomialTrajectory,'{:s}'.format("/poly"), 0)

    
    def joyCallback(self, msgJoy):
        
        if msgJoy.axes[4] > 0.5:
            msg=PolynomialTrajectory()
            msg.is_bezier = False
            msg.poly_order = 5
            msg.recompute = False
            msg.sequence = self.counter
            msg.time_start = 1.0*self.counter
            msg.time_end = 1.0*(self.counter+1)
            msg.x = [0.0,1.0,2.0,3.0,4.0,5.0]
            msg.y = [6.0,7.0,8.0,9.0,0.0,1.0]
            msg.yaw = [0.0,0.0,0.0,0.0,0.0,0.0]
            self.Pub.publish(msg)
            self.counter +=1
        
        return 
            
if __name__ == '__main__':
    rclpy.init()
    PT = PolyTraj()
    rclpy.spin(PT)
    PT.destroy_node()
    rclpy.shutdown()
