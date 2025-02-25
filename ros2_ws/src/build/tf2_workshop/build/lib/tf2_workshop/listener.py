#!/usr/bin/env python3


# commands to run:
# ros2 run turtlesim turtlesim_node
# python3 tf2_workshop/tf2_workshop/broadcaster.py turtle1 
# ros2 run turtlesim turtle_teleop_key
# ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: "turtle2"}"
# ros2 run tf2_ros tf2_echo turtle2 world
# python3 tf2_workshop/tf2_workshop/listener.py turtle1 turtle2
# 



import sys
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

class TfListener(Node):

    def __init__(self, first_turtle, second_turtle):
        super().__init__('tf_listener')
        self.first_name_ = first_turtle
        self.second_name_ = second_turtle
        self.get_logger().info("Transforming from {} to {}".format(self.second_name_, self.first_name_))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.cmd_ = Twist ()
        self.publisher_ = self.create_publisher(Twist, "{}/cmd_vel".format(self.second_name_),10)
        self.timer = self.create_timer(0.33, self.timer_callback) #30 Hz = 0.333s

    def timer_callback(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.second_name_, self.first_name_, rclpy.time.Time())
            self.cmd_.linear.x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            self.cmd_.angular.z = 4 * math.atan2(trans.transform.translation.y , trans.transform.translation.x)
            self.publisher_.publish(self.cmd_)

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

def main(argv=None):
    rclpy.init(args=argv)
    node = TfListener(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
