#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

# from std_msgs.msg import Int32
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped


class ros2_pc_node(Node):
    def __init__(self):
        super().__init__("ros2_pc_node")
        # debug variables
        x = 100
        y = 200

        self.create_timer(1.0, self.timer_callback)
        self.ros2_pc_node_pub = self.create_publisher(Twist, "pc_side_pub", 10)
        self.ros2_pc_node_lis = self.create_subscription(
            Vector3Stamped, "micro_ros_node_pub", self.pose_callback, 10
        )

    def pose_callback(self, msg: Vector3Stamped):
        string = (
            "time stamp: "
            + str(msg.header.stamp)
            + "x: "
            + str(msg.vector.x)
            + "y: "
            + str(msg.vector.y)
            + "z:"
            + str(msg.vector.z)
        )
        self.get_logger().info(string)
        """ self.get_logger().info(str(msg.header.stamp))
        self.get_logger().info(str(msg.vector.x))
        self.get_logger().info(str(msg.vector.y))
        self.get_logger().info(str(msg.vector.z)) """

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = y
        self.ros2_pc_node_pub.publish(msg)
        x += 1
        y += 1


def main(args=None):
    rclpy.init(args=args)
    node = ros2_pc_node()
    rclpy.spin(node)
    rclpy.shutdown()
