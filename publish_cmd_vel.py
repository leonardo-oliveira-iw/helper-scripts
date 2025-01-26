#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(
        self,
        frame_id: str = "base_link",
        timestep: float = 0.01,
        topic_name: str = "/robot_interface/cmd_vel",
    ):
        super().__init__("cmd_vel_publisher")
        self.frame_id = frame_id
        self.timestep = timestep
        self.t = 0.0

        # Optional QoS settings:
        qos_profile = QoSProfile(depth=10)
        # If you need transient local so that late joiners can get the last message:
        # qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Create the publisher
        self.publisher = self.create_publisher(Twist, topic_name, qos_profile)

        # Create a timer that calls `timer_callback` every `timestep` seconds
        self.timer = self.create_timer(self.timestep * 10.0, self.timer_callback)

    def timer_callback(self):
        """
        This function is called every 'timestep' seconds to publish a new point cloud.
        """
        stamp = self.get_clock().now().to_msg()
        msg = self._get_cmd_vel(self.t)

        print(f"Publishing linear x: {msg.linear.x} == angular z: {msg.angular.z}")
        self.publisher.publish(msg)

        # Increment time for the next publish
        self.t += self.timestep

    def _get_cmd_vel(self, t: float):
        msg = Twist()
        msg.angular.z = 0.1
        msg.linear.x = 0.2 + 0.15 * t
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelPublisher(timestep=0.01)

    try:
        # This will process timer callbacks and keep the node alive
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
