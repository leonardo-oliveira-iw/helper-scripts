#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from my_msgs.msg import EventStatus  # Replace with your actual package and message
from builtin_interfaces.msg import Time
import fire
import time

# Dictionaries to map string inputs to the corresponding enum values
SEVERITY_MAP = {
    'DEBUG': EventStatus.DEBUG,
    'INFO': EventStatus.INFO,
    'WARNING': EventStatus.WARNING,
    'ERROR': EventStatus.ERROR,
    'FATAL': EventStatus.FATAL
}

STATUS_MAP = {
    'OCCURRED': EventStatus.OCCURRED,
    'OCCURRED_TEMP': EventStatus.OCCURRED_TEMP,
    'CLEARED': EventStatus.CLEARED
}

class EventPublisher(Node):
    def __init__(self, code: str, severity: str, description: str, status: str, topic_name: str = 'event_status'):
        super().__init__('event_publisher')

        # Convert the provided severity and status strings to their respective enum values
        severity_value = SEVERITY_MAP.get(severity.lower())
        if severity_value is None:
            self.get_logger().error(f"Invalid severity '{severity}'. Must be one of {list(SEVERITY_MAP.keys())}")
            rclpy.shutdown()
            return

        status_value = STATUS_MAP.get(status.lower())
        if status_value is None:
            self.get_logger().error(f"Invalid status '{status}'. Must be one of {list(STATUS_MAP.keys())}")
            rclpy.shutdown()
            return

        # Create a QoS profile with transient local durability
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Create publisher with the transient local QoS profile
        self.publisher_ = self.create_publisher(EventStatus, topic_name, qos_profile)
        
        # Wait a brief moment for connections (optional)
        time.sleep(0.5)
        
        # Create and populate the message
        msg = EventStatus()
        
        # Set the current time as stamp
        now = self.get_clock().now()
        msg.timestamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        
        msg.code = code
        msg.description = description
        msg.extra_info = ""
        msg.severity = severity_value
        msg.status = status_value
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published event: code={code}, severity={severity} ({severity_value}), "
            f"description={description}, status={status} ({status_value})"
        )

        # Shutdown after publishing once
        rclpy.shutdown()

def main(code: str, severity: str, status: str, topic_name: str = 'event_status', description=""):
    rclpy.init()
    node = EventPublisher(code, severity, description, status, topic_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    # Using Fire to expose the `main` function as CLI
    fire.Fire(main)

