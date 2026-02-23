#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.get_logger().info('DEBUG driver startet - venter på laser data...')

    def scan_callback(self, msg):
        # Tjek at vi har scan-data
        if not msg.ranges:
            return

        # Brug midten af ranges som "foran" (sikrere end index 0)
        idx = len(msg.ranges) // 2
        afstand = msg.ranges[idx]
        # Afstand 45 grader til venstre og højre for midten. TO do tjek om de skal byttes om
        idx_højre = int(idx + math.radians(45)/msg.angle_increment)
        idx_venstre = int(idx - math.radians(45)/msg.angle_increment)
        afstand_venstre = msg.ranges[idx_højre]
        afstand_højre = msg.ranges[idx_venstre]

        # --- DEBUG PRINT ---
        print(f"Laser[{idx}] måling: {afstand}")
        print(f"LaserV[{idx_venstre}] måling: {afstand_venstre}")
        print(f"LaserH[{idx_højre}] måling: {afstand_højre}")
        # -------------------

        cmd = Twist()

        # Håndter 0.0, inf og NaN robust
        if not math.isfinite(afstand) or afstand == 0.0:
            afstand = float('inf')

        if 0.01 < afstand < 0.40:
            self.get_logger().info(f'Væg tæt på ({afstand:.2f}m)! Drejer...')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist()) # Stop robotten når vi lukker
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()