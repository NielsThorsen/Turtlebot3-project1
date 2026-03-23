#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time
import random

class AutoDriver(Node):
    # Counters
    collisions = 0
    linear_speeds = []

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
        

        # Distance 90 degrees left and right
        idx_right90 = int(idx + math.radians(45)/msg.angle_increment)
        idx_left90 = int(idx - math.radians(45)/msg.angle_increment)
        distance_left90 = msg.ranges[idx_left90]
        distance_right90 = msg.ranges[idx_right90]

        # --- DEBUG PRINT ---
        print(f"Laser[{idx}] måling: {afstand}")
        print(f"LaserV[{idx_venstre}] måling: {afstand_venstre}")
        print(f"LaserH[{idx_højre}] måling: {afstand_højre}")
        # -------------------

        cmd = Twist()

        # Håndter 0.0, inf og NaN
        if not math.isfinite(afstand) or afstand == 0.0:
            afstand = float('inf')

        # Check if collided and tick counter
        if afstand <= 0.20 or distance_left90 <= 0.20 or distance_right90 <= 0.20:
            self.collisions += 1
            print(f"Collisions detected {self.collisions}")

        # Velocity dependent on distances
        if self.NaNPercentage(msg) > 10:
            if self.NaNPercentage(msg) > 12:
                cmd.linear.x = 0.1
                cmd.angular.z = random.uniform(0.5,-0.5)
                print(f"WUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU {self.NaNPercentage(msg)}")
        elif 0.01 < afstand_venstre < 0.25:
            print(f'Væg tæt på ({afstand:.2f}m)! Drejer...')
            cmd.linear.x = -0.05
            cmd.angular.z = -0.3
        elif 0.01 < afstand_højre < 0.25:
            print(f'Væg tæt på ({afstand:.2f}m)! Drejer...')
            cmd.linear.x = -0.05
            cmd.angular.z = 0.3
        else:
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)
        self.linear_speeds.append(cmd.linear.x)
        print(f"Average Speed: {sum(self.linear_speeds)/len(self.linear_speeds) * -1}")

    # Find Percentage of NaN indexes between +45 and -45 degress.
    def NaNPercentage(self,msg):
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges_np = np.array(msg.ranges, dtype=float)

        # Back sector: 180° ± 30°  -> total 60°
        # Use wrapped angle difference so it works at the -pi/pi boundary.
        back_center = math.pi
        angle_diff = np.arctan2(np.sin(angles - back_center), np.cos(angles - back_center))
        sector_mask = np.abs(angle_diff) <= math.radians(30)

        sector_ranges = ranges_np[sector_mask]

        if sector_ranges.size == 0:
            nan_percent = 0.0
        else:
            nan_percent = np.isnan(sector_ranges).mean() * 100.0
        
        return nan_percent


def main(args=None):
    rclpy.init(args=args)
    node = AutoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print Counters
        print(f"Average Speed: {sum(node.linear_speeds)/len(node.linear_speeds) * -1}")
        print(f"Collisions: {node.collisions}")
        node.publisher_.publish(Twist()) # Stop robotten når vi lukker
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()