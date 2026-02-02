#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

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
        # Hent afstanden lige foran
        afstand = msg.ranges[0]
        
        # --- DEBUG PRINT ---
        # Dette vil fortælle os om koden overhovedet kører!
        print(f"Laser måling: {afstand}") 
        # -------------------

        cmd = Twist()

        # Hvis afstanden er 0.0, betyder det ofte "fejllæsning" eller "uden for rækkevidde"
        # Vi behandler 0.0 som "fri bane" for at undgå den stopper ved fejl
        if afstand == 0.0:
            afstand = 999.9

        if 0.01 < afstand < 0.40:
            self.get_logger().info(f'Væg tæt på ({afstand:.2f}m)! Drejer...')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            # Hvis vi ikke printer her, ved vi ikke om den prøver at køre
            # print("Kører frem...") 
            cmd.linear.x = 0.15
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