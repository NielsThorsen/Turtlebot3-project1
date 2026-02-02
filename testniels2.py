#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver')
        
        # 1. Vi skal kunne sende kommandoer til hjulene
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 2. Vi skal lytte til laseren
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
            
        self.get_logger().info('AutoDriver startet! Robotten kører rundt for evigt...')

    def scan_callback(self, msg):
        # Vi kigger på afstanden lige foran robotten (0 grader)
        # For at gøre det mere robust, kan vi tage det laveste tal fra en lille vifte foran
        # Men her holder vi det simpelt og kigger på index 0 (lige frem)
        afstand_foran = msg.ranges[0]
        
        cmd = Twist()

        # LOGIKKEN:
        # Hvis afstanden er over 0.0 (gyldig måling) OG mindre end 0.4 meter (40 cm)
        if 0.0 < afstand_foran < 0.40:
            # OBSTACLE DETECTED!
            self.get_logger().info(f'Væg forude ({afstand_foran:.2f}m)! Drejer...')
            
            cmd.linear.x = 0.0      # Stop med at køre frem
            cmd.angular.z = 0.5     # Drej til venstre (0.5 radianer/sekund)
            
        else:
            # FRI BANE
            # self.get_logger().info('Fri bane - kører frem')
            cmd.linear.x = 0.15     # Kør fremad (lidt hurtigere end før)
            cmd.angular.z = 0.0     # Ingen drejning

        # Send beslutningen til hjulene
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop pænt hvis man trykker Ctrl+C
        pass
    finally:
        # Send stop-signal inden vi lukker
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()