#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        # Opret publisher til /cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def stop_robot(self):
        # Stop robotten ved at sende 0 i hastighed
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Robot stoppet!')

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.1  # Kør fremad med 0.1 m/s
        msg.angular.z = 0.0

        self.get_logger().info('Kører fremad...')
        
        # Vi sender beskeden flere gange i en lille løkke for at sikre den kører
        # (I en rigtig robot-applikation ville man bruge en timer)
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.publisher_.publish(msg)
            time.sleep(0.1)  # Vent 0.1 sekund mellem hver besked

        self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    
    mover = SimpleMover()
    mover.move_forward()
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
