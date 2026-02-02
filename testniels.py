#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def move_forward():
    # 1. Initialiser din node (fortæl ROS at dette program kører)
    rospy.init_node('simple_mover', anonymous=True)

    # 2. Opret en Publisher
    # Vi sender til '/cmd_vel', og beskeden er af typen 'Twist'
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Hvor ofte skal løkken køre (10 gange i sekundet)
    rate = rospy.Rate(10) 

    # 3. Definer beskeden (hastigheden)
    vel_msg = Twist()
    
    # Sæt hastighed:
    # linear.x er frem/tilbage (meter per sekund)
    # angular.z er rotation (radianer per sekund)
    vel_msg.linear.x = 0.1   # Kør langsomt frem (0.1 m/s)
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0  # Ingen drejning

    rospy.loginfo("Robotten kører fremad...")

    # 4. Kør i en løkke i et bestemt tidsrum (f.eks. 2 sekunder)
    start_time = time.time()
    while time.time() - start_time < 2.0:
        # Tjek om ROS stadig kører (hvis man trykker Ctrl+C)
        if rospy.is_shutdown():
            break
            
        # Send kommandoen til robotten
        velocity_publisher.publish(vel_msg)
        rate.sleep()

    # 5. VIGTIGT: Stop robotten
    # Hvis vi bare lukker scriptet, kan robotten nogle gange fortsætte lidt.
    # Vi sender en 'stop' besked (0.0 hastighed).
    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("Robotten er stoppet.")

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass