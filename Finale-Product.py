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
import asyncio
import threading
import smbus
from gpiozero import LED
from time import sleep

async def rgb(node):
    try:
        bus = smbus.SMBus(1)
        # ISL29125 address, 0x44(68)
        bus.write_byte_data(0x44, 0x01, 0x05)
        node.get_logger().info('Color sensor initialized.')
    except Exception as e:
        node.get_logger().error(f"I2C Connection error: {e}")
        return

    await asyncio.sleep(1)
    led = LED(14)
    last_red_time = 0.0  # Prevents double-counting the same color spot
    cooldown = 3.0       # Wait 3 seconds between each count

    while True:
        try:
            data = bus.read_i2c_block_data(0x44, 0x09, 6)

            green = data[1] * 256 + data[0]
            red   = data[3] * 256 + data[2]
            blue  = data[5] * 256 + data[4]

            r_8 = red >> 8
            g_8 = green >> 8
            b_8 = blue >> 8

            current_time = time.time()
            if r_8 > 70 and r_8 > b_8 and g_8 > 40 and g_8 < 70 and b_8 < 40:
                if (current_time - last_red_time) > cooldown:
                    node.red_count += 1
                    last_red_time = current_time
                    color_block = f"\x1b[48;2;{r_8};{g_8};{b_8}m      \x1b[0m"
                    node.get_logger().info(f"Red detected! Total: {node.red_count} | Color: {color_block} | RGB: ({r_8}, {g_8}, {b_8})")
                    led.on()
                    sleep(1)
                    led.off()

        except Exception as e:
            pass # Ignore minor I2C errors
        
        await asyncio.sleep(0.1) # Read fast (10 Hz) so we don't miss the color while moving


def start_async_loop(loop, node):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(rgb(node))


class AutoDriver(Node):

    def __init__(self):
        super().__init__('auto_driver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        
        self.collisions = 0
        self.linear_speeds = []
        self.red_count = 0

        self.recovery_mode = False
        self.recovery_enter_threshold = 50
        self.recovery_exit_threshold = 30

        # =====================================================================
        # TUNING SECTION - FINE-TUNE ROBOT BEHAVIOR HERE
        # =====================================================================
        self.DIST_DANGER  = 0.18  # If closer than this -> REVERSE and SPIN!
        self.DIST_WARNING = 0.20  # Increased slightly to allow more time for turning
        self.DIST_SHOULDER = 0.18 # How close can boxes get to the robot's sides during turns?
        self.DIST_VSHAPE = 0.20 # When it hits a v-shape

        # Speeds (Negative X = Forward)
        self.SPEED_FORWARD = -0.17 
        self.SPEED_SLOW    = -0.07
        self.SPEED_REVERSE =  0.5 # Reverse speed
        
        # Turn speeds (Rad/s)
        self.TURN_MILD       = 0.4 
        self.TURN_AGGRESSIVE = 0.9 
        self.TURN_EXTREME    = 1.5
        # =====================================================================

        self.get_logger().info('DEBUG driver started - Running Aggressive FOV Obstacle Avoidance...')
        # function to calculate turn speed depending on distances
    def var_turning(self, dist_right, dist_left):
        if (math.isfinite(dist_left) and math.isfinite(dist_right)):
            if dist_left < dist_right:
                if (-1 * (dist_left / dist_right) < -3):
                    return -1 * (dist_right / dist_left)
                else:
                    return -3
            elif dist_right < dist_left:
                if ((dist_left / dist_right) < 3):
                    return (dist_left / dist_right)
                else:
                    return 3
            else:
                return 0.0
        else:
            return 0.0
        
    def get_min_dist(self, slice_arr):
        valid = [x for x in slice_arr if math.isfinite(x) and x > 0.02]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        if not msg.ranges:
            return

        idx = len(msg.ranges) // 2 # Center (Front)
        
        # Expand FOV (Field of View) to cover more
        offset_20  = int(math.radians(20) / msg.angle_increment)
        offset_75  = int(math.radians(75) / msg.angle_increment)
        offset_110 = int(math.radians(110) / msg.angle_increment) # Shoulder check
        offset_180 = int(math.radians(110) / msg.angle_increment) # Look backwards

        # Get ZONES (Wide slices instead of single rays)
        front_slice = msg.ranges[idx - offset_20 : idx + offset_20]
        left_slice  = msg.ranges[idx + offset_20 : idx + offset_75]
        right_slice = msg.ranges[idx - offset_75 : idx - offset_20]
        back_slice = msg.ranges[idx - offset_180: idx - offset_180]
        
        # THE NEW "SHOULDER" ZONES (Check sides to avoid clipping corners)
        shoulder_left_slice  = msg.ranges[idx + offset_75 : idx + offset_110]
        shoulder_right_slice = msg.ranges[idx - offset_110 : idx - offset_75]

        # Get the minimum distance in each zone
        dist_front = self.get_min_dist(front_slice)
        dist_left  = self.get_min_dist(left_slice)
        dist_right = self.get_min_dist(right_slice)
        dist_shoulder_l = self.get_min_dist(shoulder_left_slice)
        dist_shoulder_r = self.get_min_dist(shoulder_right_slice)
        dist_back = self.get_min_dist(back_slice)

        cmd = Twist()

        # Count collisions (Now also checking shoulders)
        if dist_front <= 0.15 or dist_shoulder_l <= 0.12 or dist_shoulder_r <= 0.12:
            self.collisions += 1

        nan_pct = self.NaNPercentage(msg)

        # --- RECOVERY MODE (Glass tests / Black holes) ---
        if nan_pct > self.recovery_enter_threshold:
            self.recovery_mode = True

        if self.recovery_mode:
            if nan_pct > self.recovery_exit_threshold:
                cmd.linear.x = self.SPEED_REVERSE
                cmd.angular.z = random.uniform(-self.TURN_EXTREME, self.TURN_EXTREME)
                self.publisher_.publish(cmd)
                print("Recovery mode")
                return 
            else:
                self.recovery_mode = False

        # --- CONTROL LOGIC ---

        # 0. THE V-SHAPE TRAP (Dead end / Corner)
        # If the front AND both sides are blocked, we are in a corner. We need to reverse and turn hard to escape.
        if dist_front < self.DIST_VSHAPE and dist_left < self.DIST_VSHAPE and dist_right < self.DIST_VSHAPE:
            self.DIST_VSHAPE = 0.25
            # cmd.linear.x = self.SPEED_REVERSE
            print("V-Shape")
            cmd.angular.z = self.TURN_EXTREME # Choose one fixed direction to break out of the V-shape
            
        # 2. WARNING ZONE: Prepare to dodge an obstacle further ahead
        elif dist_front < self.DIST_WARNING and (dist_left < self.DIST_WARNING or dist_right < self.DIST_WARNING):
            cmd.linear.x = self.SPEED_SLOW # Slow down
            print("WARNING ZONE")
            if dist_left < dist_right:
                cmd.angular.z = -self.TURN_AGGRESSIVE # Turn right
            else:
                cmd.angular.z = self.TURN_AGGRESSIVE  # Turn left

        # 3. CLEAR LINES: Forward (with slight centering)
        else:
            print("Clear Lines")
            self.DIST_VSHAPE = 0.20
            cmd.linear.x = self.SPEED_FORWARD
            
            if dist_left < 0.4 or dist_right < 0.4:
                    cmd.angular.z = self.var_turning(dist_right, dist_left) * self.TURN_MILD 
                    print(self.var_turning(dist_right, dist_left))
            else:
                cmd.angular.z = 0.0

        self.publisher_.publish(cmd)
        self.linear_speeds.append(cmd.linear.x)

    def NaNPercentage(self,msg):
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges_np = np.array(msg.ranges, dtype=float)

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
    
    loop = asyncio.new_event_loop()
    t = threading.Thread(target=start_async_loop, args=(loop, node), daemon=True)
    t.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.linear_speeds:
            print(f"Average Speed: {sum(node.linear_speeds)/len(node.linear_speeds) * -1:.3f}")
        print(f"Collisions: {node.collisions}")
        print(f"Red colors found along the way: {node.red_count}")
        
        node.publisher_.publish(Twist()) # Stop the robot
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()