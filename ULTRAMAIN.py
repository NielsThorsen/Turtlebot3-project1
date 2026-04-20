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

async def rgb(node):
    try:
        bus = smbus.SMBus(1)
        # ISL29125 address, 0x44(68)
        bus.write_byte_data(0x44, 0x01, 0x05)
        node.get_logger().info('Farvesensor initialiseret.')
    except Exception as e:
        node.get_logger().error(f"I2C Forbindelsesfejl: {e}")
        return

    await asyncio.sleep(1)

    last_red_time = 0.0  # Forhindrer "dobbelt-tælling" af samme farveklat
    cooldown = 3.0       # Vent 3 sekunder mellem hver tælling

    while True:
        try:
            data = bus.read_i2c_block_data(0x44, 0x09, 6)

            green = data[1] * 256 + data[0]
            red   = data[3] * 256 + data[2]
            blue  = data[5] * 256 + data[4]

            r_8 = red >> 8
            g_8 = green >> 8
            b_8 = blue >> 8

            if (30 <= r_8 <= 55) and (15 <= g_8 <= 45) and (5 <= b_8 <= 25):
                current_time = time.time()
                if (current_time - last_red_time) > cooldown:
                    node.red_count += 1
                    last_red_time = current_time
                    color_block = f"\x1b[48;2;{r_8};{g_8};{b_8}m      \x1b[0m"
                    node.get_logger().info(f"Rød registreret! Total: {node.red_count} | Farve: {color_block}")

        except Exception as e:
            pass # Ignorerer små I2C fejl
        
        await asyncio.sleep(0.1) # Læs hurtigt (10 Hz) så vi ikke misser farven i fart


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
        # ⚙️ TUNING SEKTION - FINJUSTER ROBOTTENS ADFÆRD HER ⚙️
        # =====================================================================
        # Distancer (i meter)
        self.DIST_DANGER  = 0.20  # Hvis tættere på end dette -> BAK og SPIND!
        self.DIST_WARNING = 0.38  # Hvis tættere på end dette -> Brems ned og sving hårdt
        
        # Hastigheder (Husk: Negativ X = Fremad på din robot)
        self.SPEED_FORWARD = -0.15 # Normal march-hastighed
        self.SPEED_SLOW    = -0.05 # Kravle-hastighed når vi svinger om hjørner
        self.SPEED_REVERSE =  0.08 # Bak-hastighed for at rive sig fri
        
        # Drejehastigheder (Rad/s)
        self.TURN_MILD       = 0.5 # Let justering
        self.TURN_AGGRESSIVE = 1.3 # Skarpt sving for at undgå at strejfe bokse
        self.TURN_EXTREME    = 1.8 # Panik-spind når vi er ved at køre galt
        # =====================================================================

        self.get_logger().info('DEBUG driver startet - Kører Aggressiv Obstacle Avoidance...')

    # Hjælpefunktion: Får den korteste afstand i et 'slice' (udsnit) af laseren
    def get_min_dist(self, slice_arr):
        valid = [x for x in slice_arr if math.isfinite(x) and x > 0.02]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        if not msg.ranges:
            return

        idx = len(msg.ranges) // 2 # Midten (Fronten)
        
        # Udregn hvor mange index der går på bestemte vinkler
        offset_15 = int(math.radians(15) / msg.angle_increment)
        offset_45 = int(math.radians(45) / msg.angle_increment)
        offset_90 = int(math.radians(90) / msg.angle_increment)

        # Hent ZONER i stedet for enkelte punkter (Dækker hjørner af bokse)
        # Front dækker -15 til +15 grader
        front_slice = msg.ranges[idx - offset_15 : idx + offset_15]
        # Venstre dækker +15 til +45 grader
        left_slice  = msg.ranges[idx + offset_15 : idx + offset_45]
        # Højre dækker -45 til -15 grader
        right_slice = msg.ranges[idx - offset_45 : idx - offset_15]

        # Få den farligste (mindste) afstand i hver zone
        dist_front = self.get_min_dist(front_slice)
        dist_left  = self.get_min_dist(left_slice)
        dist_right = self.get_min_dist(right_slice)

        # Side-kollisionstjek (90 grader) til collision counter
        dist_l90 = self.get_min_dist(msg.ranges[idx + offset_45 : idx + offset_90])
        dist_r90 = self.get_min_dist(msg.ranges[idx - offset_90 : idx - offset_45])

        cmd = Twist()

        # Tæl kollisioner
        if dist_front <= 0.15 or dist_l90 <= 0.15 or dist_r90 <= 0.15:
            self.collisions += 1

        nan_pct = self.NaNPercentage(msg)

        # --- RECOVERY MODE (Glastests og sorte huller) ---
        if nan_pct > self.recovery_enter_threshold:
            self.recovery_mode = True

        if self.recovery_mode:
            if nan_pct > self.recovery_exit_threshold:
                cmd.linear.x = self.SPEED_REVERSE  # Bak ud af problemet
                cmd.angular.z = random.uniform(-self.TURN_EXTREME, self.TURN_EXTREME)
                self.publisher_.publish(cmd)
                return  # Spring resten af logikken over mens vi recover
            else:
                self.recovery_mode = False

        # --- AGGRESSIV STYRELOGIK ---
        
        # 1. DANGER ZONE: Vi har ramt noget eller er millimeter fra!
        if dist_front < self.DIST_DANGER or dist_left < self.DIST_DANGER or dist_right < self.DIST_DANGER:
            cmd.linear.x = self.SPEED_REVERSE # BAK! (Positiv X)
            
            # Sving ekstremt hurtigt væk fra den tætteste forhindring
            if dist_left < dist_right:
                cmd.angular.z = -self.TURN_EXTREME # Sving til højre
            else:
                cmd.angular.z = self.TURN_EXTREME  # Sving til venstre

        # 2. WARNING ZONE: Der er en forhindring foran os (f.eks. boks), gør klar til undvigelse
        elif dist_front < self.DIST_WARNING or dist_left < self.DIST_WARNING or dist_right < self.DIST_WARNING:
            cmd.linear.x = self.SPEED_SLOW # Sænk farten markant (-0.05)
            
            # Beslut vej baseret på hvilken side der har mest plads
            if dist_left < dist_right:
                cmd.angular.z = -self.TURN_AGGRESSIVE # Sving højre
            else:
                cmd.angular.z = self.TURN_AGGRESSIVE  # Sving venstre

        # 3. KLARE LINJER: Tryk speederen i bund
        else:
            cmd.linear.x = self.SPEED_FORWARD
            
            # Lille ekstra feature: Centrer robotten i gange. 
            # Hvis venstre væg er lidt tættere på end højre væg, juster blødt.
            if dist_left < 0.6 and dist_right < 0.6:
                if dist_left < dist_right - 0.1:
                    cmd.angular.z = -self.TURN_MILD
                elif dist_right < dist_left - 0.1:
                    cmd.angular.z = self.TURN_MILD
                else:
                    cmd.angular.z = 0.0
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
        print(f"Røde farver fundet undervejs: {node.red_count}")
        
        node.publisher_.publish(Twist()) # Stop robotten
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()