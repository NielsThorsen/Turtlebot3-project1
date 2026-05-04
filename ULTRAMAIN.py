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
        node.get_logger().info('Farvesensor initialiseret.')
    except Exception as e:
        node.get_logger().error(f"I2C Forbindelsesfejl: {e}")
        return

    await asyncio.sleep(1)
    led = LED(14)
    last_red_time = 0.0  # Forhindrer "dobbelt-tælling" af samme farveklat
    cooldown = 3.0       # Vent 3 sekunder mellem hver tælling

    while True:
        try:
            data = bus.read_i2c_block_data(0x44, 0x09, 6)

            green = data[1] * 256 + data[0]
            red   = data[3] * 256 + data[2]
            blue  = data[5] * 256 + dataor dist_right < 0.6:
                    cmd.angular.z = self.var_turning(dist_right, dist_left) * self.TURN_MILD 
                    print(self.var_turning(dist_right, dist_left))
            else:
                cmd.angular.z = 0.0[4]

            r_8 = red >> 8
            g_8 = green >> 8
            b_8 = blue >> 8

            current_time = time.time()
            if r_8 > 70 and r_8 > b_8 and g_8 > 40 and g_8 < 70 and b_8 < 40:
                if (current_time - last_red_time) > cooldown:
                    node.red_count += 1
                    last_red_time = current_time
                    color_block = f"\x1b[48;2;{r_8};{g_8};{b_8}m      \x1b[0m"
                    node.get_logger().info(f"Rød registreret! Total: {node.red_count} | Farve: {color_block} | RGB: ({r_8}, {g_8}, {b_8}")
                    led.on()
                    sleep(1)
                    led.off()

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
        self.DIST_DANGER  = 0.18  # Hvis tættere på end dette -> BAK og SPIND!
        self.DIST_WARNING = 0.20  # Øget lidt for at give bedre tid til sving
        self.DIST_SHOULDER = 0.18 # Hvor tæt må kasser komme på robottens sider under sving?
        self.DIST_VSHAPE = 0.20 # Når den rammer en v-shape

        # Hastigheder (Negativ X = Fremad)
        self.SPEED_FORWARD = -0.17 
        self.SPEED_SLOW    = -0.07
        self.SPEED_REVERSE =  0.5 # Bak-hastighed
        
        # Drejehastigheder (Rad/s)
        self.TURN_MILD       = 0.4 
        self.TURN_AGGRESSIVE = 0.9 
        self.TURN_EXTREME    = 1.5
        # =====================================================================

        self.get_logger().info('DEBUG driver startet - Kører Aggressiv FOV Obstacle Avoidance...')
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

        idx = len(msg.ranges) // 2 # Midten (Fronten)
        
        # Udvid FOV (Field of View) for at dække mere
        offset_20  = int(math.radians(20) / msg.angle_increment)
        offset_75  = int(math.radians(75) / msg.angle_increment)
        offset_110 = int(math.radians(110) / msg.angle_increment) # Skulder-kig
        offset_180 = int(math.radians(110) / msg.angle_increment) # Kig bagud

        # Hent ZONER (Brede snit i stedet for enkelte stråler)
        front_slice = msg.ranges[idx - offset_20 : idx + offset_20]
        left_slice  = msg.ranges[idx + offset_20 : idx + offset_75]
        right_slice = msg.ranges[idx - offset_75 : idx - offset_20]
        back_slice = msg.ranges[idx - offset_180: idx - offset_180]
        
        # DE NYE "SKULDER" ZONER (Tjekker siderne for at undgå at snitte hjørner)
        shoulder_left_slice  = msg.ranges[idx + offset_75 : idx + offset_110]
        shoulder_right_slice = msg.ranges[idx - offset_110 : idx - offset_75]

        # Få den mindste afstand i hver zone
        dist_front = self.get_min_dist(front_slice)
        dist_left  = self.get_min_dist(left_slice)
        dist_right = self.get_min_dist(right_slice)
        dist_shoulder_l = self.get_min_dist(shoulder_left_slice)
        dist_shoulder_r = self.get_min_dist(shoulder_right_slice)
        dist_back = self.get_min_dist(back_slice)

        cmd = Twist()

        # Tæl kollisioner (Tjekker nu også skuldrene)
        if dist_front <= 0.15 or dist_shoulder_l <= 0.12 or dist_shoulder_r <= 0.12:
            self.collisions += 1

        nan_pct = self.NaNPercentage(msg)

        # --- RECOVERY MODE (Glastests / Sorte Huller) ---
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

        # --- STYRELOGIK ---

        # 0. V-SHAPE FÆLDEN (Blindgyde / Hjørne)
        # Hvis fronten OG begge sider er blokeret, er vi i et hjørne. Vi skal bakke og dreje hårdt for at slippe ud.
        if dist_front < self.DIST_VSHAPE and dist_left < self.DIST_VSHAPE and dist_right < self.DIST_VSHAPE:
            self.DIST_VSHAPE = 0.25
            # cmd.linear.x = self.SPEED_REVERSE
            print("V-Shape")
            cmd.angular.z = self.TURN_EXTREME # Vælg én fast retning for at bryde ud af V-shapen
            
        # 2. WARNING ZONE: Gør klar til at undvige en forhindring længere fremme
        elif dist_front < self.DIST_WARNING and (dist_left < self.DIST_WARNING or dist_right < self.DIST_WARNING):
            cmd.linear.x = self.SPEED_SLOW # Sænk farten
            print("WARNING ZONE")
            if dist_left < dist_right:
                cmd.angular.z = -self.TURN_AGGRESSIVE # Sving højre
            else:
                cmd.angular.z = self.TURN_AGGRESSIVE  # Sving venstre

        # 3. KLARE LINJER: Fremad (med let centrering)
        else:
            print("Klare Linjer")
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
        print(f"Røde farver fundet undervejs: {node.red_count}")
        
        node.publisher_.publish(Twist()) # Stop robotten
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()