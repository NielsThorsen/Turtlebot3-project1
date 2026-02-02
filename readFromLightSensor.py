import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")


def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        # Convert the data to green, red and blue int values
        # Insert code here
        green = data[1] * 256 + data[0]
        red   = data[3] * 256 + data[2]
        blue  = data[5] * 256 + data[4]
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)" % (red, green, blue))
        
        time.sleep(2) 

def bit_8_RGB():
    while True:
        # 1. Read 6 bytes of data starting from Green Low (0x09)
        data = bus.read_i2c_block_data(0x44, 0x09, 6)

        # 2. Convert to 16-bit integers (Low byte first)
        green = data[1] * 256 + data[0]
        red   = data[3] * 256 + data[2]
        blue  = data[5] * 256 + data[4]

        # 3. Scale down to 8-bit (0-255) for the terminal
        # Using bit-shifting (>> 8) is a clean way to convert 16-bit to 8-bit
        r_8 = red >> 8
        g_8 = green >> 8
        b_8 = blue >> 8

        # 4. ANSI Escape Code for TrueColor Background
        # \x1b[48;2;R;G;Bm creates a colored background block
        # \x1b[0m resets the formatting back to normal
        color_block = f"\x1b[48;2;{r_8};{g_8};{b_8}m      \x1b[0m"
        
        print(f"Detected Color: {color_block} | RGB: ({r_8}, {g_8}, {b_8})")

        time.sleep(2)

bit_8_RGB()


