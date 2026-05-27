from gpiozero import LED
from time import sleep

led = LED(17)

print("LED blinker nu... (Tryk Ctrl+C for at stoppe)")

while True:
    led.on()    # Tænd LED
    sleep(1)    # Vent 1 sekund
    led.off()   # Sluk LED
    sleep(1)    # Vent 1 sekund