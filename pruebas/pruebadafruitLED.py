import time
import board
import digitalio

led = digitalio.DigitalInOut(board.C0)
led.direction = digitalio.Direction.OUTPUT
led.value = True 

while True:
    led.value = not led.value 
    time.sleep(0.5)
