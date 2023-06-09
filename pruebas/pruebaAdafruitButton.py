import board
import digitalio

button = digitalio.DigitalInOut(board.C0)
button.direction = digitalio.Direction.OUTPUT

while True:
    button.value == False
