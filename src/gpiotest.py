import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
A2 = 19
A1 = 26
A1state = False
GPIO.setup(A2, GPIO.OUT)
GPIO.setup(A1, GPIO.OUT)

while True:
    A1state = not A1state
    GPIO.output(A2, False)
    GPIO.output(A1, A1state)
    time.sleep(1)
