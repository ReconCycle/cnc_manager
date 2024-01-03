#!/usr/bin/env python
from std_srvs.srv import SetBool
import rospy
import RPi.GPIO as GPIO

pin_open = 20
pin_close = 21
GPIO.setmode(GPIO.BCM
GPIO.setup(pin_open, GPIO.OUT)
GPIO.setup(pin_close, GPIO.OUT)

def cnc_chuck_handler(req):
    state = req.data
    if state:
        GPIO.output(pin_open, GPIO.HIGH)
        GPIO.output(pin_close, GPIO.LOW)
        msg = "CNC chuck: open"
    else:
        GPIO.output(pin_close, GPIO.HIGH)
        GPIO.output(pin_open, GPIO.LOW)
        msg = "CNC chuck: closed"
    return True, msg

def cnc_chuck_server():
    rospy.init_node('cnc_chuck')
    s = rospy.Service('/cnc_chuck_open', SetBool, cnc_chuck_handler)
    rospy.spin()

if __name__ == "__main__":
    cnc_chuck_server()
