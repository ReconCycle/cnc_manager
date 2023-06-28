#!/usr/bin/env python

import serial
import time




is_initialized = False
def initalization():
    # Use a breakpoint in the code line below to debug your script.
    cnc_port = serial.Serial('COM4',115200)
    time.sleep(0.1)
    cnc_port.write(b'$X\n')
    time.sleep(0.1)
    #cnc_port.write(b'$H\n')
    time.sleep(0.1)
    is_initialized = True
    return cnc_port

def goto(cnc_port, X=None,Y=None,Z=None, speed=100):
    string = 'G01'+'X'+str(X)+'Y'+str(Y)+'Z'+str(Z)+ 'F'+ str(speed)+'\n'

    cnc_port.write(bytes(string,'utf-8'))
    time.sleep(0.1)
if __name__ == '__main__':

    cnc_port = initalization()
    goto(cnc_port,-100,-100,-100)

