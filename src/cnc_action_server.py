#!/usr/bin/env python
import json
import numpy as np

import time
import serial

import rospy
import actionlib
#import actionlib_msgs
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from actionlib.msg import TestRequestActionGoal, TestRequestActionFeedback, TestRequestActionResult

class SmokeDetector():
    def __init__(self, smokedet_type:str, smokedet_radius:str, smokedet_height:str) -> None:
        self.height = float(smokedet_height)
        self.type = float(smokedet_type)
        self.radius= float(smokedet_radius)
        self.tabpos = None

class CNCActionServer(object):
    def __init__(self, ns = 'cnc_action_server', init_ros_node = True, initialize_cnc = False):
        """ Interface between a ROS node (disassembly_manager) and the CNC machine.
        This action server receives a string that contains:
        - the orientation of the smoke detector in degrees
        - the type of smoke detector (int 0-5) """

        self.ns = ns
        if init_ros_node:
            rospy.init_node(self.ns)

        self.delimiter = ' '

        self.load_and_check_parameters()

        # Receiving messages from disassembly_pipeline.
        self._as = actionlib.SimpleActionServer(self.ns, actionlib.msg.TestRequestAction, execute_cb = self.execute_cb, auto_start = False)

        self._as.start()
        rospy.loginfo("CNC action server /{} started".format(self.ns))

        # CNC init
        self.cnc_initialized = False
        self.cnc_port = None
        if initialize_cnc: self.initialize_cnc()

        self._goal = TestRequestActionGoal()
        self._feedback = TestRequestActionFeedback()
        self._result =  TestRequestActionResult()

        # Initialize services

    #def call_server(self, gcode_index:int = 0, rotation:float = 0):
    def load_and_check_parameters(self):
        self.DICTIONARY_FILENAME = 'dictionary.json'
        with open(self.DICTIONARY_FILENAME , 'r') as f:
            data = json.load(f)

        self.GCODE_TO_INT_DICT = data
        print(self.GCODE_TO_INT_DICT)
        #rospy.loginfo("{}\n{}".format(np.unique(self.GCODE_TO_INT_DICT.values()).size, len(self.GCODE_TO_INT_DICT.values())))
        n_unique_values = np.unique(list(self.GCODE_TO_INT_DICT.values())).size
        assert n_unique_values == len(self.GCODE_TO_INT_DICT.values()) 

        self.INT_TO_GCODE_DICT = {v: k for k, v in self.GCODE_TO_INT_DICT.items()}

    def execute_cb(self, goal):

        gcode_index = int(goal.the_result)

        if gcode_index == -1:
            # The user actually wants to run the Gcode
            gcode_string = str(goal.result_text)
            if gcode_string[-2:] != '\n': gcode_string += '\n'
            self.send_command(gcode_string)

        else:
            rotation = float(goal.result_text)

            print("GCODE_TO_INT_DICT:")
            print(self.GCODE_TO_INT_DICT)
            print("Value:", gcode_index)

            gcode_name = self.INT_TO_GCODE_DICT[gcode_index]
            #goal_string_split = goal_string.split(self.delimiter)
            # smoke detector type and rotation
            #type = goal_string_split[0]
            #rotation = goal_string_split[1]

            assert (rotation >= 0) and (rotation <= 360), 'Invalid rotation parameters'

            # Send command to run G code for smoke detector type "t" and rotation "rot"
            rospy.loginfo("{} gcode_name = {}, smoke detector_rotation = {}".format(self.ns, gcode_name, rotation))
            # Do the CNC cutting
            if gcode_name == 'homing':
                self.home()
            elif gcode_name == 'return_to_zero':
                self.go_home()
            elif gcode_name == 'smoke_detector_fumonic_case_cut':
                # TODO put all this into cutSmokedetFumonicRadionet
                cutSmokedetFumonicRadionet(self, (-241, -154))
                self.go_home()
            elif gcode_name == 'smoke_detector_fumonic_battery_tab_cut':
                0
        # Set goal to success or whatever
        self._result.result.the_result = 1
        self._as.set_succeeded(self._result.result)
        return 0

    def initialize_cnc(self):
        cnc_baudrate = 115200

        min_com_port_n = 0
        max_com_port_n = 5

        port = None

        for i in range(min_com_port_n, max_com_port_n):
            try:
                rospy.loginfo("Probing /dev/ttyUSB port {}".format(i))
                port = serial.Serial('/dev/ttyUSB{}'.format(i), cnc_baudrate)

                # Break when we find working CNC port.
                rospy.loginfo("Found port /dev/ttyUSB{}".format(i))
                break
            except Exception as e:
                rospy.loginfo("CNC server Exception: {}".format(e))
        if port is None:
            raise Exception("Did not find CNC at /dev/ttyUSB({0}-{1})".format(min_com_port_n, max_com_port_n))
        self.cnc_port = port

        self.enable_cnc()

        #self.home()

        self.cnc_initialized = True
        return 0

    def go_home(self):
        string = 'G01X0Y0Z0F1000\n'
        self.send_command(string)

    def enable_cnc(self):
        string = '$X\n'
        self.send_command(string)

    def home(self):
        string = '$H\n'
        self.send_command(string)

    def startSpindle(self):
        string = 'M3S1000\n'
        self.send_command(string)

    def stopSpindle(self):
        string = 'M5\n'
        self.send_command(string)

    def goto(self, X=None,Y=None,Z=None, feed=100):
        string = 'G01'+'X'+str(X)+'Y'+str(Y)+'Z'+str(Z)+ 'F'+ str(feed)+'\n'
        self.send_command(string)

    def cut_XY_arc(self, direction:str, target:tuple, radius:tuple, feed:float):
        """
        Arc cutting function

        Args:
        ----
            - direction(str) : Direction of cutting, either clockwise(G02) or counterclockwise(G03)
            - target(X:float, Y:float) : target coordinate at the end of the movement
            - radius(I:float, J:float) : radius center, relative to the target coordinate
            - feed(flsoat) : feed rate of the movement

        Usage:
        ------
            - If you want to cut an arch, you need to mind the relationship between the target point and the radius
            - If you want to cut a full circle, the target point should be set to the current location of the gantry

        Example:
        --------
        >>> cut_XY_arc(direction='G02', target=(13, 12), radius=(4,20), feed=6.9):
        """
        assert self.cnc_initialized == True, 'CNC not initialised'
        assert direction in ['G02', 'G03'], 'Invalid arc "direction" parameter'
        #assert (target[0] is float) and (target[1] is float), 'Invalid data types, should be (X:float, Y:float)'
        #assert (radius[0] is float) and (radius[1] is float), 'Invalid data types, should be (I:float, J:float)'

        string = 'G17'+direction+'X'+str(target[0])+'Y'+str(target[1])+'I'+str(radius[0])+'J'+str(radius[1])+'F'+str(feed)+'\n'

        self.send_command(string)

    def cut_tabs(self, tab_0_location:tuple, tab_1_location:tuple, smokedet:SmokeDetector, cut_feed:int= 100, move_feed:int=500):
        """
        Battery solder blob/tab cutting function

        Args:
        -----
            - tab_0_location(tuple) : The location of the first (left) battery connector tab, specified as a tuple of floats in the format of (X:float, Y:float, Z:float)
            - tab_1_location(tuple) : The location of the second (right) battery connector tab, specified as a tuple of floats in the format of (X:float, Y:float, Z:float)
            - smokedet:(SmokeDetector) : Instance of the SmokeDetector class, containing important information about the specific type of smoke detector, like it's radius, height, material thickness and battery tab positions        
            - cut_feed(int) : Feedrate (mm/min) for plunge cutting the solder blobs, should be kept low
            - move_feed(int) : Feedrate (mm/min) for moving between cutting locations, should be kept higher than the cutting feedrate to save time

        Usage:
        ------

        Example:
        --------
        >>> cut_tabs(tab_0_location=(-42, -69, -50), tab_1_location=(42, -69, -50), smokedet=SmokeDetector_0, cut_feed=100, move_feed=420)
        """
        # Position cutter above first tab
        string = 'G01'+'X'+str(tab_0_location[0])+'Y'+str(tab_0_location[1])+'Z'+str(tab_0_location[2]+smokedet.height+10)+'F'+str(move_feed)

        self.send_command(string)

        # Plunge onto solder blob
        string = 'G01'+'X'+str(tab_0_location[0])+'Y'+str(tab_0_location[1])+'Z'+str(tab_0_location[2])+'F'+str(cut_feed)

        self.send_command(string)

        #Return above the solder blob
        string = 'G01'+'X'+str(tab_0_location[0])+'Y'+str(tab_0_location[1])+'Z'+str(tab_0_location[2]+smokedet.height+10)+'F'+str(move_feed)
        self.send_command(string)

        #Move gantry aove the other solder point
        string = 'G01'+'X'+str(tab_1_location[0])+'Y'+str(tab_1_location[1])+'Z'+str(tab_1_location[2]+smokedet.height+10)+'F'+str(move_feed)
        self.send_command(string)

        # Plunge onto solder blob
        string = 'G01'+'X'+str(tab_1_location[0])+'Y'+str(tab_1_location[1])+'Z'+str(tab_1_location[2])+'F'+str(cut_feed)
        self.send_command(string)

        #Return above the second solder blob
        string = 'G01'+'X'+str(tab_1_location[0])+'Y'+str(tab_1_location[1])+'Z'+str(tab_1_location[2]+smokedet.height+10)+'F'+str(move_feed)
        self.send_command(string)

        return 0

    def send_command(self, string_command):

        rospy.loginfo("Sending CNC command: {}".format(string_command))
        self.cnc_port.write(bytes(string_command,'utf-8'))

        output = self.cnc_port.readline()
        rospy.loginfo("Got CNC response: {}".format(output))

        return 0

def cutSmokedetFumonicRadionet(cnc_manager_class:CNCActionServer, init_pos:tuple):
    """ Cuts the circular tab/arc one the upper side of the smoke detector."""
    CUTTING_DIAMETER = 83
    target = init_pos
    radius = (0, -CUTTING_DIAMETER/2)

    reset_zero_string = 'G92X-3Y-3Z-3\n'
    cnc_manager_class.send_command(reset_zero_string)

    move_above_center = True
    if move_above_center :
        cnc_manager_class.goto(X=init_pos[0], Y=init_pos[1], Z=-3, feed=1000)
    else:
        0
        # We will move above ARC POINT where we will start

    # Start spindle
    cnc_manager_class.startSpindle()
    # Move down to start the cut (lower Z means the drill goes lower - naturally)
    cnc_manager_class.goto(X=init_pos[0], Y=init_pos[1], Z = -29, feed = 200)

    # Move to arc edge
    cnc_manager_class.cut_XY_arc(direction='G02', target = init_pos, radius=radius, feed=200)

    cnc_manager_class.goto(X=init_pos[0], Y=init_pos[1], Z=-3, feed=1000)
    cnc_manager_class.stopSpindle()


if __name__ == '__main__':

    server = CNCActionServer(initialize_cnc = True, init_ros_node = True)
    #server.stopSpindle()
    server.home()

    #server.startSpindle()
    #server.goto(X= -241, Y= -154, Z= 20, feed = 1000)
    #server.goto(X= -100, Y= -100, Z= -15, feed=1000)
    
    #cutSmokedetFumonicRadionet(server, (-241, -152))
    #server.goto(-100,-100,-100)

    rospy.spin()
