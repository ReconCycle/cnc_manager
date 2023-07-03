#!/usr/bin/env python
import numpy as np
import os
import sys
import json

import rospy

import actionlib

#import actionlib_msgs
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from actionlib.msg import TestRequestGoal, TestRequestActionGoal, TestRequestActionFeedback, TestRequestActionResult

class CNCActionClient(object):
    _goal = TestRequestActionGoal()
    _feedback = TestRequestActionFeedback()
    _result =  TestRequestActionResult()

    def __init__(self, ns = 'cnc_action_server', wait_for_server = False,
                cnc_airblock_topic = "cnc_airblock_activate",
                cnc_gnd_topic = "cnc_GND",
                cnc_chuck_topic = "cnc_chuck_open",
                init_ros_node = True):

        """ Interface between a ROS node (disassembly_manager) and the CNC machine.
        This action client sends a goal that contains:
        - the orientation of the smoke detector in degrees
        - the type of smoke detector (int 0-5) """

        self.ns = ns

        if init_ros_node:
            rospy.init_node('cnc_action_client')

        CONTROL_PNEUMATICS = True
        if CONTROL_PNEUMATICS:
            from digital_interface_msgs.srv import PinStateWrite
            self.CNC_AIRBLOCK_TOPIC = cnc_airblock_topic
            self.CNC_GND_TOPIC = cnc_gnd_topic
            self.CNC_CHUCK_TOPIC = cnc_chuck_topic
            self.cnc_airblock_svc = rospy.ServiceProxy(self.CNC_AIRBLOCK_TOPIC, PinStateWrite)
            self.cnc_gnd_svc = rospy.ServiceProxy(self.CNC_GND_TOPIC, PinStateWrite)
            self.cnc_chuck_svc = rospy.ServiceProxy(self.CNC_CHUCK_TOPIC, PinStateWrite)
            self.prepare_pneumatics()

        self.delimiter = ' '

        self.goal = TestRequestGoal()

        # Receiving messages from disassembly_pipeline.
        self._client = actionlib.SimpleActionClient(self.ns, actionlib.msg.TestRequestAction)
        if wait_for_server:
            self._client.wait_for_server()

        rospy.loginfo("CNC action client /{} started".format(self.ns))

        self.load_and_check_parameters()

    def prepare_pneumatics(self):
        self.cnc_airblock_svc.call(True)
        self.cnc_gnd_svc.call(False)
        self.move_chuck(command = 'open')

    def move_chuck(self, command = 'close'):
        assert command in ['close', 'open']

        if command == 'close':
            self.cnc_chuck_svc.call(True)
        else:
            self.cnc_chuck_svc.call(False)
        return 0

    def load_and_check_parameters(self):
        self.DICTIONARY_FILENAME = 'dictionary.json'

        script_dir = os.path.dirname(__file__)
        abs_dict_path = os.path.join(script_dir, self.DICTIONARY_FILENAME)
        with open(abs_dict_path, "r") as f:
            data = json.load(f)

        self.GCODE_TO_INT_DICT = data
        #print(self.GCODE_TO_INT_DICT)
        #rospy.loginfo("{}\n{}".format(np.unique(self.GCODE_TO_INT_DICT.values()).size, len(self.GCODE_TO_INT_DICT.values())))
        n_unique_values = np.unique(list(self.GCODE_TO_INT_DICT.values())).size
        assert n_unique_values == len(self.GCODE_TO_INT_DICT.values())

    def call_server(self, gcode = None):
        """ Function to call the CNC Action server.
        Can be used to:
        1. Send single G-code line (specify gcode = 'G01X-10Y-10Z-10F100')
        2. Send a list of G-codes (if type(gcode) == list)
        
        Args:
        ------------------
        gcode : If not None, it can be either a type(list) or type(string)."""
        goal = self.goal


        if type(gcode) != list:
            gcode = [gcode]

        for element in gcode:
            # We actually wnat to directly send some G-code
            goal.result_text = element
            goal.the_result = -1 # -1 so the action server knows to just run G-code

            result = self.send_goal(goal)
        # Return only the last result
        return result
    
    def call_server_prespecified(self, gcode_operation_name = None, rotation = 0):
        """   Function to:
        Command the server to run one of (n) pre-specified g-codes, as seen in self.GCODE_TO_INT_DICT.
        """
        goal = self.goal

        assert gcode_operation_name in self.GCODE_TO_INT_DICT.keys()
        assert (rotation >= 0) and (rotation <= 360)
        gcode_index = self.GCODE_TO_INT_DICT[gcode_operation_name]

        goal.result_text = str(rotation)
        goal.the_result = gcode_index
        
        rospy.loginfo("CNC client sent goals: {} gcode command type name = {}, smoke detector_rotation = {}".format(self.ns, gcode_operation_name, rotation))

        result = self.send_goal(goal)
        return result

    def send_goal(self, goal):
        self._client.send_goal(goal)
        self._client.wait_for_result()
        result = self._client.get_result()

        rospy.loginfo("CNC client got result: {}".format(result))
        return result

    def example_call(self):
        self.call_server(gcode_operation_name = 'homing', rotation = 100)
        #self.call_server(gcode_operation_name = 'smoke_detector_fumonic_case_cut', rotation = 0)

if __name__ == '__main__':
    server = CNCActionClient(wait_for_server = True)
    server.example_call()
    rospy.spin()
