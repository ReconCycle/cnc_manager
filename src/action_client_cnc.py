#!/usr/bin/env python
import numpy as np
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

    def __init__(self, ns = 'cnc_manager', wait_for_server = False, 
                cnc_airblock_topic = "cnc_airblock_activate", 
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
            from digital_interface_msgs.msg import PinStateWrite
            self.CNC_AIRBLOCK_TOPIC = cnc_airblock_topic
            self.CNC_CHUCK_TOPIC = cnc_chuck_topic
            self.cnc_airblock_svc = rospy.ServiceProxy(self.CNC_AIRBLOCK_TOPIC, PinStateWrite)
            self.cnc_chuck_svc = rospy.ServiceProxy(self.CNC_CHUCK_TOPIC, PinStateWrite)

        self.delimiter = ' '

        self.goal = TestRequestGoal()

        # Receiving messages from disassembly_pipeline.
        self._client = actionlib.SimpleActionClient(self.ns, actionlib.msg.TestRequestAction)
        if wait_for_server:
            self._client.wait_for_server()

        rospy.loginfo("CNC action client /{} started".format(self.ns))

        
        self.load_and_check_parameters()

        self.prepare_pneumatics()
    #def call_server(self, gcode_index:int = 0, rotation:float = 0):
    def prepare_pneumatics(self):
        self.cnc_airblock_svc.call(True)

    def move_chuck(self, command = 'close'):
        assert command in ['close', 'open']

        if command == 'close':
            self.cnc_chuck_svc.call(True)
        else:
            self.cnc_chuck_svc.call(False)
        return 0

    def load_and_check_parameters(self):
        self.DICTIONARY_FILENAME = 'dictionary.json'
        with open(self.DICTIONARY_FILENAME , 'r') as f:
            data = json.load(f)

        self.GCODE_TO_INT_DICT = data
        #print(self.GCODE_TO_INT_DICT)
        #rospy.loginfo("{}\n{}".format(np.unique(self.GCODE_TO_INT_DICT.values()).size, len(self.GCODE_TO_INT_DICT.values())))
        n_unique_values = np.unique(list(self.GCODE_TO_INT_DICT.values())).size
        assert n_unique_values == len(self.GCODE_TO_INT_DICT.values()) 

    def call_server(self, gcode_operation_name, rotation = 0):

        assert gcode_operation_name in self.GCODE_TO_INT_DICT.keys()
        assert (rotation >= 0) and (rotation <= 360)

        gcode_index = self.GCODE_TO_INT_DICT[gcode_operation_name]

        goal = self.goal
        #goal_string_split = goal_string.split(self.delimiter)
        # smoke detector type and rotation
        #type = goal_string_split[0]
        #rotation = goal_string_split[1]

        goal.result_text = str(rotation)
        goal.the_result = gcode_index

        # Send command to run G code for smoke detector type "t" and rotation "rot"
        self._client.send_goal(goal)
        rospy.loginfo("CNC client sent goals: {} gcode command type name = {}, smoke detector_rotation = {}".format(self.ns, gcode_operation_name, rotation))

        self._client.wait_for_result()
        return self._client.get_result()

    def example_call(self):
        self.call_server(gcode_operation_name = 'homing', rotation = 100)
        self.call_server(gcode_operation_name = 'smoke_detector_fumonic_case_cut', rotation = 0)

if __name__ == '__main__':
    server = CNCActionClient(wait_for_server = True)
    server.example_call()
    rospy.spin()
