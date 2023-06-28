#!/usr/bin/env python

import rospy

import actionlib

#import actionlib_msgs
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from actionlib.msg import TestRequestGoal, TestRequestActionGoal, TestRequestActionFeedback, TestRequestActionResult

class CNCActionClient(object):
    _goal = TestRequestActionGoal()
    _feedback = TestRequestActionFeedback()
    _result =  TestRequestActionResult()

    def __init__(self, ns = 'cnc_manager', wait_for_server = False):
        """ Interface between a ROS node (disassembly_manager) and the CNC machine.
        This action client sends a goal that contains:
        - the orientation of the smoke detector in degrees
        - the type of smoke detector (int 0-5) """
        self.ns = ns

        self.delimiter = ' '

        self.goal = TestRequestGoal()

        # Receiving messages from disassembly_pipeline.
        self._client = actionlib.SimpleActionClient(self.ns, actionlib.msg.TestRequestAction)
        if wait_for_server:
            self._client.wait_for_server()

        rospy.loginfo("CNC action client /{} started".format(self.ns))

    def call_server(self, gcode_index:int = 0, rotation:float = 0):

        goal = self.goal
        #goal_string_split = goal_string.split(self.delimiter)
        # smoke detector type and rotation
        #type = goal_string_split[0]
        #rotation = goal_string_split[1]

        assert (rotation > 0) and (rotation < 360)

        goal.result_text = str(rotation)
        goal.the_result = gcode_index

        # Send command to run G code for smoke detector type "t" and rotation "rot"
        self._client.send_goal(goal)
        rospy.loginfo("CNC client sent goal: {} smoke detector type = {}, smoke detector_rotation = {}".format(self.ns, gcode_index, rotation))

        return 0

    def example_call(self):
        self.call_server(gcode_index = 0, rotation = 100)

if __name__ == '__main__':
    rospy.init_node('cnc_client')
    server = CNCActionClient(wait_for_server = True)
    server.example_call()
    rospy.spin()
