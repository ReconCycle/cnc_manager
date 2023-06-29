export ROS_MASTER_URI=http://192.168.0.101:11311
export ROS_IP=192.168.0.10

import rospy
from cnc_action_server import CNCActionServer

rospy.init_node('hello')
a = CNCActionServer(initialize_cnc=True)
a.goto(X=10, Y=10, Z=10)
