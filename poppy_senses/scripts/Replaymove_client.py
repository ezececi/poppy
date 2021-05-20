#!/usr/bin/env python3

import sys
import rospy
from ros4pro_custom.srv import Replaymove
from poppy_ros_control.recorder import Recorder
from moveit_commander.move_group import MoveGroupCommander

rospy.init_node('ros4pro_replay_move_client')

def replay_move(name):
    rospy.wait_for_service('replay_move')
    try:
        replay_request = rospy.ServiceProxy('replay_move', Replaymove)
        resp1 = replay_request(name)
        rospy.sleep(1)
        print("Request success : " + str(resp1.success))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "Usage : %s [name of the move]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        move_name_arg = str(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s..."%(move_name_arg))
    replay_move(move_name_arg)
