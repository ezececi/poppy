#!/usr/bin/env python3

import rospy
from poppy_ros_control.recorder import Player
from moveit_commander.move_group import MoveGroupCommander
from ros4pro_custom.srv import Replaymove
from ros4pro_custom.srv import ReplaymoveResponse
from std_srvs.srv import SetBool


def handle_replay_move(req):
    # init var
    ret = False

    set_compliant = rospy.ServiceProxy('set_compliant', SetBool)
    resp1 = set_compliant(False)
    if not resp1.success:
        print("Replay server error, could not set robot compliant")
        return  ReplaymoveResponse(ret)

    commander = MoveGroupCommander("arm_and_finger")
    player = Player()

    # This returns a moveit_msgs/RobotTrajectory object representing the recorded trajectory
    my_motion = player.load(req.move_name)

    # Go to the start position before replaying the motion
    commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
    commander.go()

    # Replay the exact same motion
    commander.execute(my_motion)

    ret = True

    return ReplaymoveResponse(ret, "No info")

if __name__ == "__main__":
    rospy.init_node('ros4pro_replay_move_server')
    s = rospy.Service('replay_move', Replaymove, handle_replay_move)
    print("Ready to replay moves")
    rospy.spin()


