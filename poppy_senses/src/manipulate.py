#!/usr/bin/env python3

import rospy
import std_srvs
from std_srvs.srv import SetBool
from poppy_ros_control.recorder import Recorder
from datetime import datetime as datetime
from moveit_commander.move_group import MoveGroupCommander
import math

rospy.init_node('ros4pro_custom_node')

id = "4"

# orig_pose = [-4, -90, -2, 3, 90, -24]
orig_pose = [0, 0, 0, 0, 0, 0]
orig_pose = [0, -1*math.pi/2, 0, 0, math.pi/2, 0.44]

rospy.loginfo("Start node " + id)

if not rospy.is_shutdown():
    rospy.wait_for_service('set_compliant')

    try:
        set_compliant = rospy.ServiceProxy('set_compliant', SetBool)
        resp1 = set_compliant(False)
        if resp1.success == True:
            move_name = "mot" + datetime.now().strftime("%H%M%S")

            # print("Robot is compliant, move it to the desired origin position")
            # rospy.sleep(10)

            print("Set origin position")
            commander = MoveGroupCommander("arm_and_finger", wait_for_servers=20)
            commander.set_joint_value_target(orig_pose)
            commander.go()
            rospy.sleep(4)
            # print("Pose type :")
            # print(type(orig_pose))
            # print(orig_pose)

            resp1 = set_compliant(True)
            if resp1.success == True:
                print("Robot is compliant")

                print ("Now Go record your move...")
                r = Recorder()
                r.start_recording()
                # Move your robot with your hands in compliant mode to record its trajectory
                rospy.sleep(15)

                r.stop_and_save(move_name)
                print("Move recording stopped, release robot")
                rospy.sleep(3)

                print("Robot return to origin position")
                print("Set compliant to False")
                resp1 = set_compliant(False)
                if resp1.success == True:
                    commander.set_joint_value_target(orig_pose)
                    commander.go()
                    rospy.sleep(4)

                print("Move name recorded: " + move_name)

            else:
                print("Error, abort")
        else:
            print("Error : could not set compliance, abort.")

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    rospy.loginfo("End node " + id)







# import cv2, rospy
# from poppy_controllers.srv import GetImage
# from cv_bridge import CvBridge
#
# get_image = rospy.ServiceProxy("get_image", GetImage)
# response = get_image()
# bridge = CvBridge()
# image = bridge.imgmsg_to_cv2(response.image)
# cv2.imshow("Poppy camera", image)
# cv2.waitKey(200)


""" 
from moveit_commander.move_group import MoveGroupCommander
commander = MoveGroupCommander("arm_and_finger", wait_for_servers=20)

while not rospy.is_shutdown():
    rospy.loginfo("KAlive node " + id)
    
    commander.set_pose_target([0.02, 0.09, 0.22] + [1, 0, 0, 0])
    #commander.set_joint_value_target([-0.23281901800000002, -1.4356591020000002, -0.12024910300000001, -0.145904572, 1.200222179, 0.6679148290000001])
    commander.go()    
    
    rate.sleep() """





