#!/usr/bin/env python3

import cv2
import rospy
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge
from tflite_runtime.interpreter import Interpreter
import numpy as np
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
import moveit_msgs.msg
import math
from tf.transformations import *


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


orig_pose_armfinger = [0, -2*math.pi/6, math.pi/18, math.pi/36, math.pi/4, 0.44]
# orig_pose_arm = [0, -1*math.pi/2.2, 0, 0, math.pi/2.2]
# orig_pose_arm = [0, -2*math.pi/6, -math.pi/4, 0, math.pi/2.3]
orig_pose_arm = [0, -2*math.pi/6, math.pi/18, math.pi/36, math.pi/4]


def set_input_tensor(interpreter, image):
    """Sets the input tensor."""
    tensor_index = interpreter.get_input_details()[0]['index']
    input_tensor = interpreter.tensor(tensor_index)()[0]
    input_tensor[:, :] = np.expand_dims((image-255)/255, axis=0)


def get_output_tensor(interpreter, index):
    """Returns the output tensor at the given index."""
    output_details = interpreter.get_output_details()[index]
    tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
    return tensor


def detect_objects(interpreter, image, threshold):
    """Returns a list of detection results, each a dictionary of object info."""
    set_input_tensor(interpreter, image)
    interpreter.invoke()
    # Get all output details
    boxes = get_output_tensor(interpreter, 0)
    classes = get_output_tensor(interpreter, 1)
    scores = get_output_tensor(interpreter, 2)
    count = int(get_output_tensor(interpreter, 3))

    results = []
    for i in range(count):
        if scores[i] >= threshold:
            result = {
                'bounding_box': boxes[i],
                'class_id': classes[i],
                'score': scores[i]
            }
            results.append(result)
    return results


class MoveArm(object):

    def __init__(self):

        rospy.Timer(rospy.Duration(3.0), self.plan_move)

        self.xmin = -1
        self.ymin = -1
        self.xmax = -1
        self.ymax = -1

        self.dx_move_threshold = 80 # camer width wise
        self.dy_move_threshold = 80        

        self.plan = None
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander(
            "arm", wait_for_servers=20)
        self.move_group_af = MoveGroupCommander(
            "arm_and_finger", wait_for_servers=20)            
        # self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                                     moveit_msgs.msg.DisplayTrajectory,
        #                                                     queue_size=20)

    # x = width (640 max for rpi camera), y = height

    def plan_move(self, event=None):

        while (not rospy.is_shutdown()):
            
            if (self.xmin == -1):
                return

            scale = 1

            delta_x = CAMERA_WIDTH/2 - (self.xmin + self.xmax)/2
            delta_y = CAMERA_HEIGHT/2 - (self.ymin + self.ymax)/2

            if delta_x > self.dx_move_threshold:
                x_way = 1.0
            elif delta_x < -self.dx_move_threshold:
                x_way = -1.0
            else:
                x_way = 0

            if delta_y > self.dy_move_threshold:
                y_way = 1.0
            elif delta_y < -self.dy_move_threshold:
                y_way = -1.0
            else:
                y_way = 0

            
            if ((x_way != 0) or (y_way != 0)):
                wpose = self.move_group.get_current_pose().pose

                print('Original pose : ', wpose)

                # Translation
                wpose.position.z += y_way * scale * 0.01        
                
                # Rotation
                q_rot = quaternion_from_euler(
                    0,#-y_way * 10 * scale * (math.pi/180), 
                    0,
                    x_way * 10 * scale * (math.pi/180))
                q_orig = [wpose.orientation.x, wpose.orientation.y,
                        wpose.orientation.z, wpose.orientation.w]
                q_new = quaternion_multiply(q_rot, q_orig)
                wpose.orientation.x = q_new[0]
                wpose.orientation.y = q_new[1]
                wpose.orientation.z = q_new[2]
                wpose.orientation.w = q_new[3]

                print('New pose : ', wpose)

                # waypoints = []
                # waypoints.append(copy.deepcopy(wpose))

                # # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
                # # waypoints.append(copy.deepcopy(wpose))

                # # We want the Cartesian path to be interpolated at a resolution of 1 cm
                # # which is why we will specify 0.01 as the eef_step in Cartesian
                # # translation.  We will disable the jump threshold by setting it to 0.0,
                # # ignoring the check for infeasible jumps in joint space, which is sufficient
                # # for this tutorial.
                # (plan, fraction) = self.move_group.compute_cartesian_path(
                #     waypoints,   # waypoints to follow
                #     0.01,        # eef_step
                #     0.0)         # jump_threshold
                # print('Fraction:', fraction)

                # return plan, fraction

                self.move_group.set_joint_value_target(wpose, True)
                ret, plan, pltime, errc = self.move_group.plan()
                if (moveit_msgs.msg.MoveItErrorCodes.SUCCESS == ret):
                    self.move_group.execute(plan, wait=False)
                else:
                    print('Plan failed with error code : ', errc)
                # # Calling `stop()` ensures that there is no residual movement
                self.move_group.stop()
                # # It is always good to clear your targets after planning with poses.
                # # Note: there is no equivalent function for clear_joint_value_targets()
                self.move_group.clear_pose_targets()
                # return plan, 0

    # def display_trajectory(self, event=None):
    #     print('Display Trajectory...')
    #     if (self.xmin == -1):
    #         print('None')
    #     else:
    #         self.plan_move(self.xmin, self.ymin, self.xmax, self.ymax)
    #         # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #         # display_trajectory.trajectory_start = self.robot.get_current_state()
    #         # display_trajectory.trajectory.append(self.plan)
    #         # # Publish
    #         # self.display_trajectory_publisher.publish(display_trajectory)

    #         # Execute
    #         #self.move_group.execute(self.plan, wait=True)

    #         print('OK')

    # def execute_plan(self, plan):
    #     move_group.execute(plan, wait=True)



    def repositioning(self):
        print('Repostionning start')
        rospy.loginfo('Repositioning start')

        # Set default pose
        # TODO mettre compliant false avant set pose :
        self.move_group.set_joint_value_target(orig_pose_arm)
        self.move_group.go()
        # self.move_group_af.set_joint_value_target(orig_pose_armfinger)
        # self.move_group_af.go()
        rospy.sleep(4)

        rospy.wait_for_service('get_image')

        cv2.namedWindow("Image window", 1)

        interpreter = Interpreter(
            '/home/eze/catkin_ws/src/poppy_senses/src/detect.tflite')
        interpreter.allocate_tensors()
        _, input_height, input_width, _ = interpreter.get_input_details()[
            0]['shape']

        while (not rospy.is_shutdown()):
            try:
                get_image = rospy.ServiceProxy("get_image", GetImage)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

            response = get_image()
            print('got an image')

            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(response.image)

            img = cv2.resize(cv2.cvtColor(
                image, cv2.COLOR_BGR2RGB), (320, 320))

            res = detect_objects(interpreter, img, 0.9)

            # Use the best score result only
            self.ymin = -1
            self.xmin = -1
            self.ymax = -1
            self.xmax = -1
            # res.sort(key=lambda x:x['score'])
            for result in res:
                self.ymin, self.xmin, self.ymax, self.xmax = result['bounding_box']
                self.xmin = int(max(1, self.xmin * CAMERA_WIDTH))
                self.xmax = int(min(CAMERA_WIDTH, self.xmax * CAMERA_WIDTH))
                self.ymin = int(max(1, self.ymin * CAMERA_HEIGHT))
                self.ymax = int(min(CAMERA_HEIGHT, self.ymax * CAMERA_HEIGHT))
                print('ymin, xmin, ymax, xmax :', self.ymin,
                      self.xmin, self.ymax, self.xmax)
                image = cv2.rectangle(image, (self.xmin, self.ymin),
                                      (self.xmax, self.ymax), (0, 255, 0), 1)

            mov_filter_pt1 = (int(CAMERA_WIDTH/2 - self.dx_move_threshold), int(CAMERA_HEIGHT/2 - self.dy_move_threshold))
            mov_filter_pt2 = (int(CAMERA_WIDTH/2 + self.dx_move_threshold), int(CAMERA_HEIGHT/2 + self.dy_move_threshold))
            image = cv2.rectangle(image, 
                                    mov_filter_pt1,
                                    mov_filter_pt2,
                                    (127, 127, 255),
                                     1)

            cv2.imshow('Image Window', image)
            cv2.waitKey(3)

            # rospy.loginfo('ymin, xmin, ymax, xmax : %d, %d, %d, %d', ymin, xmin, ymax, xmax)


rospy.init_node('ros4pro_custom_repositioning')

if not rospy.is_shutdown():
    poppy = MoveArm()
    poppy.repositioning()
