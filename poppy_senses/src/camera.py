#!/usr/bin/env python3

# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import cv2
import rospy
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge
import datetime


def camera():
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, store poppy camera image')  # Press Ctrl+F8 to toggle the breakpoint.

    get_image = rospy.ServiceProxy("get_image", GetImage)
    response = get_image()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(response.image)
    cv2.imshow("Poppy camera", image)
    cv2.waitKey(2000)
    time_str = datetime.datetime.now().strftime("%m%d_%H%M%S")
    cv2.imwrite("images/cible_{}.png".format(time_str), image)
    print('done')


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    camera()
    exit()

rospy.init_node('ros4pro_custom_camera')

if not rospy.is_shutdown():
    camera()






# See PyCharm help at https://www.jetbrains.com/help/pycharm/
