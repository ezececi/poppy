#!/usr/bin/env python3

import cv2
import rospy
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge
from tflite_runtime.interpreter import Interpreter
import numpy as np


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


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


def repositioning():
    print('Repostionning start')
    rospy.loginfo('Repositioning start')
    
    interpreter = Interpreter('/home/eze/catkin_ws/src/poppy_senses/src/detect.tflite')
    interpreter.allocate_tensors()
    _, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']

    rospy.wait_for_service('get_image')

    cv2.namedWindow("Image window", 1)
    
    while (not rospy.is_shutdown()):
        try:
            get_image = rospy.ServiceProxy("get_image", GetImage)
            response = get_image()
            print('got an image')

            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(response.image)


            img = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (320,320))
            res = detect_objects(interpreter, img, 0.8)

            # Use the best score result only
            ymin = 0
            xmin = 0
            ymax =0
            xmax = 0
            #res.sort(key=lambda x:x['score'])
            for result in res:
                ymin, xmin, ymax, xmax = result['bounding_box']
                xmin = int(max(1,xmin * CAMERA_WIDTH))
                xmax = int(min(CAMERA_WIDTH, xmax * CAMERA_WIDTH))
                ymin = int(max(1, ymin * CAMERA_HEIGHT))
                ymax = int(min(CAMERA_HEIGHT, ymax * CAMERA_HEIGHT))
                print('ymin, xmin, ymax, xmax :', ymin, xmin, ymax, xmax)
                image = cv2.rectangle(image,(xmin, ymin),(xmax, ymax),(0,255,0),1)
                
            
            cv2.imshow('Image Window', image)
            cv2.waitKey(3)

            #rospy.loginfo('ymin, xmin, ymax, xmax : %d, %d, %d, %d', ymin, xmin, ymax, xmax)
            

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        except Exception as err:
            print("Exception : %s"%err)
            exit()


if __name__ == '__main__':
    repositioning()
    exit()

rospy.init_node('ros4pro_custom_repositioning')

if not rospy.is_shutdown():
    repositioning()
