#!/usr/bin/env python
from __future__ import print_function
import time
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import glob
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/cv_image",Image)
    self.distance_pub = rospy.Publisher("/relative_distance",Float32MultiArray,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris/image_raw",Image,self.callback)
    self.lostnumber=0
    
  def callback(self,data):
    try:
      # tranform ROS image message into opencv image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    global  ret, mtx, dist, rvecs, tvecs

    # aruco basic setting
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # convert the image 
    cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # detect maker configuraiton
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image_gray, aruco_dict, parameters=parameters)
    if np.all(ids != None):
      self.lostnumber=0
      id=ids[0][0]
      lock_number=0
      for i in range(ids.size):
        if ids[i][0]>id:
          id=ids[i][0]
          lock_number=i
      markersize=0
      if id==1:
        markersize=0.139
      elif id==2:
        markersize=0.071
      elif id==3:
        markersize=0.0325
      elif id==4:
        markersize=0.016
      
      # pose estimation
      # 0.19: markerLength, mtx: cameraMatrix, dist: distortion coefficients
      # rvec: 
      rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[lock_number], markersize, mtx, dist)
      

      # read corners informatio
      topleftX = corners[0][0][0][0]
      topleftY = corners[0][0][0][1]

      toprightX = corners[0][0][1][0]
      toprightY = corners[0][0][1][1]

      bottomleftX = corners[0][0][2][0]
      bottomlextY = corners[0][0][2][1]

      bottomrightX = corners[0][0][3][0]
      bottomrightY = corners[0][0][3][1]
      
      # get pose information
      cor_x = tvec[0][0][0]
      cor_y = tvec[0][0][1]
      cor_z = tvec[0][0][2]
      print("x=",cor_x)
      print("y=",cor_y)
      print("z=",cor_z)
      
      midpointX = (topleftX  + bottomrightX)/2
      midpointY = (topleftY + bottomrightY)/2

      # draw axis and detected marker
      aruco.drawAxis(cv_image, mtx, dist, rvec[0], tvec[0], 0.1)
      aruco.drawDetectedMarkers(cv_image, corners)

      # draw ID text on top of image
      font = cv2.FONT_HERSHEY_SIMPLEX
      cv2.putText(cv_image, "X: {}".format(cor_x), (0,364), font, 1, (0,255,0),2,cv2.LINE_AA)
      cv2.putText(cv_image, "Y: {}".format(cor_y), (0,400), font, 1, (0,255,0),2,cv2.LINE_AA)
      cv2.putText(cv_image, "Z: {}".format(cor_z), (0,436), font, 1, (0,255,0),2,cv2.LINE_AA)

      # incorporate pose information together and print on image
      dis=Float32MultiArray()
      #dis.layout=(3,1)
      dis.data=(cor_x, cor_y, cor_z)

      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)

      # Node Publish - pose information
      self.distance_pub.publish(dis)

      # Node Publish - cv_image
      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      except CvBridgeError as e:
        print(e)
        
    else:
      self.lostnumber+=1
      if self.lostnumber>100:
        dis = Float32MultiArray()
        dis.data = (float('nan'), float('nan'), float('nan'))
        self.distance_pub.publish(dis)
        

def main(args):
    global  ret, mtx, dist, rvecs, tvecs

    # cameraMatrix and distortion coefficients
    mtx = np.array([[552.1877667, 0. , 289.37921553], [ 0. , 550.98791255, 228.87373308], [0. , 0. , 1.]])
    dist = np.array([[ 1.51623055e-03, 9.03278089e-02, 6.51492926e-03, -9.21777965e-05, -4.29890497e-01]])
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
