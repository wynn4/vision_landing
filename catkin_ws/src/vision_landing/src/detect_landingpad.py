#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    #create blob detector parameter object
    self.params = cv2.SimpleBlobDetector_Params()
    #filter by size
    self.params.filterByArea = True
    self.params.minArea = 100
    #filter by circularity
    self.params.filterByCircularity = True
    self.params.minCircularity = 0.5
    #filter by convexity
    self.params.filterByConvexity = True
    self.params.minConvexity = 0.87
    #filter by inertia
    self.params.filterByInertia = True
    self.params.minInertiaRatio = 0.35
    #create blob detector with params object
    self.blob_detect = cv2.SimpleBlobDetector(self.params)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ###############################################
    # Jesse,,,, put your image processing code here

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #__BEGIN PLAIN THRESHOLD METHOD__
    #convert to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #threshold
    _, gray_thresh = cv2.threshold(gray, 230, 255, 1)
    #erode to kill the noise
    #kernel = np.zeros((11,11),np.uint8)
    #gray_erode = cv2.erode(gray_thresh,kernel,iterations = 1)

    #detect blobs and get keypoints
    keypoints = self.blob_detect.detect(gray_thresh)

    if len(keypoints) == 1:
        for p in keypoints:
            x_coord = p.pt[0]
            y_coord = p.pt[1]

            #shift coordinates to be relative to center
            x_coord = x_coord - 320
            y_coord = y_coord - 240


    #draw keypoints
    img_w_keypoints = cv2.drawKeypoints(cv_image,keypoints,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #__END PLAIN THRESHOLD METHOD__

    cv2.imshow("Image window", img_w_keypoints)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_w_keypoints, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
