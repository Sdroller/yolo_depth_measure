#!/usr/bin/env python
import rospy
import message_filters
from darknet_ros.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


topic_depth_image = '/zed/depth/depth_registered' #Image: 32-bit depth values in meters
topic_bounding_box = 'YOLO_bboxes'

def callback(bbox_array, zed_depth_img):
    #rospy.loginfo("Num of Classes detected:%d" % (len(bbox_array.bboxes)))
    person_detected = 0
    show_depth_image = 0
    centroid_bbox = [5000,5000]
    cv_depth_image = 0
    threshold_confidence = 0.6 # prob of class must be greater than this threshold.
    last_confidence = 0.0   # used to select object with highest prob of "person"

    # This is causing the node to get stuck, at least on ssh with X-fwding
    if(show_depth_image == 1):
        cv2.imshow("Image window", cv_depth_image)
        cv2.waitKey(1)

    for i in bbox_array.bboxes:
        if(i.Class == "person" and i.prob>threshold_confidence ):
            person_detected = 1
            if(i.prob > last_confidence): #select object with highest confidence of "person"
                centroid_bbox = [(i.xmin + i.xmax)/2, (i.ymin + i.ymax)/2]
                last_confidence = i.prob


    if(zed_depth_img.encoding == '32FC1'):
        bridge = CvBridge()
        try:
          cv_depth_image = bridge.imgmsg_to_cv2(zed_depth_img, "32FC1")
        except CvBridgeError as e:
          print(e)
    else:
        rospy.logerror("ERROR: Depth Images from Zed are not in 32FC1, something is wrong")


    if(person_detected == 1 ):
        rospy.loginfo("Person Detected!\n Prediction Prob:\t %f\n Centroid of Box:\t %d, %d\n Distance to person:\t%f",\
            i.prob, centroid_bbox[1], centroid_bbox[0], cv_depth_image[centroid_bbox[1]][centroid_bbox[0]])


def listener():
    rospy.init_node('custom_listener', anonymous=False)
    #rospy.Subscriber(topic_bounding_box, bbox_array, callback)
    rospy.loginfo("Listener for Yolo Depth Control Started")

    # We use time TimeSynchronizer to sub to multiple topics
    bbox_sub = message_filters.Subscriber(topic_bounding_box, bbox_array)
    depth_sub = message_filters.Subscriber(topic_depth_image, Image)
    ts = message_filters.ApproximateTimeSynchronizer([bbox_sub, depth_sub], 5, 1, allow_headerless=True)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
