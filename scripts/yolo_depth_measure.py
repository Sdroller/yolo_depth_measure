#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import *
from darknet_ros.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from numpy import inf

topic_depth_image = '/zed/depth/depth_registered' #Image: 32-bit depth values in meters
topic_bounding_box = 'YOLO_bboxes'
img_height = 672
img_width =  376
distance_to_person = 0
print_distance_arrays = False # Setting this to true will print arrays used to detect distance to person.

class distance_detection:
    def __init__(self):
        # We use time TimeSynchronizer to sub to multiple topics
        self.bbox_sub = message_filters.Subscriber(topic_bounding_box, bbox_array)
        self.depth_sub = message_filters.Subscriber(topic_depth_image, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.bbox_sub, self.depth_sub], 5, 1, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        # We publish the distance detected
        self.dist_pub = rospy.Publisher('distance_to_person', Float32, queue_size=10)



    def callback(self, bbox_array, zed_depth_img):

        person_detected = 0         # Value of 1 means a person has been detected
        centroid_bbox = [5000,5000]
        cv_depth_image = 0
        threshold_confidence = 0.6  # prob of class must be greater than this threshold for us to use it.
        last_confidence = 0.0       # used to select object with highest prob of "person"
        detected_person_bbox = [0,0,0,0]

        # Get the depth img
        if(zed_depth_img.encoding == '32FC1'):
            bridge = CvBridge()
            try:
              cv_depth_image = bridge.imgmsg_to_cv2(zed_depth_img, "32FC1")
            except CvBridgeError as e:
              print(e)
              return
        else:
            rospy.logerror("ERROR: Depth Images from Zed are not in 32FC1 format")
            return

        # Get the bounding box of detected person, if any
        rospy.loginfo("Num of Classes detected by Yolo: %d" % (len(bbox_array.bboxes)))
        for detected_object in bbox_array.bboxes:
            if(detected_object.Class == "person" and detected_object.prob > threshold_confidence):
                person_detected = 1
                # Keep only object with highest confidence of "person"
                if(detected_object.prob > last_confidence):
                    detected_person_bbox = [detected_object.xmin, detected_object.ymin, \
                                            detected_object.xmax, detected_object.ymax]
                    last_confidence = detected_object.prob


        if( person_detected == 1 ):
            # Calculate distance to person based on mean of 10px around the centroid of bounding box
            window_dim = int(10/2) #the width of window around centroid which we sample for depth
            centroid_bbox[0] = (detected_person_bbox[0] + detected_person_bbox[2])/2
            centroid_bbox[1] = (detected_person_bbox[1] + detected_person_bbox[3])/2

            centroid_bbox[0] = np.clip(centroid_bbox[0], (0 + window_dim), (img_width  - window_dim) )
            centroid_bbox[1] = np.clip(centroid_bbox[1], (0 + window_dim), (img_height - window_dim) )
            mean_depth_image = cv_depth_image[centroid_bbox[1]-window_dim:centroid_bbox[1]+window_dim,\
                                              centroid_bbox[0]-window_dim:centroid_bbox[0]+window_dim]

            # Clean the values and take avg
            mean_depth_image_clean = np.copy(mean_depth_image)
            mean_depth_image_clean[~np.isfinite(mean_depth_image)] = 0
            distance_to_person = np.mean(mean_depth_image_clean)

            #Publish the distance
            self.dist_pub.publish(distance_to_person)


            rospy.loginfo("Person Detected!\n Bounding Box:\t (%d,%d), (%d,%d)\n Centroid of Box:\t %d,%d\n \
    Prediction Prob:\t %0.3f\n Distance to person:\t%0.3f m\n\n",\
            detected_person_bbox[0], detected_person_bbox[1], detected_person_bbox[2], detected_person_bbox[3],\
            centroid_bbox[1], centroid_bbox[0], last_confidence, distance_to_person)

            if(print_distance_arrays):
                print("Raw mean depth img:")
                print np.array_str(mean_depth_image, precision=2)
                print("\n\nClean depth img:")
                print np.array_str(mean_depth_image_clean, precision=2)
                print("\n\n")
        else:
            rospy.loginfo("No Person Detected...")


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('yolo_depth_measure_node', anonymous=False)
    rospy.loginfo("Yolo Depth Measure node started")
    dt = distance_detection()
    
    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

