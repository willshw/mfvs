#!/usr/bin/env python


import sys
#FIXME cv2 importing from opencv3.4.3 path, ros-kinetic-opencv version 3.3.1 still installed
sys.path.insert(1, "/usr/local/lib/python2.7/dist-packages")

import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from arm_vs.msg import Rect
from cv_bridge import CvBridge, CvBridgeError

class Tracker:
    
    def __init__(self):
        self.input_image_topic = rospy.get_param("~input_image_topic", "/usb_cam/image_raw")
        rospy.loginfo("Input Image Topic: %s", self.input_image_topic)

        self.output_image_topic = rospy.get_param("~output_image_topic", "/tracking_image")
        rospy.loginfo("Output Image Topic: %s", self.output_image_topic)

        self.tracked_obj_pixels_topic = rospy.get_param("~tracked_obj_pixels_topic", "/tracked_obj_pixels")
        rospy.loginfo("Tracked Object Pixels Topic: %s", self.tracked_obj_pixels_topic)

        self.image_pub = rospy.Publisher(self.output_image_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.input_image_topic, Image, self.imageSubCallback)
        self.image_bridge = CvBridge()
        self.tracked_obj_bbox_pub = rospy.Publisher(self.tracked_obj_pixels_topic, Rect, queue_size=10)

        self.image_tracker_type = rospy.get_param("~tracker_type", "CSRT")
        rospy.loginfo("Image Tracker Type: %s", self.image_tracker_type)

        self.ROI_initialized = False
        self.image_tracking_bbox = (287, 23, 86, 320)

        if self.image_tracker_type == 'BOOSTING':
            self.tracker = cv2.TrackerBoosting_create()

        if self.image_tracker_type == 'MIL':
            self.tracker = cv2.TrackerMIL_create()

        if self.image_tracker_type == 'KCF':
            self.tracker = cv2.TrackerKCF_create()

        if self.image_tracker_type == 'TLD':
            self.tracker = cv2.TrackerTLD_create()

        if self.image_tracker_type == 'MEDIANFLOW':
            self.tracker = cv2.TrackerMedianFlow_create()

        if self.image_tracker_type == 'GOTURN':
            self.tracker = cv2.TrackerGOTURN_create()

        if self.image_tracker_type == 'MOSSE':
            self.tracker = cv2.TrackerMOSSE_create()

        if self.image_tracker_type == "CSRT":
            self.tracker = cv2.TrackerCSRT_create()

    def imageSubCallback(self, data):
        try:
            cv_image = self.image_bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # process image
        if not self.ROI_initialized:
            self.image_tracking_bbox = cv2.selectROI(cv_image, False)
            self.ROI_initialized = True
            self.tracker.init(cv_image, self.image_tracking_bbox)
            cv2.destroyAllWindows()

        else:
            # Start timer
            timer = cv2.getTickCount()
    
            # Update tracker
            ok, self.image_tracking_bbox = self.tracker.update(cv_image)
    
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    
            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(self.image_tracking_bbox[0]), int(self.image_tracking_bbox[1]))
                p2 = (int(self.image_tracking_bbox[0] + self.image_tracking_bbox[2]),
                        int(self.image_tracking_bbox[1] + self.image_tracking_bbox[3]))
                cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(cv_image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
            # Display tracker type on frame
            cv2.putText(cv_image, self.image_tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        
            # Display FPS on frame
            cv2.putText(cv_image, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

            try:
                self.image_pub.publish(self.image_bridge.cv2_to_imgmsg(cv_image, "mono8"))
            except CvBridgeError as e:
                rospy.logerr(e)

            tracked_pixel_bbox = Rect()
            tracked_pixel_bbox.x = self.image_tracking_bbox[0]
            tracked_pixel_bbox.y = self.image_tracking_bbox[1]
            tracked_pixel_bbox.width = self.image_tracking_bbox[2]
            tracked_pixel_bbox.height = self.image_tracking_bbox[3]
            
            try:
                self.tracked_obj_bbox_pub.publish(tracked_pixel_bbox)
            except CvBridgeError as e:
                rospy.logerr(e)

def main(args):
    rospy.init_node("image_tracker", anonymous=True)
    
    # d = rospy.Duration(2, 0)
    # rospy.sleep(d)

    image_tracker = Tracker()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)