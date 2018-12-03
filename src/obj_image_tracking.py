#!/usr/bin/env python

import sys
#FIXME cv2 importing from opencv3.4.3 path, ros-kinetic-opencv version 3.3.1 still installed
sys.path.insert(1, "/usr/local/lib/python2.7/dist-packages")

import rospy
import message_filters
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from pcl_msgs.msg import PointIndices

class Tracker:
    
    def __init__(self):
        # Getting topics
        self.input_pointcloud_topic = rospy.get_param("~input_pointcloud_topic", "/camera/depth/points")
        self.input_image_topic = rospy.get_param("~input_image_topic", "/usb_cam/image_raw")
        self.output_image_topic = rospy.get_param("~output_image_topic", "/tracking_image")
        self.output_indices_topic = rospy.get_param("~output_indices_topic", "tracked_object_points_indices")

        # Subscribers
        self.ptcld_sub = message_filters.Subscriber(self.input_pointcloud_topic, PointCloud2)
        self.image_sub = message_filters.Subscriber(self.input_image_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.ptcld_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.callback)

        # Publishers
        self.tracked_points_indices_pub = rospy.Publisher(self.output_indices_topic, PointIndices, queue_size=1)
        self.image_pub = rospy.Publisher(self.output_image_topic, Image, queue_size=1)
        self.image_bridge = CvBridge()

        # OpenCV tracker
        self.image_tracker_type = rospy.get_param("~tracker_type", "CSRT")
        rospy.loginfo("Image Tracker Type: %s", self.image_tracker_type)
        if self.image_tracker_type not in ["KCF", "MOSSE", "CSRT", "GOTURN"]:
            exit(0)

        self.ROI_initialized = False
        self.image_tracking_bbox = (0, 0, 639, 479)

        if self.image_tracker_type == 'KCF':
            self.tracker = cv2.TrackerKCF_create()

        if self.image_tracker_type == 'MOSSE':
            self.tracker = cv2.TrackerMOSSE_create()

        if self.image_tracker_type == "CSRT":
            self.tracker = cv2.TrackerCSRT_create()

        if self.image_tracker_type == "GOTURN":
            self.tracker = cv2.TrackerGOTURN_create()

    def getPointIndices(self, p1, p2, image_res_w, image_res_h):
        height = p2[1] - p1[1] + 1
        width = p2[0] - p1[0] + 1

        x = np.linspace(p1[0] + p1[1] * image_res_w, p2[0] + p1[1] * image_res_w, width)
        y = np.linspace(0, (p2[1] - p1[1]) * image_res_w, height)

        xv, yv = np.meshgrid(x, y)

        grid = np.int32(xv + yv)
        indices = grid.ravel()

        return indices

    def callback(self, image, pointcloud):

        try:
            cv_image = self.image_bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # shrink image for faster tracking
        cv_image = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)

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
                # Calculate bounding box's top left and bottom right pixel location
                p1 = np.array([int(self.image_tracking_bbox[0]), int(self.image_tracking_bbox[1])])
                p2 = np.array([int(self.image_tracking_bbox[0] + self.image_tracking_bbox[2]),
                        int(self.image_tracking_bbox[1] + self.image_tracking_bbox[3])])

                # find all points indices according to the pixel surrounded by the bounding box
                tracked_point_indices = self.getPointIndices(p1*2, p2*2, 640, 480)
                extract_indices = PointIndices()
                extract_indices.header = pointcloud.header
                extract_indices.indices = tracked_point_indices

                try:
                    self.tracked_points_indices_pub.publish(extract_indices)
                except CvBridgeError as e:
                    rospy.logerr(e)

                cv2.rectangle(cv_image, tuple(p1), tuple(p2), (255,0,0), 2, 1)

            else :
                # Tracking failure
                cv2.putText(cv_image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

            # Display tracker type on frame
            cv2.putText(cv_image, self.image_tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        
            # Display FPS on frame
            cv2.putText(cv_image, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

            img_msg = self.image_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            img_msg.header.frame_id = image.header.frame_id

            try:
                self.image_pub.publish(img_msg)
            except CvBridgeError as e:
                rospy.logerr(e)

def main(args):
    rospy.init_node("image_tracker", anonymous=True)

    image_tracker = Tracker()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)