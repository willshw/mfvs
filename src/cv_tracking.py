#!/usr/bin/env python


import sys
#FIXME cv2 importing from opencv3.4.3 path, ros-kinetic-opencv version 3.3.1 still installed
sys.path.insert(1, "/usr/local/lib/python2.7/dist-packages")

import cv2
import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
# from arm_vs.msg import Rect

from cv_bridge import CvBridge, CvBridgeError
import struct
import message_filters
import dynamic_reconfigure.client

fmt_full = ''

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

_NP_TYPES = {
    np.dtype('uint8')   :   (PointField.UINT8,  1),
    np.dtype('int8')    :   (PointField.INT8,   1),
    np.dtype('uint16')  :   (PointField.UINT16, 2),
    np.dtype('int16')   :   (PointField.INT16,  2),
    np.dtype('uint32')  :   (PointField.UINT32, 4),
    np.dtype('int32')   :   (PointField.INT32,  4),
    np.dtype('float32') :   (PointField.FLOAT32,4),
    np.dtype('float64') :   (PointField.FLOAT64,8)
}

class Tracker:
    
    def __init__(self):
        self.input_pointcloud_topic = rospy.get_param("~input_pointcloud_topic", "/camera/depth/points")
        rospy.loginfo("Input Point Cloud Topic: %s", self.input_pointcloud_topic)

        self.input_image_topic = rospy.get_param("~input_image_topic", "/usb_cam/image_raw")
        rospy.loginfo("Input Image Topic: %s", self.input_image_topic)

        self.output_image_topic = rospy.get_param("~output_image_topic", "/tracking_image")
        rospy.loginfo("Output Image Topic: %s", self.output_image_topic)

        self.ptcld_sub = message_filters.Subscriber(self.input_pointcloud_topic, PointCloud2)
        self.image_sub = message_filters.Subscriber(self.input_image_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.ptcld_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.image_pub = rospy.Publisher(self.output_image_topic, Image, queue_size=1)
        self.image_bridge = CvBridge()
        # self.tracked_obj_bbox_pub = rospy.Publisher(self.tracked_obj_pixels_topic, Rect, queue_size=10)

        self.image_tracker_type = rospy.get_param("~tracker_type", "CSRT")
        rospy.loginfo("Image Tracker Type: %s", self.image_tracker_type)

        self.ROI_initialized = False
        self.image_tracking_bbox = (287, 23, 86, 320)

        self.client = dynamic_reconfigure.client.Client("cropbox")

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

    def pointcloud2_to_array(self, msg):
        global fmt_full
        if not fmt_full:
            fmt = self._get_struct_fmt(msg)
            fmt_full = '>' if msg.is_bigendian else '<' + fmt.strip('<>')*msg.width*msg.height
        # import pdb; pdb.set_trace()
        unpacker = struct.Struct(fmt_full)
        unpacked = np.asarray(unpacker.unpack_from(msg.data))
        return unpacked
        # unpacked.reshape(msg.height, msg.width, len(msg.fields))

    def _get_struct_fmt(self, cloud, field_names=None):
        fmt = '>' if cloud.is_bigendian else '<'
        offset = 0
        for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt

    def pixelTo3DPoint(self, u, v, pointcloud):
        arrayPos = v*pointcloud.row_step + u*pointcloud.point_step

        arrayPos_X = arrayPos + pointcloud.fields[0].offset
        arrayPos_Y = arrayPos + pointcloud.fields[1].offset
        arrayPos_Z = arrayPos + pointcloud.fields[2].offset

        xb = pointcloud.data[arrayPos_X:arrayPos_X+4]
        X = struct.unpack('f', xb)

        yb = pointcloud.data[arrayPos_Y:arrayPos_Y+4]
        Y = struct.unpack('f', yb)

        zb = pointcloud.data[arrayPos_Z:arrayPos_Z+4]
        Z = struct.unpack('f', zb)

        return X[0], Y[0], Z[0]

    def callback(self, image, pointcloud):

        try:
            cv_image = self.image_bridge.imgmsg_to_cv2(image, "mono8")
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

            # Set Point Cloud Crop Boundary
            pointcloud_x_min = 0.0
            pointcloud_x_max = 0.0
            pointcloud_y_min = 0.0
            pointcloud_y_max = 0.0

            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(self.image_tracking_bbox[0]), int(self.image_tracking_bbox[1]))
                p2 = (int(self.image_tracking_bbox[0] + self.image_tracking_bbox[2]),
                        int(self.image_tracking_bbox[1] + self.image_tracking_bbox[3]))
                cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)

                # Set Point Cloud Crop Box
                pointcloud_x_min, pointcloud_y_min, _ = self.pixelTo3DPoint(p1[0], p1[1], pointcloud)
                pointcloud_x_max, pointcloud_y_max, _ = self.pixelTo3DPoint(p2[0], p2[1], pointcloud)

            else :
                # Tracking failure
                cv2.putText(cv_image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

            # Reconfigure Point Cloud Crop Box nodelet
            params = {"min_x": pointcloud_x_min, "max_x": pointcloud_x_max, "min_y": pointcloud_y_min, "max_y": pointcloud_y_max}
            config = self.client.update_configuration(params)

            # Display tracker type on frame
            cv2.putText(cv_image, self.image_tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        
            # Display FPS on frame
            cv2.putText(cv_image, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

            try:
                self.image_pub.publish(self.image_bridge.cv2_to_imgmsg(cv_image, "mono8"))
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