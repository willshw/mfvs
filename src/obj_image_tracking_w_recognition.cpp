/**
 * This node uses OpenCV provided tracker to track object of interest in the rgb image
 * with object recognition infomation and publish the bounding box of the tracked object
 * for point cloud cropping.
 * Object recognition information helps with reinitializing tracker when tracker operates
 * for too long and start drifting. Also, helps with signaling tracker if object is still
 * in frame in some cases tracker will transfer to other objects when object exits frame. 
 * 
 * flowchat_obj_image_recog_track.md illustrates the flow of this node.
 */

#include <vector>
#include <iterator>
#include <string>
#include <unordered_set>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "opencv2/tracking/tracker.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core/ocl.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "boost/shared_ptr.hpp"
#include "boost/bind.hpp"

// msgs
#include "arm_vs/BBox.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "object_msgs/ObjectInBox.h"
#include "object_msgs/ObjectsInBoxes.h"

/**
 * FIXME: workaround to CMake unable to find the correct OpenCV directory
 */
namespace cv
{
    class CV_EXPORTS_W TrackerCSRT : public Tracker
    {
    public:
    CV_WRAP static Ptr<TrackerCSRT> create();
    };
}

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, object_msgs::ObjectsInBoxes> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

struct ROIBox{
    int x;
    int y;
    int width;
    int height;
};

class TrackerCV {
    private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<object_msgs::ObjectsInBoxes> obj_inf_sub;
    boost::shared_ptr<Sync> sync;

    image_transport::ImageTransport img_trans;
    image_transport::Publisher img_pub;
    ros::Publisher bbox_pub;

    std::string input_image_topic;
    std::string input_obj_inf_topic;
    std::string obj_of_interest;
    std::string tracker_type;
    std::string output_image_topic;
    std::string output_bbox_topic;

    bool initialized; // status indicating if tracker is initialized
    int tracked_frame_counter;
    int tracked_frame_limit;
    int missing_obj_frame_counter;
    int missing_obj_frame_limit;

    cv::Rect2d bbox; // bounding box for tracked object in image
    cv_bridge::CvImagePtr cv_ptr;
    cv::Ptr<cv::Tracker> tracker;

    /**
     * Getting parameters
     */
    void getParametersValues ()
    {
        nh.param<std::string>("input_image_topic", input_image_topic, "/usb_cam/image_raw");
        nh.param<std::string>("input_obj_inf_topic", input_obj_inf_topic, "/opencl_caffe/inference");
        nh.param<std::string>("obj_of_interest", obj_of_interest, "person");
        nh.param<std::string>("tracker_type", tracker_type, "GOTURN");
        std::unordered_set<std::string> trackers = std::unordered_set<std::string>{"KCF", "MOSSE", "CSRT", "GOTURN"};
        if(trackers.find(tracker_type) == trackers.end())
        {
            ROS_ERROR("Tracker %s not available !!!", tracker_type.c_str());
            exit(1);
        }
        nh.param<int>("tracked_frame_limit", tracked_frame_limit, 10);
        nh.param<int>("missing_obj_frame_limit", missing_obj_frame_limit, 10);
        nh.param<std::string>("output_image_topic", output_image_topic, "/tracking_image");
        nh.param<std::string>("output_bbox_topic", output_bbox_topic, "/tracked_object_bbox");
    }

    /**
     * Function to reset tracker when tracked frame number exceeds limit or
     * object cannot be detected in the scene (no object recognition inference input)
     */
    void resetTracker()
    {
        initialized = false;
        tracked_frame_counter = 0;

        tracker.release();

        if (tracker_type == "KCF")
            tracker = cv::TrackerKCF::create();
        if (tracker_type == "MOSSE")
            tracker = cv::TrackerMOSSE::create();
        if (tracker_type == "CSRT")
            tracker = cv::TrackerCSRT::create();
        if (tracker_type == "GOTURN")
            tracker = cv::TrackerGOTURN::create();
    }

    /**
     * Image subscriber callback subscribes the latest image and track the object of interest in the image
     * and publish the bounding box as customized msg BBox.
     */
    void SyncImgObjInfCallback (const sensor_msgs::ImageConstPtr& img_msg, const object_msgs::ObjectsInBoxes::ConstPtr& obj_inf_boxes)
    {   
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /**
         * FIXME: Only work in the scene if object of interest is the only one in the scene
         * write more to work for more object, better object recognition also needed.
         */
        ROIBox *roi = nullptr;
        for(int i = 0; i < obj_inf_boxes->objects_vector.size(); i++)
        {   
            if(obj_inf_boxes->objects_vector[i].object.object_name == obj_of_interest){
                roi = new ROIBox();
                roi->x = obj_inf_boxes->objects_vector[i].roi.x_offset;
                roi->y = obj_inf_boxes->objects_vector[i].roi.y_offset;
                roi->width = obj_inf_boxes->objects_vector[i].roi.width;
                roi->height = obj_inf_boxes->objects_vector[i].roi.height;
                break;
            }
        }

        if(roi != nullptr)
        {
            missing_obj_frame_counter = 0;

            if(tracked_frame_counter > tracked_frame_limit - 1)
            {
                TrackerCV::resetTracker();
            }
        }
        else
        {   
            missing_obj_frame_counter++;
        }

        if(missing_obj_frame_counter < missing_obj_frame_limit){

            if(!initialized)
            {
                // Display bounding box.
                bbox = cv::Rect2d(roi->x, roi->y, roi->width, roi->height);
                cv::rectangle(cv_ptr->image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );

                tracker->init(cv_ptr->image, bbox);
                initialized = true;
            }
            else
            {
                // Start timer
                double timer = (double)cv::getTickCount();
                
                // Update the tracking result
                bool ok = tracker->update(cv_ptr->image, bbox);
                tracked_frame_counter++;

                // Assemble output bounding box message
                arm_vs::BBox bbox_msg_out;
                bbox_msg_out.header.stamp = img_msg->header.stamp;
                bbox_msg_out.x = static_cast<uint>(bbox.x);
                bbox_msg_out.y = static_cast<uint>(bbox.y);
                bbox_msg_out.width = static_cast<uint>(bbox.width);
                bbox_msg_out.height = static_cast<uint>(bbox.height);
                bbox_pub.publish(bbox_msg_out);
                
                // Calculate Frames per second (FPS)
                float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
                
                if (ok)
                {
                    // Tracker success : Draw the tracked object
                    cv::rectangle(cv_ptr->image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
                }
                else
                {
                    // Tracking failure detected.
                    cv::putText(cv_ptr->image, "Tracking failure detected", cv::Point(100,80), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
                }
                
                // Display tracker type on cv_ptr->image
                cv::putText(cv_ptr->image, tracker_type + " Tracker", cv::Point(100,20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
                
                // Display FPS on image
                cv::putText(cv_ptr->image, "FPS : " + std::to_string(int(fps)), cv::Point(100,50), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);

                img_pub.publish(cv_ptr->toImageMsg());
            }
        }
        else
        {   
            arm_vs::BBox bbox_msg_out;
            bbox_msg_out.header.stamp = img_msg->header.stamp;
            bbox_msg_out.x = 0;
            bbox_msg_out.y = 0;
            bbox_msg_out.width = 0;
            bbox_msg_out.height = 0;
            bbox_pub.publish(bbox_msg_out);
            img_pub.publish(cv_ptr->toImageMsg());

            if(initialized){
                TrackerCV::resetTracker();
            }
        }
    }

    public:
    TrackerCV (ros::NodeHandle node_handle):img_trans (node_handle)
    {
        nh = node_handle;
        TrackerCV::getParametersValues();

        initialized = false;
        tracked_frame_counter = tracked_frame_limit;
        missing_obj_frame_counter = 0;

        bbox = cv::Rect2d(0, 0, 32, 32);

        if (tracker_type == "KCF")
            tracker = cv::TrackerKCF::create();
        if (tracker_type == "MOSSE")
            tracker = cv::TrackerMOSSE::create();
        if (tracker_type == "CSRT")
            tracker = cv::TrackerCSRT::create();
        if (tracker_type == "GOTURN")
            tracker = cv::TrackerGOTURN::create();

        image_sub.subscribe(nh, input_image_topic, 10);
        obj_inf_sub.subscribe(nh, input_obj_inf_topic, 10);

        sync.reset(new Sync(MySyncPolicy(10), image_sub, obj_inf_sub));
        sync->registerCallback(boost::bind(&TrackerCV::SyncImgObjInfCallback, this, _1, _2));

        img_pub = img_trans.advertise(output_image_topic, 1);
        bbox_pub = nh.advertise<arm_vs::BBox>(output_bbox_topic, 60);
    }
};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "obj_tracking");
    ros::NodeHandle nh ("~");
    TrackerCV tracker (nh);

    ros::spin();
    return 0;
}