#include <vector>
#include <string>
#include <unordered_set>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "opencv2/tracking/tracker.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core/ocl.hpp"

// msgs
#include "arm_vs/BBox.h"

/**
 * workaround to CMake unable to find the correct OpenCV directory
 */
namespace cv
{
    class CV_EXPORTS_W TrackerCSRT : public Tracker
    {
    public:
    CV_WRAP static Ptr<TrackerCSRT> create();
    };
}

class TrackerCV {
    private:
        ros::NodeHandle nh;

        image_transport::ImageTransport img_trans;
        image_transport::Subscriber img_sub;
        image_transport::Publisher img_pub;
        ros::Publisher bbox_pub;

        std::string input_image_topic;
        std::string output_image_topic;
        std::string output_bbox_topic;

        cv_bridge::CvImagePtr cv_ptr;

        cv::Ptr<cv::Tracker> tracker;
        std::string tracker_type;

        bool initialized;

        //Define tracking bounding box
        cv::Rect2d bbox;

        void getParametersValues (){
            nh.param<std::string>("input_image_topic", input_image_topic, "/usb_cam/image_raw");
            // ROS_INFO("Input Image Topic: %s", input_image_topic.c_str());

            nh.param<std::string>("output_image_topic", output_image_topic, "/tracking_image");
            // ROS_INFO("Output Image Topic: %s", output_image_topic.c_str());
            
            nh.param<std::string>("tracker_type", tracker_type, "GOTURN");
            // ROS_INFO("Image Tracker Type: %s", tracker_type.c_str());

            std::unordered_set<std::string> trackers = std::unordered_set<std::string>{"KCF", "MOSSE", "CSRT", "GOTURN"};
            if(trackers.find(tracker_type) == trackers.end()){
                ROS_ERROR("Tracker %s not available !!!", tracker_type.c_str());
                exit(1);
            }

            nh.param<std::string>("output_bbox_topic", output_bbox_topic, "/tracked_object_bbox");

        }

        void imageSubCallback (const sensor_msgs::ImageConstPtr& img_msg){

            try{
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            if(!initialized){
                // Select and initialized tracking interest
                bbox = cv::selectROI("Select Object", cv_ptr->image, true, false);
                // Display bounding box.
                cv::rectangle(cv_ptr->image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );

                tracker->init(cv_ptr->image, bbox);
                initialized = true;

                cv::destroyWindow("Select Object");
            }
            else{
                // Start timer
                double timer = (double)cv::getTickCount();
                
                // Update the tracking result
                bool ok = tracker->update(cv_ptr->image, bbox);

                arm_vs::BBox bbox_msg_out;
                bbox_msg_out.header.stamp = img_msg->header.stamp;
                bbox_msg_out.x = static_cast<uint>(bbox.x);
                bbox_msg_out.y = static_cast<uint>(bbox.y);
                bbox_msg_out.width = static_cast<uint>(bbox.width);
                bbox_msg_out.height = static_cast<uint>(bbox.height);
                bbox_pub.publish(bbox_msg_out);
                
                // Calculate Frames per second (FPS)
                float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
                
                if (ok){
                    // Tracker success : Draw the tracked object
                    cv::rectangle(cv_ptr->image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
                }
                else{
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

    public:
        TrackerCV (ros::NodeHandle node_handle)
        :img_trans (node_handle){
            nh = node_handle;
            TrackerCV::getParametersValues();

            if (tracker_type == "KCF")
                tracker = cv::TrackerKCF::create();
            if (tracker_type == "MOSSE")
                tracker = cv::TrackerMOSSE::create();
            if (tracker_type == "CSRT")
                tracker = cv::TrackerCSRT::create();
            if (tracker_type == "GOTURN")
                tracker = cv::TrackerGOTURN::create();

            bbox = cv::Rect2d(0, 0, 32, 32);
            initialized = false;

            img_sub = img_trans.subscribe(input_image_topic, 1, &TrackerCV::imageSubCallback, this);
            img_pub = img_trans.advertise(output_image_topic, 1);
            bbox_pub = nh.advertise<arm_vs::BBox>(output_bbox_topic, 50);
        }
};

int main (int argc, char** argv){
    ros::init (argc, argv, "obj_tracking");
    ros::NodeHandle nh ("~");
    TrackerCV tracker (nh);

    ros::spin();
    return 0;
}