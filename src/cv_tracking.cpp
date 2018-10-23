#include <vector>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

class TrackerCV {
    private:
        ros::NodeHandle nh;

        image_transport::ImageTransport img_trans;
        image_transport::Subscriber img_sub;
        image_transport::Publisher img_pub;

        std::string input_image_topic;
        std::string output_image_topic;

        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat *frame;

        cv::Ptr<cv::Tracker> tracker;
        std::string tracker_type;

        bool initialized;

        //Define tracking bounding box
        cv::Rect2d bbox;

        void getParametersValues (){
            nh.param<std::string>("input_image_topic", input_image_topic, "/usb_cam/image_raw");
            ROS_INFO("Input Image Topic: %s", input_image_topic.c_str());

            nh.param<std::string>("output_image_topic", output_image_topic, "/tracking_image");
            ROS_INFO("Output Image Topic: %s", output_image_topic.c_str());
            
            nh.param<std::string>("tracker_type", tracker_type, "KCF");
            ROS_INFO("Output Image Topic: %s", output_image_topic.c_str());
        }

        void imageSubCallback (const sensor_msgs::ImageConstPtr& msg){

            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            frame = &cv_ptr->image;

            if(!initialized){
                // Select and initialized tracking interest
                bbox = cv::selectROI("Select Object", *frame, true, false);
                // Display bounding box.
                cv::rectangle(*frame, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );

                tracker->init(*frame, bbox);
                initialized = true;

                cv::destroyWindow("Select Object");
            }
            else{
                // Start timer
                double timer = (double)cv::getTickCount();
                
                // Update the tracking result
                bool ok = tracker->update(*frame, bbox);
                
                // Calculate Frames per second (FPS)
                float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
                
                if (ok){
                    // Tracker success : Draw the tracked object
                    cv::rectangle(*frame, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
                }
                else{
                    // Tracking failure detected.
                    cv::putText(*frame, "Tracking failure detected", cv::Point(100,80), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
                }
                
                // Display tracker type on frame
                cv::putText(*frame, tracker_type + " Tracker", cv::Point(100,20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
                
                // Display FPS on image
                cv::putText(*frame, "FPS : " + std::to_string(int(fps)), cv::Point(100,50), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);

                cv_ptr->image = *frame;

                img_pub.publish(cv_ptr->toImageMsg());
            }

        }

    public:
        TrackerCV (ros::NodeHandle node_handle)
        :img_trans (nh){
            nh = node_handle;
            TrackerCV::getParametersValues();

            img_sub = img_trans.subscribe(input_image_topic, 1, &TrackerCV::imageSubCallback, this);
            img_pub = img_trans.advertise(output_image_topic, 1);

            if (tracker_type == "BOOSTING")
                tracker = cv::TrackerBoosting::create();
            if (tracker_type == "MIL")
                tracker = cv::TrackerMIL::create();
            if (tracker_type == "KCF")
                tracker = cv::TrackerKCF::create();
            if (tracker_type == "TLD")
                tracker = cv::TrackerTLD::create();
            if (tracker_type == "MEDIANFLOW")
                tracker = cv::TrackerMedianFlow::create();
            if (tracker_type == "MOSSE")
                tracker = cv::TrackerMOSSE::create();
            // if (tracker_type == "CSRT")
            //     tracker = TrackerCSRT::create();

            bbox = cv::Rect2d(0, 0, 1, 1);
            initialized = false;
        }
};

int main (int argc, char** argv){
    ros::init (argc, argv, "obj_tracking");
    ros::NodeHandle nh ("~");
    TrackerCV tracker (nh);

    ros::spin();
    return 0;
}