#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ViSP

// msgs
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>


class VisualTracking{
    private:
        ros::NodeHandle nh;

        // ros::Subscriber sub;
        // ros::Publisher pub;
        image_transport::ImageTransport img_trans;
        image_transport::Subscriber img_sub;
        image_transport::Publisher img_pub;

        std::string input_image_topic;
        std::string output_image_topic;

    public:
        VisualTracking(ros::NodeHandle node_handle)
        : img_trans(nh)
        {
            nh = node_handle;
            VisualTracking::getParametersValues();

            img_sub = img_trans.subscribe(input_image_topic, 1, &VisualTracking::imageSubCallback, this);
            img_pub = img_trans.advertise(output_image_topic, 1);

        }

        void getParametersValues(){
                nh.param<std::string>("input_image_topic", input_image_topic, "/usb_cam/image_raw");
                ROS_INFO("Input Image Topic: %s", input_image_topic.c_str());

                nh.param<std::string>("output_image_topic", output_image_topic, "/track_img");
                ROS_INFO("Output Image Topic: %s", output_image_topic.c_str());
        }

        void imageSubCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Draw an example circle on the video stream
            if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

            // Update GUI Window
            cv::imshow("Image Window", cv_ptr->image);
            cv::waitKey(3);

            // Output modified video stream
            img_pub.publish(cv_ptr->toImageMsg());
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "visual_tracking");
    ros::NodeHandle nh("~");
    VisualTracking vt(nh);
    ros::spin();
    return 0;
}