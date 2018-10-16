#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ViSP
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpException.h>
// #include <visp3/core/vpColor.h>
// #include <visp3/core/vpCameraParameters.h>
// #include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpDisplayOpenCV.h>
// ViSP trackers
// #include <visp3/me/vpMe.h>
// #include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/mbt/vpMbEdgeTracker.h>

// msgs
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>


class VisualTracking{
    public:
        ros::NodeHandle nh;

        image_transport::ImageTransport img_trans;
        image_transport::Subscriber img_sub;
        image_transport::Publisher img_pub;

        std::string input_image_topic;
        std::string output_image_topic;

        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat *mat_image;

        cv::Mat grey_mat;
        vpImage<unsigned char> I;

        bool first_image = false;

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
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            mat_image = &cv_ptr->image;

            cv::cvtColor(*mat_image, grey_mat, CV_BGR2GRAY);
            vpImageConvert::convert(grey_mat, I, true);

            first_image = true;
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "visual_tracking");
    ros::NodeHandle nh("~");
    VisualTracking vt(nh);

    bool initialized = false;

    std::string objectname = "/home/williamshwang/workspace/catkin_ws/src/arm_vs/visp_models/25mm_cube";

    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;

    vpDisplay *display = NULL;
    display = new vpDisplayOpenCV;

    vpMbTracker *tracker;
    tracker = new vpMbEdgeTracker;
    
    vpMe me;
    me.setMaskSize(5);
    me.setMaskNumber(180);
    me.setRange(8);
    me.setThreshold(0.4);
    me.setMu1(0.5);
    me.setMu2(0.5);
    me.setSampleStep(4);
    dynamic_cast<vpMbEdgeTracker *>(tracker)->setMovingEdge(me);

    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    tracker->setCameraParameters(cam);

    tracker->loadModel(objectname + ".cao");
    tracker->setDisplayFeatures(true);

    while(ros::ok()){
        if(vt.first_image && !initialized){
            display->init(vt.I, 100, 100, "Model-based tracker");
            tracker->initClick(vt.I, objectname + ".init", true);
            initialized = true;
        }

        if(initialized){
            vpDisplay::display(vt.I);
            tracker->track(vt.I);
            tracker->getPose(cMo);
            tracker->getCameraParameters(cam);
            tracker->display(vt.I, cMo, cam, vpColor::red, 2, true);
            vpDisplay::displayFrame(vt.I, cMo, cam, 0.025, vpColor::none, 3);
            vpDisplay::displayText(vt.I, 10, 10, "A click to exit...", vpColor::red);
            vpDisplay::flush(vt.I);

            vpImageConvert::convert(vt.I, *(vt.mat_image), true);
                
            // Output modified video stream
            vt.img_pub.publish(vt.cv_ptr->toImageMsg());

            if (vpDisplay::getClick(vt.I, false))
                break;
        }

        ros::spinOnce();
    }

    vpDisplay::getClick(vt.I);
    delete display;
    delete tracker;

    // ros::spin();
    return 0;
}