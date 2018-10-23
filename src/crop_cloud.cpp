#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//msgs
#include <sensor_msgs/PointCloud2.h>
#include <arm_vs/Rect.h>

class CropCloud{
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;
        ros::Publisher pub;

        std::string input_pixel_points_topic;
        std::string input_pointcloud_topic;
        std::string output_pointcloud_topic;

        arm_vs::Rect pixel_points;
        sensor_msgs::PointCloud2 points;

        void getParametersValues(){
            nh.param<std::string>("input_pixel_points_topic", input_pixel_points_topic, "/tracked_pixel_points");
            ROS_INFO("Input Pixel Points Topic: %s", input_pixel_points_topic.c_str());

            nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/camera/depth/points");
            ROS_INFO("Input Point Cloud Topic: %s", input_pointcloud_topic.c_str());

            nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/cropped_cloud");
            ROS_INFO("Output Point Cloud Topic: %s", output_pointcloud_topic.c_str());
        }

        void sub_callback(const sensor_msgs::PointCloud2ConstPtr& ptcld, const arm_vs::Rect& rect){
            
        }

    public:
        CropCloud(ros::NodeHandle node_handle){
            nh = node_handle;
            CropCloud::getParametersValues();
    
            pub = nh.advertise<sensor_msgs::PointCloud2>(output_pointcloud_topic, 1);

            message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, input_pointcloud_topic, 1);
            message_filters::Subscriber<arm_vs::Rect> rect_sub(nh, input_pixel_points_topic, 1);
            message_filters::TimeSynchronizer<sensor_message::PointCloud2, arm_vs::Rect> sync(pointcloud_sub, rect_sub, 10);
            sync.registerCallback(boost::bind(&CropCloud::sub_callback, _1, _2))
        
            
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "crop_cloud");
    ros::NodeHandle nh("~");
    CropCloud cc(nh);
    ros::spin();
    return 0;
}