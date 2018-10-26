#include <vector>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//msgs
#include <sensor_msgs/PointCloud2.h>
#include <arm_vs/Rect.h>

class CropCloud{
    private:
        ros::NodeHandle nh;

        ros::Publisher pub;

        std::string input_pixel_points_topic;
        std::string input_pointcloud_topic;
        std::string output_pointcloud_topic;

        arm_vs::Rect pixel_points;
        sensor_msgs::PointCloud2 cropped_cloud;

        void getParametersValues(){
            nh.param<std::string>("input_pixel_points_topic", input_pixel_points_topic, "/tracked_pixel_points");
            ROS_INFO("Input Pixel Points Topic: %s", input_pixel_points_topic.c_str());

            nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/camera/depth/points");
            ROS_INFO("Input Point Cloud Topic: %s", input_pointcloud_topic.c_str());

            nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/cropped_cloud");
            ROS_INFO("Output Point Cloud Topic: %s", output_pointcloud_topic.c_str());
        }

    public:
        void sub_callback(const boost::shared_ptr<sensor_msgs::PointCloud2>& ptcld, const boost::shared_ptr<arm_vs::Rect>& rect){
            int array_pos_first_pixel = 0;
            
            cropped_cloud.header.frame_id = ptcld->header.frame_id;
            cropped_cloud.height = rect->height;
            cropped_cloud.width = rect->width;
            cropped_cloud.fields = ptcld->fields;
            cropped_cloud.is_bigendian = ptcld->is_bigendian;
            cropped_cloud.point_step = ptcld->point_step;
            cropped_cloud.row_step = ptcld->row_step;
            cropped_cloud.is_dense = ptcld->is_dense;

            for(int h = 0; h < rect->height; h++){
                array_pos_first_pixel = (rect->y + h) * ptcld->row_step + rect->x * ptcld->point_step;
                memcpy(&cropped_cloud.data[h * ptcld->row_step], &ptcld->data[array_pos_first_pixel], rect->width * ptcld->point_step);
            }


            pub.publish(cropped_cloud);
        }
        
        CropCloud(ros::NodeHandle node_handle){
            nh = node_handle;
            CropCloud::getParametersValues();
    
            pub = nh.advertise<sensor_msgs::PointCloud2>(output_pointcloud_topic, 1);

            message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, input_pointcloud_topic, 1);
            message_filters::Subscriber<arm_vs::Rect> rect_sub(nh, input_pixel_points_topic, 1);
            message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, arm_vs::Rect> sync(pointcloud_sub, rect_sub, 10);
            sync.registerCallback(sub_callback);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "crop_cloud");
    ros::NodeHandle nh("~");
    CropCloud cc(nh);
    ros::spin();
    return 0;
}