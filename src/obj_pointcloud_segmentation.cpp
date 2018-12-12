#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

#include "pcl/features/integral_image_normal.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/segmentation/organized_connected_component_segmentation.h"
#include "pcl/segmentation/organized_multi_plane_segmentation.h"
#include "pcl/segmentation/euclidean_cluster_comparator.h"

// Boost
#include "boost/thread/thread.hpp"

// msgs
#include "sensor_msgs/PointCloud2.h"

//srv
#include "arm_vs/SetObjPtCldTemplate.h"

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<RefPointType> RefPointCloudType;

class Segmenter
{
    private:
    ros::NodeHandle nh;

    // ros::Subscriber cloud_sub;
    ros::ServiceServer object_template_getter_service;

    std::string input_pointcloud_topic;
    std::string object_template_getter_service_name;

    void getParametersValues()
    {
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/realsense/depth_registered/points");
        nh.param<std::string>("object_template_getter_service_name", object_template_getter_service_name, "get_object_template");
    }

    bool getObjectTemplate(arm_vs::SetObjPtCldTemplate::Request &req, arm_vs::SetObjPtCldTemplate::Response &res)
    {
        sensor_msgs::PointCloud2ConstPtr input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(input_pointcloud_topic, ros::Duration(10));
        RefPointCloudType::Ptr cloud;
        pcl::fromROSMsg(*input_cloud, *cloud);



        ROS_INFO("Object point cloud template captured.");

        return true;
    }

    public:
    Segmenter(ros::NodeHandle node_handle){
        nh = node_handle;

        Segmenter::getParametersValues();

        // cloud_sub = nh.subscribe<RefPointCloud>(input_pointcloud_topic, 1, &Segmenter::callback, this);
        object_template_getter_service = nh.advertiseService(object_template_getter_service_name, &Segmenter::getObjectTemplate, this);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_segmentation_node");
    ros::NodeHandle nh("~");
    Segmenter node(nh);
    ros::spin();
    return 0;
}