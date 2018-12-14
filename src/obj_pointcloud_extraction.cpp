/**
 * This node subscribes to the bouding box message for tracked object in image,
 * and structured point cloud (point cloud stuctured in the way that aligns with color image).
 * By using message filter, this node synchronizes bouding box and point could, and usez the bounding
 * box information to crop the point cloud and preserve the region of interest, and publishes the
 * point cloud in the region of interest. 
 */
#include <algorithm>
#include <vector>

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_msgs/PointIndices.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/point_types.h"
#include "pcl/common/point_tests.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"

#include "pcl/segmentation/organized_connected_component_segmentation.h"
#include "pcl/features/integral_image_normal.h"
#include "pcl/segmentation/organized_multi_plane_segmentation.h"
#include "pcl/segmentation/euclidean_cluster_comparator.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "boost/shared_ptr.hpp"
#include "boost/bind.hpp"
#include "boost/iterator/counting_iterator.hpp"

// msgs
#include "arm_vs/BBox.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<RefPointType> RefPointCloud;

// setup synchronized subscriber using approximate time sync policy
typedef message_filters::sync_policies::ApproximateTime<arm_vs::BBox, RefPointCloud> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class Extraction
{
    private:
    ros::NodeHandle nh;

    message_filters::Subscriber<arm_vs::BBox> bbox_sub;
    message_filters::Subscriber<RefPointCloud> pt_sub;
    ros::Publisher extracted_pt_pub;
    ros::Publisher labeled_cloud_pub;

    boost::shared_ptr<Sync> sync;

    std::string input_pointcloud_topic;
    std::string input_bbox_topic;
    std::string output_pointcloud_topic;

    /**
     * Get parameters
     */
    void getParametersValues()
    {
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/camera/depth/points");
        nh.param<std::string>("input_bbox_topic", input_bbox_topic, "/tracked_object_bbox");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/tracked_object_pointcloud");
    }

    void check(std::pair<int, int> *p, int width, int height)
    {
        p->first = std::max(p->first, 0);
        p->first = std::min(p->first, width - 1);
        p->second = std::max(p->second, 0);
        p->second = std::min(p->second, height - 1);
    }

    /**
     * Callback takes in synchronized bounding box message and point cloud message, and extract points in the boudning box
     * region (in x and y), and publishes extracted point cloud.
     */
    void callback(const arm_vs::BBox::ConstPtr bbox, const RefPointCloud::ConstPtr& input_cloud)
    {
        RefPointCloud::Ptr extracted_cloud(new RefPointCloud);
        pcl::PointIndices::Ptr extracted_indices(new pcl::PointIndices);
        
        extracted_indices->header = input_cloud->header;

        // prep the output cloud
        extracted_cloud->header = input_cloud->header;
        extracted_cloud->sensor_origin_ = input_cloud->sensor_origin_;
        extracted_cloud->sensor_orientation_ = input_cloud->sensor_orientation_;

        // bouding box top left pixel location and bottom right pixel location
        std::pair<int, int> tl = std::make_pair(bbox->x, bbox->y);
        std::pair<int, int> br = std::make_pair(bbox->x + bbox->width - 1, bbox->y + bbox->height - 1);

        check(&tl, input_cloud->width, input_cloud->height);
        check(&br, input_cloud->width, input_cloud->height);

        int new_width = br.first - tl.first + 1;
        int new_height = br.second - tl.second + 1;

        // construct point indices array from input_cloud in bounding box
        std::vector<int> tracked_indices(new_width * new_height);
 
        for(auto row = 0; row < new_height; row++)
        {
            std::vector<int> inds(
                boost::counting_iterator<int>(tl.first + (tl.second + row) * input_cloud->width), 
                boost::counting_iterator<int>(br.first + (tl.second + row) * input_cloud->width)
            );

            std::copy(inds.begin(), inds.end(), tracked_indices.begin() + row * new_width);
        }
        
        extracted_indices->indices = tracked_indices;

        // extract points from input point cloud
        pcl::ExtractIndices<RefPointType> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(extracted_indices);
        extract.setNegative(false);
        extract.filter(*extracted_cloud);

        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*extracted_cloud, *extracted_cloud, indices);

        // publish extracted points
        extracted_cloud->header = input_cloud->header;
        extracted_pt_pub.publish(*extracted_cloud);
    }

    public:
    Extraction(ros::NodeHandle node_handle){
        nh = node_handle;

        Extraction::getParametersValues();

        bbox_sub.subscribe(nh, input_bbox_topic, 10);
        pt_sub.subscribe(nh, input_pointcloud_topic, 10);

        extracted_pt_pub = nh.advertise<RefPointCloud>(output_pointcloud_topic, 1);
        labeled_cloud_pub = nh.advertise<RefPointCloud>("/labeled", 1);

        // setup synchronized subscriber using approximate time sync policy
        sync.reset(new Sync(MySyncPolicy(10), bbox_sub, pt_sub));
        sync->registerCallback(boost::bind(&Extraction::callback, this, _1, _2));

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extract_tracked_obj_points_node");
    ros::NodeHandle nh("~");
    Extraction node(nh);
    ros::spin();
    return 0;
}