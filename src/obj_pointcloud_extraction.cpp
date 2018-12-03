/**
 * This node subscribes to the bouding box message for tracked object in image,
 * and structured point cloud (point cloud stuctured in the way that aligns with color image).
 * By using message filter, this node synchronizes bouding box and point could, and usez the bounding
 * box information to crop the point cloud and preserve the region of interest, and publishes the
 * point cloud in the region of interest. 
 */
#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_msgs/PointIndices.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/point_types.h"
#include "pcl/filters/extract_indices.h"

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

class Extraction{
    private:
    ros::NodeHandle nh;

    message_filters::Subscriber<arm_vs::BBox> bbox_sub;
    message_filters::Subscriber<RefPointCloud> pt_sub;
    ros::Publisher extracted_pt_pub;

    boost::shared_ptr<Sync> sync;

    std::string input_pointcloud_topic;
    std::string input_bbox_topic;
    std::string output_pointcloud_topic;

    /**
     * Get parameters
     */
    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/camera/depth/points");
        nh.param<std::string>("input_bbox_topic", input_bbox_topic, "/tracked_object_bbox");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/tracked_object_pointcloud");
    }

    /**
     * Callback takes in synchronized bounding box message and point cloud message, and extract points in the boudning box
     * region (in x and y), and publishes extracted point cloud.
     */
    void callback(const arm_vs::BBox::ConstPtr bbox, const RefPointCloud::ConstPtr& points)
    {
        RefPointCloud::Ptr extracted_cloud(new RefPointCloud);
        pcl::PointIndices::Ptr extracted_indices(new pcl::PointIndices);

        // bouding box top left pixel location and bottom right pixel location
        std::pair<int, int> tl = std::make_pair(bbox->x, bbox->y);
        std::pair<int, int> br = std::make_pair(bbox->x + bbox->width, bbox->y + bbox->height);

        // construct point indices array from points in bounding box
        std::vector<int> tracked_indices((bbox->width + 1) * (bbox->height + 1));

        for(auto row = 0; row < bbox->height + 1; row++){
            std::vector<int> inds(
                boost::counting_iterator<int>(tl.first + (tl.second + row) * points->width), 
                boost::counting_iterator<int>(br.first + (tl.second + row) * points->width)
            );

            std::copy(inds.begin(), inds.end(), tracked_indices.begin() + row * (bbox->width + 1));
        }

        extracted_indices->indices = tracked_indices;

        // extract points from input point cloud
        pcl::ExtractIndices<RefPointType> extract;
        extract.setInputCloud(points);
        extract.setIndices(extracted_indices);
        extract.setNegative(false);
        extract.filter(*extracted_cloud);

        // publish extracted points
        extracted_cloud->header = points->header;
        extracted_pt_pub.publish(*extracted_cloud);
    }

    public:
    Extraction(ros::NodeHandle node_handle){
        nh = node_handle;

        Extraction::getParametersValues();

        bbox_sub.subscribe(nh, input_bbox_topic, 1);
        pt_sub.subscribe(nh, input_pointcloud_topic, 1);

        extracted_pt_pub = nh.advertise<RefPointCloud>(output_pointcloud_topic, 1);

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