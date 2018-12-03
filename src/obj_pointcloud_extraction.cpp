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
#include <boost/iterator/counting_iterator.hpp>

// msgs
#include "arm_vs/BBox.h"

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<RefPointType> RefPointCloud;
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

    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/camera/depth/points");
        nh.param<std::string>("input_bbox_topic", input_bbox_topic, "/tracked_object_bbox");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/tracked_object_pointcloud");
    }

    void getInicies(const arm_vs::BBox::ConstPtr box, const RefPointCloud::ConstPtr& cloud, pcl::PointIndices::Ptr extracted_indices){
        std::pair<int, int> tl = std::make_pair(box->x, box->y);
        std::pair<int, int> br = std::make_pair(box->x + box->width, box->y + box->height);
        

        ROS_INFO("%d,  %d,  %d", cloud->width, cloud->height, cloud->is_dense);
        std::vector<int> tracked_points_indices((box->width + 1) * (box->height + 1));

        // for(auto row = 0; row < box->height + 1; row++){
        //     for(auto col=0; col < box->width + 1; col++){
        //         tracked_points_indices[col + row * (box->width+1)] = (tl.first + col) + (tl.second + row * cloud->width));
        //     }
        // }
            // std::vector<int> inds(
            //     boost::counting_iterator<int>(tl.first + (tl.second + row) * (cloud->width + 1)), 
            //     boost::counting_iterator<int>(br.first + (tl.second + row) * (cloud->width + 1))
            // );
        //     std::copy(
        //         boost::counting_iterator<int>(tl.first + (tl.second + row) * (cloud->width + 1)),
        //         boost::counting_iterator<int>(br.first + (tl.second + row) * (cloud->width + 1)),
        //         tracked_points_indices.begin() + row * (box->width + 1));
        // }

        extracted_indices->header = cloud->header;
        extracted_indices->indices = tracked_points_indices;
    }

    void callback(const arm_vs::BBox::ConstPtr bbox, const RefPointCloud::ConstPtr& points)
    {
        pcl::PointIndices::Ptr extracted_indices(new pcl::PointIndices);
        RefPointCloud::Ptr extracted_cloud(new RefPointCloud);

        Extraction::getInicies(bbox, points, extracted_indices);

        pcl::ExtractIndices<RefPointType> extract;
        extract.setInputCloud(points);
        extract.setIndices(extracted_indices);
        extract.setNegative(false);
        extract.filter(*extracted_cloud);

        extracted_pt_pub.publish(*extracted_cloud);
    }

    public:
    Extraction(ros::NodeHandle node_handle){
        nh = node_handle;

        Extraction::getParametersValues();

        bbox_sub.subscribe(nh, input_bbox_topic, 1);
        pt_sub.subscribe(nh, input_pointcloud_topic, 1);

        extracted_pt_pub = nh.advertise<RefPointCloud>(output_pointcloud_topic, 1);

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