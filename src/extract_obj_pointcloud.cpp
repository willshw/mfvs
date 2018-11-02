#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<pcl_msgs::PointIndices, PointCloud> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class Extraction{
    private:
    ros::NodeHandle nh;

    message_filters::Subscriber<pcl_msgs::PointIndices> ind_sub;
    message_filters::Subscriber<PointCloud> pt_sub;
    ros::Publisher extracted_pt_pub;

    boost::shared_ptr<Sync> sync;

    std::string input_pointcloud_topic;
    std::string input_indices_topic;
    std::string output_pointcloud_topic;

    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/camera/depth/points");
        nh.param<std::string>("input_indices_topic", input_indices_topic, "/tracked_object_points_indices");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/tracked_object_pointcloud");
    }

    void callback(const pcl_msgs::PointIndices::ConstPtr& indices, const PointCloud::ConstPtr& points)
    {
        PointCloud::Ptr extracted_points(new PointCloud);

        pcl::PointIndices::Ptr indices_cvt(new pcl::PointIndices); 
        pcl_conversions::toPCL(*indices, *indices_cvt);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(points);
        extract.setIndices(indices_cvt);
        extract.setNegative(false);
        extract.filter(*extracted_points);

        extracted_pt_pub.publish(*extracted_points);
    }

    public:
    Extraction(ros::NodeHandle node_handle){
        nh = node_handle;

        Extraction::getParametersValues();

        ind_sub.subscribe(nh, input_indices_topic, 1);
        pt_sub.subscribe(nh, input_pointcloud_topic, 1);

        extracted_pt_pub = nh.advertise<PointCloud>(output_pointcloud_topic, 1);

        sync.reset(new Sync(MySyncPolicy(10), ind_sub, pt_sub));
        sync->registerCallback(boost::bind(&Extraction::callback, this, _1, _2));
    }

};
ros::Publisher pub;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "extract_tracked_obj_points_node");
    ros::NodeHandle nh("~");
    Extraction node(nh);
    ros::spin();
    return 0;
}