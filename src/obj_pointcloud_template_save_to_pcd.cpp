/**
 * This node subscribe to point cloud topic and save the point cloud data to pcd file.
 * pointcloud_to_pcd in pcl_ros only saves the XYZ pointcloud; RGB info can be saved using this node
 */

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

// msgs
#include "sensor_msgs/PointCloud2.h"


#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>


// Boost
#include <boost/thread/thread.hpp>
// PCL

#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<RefPointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class PCDSaver{
    private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string input_pointcloud_topic;
    std::string target_folder;
    std::string filename_prefix;

    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/realsense/voxel_grid/output");
        nh.param<std::string>("target_folder", target_folder, "~/Document/");
        nh.param<std::string>("filename_prefix", filename_prefix, "ptcld_");
    }

    void callback(const CloudConstPtr& pt_cld){
        std::string filename = target_folder + "/" + filename_prefix + std::to_string(pt_cld->header.seq) + ".pcd";
        pcl::io::savePCDFileASCII (filename, *pt_cld);
        ROS_INFO("Saved %s", filename.c_str());
        ROS_INFO("Width: %d, Height: %d", pt_cld->width, pt_cld->height);
    }

    public:
    PCDSaver(ros::NodeHandle node_handle){
        nh = node_handle;
        PCDSaver::getParametersValues();
        sub = nh.subscribe(input_pointcloud_topic, 0, &PCDSaver::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_to_pcd");
    ros::NodeHandle nh("~");
    PCDSaver save_pcd(nh);
    ros::spin();
    return 0;
}