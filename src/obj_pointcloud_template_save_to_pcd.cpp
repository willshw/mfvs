/**
 * This node subscribe to point cloud topic and save the point cloud data to pcd file.
 * pointcloud_to_pcd in pcl_ros only saves the XYZ pointcloud; RGB info can be saved using this node
 */
#include <vector>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// msgs
#include <sensor_msgs/PointCloud2.h>

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