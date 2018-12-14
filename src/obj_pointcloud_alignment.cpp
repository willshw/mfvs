/**
 * This node takes in alignment from particle filter and refine it with ICP
*/
#include <vector>
#include <Eigen/Core>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<RefPointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

class Alignment{
    private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    // parameters
    std::string input_pointcloud_topic;
    std::string output_pointcloud_topic;
    std::string template_cloud_filename;
    std::string child_frame_id;

    pcl::IterativeClosestPoint<RefPointType, RefPointType> icp;
    PointCloudPtr template_cloud;

    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/kinect2/qhd/points");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/matched_cloud");
        nh.param<std::string>("template_cloud_filename", template_cloud_filename, "cloud.pcd");
        nh.param<std::string>("icp_alignment_frame", child_frame_id, "icp_alignment_frame");
    }

    void callback_icp(const PointCloud::ConstPtr& scene_cloud){
        icp.setInputSource(template_cloud);
        icp.setInputTarget(scene_cloud);

        PointCloud::Ptr object_aligned (new PointCloud);

        pcl::ScopeTime t("Alignment");
        icp.align(*object_aligned);

        ROS_INFO("Alignment Fitness Score: %f",(float) icp.getFitnessScore());

        if(icp.hasConverged()){
            ROS_INFO("Alignment converged");

            // get the final transformation
            Eigen::Affine3f transformation_eigen;
            transformation_eigen.matrix() = icp.getFinalTransformation(); // icp.getFinalTransformation() returns Eigen::Matrix4f

            // Broadcast TF of the pointcloud template
            geometry_msgs::TransformStamped transformation_stamped;

            transformation_stamped = tf2::eigenToTransform(transformation_eigen.cast<double>());
            // transformation_stamped.header = msg_cloud->header;
            transformation_stamped.header.frame_id = scene_cloud->header.frame_id;
            transformation_stamped.child_frame_id = child_frame_id;

            static tf2_ros::TransformBroadcaster br;
            br.sendTransform(transformation_stamped);

            object_aligned->header.frame_id = scene_cloud->header.frame_id;
            pub.publish(*object_aligned);

        }else{
            ROS_INFO("Alignment failed to converge");
        } 
    }

    public:
    Alignment(ros::NodeHandle node_handle){
        nh = node_handle;
        Alignment::getParametersValues();
        
        template_cloud.reset(new PointCloud());
        if(pcl::io::loadPCDFile(template_cloud_filename, *template_cloud) == -1){
            ROS_FATAL("Target cloud pcd file not found!");
        }
        else{
            ROS_INFO("Target cloud pcd file: %s successfully loaded!", template_cloud_filename.c_str());
        }

        // initialize subscribers, publishers and services
        sub = nh.subscribe<PointCloud>(input_pointcloud_topic, 1, &Alignment::callback_icp, this);
        pub = nh.advertise<PointCloud>(output_pointcloud_topic, 1);
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "align_obj_pointcloud_node");
    ros::NodeHandle nh("~");
    Alignment pt_alignment(nh);
    ros::spin();
    return 0;
}