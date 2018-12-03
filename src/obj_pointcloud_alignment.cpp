#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.h"

#include "vector"
#include "Eigen/Core"

#include "pcl/point_types.h"
#include "pcl/common/time.h"


#include "pcl/kdtree/kdtree_flann.h"

#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/fpfh_omp.h"

#include "pcl/registration/icp.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/registration/sample_consensus_prerejective.h"

#include "pcl/segmentation/sac_segmentation.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "boost/shared_ptr.hpp"
#include "boost/bind.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloud_Normal;
typedef pcl::PointCloud<pcl::FPFHSignature33> PointCloud_Feature;
typedef pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> FeatureEstimation;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

typedef message_filters::sync_policies::ApproximateTime<PointCloud, PointCloud> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class Alignment{
    private:
    ros::NodeHandle nh;
    message_filters::Subscriber<PointCloud> template_cloud_sub;
    message_filters::Subscriber<PointCloud> scene_cloud_sub;
    ros::Publisher algined_cloud_pub;
    boost::shared_ptr<Sync> sync;

    // parameters
    std::string input_template_pointcloud_topic;
    std::string input_scene_pointcloud_topic;
    std::string output_pointcloud_topic;
    std::string child_frame_id;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_est;
    FeatureEstimation feature_est;
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
    // pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    const float leaf = 0.005f;

    void getParametersValues(){
        nh.param<std::string>("input_template_pointcloud_topic", input_template_pointcloud_topic, "/camera/depth/points");
        nh.param<std::string>("input_scene_pointcloud_topic", input_scene_pointcloud_topic, "/tracked_object_points_indices");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/tracked_object_pointcloud");
        nh.param<std::string>("icp_alignment_frame", child_frame_id, "icp_alignment_frame");
    }

    void callback_icp(const PointCloud::ConstPtr& template_cloud, const PointCloud::ConstPtr& scene_cloud){
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

            algined_cloud_pub.publish(*object_aligned);

        }else{
            ROS_INFO("Alignment failed to converge");
        } 
    }

    void callback_FPFH(const PointCloud::ConstPtr& template_cloud, const PointCloud::ConstPtr& scene_cloud){
        PointCloud_Normal::Ptr scene (new PointCloud_Normal);
        PointCloud_Normal::Ptr object (new PointCloud_Normal);

        PointCloud_Feature::Ptr scene_features (new PointCloud_Feature);
        PointCloud_Feature::Ptr object_features (new PointCloud_Feature);

        PointCloud::Ptr object_aligned (new PointCloud);

        // SearchMethod::Ptr search_method (new SearchMethod);

        normal_est.setInputCloud (scene_cloud);
        // normal_est.setSearchMethod(search_method);
        // normal_est.setRadiusSearch(0.02f);
        normal_est.compute (*scene);

        feature_est.setInputCloud (scene_cloud);
        feature_est.setInputNormals (scene);
        // feature_est.setSearchMethod(search_method);
        // feature_est.setRadiusSearch(0.02f);
        feature_est.compute (*scene_features);

        normal_est.setInputCloud (template_cloud);
        // normal_est.setSearchMethod(search_method);
        // normal_est.setRadiusSearch(0.02f);
        normal_est.compute (*object);

        feature_est.setInputCloud (template_cloud);
        feature_est.setInputNormals (object);
        // feature_est.setSearchMethod(search_method);
        // feature_est.setRadiusSearch(0.02f);
        feature_est.compute (*object_features);

        align.setInputSource (template_cloud);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene_cloud);
        align.setTargetFeatures (scene_features);

        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (7); // Number of nearest features to use
        align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
        align.setInlierFraction (0.30f); // Required inlier fraction for accepting a pose hypothesis
        // align.setMinSampleDistance(0.05f);

        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);

        ROS_INFO("Alignment Fitness Score: %f",(float) align.getFitnessScore(2.5f * leaf));

        if(align.hasConverged()){
            ROS_INFO("Alignment converged");
            algined_cloud_pub.publish(*object_aligned);

        }else{
            ROS_INFO("Alignment failed to converge");
        } 
    }

    public:
    Alignment(ros::NodeHandle node_handle){
        nh = node_handle;

        Alignment::getParametersValues();

        template_cloud_sub.subscribe(nh, input_template_pointcloud_topic, 10);
        scene_cloud_sub.subscribe(nh, input_scene_pointcloud_topic, 10);

        sync.reset(new Sync(MySyncPolicy(10), template_cloud_sub, scene_cloud_sub));
        sync->registerCallback(boost::bind(&Alignment::callback_icp, this, _1, _2));
    
        algined_cloud_pub = nh.advertise<PointCloud>(output_pointcloud_topic, 1);

        normal_est.setRadiusSearch(0.01);
        feature_est.setRadiusSearch(0.025);
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "align_obj_pointcloud_node");
    ros::NodeHandle nh("~");
    Alignment pt_alignment(nh);
    ros::spin();
    return 0;
}