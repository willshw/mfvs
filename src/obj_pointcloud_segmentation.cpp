#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

#include "pcl/segmentation/organized_connected_component_segmentation.h"
#include "pcl/features/integral_image_normal.h"
#include "pcl/segmentation/organized_multi_plane_segmentation.h"
#include "pcl/segmentation/euclidean_cluster_comparator.h"

// Boost
#include "boost/thread/thread.hpp"

// msgs
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<RefPointType> RefPointCloud;

class Segmenter{
    private:
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    std::string input_pointcloud_topic;
    std::string output_pointcloud_topic;

    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/realsense/depth_registered/points");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/segmented_cloud");
    }

    void callbackOrganizedCloud(const RefPointCloud::ConstPtr& points){

        pcl::IntegralImageNormalEstimation<RefPointType, pcl::Normal> ne;
        pcl::OrganizedMultiPlaneSegmentation<RefPointType, pcl::Normal, pcl::Label> mps;

        pcl::EuclideanClusterComparator<RefPointType, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_
            = pcl::EuclideanClusterComparator<RefPointType, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<RefPointType, pcl::Normal, pcl::Label> ());

        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);


        ne.setInputCloud (points);
        // ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor (0.02f);
        ne.setNormalSmoothingSize (10.0f);
        ne.compute (*normal_cloud);

        // Segment Planes
        std::vector<pcl::PlanarRegion<RefPointType>, Eigen::aligned_allocator<pcl::PlanarRegion<RefPointType> > > regions;
        std::vector<pcl::ModelCoefficients> model_coefficients;
        std::vector<pcl::PointIndices> inlier_indices;  
        pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
        std::vector<pcl::PointIndices> label_indices;
        std::vector<pcl::PointIndices> boundary_indices;
        
        mps.setInputNormals (normal_cloud);
        mps.setInputCloud (points);
        mps.setMinInliers (5000);
        mps.setAngularThreshold (pcl::deg2rad (2.0)); //3 degrees
        mps.setDistanceThreshold (0.01); //2cm
        mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

        // mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

        pcl::PointCloud<RefPointType> clusters;
        unsigned char red [6] = {255,   0,   0, 255, 255,   0};
        unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
        unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

        // check clustered region
        if (regions.size () > 0)
        {
            std::vector<bool> plane_labels;
            plane_labels.resize (label_indices.size (), false);
            for (size_t i = 0; i < label_indices.size (); i++)
            {
                if (label_indices[i].indices.size () > 5000)
                {
                    plane_labels[i] = true;
                }
            }  

            euclidean_cluster_comparator_->setInputCloud (points);
            euclidean_cluster_comparator_->setLabels (labels);
            euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
            euclidean_cluster_comparator_->setDistanceThreshold (0.03f, false);

            pcl::PointCloud<pcl::Label> euclidean_labels;
            std::vector<pcl::PointIndices> euclidean_label_indices;

            pcl::OrganizedConnectedComponentSegmentation<RefPointType,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
            euclidean_segmentation.setInputCloud (points);
            euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

            for (size_t i = 0; i < euclidean_label_indices.size (); i++)
            {
                if (euclidean_label_indices[i].indices.size () > 1000)
                {   
                    pcl::PointCloud<RefPointType> cluster;
                    pcl::copyPointCloud (*points, euclidean_label_indices[i].indices, cluster);
                    int32_t rgb = (static_cast<uint32_t>(red[i%6]) << 16 | static_cast<uint32_t>(grn[i%6]) << 8 | static_cast<uint32_t>(blu[i%6]));
                    
                    for(auto &p: cluster.points)
                    {
                        p.rgb = rgb;
                    }
                    clusters += cluster;
                }
            }
        }

        clusters.header = points->header;;
        cloud_pub.publish(clusters);
    }

    public:
    Segmenter(ros::NodeHandle node_handle){
        nh = node_handle;

        Segmenter::getParametersValues();

        cloud_sub = nh.subscribe<RefPointCloud>(input_pointcloud_topic, 1, &Segmenter::callback, this);
        cloud_pub = nh.advertise<RefPointCloud>(output_pointcloud_topic, 1);
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