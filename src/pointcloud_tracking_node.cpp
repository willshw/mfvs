#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

//msgs
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointcloudTracking{
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;
        ros::Publisher pub;

        std::string input_pointcloud_topic;
        std::string output_pointcloud_topic;
        std::string track_cloud_filename;

        PointCloud::Ptr data;
        PointCloud::Ptr target_cloud;
        PointCloud::Ptr cloud_pass_;
        PointCloud::Ptr cloud_pass_downsampled_;

        double downsampling_grid_size;

        boost::shared_ptr<pcl::tracking::ParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>> tracker_;

        bool new_cloud_;
        double downsampling_grid_size_;
        int counter;

        void getParametersValues(){
            nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/kinect2/qhd/points");
            ROS_INFO("Input Point Cloud Topic: %s", input_pointcloud_topic.c_str());

            nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/matched_cloud");
            ROS_INFO("Output Point Cloud Topic: %s", output_pointcloud_topic.c_str());

            nh.param<std::string>("track_cloud_filename", track_cloud_filename, "cloud.pcd");
            ROS_INFO("Tracking Object Cloud Filename: %s", track_cloud_filename.c_str());
        }

        void pointcloud_sub_callback(const PointCloudRGB::ConstPtr& msg){
            ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);

            pcl::copyPointCloud(*msg, *data);

            cloud_pass_.reset (new PointCloud);
            cloud_pass_downsampled_.reset (new PointCloud);
            PointcloudTracking::filterPassThrough (data, *cloud_pass_);
            PointcloudTracking::gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

            if(counter < 10){
                counter++;
            }else{
                //Track the object
                tracker_->setInputCloud (cloud_pass_downsampled_);
                tracker_->compute ();
                new_cloud_ = true;
            }

            pcl::tracking::ParticleXYZRPY result = tracker_->getResult ();
            Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
            //move close to camera a little for better visualization
            transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
            PointCloud::Ptr result_cloud (new PointCloud());
            pcl::transformPointCloud<pcl::PointXYZ> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

            result_cloud->header.frame_id = data->header.frame_id;
            pub.publish(*result_cloud);
        }

        //Filter along a specified dimension
        void filterPassThrough (const PointCloud::Ptr &cloud, PointCloud &result)
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 10.0);
            pass.setKeepOrganized (false);
            pass.setInputCloud (cloud);
            pass.filter (result);
        }


        void gridSampleApprox (const PointCloud::Ptr &cloud, PointCloud &result, double leaf_size)
        {
            pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
            grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
            grid.setInputCloud (cloud);
            grid.filter (result);
        }


    public:
        PointcloudTracking(ros::NodeHandle node_handle){
            nh = node_handle;
            PointcloudTracking::getParametersValues();

            sub = nh.subscribe<PointCloudRGB>(input_pointcloud_topic, 1, &PointcloudTracking::pointcloud_sub_callback, this);
            pub = nh.advertise<PointCloud>(output_pointcloud_topic, 1);

            target_cloud.reset(new PointCloud());
            if(pcl::io::loadPCDFile(track_cloud_filename, *target_cloud) == -1){
                ROS_FATAL("Target cloud pcd file not found!");
            }
            else{
                ROS_INFO("Target cloud pcd file: %s successfully loaded!", track_cloud_filename.c_str());
            }

            data.reset(new PointCloud());

            counter = 0;

            //Set parameters
            new_cloud_  = false;
            downsampling_grid_size_ =  0.002;

            std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
            default_step_covariance[3] *= 40.0;
            default_step_covariance[4] *= 40.0;
            default_step_covariance[5] *= 40.0;

            std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
            std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

            boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>> tracker
                (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY> (8));

            pcl::tracking::ParticleXYZRPY bin_size;
            bin_size.x = 0.1f;
            bin_size.y = 0.1f;
            bin_size.z = 0.1f;
            bin_size.roll = 0.1f;
            bin_size.pitch = 0.1f;
            bin_size.yaw = 0.1f;

            //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
            tracker->setMaximumParticleNum (1000);
            tracker->setDelta (0.99);
            tracker->setEpsilon (0.2);
            tracker->setBinSize (bin_size);

            //Set all parameters for  ParticleFilter
            tracker_ = tracker;
            tracker_->setTrans (Eigen::Affine3f::Identity ());
            tracker_->setStepNoiseCovariance (default_step_covariance);
            tracker_->setInitialNoiseCovariance (initial_noise_covariance);
            tracker_->setInitialNoiseMean (default_initial_mean);
            tracker_->setIterationNum (1);
            tracker_->setParticleNum (600);
            tracker_->setResampleLikelihoodThr(0.00);
            tracker_->setUseNormal (false);

            //Setup coherence object for tracking
            pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr
                (new pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ> ());

            boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZ> > distance_coherence
                = boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZ> > (new pcl::tracking::DistanceCoherence<pcl::PointXYZ> ());
            coherence->addPointCoherence (distance_coherence);

            boost::shared_ptr<pcl::search::Octree<pcl::PointXYZ> > search (new pcl::search::Octree<pcl::PointXYZ> (0.01));
            coherence->setSearchMethod (search);
            coherence->setMaximumDistance (0.01);

            tracker_->setCloudCoherence (coherence);

            //prepare the model of tracker's target
            Eigen::Vector4f c;
            Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
            PointCloud::Ptr transed_ref (new PointCloud);
            PointCloud::Ptr transed_ref_downsampled (new PointCloud);

            pcl::compute3DCentroid<pcl::PointXYZ> (*target_cloud, c);
            trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
            pcl::transformPointCloud<pcl::PointXYZ> (*target_cloud, *transed_ref, trans.inverse());
            gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

            //set reference model and trans
            tracker_->setReferenceCloud (transed_ref_downsampled);
            tracker_->setTrans (trans);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh("~");
    PointcloudTracking pt(nh);
    ros::spin();
    return 0;
}