/**
 * This node used object point cloud template and particle filter to track object in
 * current point cloud scene.
 */

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

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

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// srv
#include "std_srvs/Trigger.h"

typedef pcl::PointXYZRGB RefPointType;

typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<RefPointType> Cloud;

typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

class PointcloudTracking
{
    private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::ServiceServer reset_service;

    // parameters
    std::string input_pointcloud_topic;
    std::string output_pointcloud_topic;
    std::string template_cloud_filename;
    std::string child_frame_id;
    std::string tracker_reset_service;

    // PCL stuff
    CloudPtr template_cloud;
    boost::shared_ptr<ParticleFilter> tracker_;
    int counter;

    tf2_ros::TransformBroadcaster br;

    void getParametersValues()
    {
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/kinect2/qhd/points");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/matched_cloud");
        nh.param<std::string>("template_cloud_filename", template_cloud_filename, "cloud.pcd");
        nh.param<std::string>("particle_filter_tracked_template_frame", child_frame_id, "particle_filter_tracked_template_frame");
        nh.param<std::string>("particle_filter_tracker_reset_service", tracker_reset_service, "reset_particle_filter_tracker");
    }

    bool reset_tracker_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {

        tracker_->resetTracking();
        
        res.success = true;
        res.message = "Particle filter tracker has been successfully reset.";
        
        ROS_INFO("Particle filter tracker reset.");

        return true;
    }

    void pointcloud_sub_callback(const CloudConstPtr& msg_cloud)
    {
        if(counter < 10)
        {
            counter++;
        }
        else if(msg_cloud->points.size() > 0)
        {
        
            // Track the object
            tracker_->setInputCloud(msg_cloud);
            tracker_->compute ();

            ParticleT result = tracker_->getResult ();
            Eigen::Affine3f transformation_eigen = tracker_->toEigenMatrix (result);

            // Get object template pointcloud pose from tracker
            CloudPtr result_cloud (new Cloud());
            pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation_eigen);

            // Broadcast TF of the pointcloud template
            geometry_msgs::TransformStamped transformation_stamped;

            transformation_stamped = tf2::eigenToTransform(transformation_eigen.cast<double>());
            // transformation_stamped.header = msg_cloud->header;
            transformation_stamped.header.frame_id = msg_cloud->header.frame_id;
            transformation_stamped.child_frame_id = child_frame_id;
            transformation_stamped.header.stamp = ros::Time::now();
            br.sendTransform(transformation_stamped);
            
            result_cloud->header.frame_id = msg_cloud->header.frame_id;
            // result_cloud->header = msg_cloud->header;
            pub.publish(*result_cloud);
        }
    }


    public:
    PointcloudTracking(ros::NodeHandle node_handle){
        nh = node_handle;
        PointcloudTracking::getParametersValues();

        template_cloud.reset(new Cloud());
        if(pcl::io::loadPCDFile(template_cloud_filename, *template_cloud) == -1){
            ROS_FATAL("Target cloud pcd file not found!");
        }
        else{
            ROS_INFO("Target cloud pcd file: %s successfully loaded!", template_cloud_filename.c_str());
        }

        counter = 0;

        // Set tracker parameters
        std::vector<double> default_step_covariance = std::vector<double> (6, 0.010 * 0.010);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;

        std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

        boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>> tracker
            (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8)); // 8 is the number of threads

        ParticleT bin_size;
        bin_size.x = 0.10f;
        bin_size.y = 0.10f;
        bin_size.z = 0.10f;
        bin_size.roll = 0.10f;
        bin_size.pitch = 0.10f;
        bin_size.yaw = 0.10f;

        //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
        tracker->setMaximumParticleNum (1000);
        tracker->setDelta (0.99);
        tracker->setEpsilon (0.2);
        tracker->setBinSize (bin_size);

        //Set all parameters for ParticleFilter
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
        
        pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
            (new pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType> ());

        //Distance Coherence
        boost::shared_ptr<pcl::tracking::DistanceCoherence <RefPointType> > distance_coherence
            = boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > (new pcl::tracking::DistanceCoherence<RefPointType> ());
        
        //Color Coherence
        boost::shared_ptr<pcl::tracking::HSVColorCoherence <RefPointType> > hsv_color_coherence
            = boost::shared_ptr<pcl::tracking::HSVColorCoherence<RefPointType> > (new pcl::tracking::HSVColorCoherence<RefPointType> ());

        //Add coherence
        coherence->addPointCoherence (distance_coherence);
        coherence->addPointCoherence (hsv_color_coherence);

        boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
        coherence->setSearchMethod (search);
        coherence->setMaximumDistance (0.01);

        tracker_->setCloudCoherence (coherence);

        //prepare the model of tracker's target
        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
        CloudPtr transed_ref (new Cloud);

        pcl::compute3DCentroid<RefPointType> (*template_cloud, c);
        trans.translation().matrix() = Eigen::Vector3f (c[0], c[1], c[2]);
        
        pcl::transformPointCloud<RefPointType> (*template_cloud, *transed_ref, trans.inverse());

        //set reference model and trans
        tracker_->setReferenceCloud (transed_ref);
        tracker_->setTrans (trans);

        // initialize subscribers, publishers and services
        sub = nh.subscribe<Cloud>(input_pointcloud_topic, 1, &PointcloudTracking::pointcloud_sub_callback, this);
        pub = nh.advertise<Cloud>(output_pointcloud_topic, 1);

        reset_service = nh.advertiseService(tracker_reset_service, &PointcloudTracking::reset_tracker_srv_callback, this);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_obj_pointcloud_node");
    ros::NodeHandle nh("~");
    PointcloudTracking pt(nh);
    ros::spin();
    return 0;
}