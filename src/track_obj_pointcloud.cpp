#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

//msgs
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointcloudTracking{
    private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pub;

    std::string input_pointcloud_topic;
    std::string output_pointcloud_topic;
    std::string template_cloud_filename;

    PointCloud::Ptr template_cloud;

    boost::shared_ptr<pcl::tracking::ParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>> tracker_;

    int counter;

    void getParametersValues(){
        nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/kinect2/qhd/points");
        nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/matched_cloud");
        nh.param<std::string>("template_cloud_filename", template_cloud_filename, "cloud.pcd");
    }

    void pointcloud_sub_callback(const PointCloud::ConstPtr& msg_cloud){
        // ROS_INFO("Cloud: width = %d, height = %d\n", msg_cloud->width, msg_cloud->height);

        if(counter < 10){
            counter++;
        }else{
            //Track the object
            tracker_->setInputCloud (msg_cloud);
            tracker_->compute ();

            pcl::tracking::ParticleXYZRPY result = tracker_->getResult ();
            Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

            //move close to camera a little for better visualization
            PointCloud::Ptr result_cloud (new PointCloud());
            pcl::transformPointCloud<pcl::PointXYZ> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

            
            // result_cloud->header.frame_id = msg_cloud->header.frame_id;
            result_cloud->header = msg_cloud->header;
            pub.publish(*result_cloud);
        }
    }


    public:
    PointcloudTracking(ros::NodeHandle node_handle){
        nh = node_handle;

        PointcloudTracking::getParametersValues();

        template_cloud.reset(new PointCloud());
        if(pcl::io::loadPCDFile(template_cloud_filename, *template_cloud) == -1){
            ROS_FATAL("Target cloud pcd file not found!");
        }
        else{
            ROS_INFO("Target cloud pcd file: %s successfully loaded!", template_cloud_filename.c_str());
        }

        counter = 0;

        // Set parameter
        std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;

        std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

        boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>> tracker
            (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY> (8)); // 8 is the number of threads

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
        pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr
            (new pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ> ());

        boost::shared_ptr<pcl::tracking::DistanceCoherence <pcl::PointXYZ> > distance_coherence
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

        pcl::compute3DCentroid<pcl::PointXYZ> (*template_cloud, c);
        trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
        pcl::transformPointCloud<pcl::PointXYZ> (*template_cloud, *transed_ref, trans.inverse());

        //set reference model and trans
        tracker_->setReferenceCloud (transed_ref);
        tracker_->setTrans (trans);

        sub = nh.subscribe<PointCloud>(input_pointcloud_topic, 1, &PointcloudTracking::pointcloud_sub_callback, this);
        pub = nh.advertise<PointCloud>(output_pointcloud_topic, 1);
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