/**
 * This node takes in desired camera pose and current camera pose and uses VISP code to
 * calculate the end effector velocity needed to minized the error between two poses mentioned.
 */
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// ViSP
#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

// msgs
#include "std_msgs/String.h"
// #include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "arm_vs/CartVelCmd.h"

#define PI 3.14159265

class VisualServoing{
    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;
        geometry_msgs::TransformStamped tf_transform;
        ros::Publisher cart_vel_pub;

        //parameters
        double pbvs_control_loop_hz;
        double pbvs_control_law_gain_lambda;
        double pbvs_control_deadband_error;

        double xyz_vel_limit;
        double rpy_vel_limit;

        std::string desired_camera_frame;
        std::string current_camera_frame;
        std::string robot_base_frame;
        std::string control_input_topic;

        arm_vs::CartVelCmd control_input;

    public:
        VisualServoing(ros::NodeHandle node_handle){
            nh = node_handle;
            VisualServoing::getParametersValues();

            tf_listener = new tf2_ros::TransformListener(tf_buffer);
            cart_vel_pub = nh.advertise<arm_vs::CartVelCmd>(control_input_topic, 1);

            VisualServoing::pbvs();
        }
        
        void getParametersValues(){
            nh.param<double>("pbvs_control_loop_hz", pbvs_control_loop_hz, 60.0);
            nh.param<double>("pbvs_control_law_gain_lambda", pbvs_control_law_gain_lambda, 1.0);
            nh.param<double>("pbvs_control_deadband_error", pbvs_control_deadband_error, 0.0001);

            nh.param<double>("xyz_vel_limit", xyz_vel_limit, 0.18);
            nh.param<double>("rpy_vel_limit", rpy_vel_limit, 0.18);

            nh.param<std::string>("desired_camera_frame", desired_camera_frame, "/desired_cam_frame");
            nh.param<std::string>("current_camera_frame", current_camera_frame, "/camera_rgb_optical_frame");
            nh.param<std::string>("robot_base_frame", robot_base_frame, "robot_base_frame");
            nh.param<std::string>("control_input_topic", control_input_topic, "/control_input");
        }

        void getTwistVectorBodyFrame(Eigen::VectorXd& Vb, Eigen::VectorXd Vc, Eigen::Matrix4d bMc){
            // calculate adjoint using input transformation
            // [  R  0]
            // [[tR] R]
            Eigen::Matrix3d bRc = bMc.block<3,3>(0,0); // rotation
            Eigen::Vector3d btc = bMc.block<3,1>(0,3); // translation

            // skew symmetric [t]
            Eigen::Matrix3d btc_s;
            btc_s << 0, -btc(2), btc(1),
                    btc(2), 0, -btc(0),
                    -btc(1), btc(0), 0;

            // Adjoint
            Eigen::MatrixXd bAdc(6, 6);
            bAdc << bRc, Eigen::Matrix3d::Zero(), 
                    btc_s*bRc, bRc;

            // calculate twist in body frame using adjoint and twist in camera frame
            Vb = bAdc * Vc;

            // std::stringstream ss;
            // ss.str(std::string());
            // ss << "Vb:\n" << Vb << "\n";
            // ROS_INFO("\n%s", ss.str().c_str());
        }

        void fixQuat(double &qx, double &qy, double &qz, double &qw){
            double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            qx = qx/norm;
            qy = qy/norm;
            qz = qz/norm;
            qw = qw/norm;

            if(2*acos(qw) > PI)
            {
                qx = -qx;
                qy = -qy;
                qz = -qz;
                qw = -qw;
            }
        }

        void limitLinVel(double &v){
            v = std::min(v, xyz_vel_limit);
            v = std::max(v, -xyz_vel_limit);
        }

        void limitRotVel(double &w){
            w = std::min(w, rpy_vel_limit);
            w = std::max(w, -rpy_vel_limit);
        }

        void pbvs(){
            vpHomogeneousMatrix cdMc; // cdMc is the result of a pose estimation; cd: desired camera frame, c:current camera frame

            // Creation of the current visual feature s = (c*_t_c, ThetaU)
            vpFeatureTranslation s_t(vpFeatureTranslation::cdMc);
            vpFeatureThetaU s_tu(vpFeatureThetaU::cdRc);

            // Set the initial values of the current visual feature s = (c*_t_c, ThetaU)
            s_t.buildFrom(cdMc);
            s_tu.buildFrom(cdMc);

            // Build the desired visual feature s* = (0,0)
            vpFeatureTranslation s_star_t(vpFeatureTranslation::cdMc); // Default initialization to zero
            vpFeatureThetaU s_star_tu(vpFeatureThetaU::cdRc); // Default initialization to zero
            vpColVector v; // Camera velocity
            double error = 1.0;  // Task error

            // Creation of the visual servo task.
            vpServo task;
            // Visual servo task initialization
            // - Camera is monted on the robot end-effector and velocities are
            //   computed in the camera frame
            task.setServo(vpServo::EYEINHAND_CAMERA);
            // - Interaction matrix is computed with the current visual features s
            task.setInteractionMatrixType(vpServo::CURRENT);

            // vpAdaptiveGain lambda;
            // lambda.initStandard(4, 0.4, 30);

            // - Set the contant gain to 1
            task.setLambda(pbvs_control_law_gain_lambda);
            // - Add current and desired translation feature
            task.addFeature(s_t, s_star_t);
            // - Add current and desired ThetaU feature for the rotation
            task.addFeature(s_tu, s_star_tu);
            // Visual servoing loop. The objective is here to update the visual
            // features s = (c*_t_c, ThetaU), compute the control law and apply
            // it to the robot

            ros::Rate rate(pbvs_control_loop_hz);
            while(nh.ok()){
                try{
                    // lookup desired camera from and current camera frame transform
                    tf_transform = tf_buffer.lookupTransform(desired_camera_frame, current_camera_frame, ros::Time::now(), ros::Duration(0.5));

                    // convert transform to vpHomogeneousMatrix
                    double t_x = tf_transform.transform.translation.x;
                    double t_y = tf_transform.transform.translation.y;
                    double t_z = tf_transform.transform.translation.z;
                    vpTranslationVector trans_vec;
                    trans_vec.buildFrom(t_x, t_y, t_z);

                    double q_x = tf_transform.transform.rotation.x;
                    double q_y = tf_transform.transform.rotation.y;
                    double q_z = tf_transform.transform.rotation.z;
                    double q_w = tf_transform.transform.rotation.w;
                    fixQuat(q_x, q_y, q_z, q_w);
                    vpQuaternionVector quat_vec;
                    quat_vec.buildFrom(q_x, q_y, q_z, q_w);

                    // vpHomogeneousMatrix cdMc;
                    cdMc.buildFrom(trans_vec, quat_vec);

                    // PBVS
                    // ... cdMc is here the result of a pose estimation
                    // Update the current visual feature s
                    s_t.buildFrom(cdMc);  // Update translation visual feature
                    s_tu.buildFrom(cdMc); // Update ThetaU visual feature
                    v = task.computeControlLaw(); // Compute camera velocity skew
                    error = (task.getError()).sumSquare(); // error = s^2 - s_star^2

                    // convert twist in camera frame to body frame
                    // rearranging twist from [v w] to [w v]
                    Eigen::VectorXd Vc(6);
                    Vc << v[3], v[4], v[5], v[0], v[1], v[2];

                    // lookup desired camera from and robot body frame transform
                    Eigen::VectorXd Vb(6);
                    tf_transform = tf_buffer.lookupTransform(robot_base_frame, current_camera_frame, ros::Time::now(), ros::Duration(3.0));
                    getTwistVectorBodyFrame(Vb, Vc, tf2::transformToEigen(tf_transform).matrix());

                    // limit linear and rotational velocity
                    limitLinVel(Vb[3]);
                    limitLinVel(Vb[4]);
                    limitLinVel(Vb[5]);
                    limitRotVel(Vb[0]);
                    limitRotVel(Vb[1]);
                    limitRotVel(Vb[2]);

                    // command end effector twist to robot
                    if(error >= pbvs_control_deadband_error){
                        control_input.velocity.data = {
                            static_cast<float>(Vb[3]), static_cast<float>(Vb[4]), static_cast<float>(Vb[5]), 
                            static_cast<float>(Vb[0]), static_cast<float>(Vb[1]), static_cast<float>(Vb[2])
                        };
                    }
                    else{
                        control_input.velocity.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    }
                }
                catch(tf2::TransformException ex){
                    ROS_ERROR("%s", ex.what());
                    control_input.velocity.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                }

                cart_vel_pub.publish(control_input);

                // std::stringstream ss0;
                // ss0.str(std::string());
                // ss0 << "body frame v[v w]:\n";
                // ss0 << control_input.velocity.data[0] << " \n";
                // ss0 << control_input.velocity.data[1] << " \n";
                // ss0 << control_input.velocity.data[2] << " \n";
                // ss0 << control_input.velocity.data[3] << " \n";
                // ss0 << control_input.velocity.data[4] << " \n";
                // ss0 << control_input.velocity.data[5] << " \n";
                // ss0 << "\n";
                // ROS_INFO("\n%s", ss0.str().c_str());

                rate.sleep();
            }
            task.kill();
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "visual_servoing");
    ros::NodeHandle nh("~");
    VisualServoing vs(nh);
    ros::spin();
    return 0;
}