#include <vector>
#include <iostream>

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
#include <std_msgs/String.h>
// #include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <arm_vs/CartVelCmd.h>


class VisualServoing{
    private:
        ros::NodeHandle nh;
        ros::Publisher cart_vel_pub = nh.advertise<arm_vs::CartVelCmd>("control_input", 50);

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;
        geometry_msgs::TransformStamped tf_transform;

        std::string desired_camera_frame;
        std::string current_camera_frame;
        std::string robot_body_frame;

        // camera velocity
        // end effector velocity

    public:
        void getParametersValues(){
            if(!nh.hasParam("desired_camera_frame") || !nh.hasParam("current_camera_frame")){
                ROS_INFO("Camera frame parameters are not correctly set!");
            }
            else{
                nh.param<std::string>("desired_camera_frame", desired_camera_frame, "/desired_cam_frame");
                nh.param<std::string>("current_camera_frame", current_camera_frame, "/camera_rgb_optical_frame");
            
                ROS_INFO("\nDesired camera frame: %s \nCurrent camera frame: %s", desired_camera_frame.c_str(), current_camera_frame.c_str());
            }

            if(!nh.hasParam("robot_body_frame")){
                ROS_INFO("Robot body frame is not correctly set!");
            }
            else{
                nh.param<std::string>("robot_body_frame", robot_body_frame, "/robot_body_frame");

                ROS_INFO("Robot body frame: %s", robot_body_frame.c_str());
            }
        }

        void getTwistVectorBodyFrame(Eigen::VectorXd& Vb, Eigen::VectorXd Vc, Eigen::Matrix4d bMc){
            std::stringstream ss;
            // ROS_INFO("Here");
            // Eigen::Matrix4d cMb = cMb_affine.matrix();

            ss << "Vc:\n" << Vc << "\n";
            ROS_INFO("\n%s", ss.str().c_str());

            // calculate adjoint using input transformation
            // [  R  0]
            // [[tR] R]
            Eigen::Matrix3d bRc = bMc.block<3,3>(0,0); // rotation
            Eigen::Vector3d btc = bMc.block<3,1>(0,3); // translation

            // ss.str(std::string());
            // ss << "Roataion Matrix:\n" << cRb << "\n" << "Translation Vector:\n" << ctb << "\n";
            // ROS_INFO("\n%s", ss.str().c_str());

            // skew symmetric [t]
            Eigen::Matrix3d btc_s;
            btc_s << 0, -btc(2), btc(1),
                    btc(2), 0, -btc(0),
                    -btc(1), btc(0), 0;

            // ss.str(std::string());
            // ss << "Roataion Skew Matrix:\n" << ctb_s << "\n";
            // ROS_INFO("\n%s", ss.str().c_str());

            Eigen::MatrixXd bAdc(6, 6);
            
            bAdc << bRc, Eigen::Matrix3d::Zero(), 
                    btc_s*bRc, bRc;

            // ss.str(std::string());
            // ss << "Adjoint Matrix:\n" << cAdb << "\n";
            // ROS_INFO("\n%s", ss.str().c_str());

            // calculate twist in body frame using adjoint and twist in camera frame
            Vb = bAdc * Vc;

            ss.str(std::string());
            ss << "Vb:\n" << Vb << "\n";
            ROS_INFO("\n%s", ss.str().c_str());
        }

        void pbvs(){

            vpHomogeneousMatrix cdMc;
            // ... cdMc is here the result of a pose estimation
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
            // - Set the contant gain to 1
            task.setLambda(1);
            // - Add current and desired translation feature
            task.addFeature(s_t, s_star_t);
            // - Add current and desired ThetaU feature for the rotation
            task.addFeature(s_tu, s_star_tu);
            // Visual servoing loop. The objective is here to update the visual
            // features s = (c*_t_c, ThetaU), compute the control law and apply
            // it to the robot

            ros::Rate rate(30.0);
            while(nh.ok() && (error > 0.0001)){
                try{
                    // lookup desired camera from and current camera frame transform
                    tf_transform = tf_buffer.lookupTransform(current_camera_frame, desired_camera_frame, ros::Time::now(), ros::Duration(3.0));

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
                    vpQuaternionVector quat_vec;
                    quat_vec.buildFrom(q_x, q_y, q_z, q_w);

                    vpHomogeneousMatrix cdMc;
                    cdMc.buildFrom(trans_vec, quat_vec);

                    // PBVS
                    // ... cdMc is here the result of a pose estimation
                    // Update the current visual feature s
                    s_t.buildFrom(cdMc);  // Update translation visual feature
                    s_tu.buildFrom(cdMc); // Update ThetaU visual feature
                    v = task.computeControlLaw(); // Compute camera velocity skew
                    error =  ( task.getError() ).sumSquare(); // error = s^2 - s_star^2
                
                    // std_msgs::String msg;
                    std::stringstream ss;
                    ss << "V:";
                    for (int i = 0; i < v.size(); i++)
                        ss << ' ' << v[i];
                    ss << '\n';
                    // msg.data = ss.str();
                    // pub.publish(msg);
                    ROS_INFO("%s", ss.str().c_str());

                    // convert twist in camera frame to body frame
                    // rearranging twist from [v w] to [w v]
                    Eigen::VectorXd Vc(6);
                    Vc << v[3], v[4], v[5], v[0], v[1], v[2];

                    // lookup desired camera from and robot body frame transform
                    Eigen::VectorXd Vb(6);
                    tf_transform = tf_buffer.lookupTransform(current_camera_frame, robot_body_frame, ros::Time::now(), ros::Duration(3.0));
                    getTwistVectorBodyFrame(Vb, Vc, tf2::transformToEigen(tf_transform).matrix());

                    // try{
                    //     tf_listener.lookupTransform(current_camera_frame, desired_camera_frame, ros::Time(0), tf_transform);
                    // }
                    // catch(tf::TransformException ex){
                    //     ROS_ERROR("%s", ex.what());
                    //     ros::Duration(1.0).sleep();
                    // }


                    // command end effector twist to robot
                    arm_vs::CartVelCmd control_input;
                    control_input.velocity.data = {Vb[3], Vb[4], Vb[5], Vb[0], Vb[1], Vb[2]};
                    cart_vel_pub.publish(control_input);
                }
                catch(tf2::TransformException ex){
                    ROS_ERROR("%s", ex.what());
                }

                rate.sleep();
            }
            task.kill();
        }

        VisualServoing(ros::NodeHandle node_handle){
            nh = node_handle;
            tf_listener = new tf2_ros::TransformListener(tf_buffer);

            getParametersValues();
            pbvs();
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "visual_servoing");
    ros::NodeHandle nh("~");
    VisualServoing vs(nh);
    ros::spin();
    return 0;
}