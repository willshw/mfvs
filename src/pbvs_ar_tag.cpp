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

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;
        geometry_msgs::TransformStamped tf_transform;
        ros::Publisher cart_vel_pub;

        //parameters
        double pbvs_control_loop_hz;
        double pbvs_control_law_gain_lambda;
        double pbvs_control_deadband_error;

        std::string desired_camera_frame;
        std::string current_camera_frame;
        std::string robot_body_frame;

        std::string control_input_topic;
        double control_input_topic_hz;

        arm_vs::CartVelCmd control_input;

    public:
        VisualServoing(ros::NodeHandle node_handle){
            nh = node_handle;
            VisualServoing::getParametersValues();

            tf_listener = new tf2_ros::TransformListener(tf_buffer);
            cart_vel_pub = nh.advertise<arm_vs::CartVelCmd>(control_input_topic, control_input_topic_hz);

            VisualServoing::pbvs();
        }
        
        void getParametersValues(){
            nh.param<double>("pbvs_control_loop_hz", pbvs_control_loop_hz, 50.0);
            nh.param<double>("pbvs_control_gain_lambda", pbvs_control_law_gain_lambda, 1.0);
            nh.param<double>("pbvs_control_deadband_error", pbvs_control_deadband_error, 0.0001);

            if(!nh.hasParam("desired_camera_frame") || !nh.hasParam("current_camera_frame")){
                ROS_FATAL("Camera frame parameters are not correctly set!");
            }
            else{
                nh.param<std::string>("desired_camera_frame", desired_camera_frame, "/desired_cam_frame");
                nh.param<std::string>("current_camera_frame", current_camera_frame, "/camera_rgb_optical_frame");
            }

            if(!nh.hasParam("robot_body_frame")){
                ROS_FATAL("Robot body frame is not correctly set!");
            }
            else{
                nh.param<std::string>("robot_body_frame", robot_body_frame, "/robot_body_frame");
            }

            if(!nh.hasParam("control_input_topic")){
                ROS_FATAL("Control input topic is not correctly set!");
            }
            else{
                nh.param<std::string>("control_input_topic", control_input_topic, "/control_input");
            }

            nh.param<double>("control_input_topic_hz", control_input_topic_hz, 50.0);
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

        // void ibvs(){
        //     vpHomogeneousMatrix cdMo(0.0, 0.0, 0.5, 0.0, 0.0, 0.0); // cdMo is the result of a pose estimation; cd: desired camera frame, o:object frame

        //     vpPoints point[1];
        //     point[0].setWorldCoordinates(0.0, 0.0, 0.0);

        //     vpServo task;
        //     task.setServo(vpServo::EYEINHAND_CAMERA);
        //     task.setInteractionMatrixType(vpServo::CURRENT);
        //     task.setLambda(0.5);

        //     vpFeaturePoint p[1], pd[1];
        //     point[0].track(cdMo);
        //     vpFeatureBuilder::create(pd[0], point[0]);
        //     point[0].track(cMo);
        //     vpFeatureBuilder::create(p[0], point[0]);

        //     task.addFeature(p[0], pd[0]);
            
        //     vpHomogeneousMatrix wMc, wMo;

        // }

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
                    tf_transform = tf_buffer.lookupTransform(desired_camera_frame, current_camera_frame, ros::Time::now(), ros::Duration(3.0));

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

                    std::stringstream ss;
                    ss.str(std::string());
                    ss << "v:\n" << Vc << "\n";
                    ss << "error: " << error << "\n";
                    ROS_INFO("\n%s", ss.str().c_str());

                    // lookup desired camera from and robot body frame transform
                    Eigen::VectorXd Vb(6);
                    tf_transform = tf_buffer.lookupTransform(current_camera_frame, robot_body_frame, ros::Time::now(), ros::Duration(3.0));
                    getTwistVectorBodyFrame(Vb, Vc, tf2::transformToEigen(tf_transform).matrix());

                    // command end effector twist to robot
                    if(error >= pbvs_control_deadband_error){
                        control_input.velocity.data = {Vb[3], Vb[4], Vb[5], Vb[0], Vb[1], Vb[2]};
                    }
                    else{
                        control_input.velocity.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    }
                    cart_vel_pub.publish(control_input);
                }
                catch(tf2::TransformException ex){
                    ROS_ERROR("%s", ex.what());
                }

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