#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include "stonefish_ros/BeaconInfo.h"
#include "transforms3d/euler.h"

using namespace Eigen;

class DifferentialDrive {
public:
    DifferentialDrive() {
        // camera position with respect to the robot
        r_cx = 0.122; // this will be fixed
        r_cy = -0.033;
        r_cz = 0.082;
        r_c_roll = M_PI / 2.0;
        r_c_pitch = 0.0;
        r_c_yaw = M_PI / 2.0;
        current_time = 0;
        v = 0;
        w = 0;
        m = 1;

        // robot constants
        wheel_radius = 0.035; // meters
        wheel_base_distance = 0.23; // meters

        camera << r_cx, r_cy, r_cz, r_c_roll, r_c_pitch, r_c_yaw;

        // initial pose of the robot
        Xk = Vector4d::Zero();

        // covariance matrices
        Pk << 0.2*0.2, 0, 0, 0,
              0, 0.2*0.2, 0, 0,
              0, 0, 0.0, 0,
              0, 0, 0, 0.5*0.5;

        Qk << 0.02*0.02, 0,
              0, 0.02*0.02;

        Rk << 0.2*0.2, 0, 0,
              0, 0.4*0.4, 0,
              0, 0, 0.4*0.4;

        odom_pub = nh.advertise<nav_msgs::Odometry>("kobuki/odom", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/beacons_viz", 10);
        marker_pub1 = nh.advertise<visualization_msgs::Marker>("/gps/ellipsoid_marker", 10);

        js_sub = nh.subscribe("kobuki/joint_states", 1, &DifferentialDrive::joint_state_callback, this);
        imu_sub = nh.subscribe("/kobuki/sensors/imu", 1, &DifferentialDrive::imu_callback, this);
        aruco_pose_sub = nh.subscribe("/aruco_position", 1, &DifferentialDrive::aruco_position, this);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        imu_vel = imu_msg->angular_velocity.z;
        tf2::Quaternion q;
        tf2::fromMsg(imu_msg->orientation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(Imu_roll, Imu_pitch, Imu_yaw);

        // observation from IMU angle
        double Zk = Imu_yaw;

        double Vk = 1.0;
        double Rk = 0.2*0.2;

        MatrixXd I = MatrixXd::Identity(Pk.rows(), Pk.cols());
        MatrixXd Hk = MatrixXd::Zero(1, Pk.cols());
        Hk(0, 3) = 1;

        double Ykn_angle = Zk - Hk * Xk;

        MatrixXd before_inv = Hk * (Pk * Hk.transpose()) + (Vk * Rk * Vk);
        MatrixXd inv = before_inv.inverse();
        MatrixXd K_gain = Pk * Hk.transpose() * inv;

        Xk = Xk + K_gain * Ykn_angle;
        Pk = (I - K_gain * Hk) * Pk;
    }

    void transform_r_c(double x, double y, double z, double roll, double pitch, double yaw, Matrix4d& Transf) {
        Matrix3d Rx;
        Rx << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);

        Matrix3d Rz;
        Rz << cos(yaw), -sin(yaw), 0,
              sin(yaw), cos(yaw), 0,
              0, 0, 1;

        Matrix3d R = Rz * Rx;

        Vector3d Trans;
        Trans << x, y, z;

        Transf.block<3, 3>(0, 0) = R;
        Transf.block<3, 1>(0, 3) = Trans;
    }

    void transform_r_c_inv(double x, double y, double z, double roll, double pitch, double yaw, Matrix4d& Transf) {
        Matrix3d Rx;
        Rx << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);

        Matrix3d Rz;
        Rz << cos(yaw), -sin(yaw), 0,
              sin(yaw), cos(yaw), 0,
              0, 0, 1;

        Matrix3d R = Rz * Rx;

        Vector3d Trans;
        Trans << x, y, z;

        Transf.block<3, 3>(0, 0) = R.transpose();
        Transf.block<3, 1>(0, 3) = -R.transpose() * Trans;
    }

    double wrap_angle(double ang) {
        return ang + (2.0 * M_PI * std::floor((M_PI - ang) / (2.0 * M_PI)));
    }

    void aruco_position(const std_msgs::Float64MultiArray::ConstPtr& beacon) {
        c_fx = beacon->data[0];
        c_fy = beacon->data[1];
        c_fz = beacon->data[2];
        id = static_cast<int>(beacon->data[3]);

        if (id != 0) {
            c_f << c_fx, c_fy, c_fz;

            range = sqrt(c_fx * c_fx + c_fy * c_fy + c_fz * c_fz);
            azimuth = atan2(c_fy, c_fx);
            azimuth = wrap_angle(azimuth);
            elevation = atan2(c_fz, sqrt(c_fx * c_fx + c_fy * c_fy));
            elevation = wrap_angle(elevation);

            Matrix3d J_p2c;
            J_p2c << cos(elevation) * cos(azimuth), -range * cos(elevation) * sin(azimuth), -range * sin(elevation) * cos(azimuth),
                    cos(elevation) * sin(azimuth), range * cos(elevation) * cos(azimuth), -range * sin(elevation) * sin(azimuth),
                    sin(azimuth), 0, range * cos(elevation);

            Matrix3d Rk_c = J_p2c * Rk * J_p2c.transpose();

            update(c_f, Rk_c, id);
            quad(Xk, Pk, current_time, v, w);
        }
    }

    void update(const Eigen::Vector3d& c_f, const Eigen::Matrix3d& Rk_c, int id) {
        this->c_f = c_f;
        this->c_fx = c_f(0,0);
        this->c_fy = c_f(1,0);
        this->c_fz = c_f(2,0);
        this->id = id;

        double roll = camera[3];
        double yaw = camera[5];

        if (std::find(beacon_list.begin(), beacon_list.end(), id) == beacon_list.end()) {
            Xk(3,0) = wrap_angle(Xk(3,0));

            beacon_list.push_back(id);
            std::cout << "self.beacon_list: ";
            for (auto& elem : beacon_list) {
                std::cout << elem << " ";
            }
            std::cout << std::endl;

            Xk.conservativeResize(Xk.rows() + 3, Eigen::NoChange);
            Hk.conservativeResize(Hk.rows(), Hk.cols() + 3);
            g1.conservativeResize(g1.rows() + 3, g1.cols() + 3);
            g2.conservativeResize(g2.rows() + 3, g2.cols());

            F1.conservativeResize(F1.rows() + 3, F1.cols() + 3);
            F1.bottomRightCorner(3, 3) = F1_I;
            F2.conservativeResize(F2.rows() + 3, F2.cols());

            Eigen::MatrixXd I4 = Eigen::MatrixXd::Identity(4, 4);
            Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
            Eigen::MatrixXd O = Eigen::MatrixXd::Zero(3, 2);
            Eigen::MatrixXd Vk = Eigen::MatrixXd::Identity(3, 3);

            auto [Trans, rot, _] = transform_r_c(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5]);
            rm_r_c = Trans + rot * c_f;

            x_b = rm_r_c(0, 0);
            y_b = rm_r_c(1, 0);
            z_b = rm_r_c(2, 0);

            Eigen::MatrixXd J2 = rot;

            Rk_r = J2 * Rk_c * J2.transpose();

            Eigen::MatrixXd Zk = Eigen::Vector3d(x_b, y_b, z_b);
            Eigen::MatrixXd Zk_p(3, 1);
            Zk_p << Xk(0, 0) + Zk(0, 0) * std::cos(Xk(3, 0)) - Zk(1, 0) * std::sin(Xk(3, 0)),
                    Xk(1, 0) + Zk(0, 0) * std::sin(Xk(3, 0)) + Zk(1, 0) * std::cos(Xk(3, 0)),
                    Xk(2, 0) + Zk(2, 0);

            Xk.bottomRows(3) = Zk_p;

            fx_x = c_fy * std::sin(yaw) * std::sin(roll) + c_fz * std::sin(yaw) * std::cos(roll);
            fx_y = -c_fy * std::cos(yaw) * std::sin(roll) - c_fz * std::cos(yaw) * std::cos(roll);

            Eigen::MatrixXd J_1_p(3, 4);
            J_1_p << 1, 0, 0, -Zk(0, 0) * std::sin(Xk(3, 0)) - Zk(1, 0) * std::cos(Xk(3, 0)),
                    0, 1, 0, Zk(0, 0) * std::cos(Xk(3, 0)) - Zk(1, 0) * std::sin(Xk(3, 0)),
                    0, 0, 1, 0;

            Eigen::MatrixXd J_2_p(3, 3);
            J_2_p << std::cos(Xk(3, 0)), -std::sin(Xk(3, 0)), 0,
                    std::sin(Xk(3, 0)), std::cos(Xk(3, 0)), 0,
                    0, 0, 1;

            if (m == 1) {
                g1.block(0, 0, 4, 4) = I4;
                g1.bottomLeftCorner(3, 4) = J_1_p;
                m = 2;
            } else {
                g1.block(g1.rows() - 6, g1.cols() - 3, 3, 3) = I3;
                g1.bottomLeftCorner(3, 4) = J_1_p;
            }

            g2.setZero();
            g2.bottomRows(3) = J_2_p;

            Pk = g1 * Pk * g1.transpose() + g2 * Rk_r * g2.transpose();

            g1.bottomLeftCorner(3, 4).setZero();

            map12[num_2] = 0.0;
            num_2++;
        } else {
            int idx = std::distance(beacon_list.begin(), std::find(beacon_list.begin(), beacon_list.end(), id));
            int f_i_1 = 4 + 3 * idx;
            int f_i_2 = 7 + 3 * idx;

            Xk(3, 0) = wrap_angle(Xk(3, 0));
            double new_angle = Xk(3, 0);

            Eigen::MatrixXd Zk1 = c_f;
            Eigen::MatrixXd I_1 = Eigen::MatrixXd::Identity(Pk.rows(), Pk.cols());
            Eigen::MatrixXd Vk = Eigen::MatrixXd::Identity(3, 3);

            Eigen::MatrixXd map = Xk.block(f_i_1, 0, 3, 1);

            double th = Xk(3, 0);
            Eigen::MatrixXd Z_r_f(3, 1);
            Z_r_f << -Xk(0, 0) * std::cos(th) - Xk(1, 0) * std::sin(th) + map(0, 0) * std::cos(th) + map(1, 0) * std::sin(th),
                    Xk(0, 0) * std::sin(th) - Xk(1, 0) * std::cos(th) - map(0, 0) * std::sin(th) + map(1, 0) * std::cos(th),
                    -rz + map(2, 0);

            auto [Trans_c_r, rot_c_r, _] = transform_r_c_inv(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5]);
            Eigen::MatrixXd h_x = Trans_c_r + rot_c_r * Z_r_f;

            Eigen::MatrixXd Ykn = Zk1 - h_x;

            Hk.setZero();
            Hk.block(0, 0, 3, 4) << -std::cos(th) * std::cos(yaw) + std::sin(yaw) * std::sin(th),
                                    -std::cos(yaw) * std::sin(th) - std::sin(yaw) * std::cos(th),
                                    0,
                                    std::cos(roll) * std::cos(yaw) * std::sin(th) + std::cos(roll) * std::sin(yaw) * std::cos(th),
                                    -std::cos(roll) * std::cos(yaw) * std::cos(th) + std::cos(roll) * std::sin(yaw) * std::sin(th),
                                    -std::sin(roll),
                                    -std::sin(roll) * std::cos(yaw) * std::sin(th) - std::sin(roll) * std::sin(yaw) * std::cos(th),
                                    std::sin(roll) * std::cos(yaw) * std::cos(th) - std::sin(roll) * std::sin(yaw) * std::sin(th),
                                    std::cos(roll);

            Hk.block(0, f_i_1, 3, 3) << std::cos(th) * std::cos(yaw) - std::sin(yaw) * std::sin(th),
                                        -std::cos(yaw) * std::sin(th) - std::sin(yaw) * std::cos(th),
                                        0,
                                        std::cos(roll) * std::cos(yaw) * std::sin(th) + std::cos(roll) * std::sin(yaw) * std::cos(th),
                                        -std::cos(roll) * std::cos(yaw) * std::cos(th) + std::cos(roll) * std::sin(yaw) * std::sin(th),
                                        -std::sin(roll),
                                        -std::sin(roll) * std::cos(yaw) * std::sin(th) - std::sin(roll) * std::sin(yaw) * std::cos(th),
                                        std::sin(roll) * std::cos(yaw) * std::cos(th) - std::sin(roll) * std::sin(yaw) * std::sin(th),
                                        std::cos(roll);

            Eigen::MatrixXd before_inv = Hk * (Pk * Hk.transpose()) + Vk * (Rk_c * Vk.transpose());
            Eigen::MatrixXd inv = before_inv.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd K_gain = (Pk * Hk.transpose()) * inv;

            Xk += K_gain * Ykn;
            Xk(3, 0) = new_angle;

            Pk = ((I_1 - K_gain * Hk) * Pk * (I_1 - K_gain * Hk).transpose()).eval();

            for (int m = 0; m < map12.size(); ++m) {
                map12[m] = { Xk(3 * m + 4, 0), Xk(3 * m + 5, 0), Xk(3 * m + 6, 0) };
            }
            modem_visualization(map12);
        }
    }

    void modem_visualization(const std::vector<std::vector<double>>& map) {
        this->map = map;

        visualization_msgs::MarkerArray ma;

        for (int i = 0; i < map.size(); ++i) {
            int idx = i;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.id = i * 2;

            marker.header.stamp = ros::Time::now();
            marker.pose.position.x = map[idx][0];
            marker.pose.position.y = map[idx][1];
            marker.pose.position.z = map[idx][2];
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.color.r = 1.0;
            marker.color.a = 0.9;

            ma.markers.push_back(marker);

            visualization_msgs::Marker marker_text;
            marker_text.header.frame_id = "world";
            marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_text.action = visualization_msgs::Marker::ADD;
            marker_text.id = i * 2 + 1;

            marker_text.header.stamp = ros::Time::now();
            marker_text.pose.position.x = map[idx][0];
            marker_text.pose.position.y = map[idx][1];
            marker_text.pose.position.z = map[idx][2] + 1.0;
            marker_text.pose.orientation.w = 1.0;

            marker_text.scale.x = 0.1;
            marker_text.scale.y = 0.1;
            marker_text.scale.z = 0.3;

            marker_text.color.r = 1.0;
            marker_text.color.a = 0.9;

            std::stringstream ss;
            ss << "Aruco " << idx;
            marker_text.text = ss.str();

            ma.markers.push_back(marker_text);

            Eigen::MatrixXd cov = Pk.block(i * 3 + 4, i * 3 + 4, 3, 3);

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(cov);
            Eigen::MatrixXd eigenvalues = eigensolver.eigenvalues();
            Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors();

            Eigen::VectorXd major_axis_eigenvector = eigenvectors.col(eigensolver.eigenvalues().argMax());

            double roll_m = std::atan2(major_axis_eigenvector(1), major_axis_eigenvector(0));

            double confidence_factor = 2.0;
            double major_axis_length = confidence_factor * std::sqrt(eigenvalues.maxCoeff());
            double minor_axis_length = confidence_factor * std::sqrt(eigenvalues.minCoeff());

            double orientation_angle = std::atan2(major_axis_eigenvector(1), major_axis_eigenvector(0));

            double pitch_m = std::asin(-major_axis_eigenvector(2));
            double yaw_m = std::atan2(cov(2, 1), cov(2, 2));

            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll_m, pitch_m, yaw_m);

            visualization_msgs::Marker marker1;
            marker1.header.frame_id = "world";
            marker1.header.stamp = ros::Time::now();
            marker1.ns = "feature_ellipsoid";
            marker1.id = i * 100 + 1;
            marker1.type = visualization_msgs::Marker::SPHERE;
            marker1.action = visualization_msgs::Marker::ADD;
            marker1.color = std_msgs::ColorRGBA(0.0, 1.0, 0.0, 1.0);

            marker1.pose.position.x = map[idx][0];
            marker1.pose.position.y = map[idx][1];
            marker1.pose.position.z = map[idx][2];

            marker1.pose.orientation.x = 0.0;
            marker1.pose.orientation.y = 0.0;
            marker1.pose.orientation.z = 0.0;
            marker1.pose.orientation.w = 1.0;

            marker1.scale.x = minor_axis_length * 2;
            marker1.scale.y = major_axis_length;
            marker1.scale.z = 0.01;

            ma.markers.push_back(marker1);
        }

        marker_pub.publish(ma);
    }


    void prediction(double left_wheel_velocity, double right_wheel_velocity, const sensor_msgs::JointState& msg) {
        this->left_wheel_velocity = left_wheel_velocity;
        this->right_wheel_velocity = right_wheel_velocity;

        double left_lin_vel = left_wheel_velocity * wheel_radius;
        double right_lin_vel = right_wheel_velocity * wheel_radius;

        v = (left_lin_vel + right_lin_vel) / 2.0;
        w = (left_lin_vel - right_lin_vel) / wheel_base_distance;

        // Calculate dt
        current_time = ros::Time(msg.header.stamp.sec, msg.header.stamp.nsec);
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        // Predicted state update
        Xk(3, 0) = wrap_angle(Xk(3, 0));
        Xk(0, 0) += std::cos(Xk(3, 0)) * v * dt;
        Xk(1, 0) += std::sin(Xk(3, 0)) * v * dt;
        Xk(2, 0) = rz; // Assuming rz is defined somewhere
        Xk(3, 0) += w * dt;
        Xk(3, 0) = wrap_angle(Xk(3, 0)); // Wrap angle

        // Jacobians
        Eigen::MatrixXd Ak(4, 4);
        Ak << 1, 0, 0, -std::sin(Xk(3, 0)) * v * dt,
              0, 1, 0, std::cos(Xk(3, 0)) * v * dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

        Eigen::MatrixXd Bk(4, 2);
        Bk << std::cos(Xk(3, 0)) * dt * 0.5 * wheel_radius, std::cos(Xk(3, 0)) * dt * 0.5 * wheel_radius,
              std::sin(Xk(3, 0)) * dt * 0.5 * wheel_radius, std::sin(Xk(3, 0)) * dt * 0.5 * wheel_radius,
              0, 0,
              (dt * wheel_radius) / wheel_base_distance, -(dt * wheel_radius) / wheel_base_distance;

        // Update covariance
        F1.block<4, 4>(0, 0) = Ak;
        F2.block<4, 2>(0, 0) = Bk;
        Pk = F1 * Pk * F1.transpose() + F2 * Qk * F2.transpose();

        // Return Xk and Pk
        // In C++ we directly modify the class members, no need to return them
    }

    void quad(const Eigen::MatrixXd& Xk, const Eigen::MatrixXd& Pk, const ros::Time& current_time, double v, double w) {
        this->Xk = Xk;
        this->Pk = Pk;
        this->current_time = current_time;
        this->v = v;
        this->w = w;

        // Create quaternion from Euler angles (assuming Xk[3, 0] is the yaw)
        tf::Quaternion q;
        q.setRPY(0, 0, Xk(3, 0));

        // Create odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";
        odom.child_frame_id = "kobuki/base_footprint";

        odom.pose.pose.position.x = Xk(0, 0);
        odom.pose.pose.position.y = Xk(1, 0);
        odom.pose.pose.position.z = Xk(2, 0);

        odom.pose.pose.orientation.x = q.getX();
        odom.pose.pose.orientation.y = q.getY();
        odom.pose.pose.orientation.z = q.getZ();
        odom.pose.pose.orientation.w = q.getW();

        // Set covariance matrix
        Eigen::Map<Eigen::Matrix<double, 6, 6>> covariance(odom.pose.covariance.data());
        covariance <<
            Pk(0, 0), Pk(0, 1), Pk(0, 2), 0, 0, Pk(0, 3),
            Pk(1, 0), Pk(1, 1), Pk(1, 2), 0, 0, Pk(1, 3),
            Pk(2, 0), Pk(2, 1), Pk(2, 2), 0, 0, Pk(2, 3),
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            Pk(3, 0), Pk(3, 1), Pk(3, 2), 0, 0, Pk(3, 3);

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        // Publish odometry
        odom_pub.publish(odom);

        // Broadcast tf transform
        tf_br.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW()),
                          tf::Vector3(Xk(0, 0), Xk(1, 0), Xk(2, 0))),
                          current_time, "world", "kobuki/base_footprint"));
    }

    void joint_state_callback(const sensor_msgs::JointState& msg) {
        if (msg.name[0] == "kobuki/wheel_left_joint") {
            left_wheel_velocity = msg.velocity[0];
            left_wheel_received = true;
        } else if (msg.name[0] == "kobuki/wheel_right_joint") {
            right_wheel_velocity = msg.velocity[0];
            if (left_wheel_received) {
                left_wheel_received = false;

                // Predict and publish odometry
                prediction(left_wheel_velocity, right_wheel_velocity, msg);
                quad(Xk, Pk, current_time, v, w);
            }
        }
    }

private:
    double wrap_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_br;

    double left_wheel_velocity;
    double right_wheel_velocity;
    double v;
    double w;

    ros::Time current_time;
    ros::Time last_time;

    Eigen::MatrixXd Xk;
    Eigen::MatrixXd Pk;
    Eigen::MatrixXd F1;
    Eigen::MatrixXd F2;
    Eigen::MatrixXd Qk; // Define Qk appropriately

    double wheel_radius;
    double wheel_base_distance;

    bool left_wheel_received = false;

    double rz; // Assuming rz is defined somewhere
};

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "differential_drive");
    ros::NodeHandle nh;

    // Create an instance of DifferentialDrive class
    DifferentialDrive robot;

    // Enter ROS event loop
    ros::spin();

    return 0;
}