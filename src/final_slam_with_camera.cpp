#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include "stonefish_ros/BeaconInfo.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <cmath>
#include <vector>
#include <unordered_map>

class DifferentialDrive {
public:
    DifferentialDrive(double sensor_noise, double sensor_noise_angle)
        : r_cx(0.5), r_cy(-0.06), r_cz(0.082), r_c_roll(M_PI / 2), r_c_pitch(0.0), r_c_yaw(M_PI / 2),
          m(1), wheel_radius(0.035), wheel_base_distance(0.23),
          camera({r_cx, r_cy, r_cz, r_c_roll, r_c_pitch, r_c_yaw}),
          Xk(Eigen::VectorXd::Zero(10)),
          lin_vel(0.0), ang_vel(0.0), current_time(0), v(0), w(0),
          left_wheel_velocity(0.0), right_wheel_velocity(0.0),
          left_wheel_received(false),
          last_time(ros::Time::now()),
          id(0), beacon_list(), rz(0.0),
          Hk(Eigen::MatrixXd::Zero(3, 10)),
          F1(Eigen::MatrixXd::Zero(10, 10)),
          F1_I(Eigen::MatrixXd::Identity(3, 3)),
          F2(Eigen::MatrixXd::Zero(10, 2)),
          F2_o(Eigen::MatrixXd::Zero(3, 3)),
          g1(Eigen::MatrixXd::Zero(10, 7)),
          g2(Eigen::MatrixXd::Zero(10, 3)),
          num(0), num_2(0),
          map12(),
          Pk_r(Eigen::MatrixXd(4, 4)),
          Qk(Eigen::MatrixXd(2, 2)),
          Rk(Eigen::MatrixXd(3, 3)),
          r_P_c(Eigen::MatrixXd::Zero(6, 6)),
          Pk(Eigen::MatrixXd::Zero(10, 10)),
          fv(nullptr),
          odom_pub(nh.advertise<nav_msgs::Odometry>("kobuki/odom", 10)),
          marker_pub(nh.advertise<visualization_msgs::MarkerArray>("/beacons_viz", 10)),
          marker_pub1(nh.advertise<visualization_msgs::Marker>("/gps/ellipsoid_marker", 10)),
          js_sub(nh.subscribe("kobuki/joint_states", 10, &DifferentialDrive::joint_state_callback, this)),
          aruco_pose(nh.subscribe("/aruco_position", 10, &DifferentialDrive::aruco_position, this))
    {
        Pk_r << std::pow(0.2, 2), 0, 0, 0,
                0, std::pow(0.2, 2), 0, 0,
                0, 0, 0.0, 0,
                0, 0, 0, std::pow(0.05, 2);

        Qk << std::pow(0.2, 2), 0,
              0, std::pow(0.2, 2);

        Rk << std::pow(0.2, 2), 0, 0,
              0, std::pow(0.4, 2), 0,
              0, 0, std::pow(0.4, 2);

        r_P_c.diagonal().setConstant(std::pow(2, 2));

        Pk.topLeftCorner(4, 4) = Pk_r;
        Pk.block(4, 4, 6, 6) = r_P_c;
    }

    void transform_r_c(double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d Transf = Eigen::Matrix4d::Identity();

        Eigen::Matrix3d Rx;
        Rx << 1, 0, 0,
              0, std::cos(roll), -std::sin(roll),
              0, std::sin(roll), std::cos(roll);

        Eigen::Matrix3d Rz;
        Rz << std::cos(yaw), -std::sin(yaw), 0,
              std::sin(yaw), std::cos(yaw), 0,
              0, 0, 1;

        Eigen::Matrix3d R = Rz * Rx;

        Eigen::Vector3d Trans;
        Trans << x, y, z;

        Transf.block<3, 3>(0, 0) = R;
        Transf.block<3, 1>(0, 3) = Trans;

        return Transf;
    }

    void transform_r_c_inv(double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d Transf = Eigen::Matrix4d::Identity();

        Eigen::Matrix3d Rx;
        Rx << 1, 0, 0,
              0, std::cos(roll), -std::sin(roll),
              0, std::sin(roll), std::cos(roll);

        Eigen::Matrix3d Rz;
        Rz << std::cos(yaw), -std::sin(yaw), 0,
              std::sin(yaw), std::cos(yaw), 0,
              0, 0, 1;

        Eigen::Matrix3d R = Rz * Rx;

        Eigen::Vector3d Trans;
        Trans << x, y, z;

        Eigen::Vector3d Trans_T = -R.transpose() * Trans;

        Transf.block<3, 3>(0, 0) = R.transpose();
        Transf.block<3, 1>(0, 3) = Trans_T;

        return Transf;
    }

    double wrap_angle(double ang) {
        if (ang > M_PI)
            ang -= 2 * M_PI;
        return ang;
    }

    void aruco_position(const std_msgs::Float64MultiArray& beacon) {
        double c_fx = beacon.data[0];
        double c_fy = beacon.data[1];
        double c_fz = beacon.data[2];
        id = static_cast<int>(beacon.data[3]);

        if (id != 0) {
            Eigen::Vector3d c_f(c_fx, c_fy, c_fz);
            double range = std::sqrt(std::pow(c_fx, 2) + std::pow(c_fy, 2) + std::pow(c_fz, 2));
            double azimuth = std::atan2(c_fy, c_fx);
            azimuth = wrap_angle(azimuth);
            double elevation = std::atan2(c_fz, std::sqrt(std::pow(c_fx, 2) + std::pow(c_fy, 2)));
            elevation = wrap_angle(elevation);

            Eigen::Matrix3d J_p2c;
            J_p2c << std::cos(elevation) * std::cos(azimuth), -range * std::cos(elevation) * std::sin(azimuth), -range * std::sin(elevation) * std::cos(azimuth),
                    std::cos(elevation) * std::sin(azimuth), range * std::cos(elevation) * std::cos(azimuth), -range * std::sin(elevation) * std::sin(azimuth),
                    std::sin(azimuth), 0, range * std::cos(elevation);

            Eigen::Matrix3d Rk_c = J_p2c * Rk * J_p2c.transpose();
            std::tie(Xk, Pk) = update(c_f, Rk_c, id);
            quad(Xk, Pk, current_time, v, w);
        }
    }


    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> update(const Eigen::Vector3d& c_f, const Eigen::MatrixXd& Rk_c, int id) {
        // Define constants and variables used in the function
        c_f = c_f;
        c_fx = c_f[0,0];
        c_fy = c_f[1,0];
        c_fz = c_f[2,0];
        Rk_c = Rk_c;
        id = id;
        roll = Xk[7,0];
        yaw = Xk[9,0];
        global fx_x, fx_y, fxx_x, fxx_y, Zk, Rk_r;

        if (std::find(beacon_list.begin(), beacon_list.end(), id) == beacon_list.end()) {

            // Wrapping the angles
            Xk[3,0] = wrap_angle(Xk[3,0]);
            Xk[7,0] = wrap_angle(Xk[7,0]);
            Xk[8,0] = wrap_angle(Xk[8,0]);
            Xk[9,0] = wrap_angle(Xk[9,0]);

            // Adding new beacon id to beacon_list
            beacon_list.push_back(id);

            // Increasing matrix size
            Xk.conservativeResize(Xk.size() + 3, Eigen::NoChange);
            Hk.conservativeResize(Hk.rows(), Hk.cols() + 3);
            g1.conservativeResize(g1.rows() + 3, g1.cols() + 3);
            g2.conservativeResize(g2.rows() + 3, g2.cols());
            F1.conservativeResize(F1.rows() + 3, F1.cols() + 3);
            F1.bottomRightCorner(3, 3) = F1_I;
            F2.conservativeResize(F2.rows() + 3, F2.cols());

            Eigen::Matrix4d Trans, rot;
            Eigen::Vector3d unused;
            std::tie(Trans, rot, unused) = transform_r_c(Xk[4,0], Xk[5,0], Xk[6,0], Xk[7,0], Xk[8,0], Xk[9,0]);

            rm_r_c = Trans + rot * c_f;

            // Position of the feature in robot frame
            x_b = rm_r_c[0,0];
            y_b = rm_r_c[1,0];
            z_b = rm_r_c[2,0];

            // Partial derivative of camera to robot compounding with respect to T_r_c
            Eigen::Matrix<double, 3, 6> J1;
            J1 << 1, 0, 0, c_fy * sin(yaw) * sin(roll) + c_fz * sin(yaw) * cos(roll), 0, -c_fx * sin(yaw) - c_fy * cos(yaw) * cos(roll) + c_fz * cos(yaw) * sin(roll),
                0, 1, 0, -c_fy * cos(yaw) * sin(roll) - c_fz * cos(yaw) * cos(roll), 0, c_fx * cos(yaw) - c_fy * sin(yaw) * cos(roll) + c_fz * sin(yaw) * sin(roll),
                0, 0, 1, c_fy * cos(roll) - c_fz * sin(roll), 0, 0;

            // Partial derivative of camera to robot compounding with respect to feature_in_camera
            Eigen::Matrix<double, 3, 3> J2;
            J2 << cos(yaw), -sin(yaw) * cos(roll), sin(yaw) * sin(roll),
                sin(yaw), cos(yaw) * cos(roll), -cos(yaw) * sin(roll),
                0, sin(roll), cos(roll);

            Rk_r = J1 * r_P_c * J1.transpose() + J2 * Rk_c * J2.transpose(); // Uncertainty in robot frame in cartesian coordinates

            // Position of the feature in robot frame in cartesian coordinates
            Zk = Eigen::Vector3d(x_b, y_b, z_b);

            // Feature in the world frame
            Eigen::Vector3d Zk_p(Xk[0,0] + Zk[0,0] * cos(Xk[3,0]) - Zk[1,0] * sin(Xk[3,0]),
                                Xk[1,0] + Zk[0,0] * sin(Xk[3,0]) + Zk[1,0] * cos(Xk[3,0]),
                                Xk[2,0] + Zk[2,0]);

            Xk.tail(3) = Zk_p;

            // Calculation of derivatives fx_x, fx_y, fxx_x, fxx_y
            fx_x = c_fy * sin(yaw) * sin(roll) + c_fz * sin(yaw) * cos(roll);
            fx_y = -c_fy * cos(yaw) * sin(roll) - c_fz * cos(yaw) * cos(roll);
            fxx_x = -c_fx * sin(yaw) - c_fy * cos(yaw) * cos(roll) + c_fz * cos(yaw) * sin(roll);
            fxx_y = c_fx * cos(yaw) - c_fy * sin(yaw) * cos(roll) + c_fz * sin(yaw) * sin(roll);

            // Jacobian matrices
            Eigen::Matrix<double, 3, 10> J_1_p;
            J_1_p << 1, 0, 0, -Zk[0,0] * sin(Xk[3,0]) - Zk[1,0] * cos(Xk[3,0]), cos(Xk[3,0]), -sin(Xk[3,0]), 0,
                    fx_x * cos(Xk[3,0]) - fx_y * sin(Xk[3,0]), 0, fxx_x * cos(Xk[3,0]) - fxx_y * sin(Xk[3,0]),
                    0, 1, 0, Zk[0,0] * cos(Xk[3,0]) - Zk[1,0] * sin(Xk[3,0]), sin(Xk[3,0]), cos(Xk[3,0]), 0,
                    fx_x * sin(Xk[3,0]) + fx_y * cos(Xk[3,0]), 0, fxx_x * sin(Xk[3,0]) + fxx_y * cos(Xk[3,0]),
                    0, 0, 1, 0, 0, 0;

            Eigen::Matrix<double, 3, 3> J_2_p;
            J_2_p << cos(Xk[3,0]), -sin(Xk[3,0]), 0,
                    sin(Xk[3,0]), cos(Xk[3,0]), 0,
                    0, 0, 1;

            // Accumulated uncertainty
            if (m == 1) {
                g1.topLeftCorner(10, 10) = Eigen::MatrixXd::Identity(10, 10);
                m = 2;
            }
            g1.bottomRightCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6);
            g1.bottomRows(3).leftCols(10) = J_1_p;

            g2.setZero();
            g2.bottomRows(3) = J_2_p;

            Pk = g1 * Pk * g1.transpose() + g2 * Rk_c * g2.transpose();
            g1.bottomRows(3).leftCols(10).setZero();

            // Increase map size to include features
            map12[num_2] = Eigen::Vector3d(0.0, 0.0, 0.0);
            num_2++;

        } else {
            // Update existing beacon id
            int idx = std::distance(beacon_list.begin(), std::find(beacon_list.begin(), beacon_list.end(), id));
            int f_i_1 = 10 + 3 * idx;
            int f_i_2 = 13 + 3 * idx;

            // Wrapping the angles
            Xk[3,0] = wrap_angle(Xk[3,0]);
            Xk[7,0] = wrap_angle(Xk[7,0]);
            Xk[8,0] = wrap_angle(Xk[8,0]);
            Xk[9,0] = wrap_angle(Xk[9,0]);

            // Position of the feature in camera frame in cartesian coordinate
            Eigen::Vector3d Zk1 = c_f;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I_1 = Eigen::MatrixXd::Identity(Pk.rows(), Pk.cols());
            Eigen::Matrix<double, 3, 3> Vk = Eigen::MatrixXd::Identity(3, 3);

            Eigen::Matrix<double, 3, 1> map = Xk.middleRows(f_i_1, f_i_2 - f_i_1);

            // Feature in robot frame
            Eigen::Vector3d Z_r_f(-Xk[0,0] * cos(Xk[3,0]) - Xk[1,0] * sin(Xk[3,0]) + map[0,0] * cos(Xk[3,0]) + map[1,0] * sin(Xk[3,0]),
                                Xk[0,0] * sin(Xk[3,0]) - Xk[1,0] * cos(Xk[3,0]) - map[0,0] * sin(Xk[3,0]) + map[1,0] * cos(Xk[3,0]),
                                -rz + map[2,0]);

            Eigen::Matrix<double, 3, 1> Trans_c_r, rot_c_r;
            std::tie(Trans_c_r, rot_c_r, unused) = transform_r_c_inv(Xk[4,0], Xk[5,0], Xk[6,0], camera[3], camera[4], camera[5]);

            // Position of feature in camera frame
            Eigen::Matrix<double, 3, 1> h_x = Trans_c_r + rot_c_r * Z_r_f;

            // Innovation
            Eigen::Matrix<double, 3, 1> Ykn = Zk1 - h_x;

            // Jacobian matrices
            Hk.setZero();
            Eigen::Matrix3d repeat;
            repeat << cos(yaw) * cos(Xk[3,0]) - sin(yaw) * sin(Xk[3,0]), cos(yaw) * sin(Xk[3,0]) + sin(yaw) * cos(Xk[3,0]), 0,
                    -cos(roll) * cos(yaw) * sin(Xk[3,0]) - cos(roll) * sin(yaw) * cos(Xk[3,0]), cos(roll) * cos(yaw) * cos(Xk[3,0]) - cos(roll) * sin(yaw) * sin(Xk[3,0]), sin(roll),
                    sin(roll) * cos(yaw) * sin(Xk[3,0]) + sin(roll) * sin(yaw) * cos(Xk[3,0]), sin(roll) * sin(yaw) * sin(Xk[3,0]) - sin(roll) * cos(yaw) * cos(Xk[3,0]), cos(roll);

            double f1_th = -map[1,0] * sin(Xk[3,0]) + Xk[1,0] * sin(Xk[3,0]) - map[0,0] * cos(Xk[3,0]) + Xk[0,0] * cos(Xk[3,0]);
            double f2_th = -map[0,0] * sin(Xk[3,0]) + Xk[0,0] * sin(Xk[3,0]) + map[1,0] * cos(Xk[3,0]) - Xk[1,0] * cos(Xk[3,0]);

            Hk << -cos(Xk[3,0]) * cos(yaw) + sin(yaw) * sin(Xk[3,0]), -cos(yaw) * sin(Xk[3,0]) - sin(yaw) * cos(Xk[3,0]), 0,
                cos(roll) * cos(yaw) * sin(Xk[3,0]) + cos(roll) * sin(yaw) * cos(Xk[3,0]), -cos(roll) * cos(yaw) * cos(Xk[3,0]) + cos(roll) * sin(yaw) * sin(Xk[3,0]), -sin(roll),
                -sin(roll) * cos(yaw) * sin(Xk[3,0]) - sin(roll) * sin(yaw) * cos(Xk[3,0]), sin(roll) * cos(yaw) * cos(Xk[3,0]) - sin(roll) * sin(yaw) * sin(Xk[3,0]), -cos(roll),
                cos(yaw) * f2_th + sin(yaw) * f1_th, -cos(yaw), -sin(yaw), 0,
                0, 0, Xk[4,0] * sin(yaw) - Xk[5,0] * cos(yaw) + cos(yaw) * f2_th + sin(yaw) * f1_th,
                cos(roll) * cos(yaw) * sin(Xk[3,0]) + cos(roll) * sin(yaw) * cos(Xk[3,0]), -cos(roll) * cos(yaw) * cos(Xk[3,0]) + cos(roll) * sin(yaw) * sin(Xk[3,0]), -sin(roll),
                -Xk[4,0] * sin(yaw) * sin(roll) + Xk[5,0] * cos(yaw) * sin(roll) - Xk[6,0] * cos(roll) + (map[2,0] - Xk[2,0]) * cos(roll) - sin(roll) * cos(yaw) * f2_th - sin(roll) * sin(yaw) * f1_th, 0,
                Xk[4,0] * cos(roll) * cos(yaw) + Xk[5,0] * cos(roll) * sin(yaw) - cos(roll) * sin(yaw) * f2_th + cos(roll) * cos(yaw) * f1_th,
                -sin(roll) * cos(yaw) * sin(Xk[3,0]) - sin(roll) * sin(yaw) * cos(Xk[3,0]), sin(roll) * cos(yaw) * cos(Xk[3,0]) - sin(roll) * sin(yaw) * sin(Xk[3,0]), -cos(roll),
                -Xk[4,0] * sin(yaw) * cos(roll) + Xk[5,0] * cos(roll) * cos(yaw) + Xk[6,0] * sin(roll) - sin(roll) * (map[2,0] - Xk[2,0]) - cos(yaw) * cos(roll) * f2_th - cos(roll) * sin(yaw) * f1_th, 0,
                -Xk[4,0] * sin(roll) * cos(yaw) - Xk[5,0] * sin(roll) * sin(yaw) + sin(roll) * sin(yaw) * f2_th - sin(roll) * cos(yaw) * f1_th;

            Hk.block(0, f_i_1, Hk.rows(), f_i_2 - f_i_1) = repeat;

            // Kalman gain calculation
            Eigen::MatrixXd before_inv = (Hk * (Pk * Hk.transpose())) + (Vk * (Rk_c * Vk.transpose()));
            Eigen::MatrixXd inv = before_inv.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd K_gain = (Pk * Hk.transpose()) * inv;

            // Update state and covariance
            Xk += K_gain * Ykn;
            Pk = (I_1 - (K_gain * Hk)) * Pk * (I_1 - (K_gain * Hk)).transpose();

            // Visualizing the aruco as a point in the simulation
            for (int m = 0; m < map12.size(); m++) {
                map12[m] = Eigen::Vector3d(Xk[3 * m + 10, 0], Xk[3 * m + 11, 0], Xk[3 * m + 12, 0]);
            }
            modem_visualization(map12);
        }

        return std::make_tuple(Xk, Pk);
    }


    void modem_visualization(const std::vector<Eigen::Vector3d>& map) {
        this->map = map;

        visualization_msgs::MarkerArray ma;

        for (size_t i = 0; i < this->map.size(); ++i) {
            Eigen::Vector3d pos = this->map[i];

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i * 2;

            marker.header.stamp = ros::Time::now();
            marker.pose.position.x = pos.x();
            marker.pose.position.y = pos.y();
            marker.pose.position.z = -1.0;
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
            marker_text.pose.position.x = pos.x();
            marker_text.pose.position.y = pos.y();
            marker_text.pose.position.z = -2.0;
            marker_text.pose.orientation.w = 1.0;

            marker_text.scale.x = 0.1;
            marker_text.scale.y = 0.1;
            marker_text.scale.z = 0.3;

            marker_text.color.r = 1.0;
            marker_text.color.a = 0.9;

            std::stringstream ss;
            ss << "Aruco " << i;
            marker_text.text = ss.str();

            ma.markers.push_back(marker_text);

            // Uncertainty calculation
            Eigen::Matrix3d cov = Pk.block<3, 3>(i * 3 + 10, i * 3 + 10); // Assuming Pk is Eigen::MatrixXd

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
            Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
            Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();

            Eigen::Vector3d major_axis_eigenvector = eigenvectors.col(2); // Assuming largest eigenvalue is last

            double roll_m = std::atan2(major_axis_eigenvector.y(), major_axis_eigenvector.x());

            double confidence_factor = 2.0;
            double major_axis_length = confidence_factor * std::sqrt(eigenvalues(2));
            double minor_axis_length = confidence_factor * std::sqrt(eigenvalues(0));

            double orientation_angle = std::atan2(major_axis_eigenvector.y(), major_axis_eigenvector.x());
            double pitch_m = std::asin(-major_axis_eigenvector.z());
            double yaw_m = std::atan2(cov(2, 1), cov(2, 2));

            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll_m, pitch_m, yaw_m);

            visualization_msgs::Marker marker1;
            marker1.header.frame_id = "world";
            marker1.header.stamp = ros::Time::now();
            marker1.ns = "feature_ellipsoid";
            marker1.id = i * 100 + 1;
            marker1.type = visualization_msgs::Marker::SPHERE;
            marker1.action = visualization_msgs::Marker::ADD;
            marker1.color.r = 0.0;
            marker1.color.g = 1.0;
            marker1.color.b = 0.0;
            marker1.color.a = 1.0;

            marker1.pose.position.x = pos.x();
            marker1.pose.position.y = pos.y();
            marker1.pose.position.z = -1.0;

            marker1.pose.orientation = quat;

            marker1.scale.x = major_axis_length;
            marker1.scale.y = minor_axis_length;
            marker1.scale.z = 0.01;

            ma.markers.push_back(marker1);
        }

        marker_pub.publish(ma);
    }


    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> prediction(double left_wheel_velocity, double right_wheel_velocity, double msg_sec, double msg_nsec) {
        this->left_wheel_velocity = left_wheel_velocity;
        this->right_wheel_velocity = right_wheel_velocity;

        double left_lin_vel = this->left_wheel_velocity * this->wheel_radius;
        double right_lin_vel = this->right_wheel_velocity * this->wheel_radius;

        this->v = (left_lin_vel + right_lin_vel) / 2.0;
        this->w = (left_lin_vel - right_lin_vel) / this->wheel_base_distance;

        // Calculate dt
        this->current_time = ros::Time(msg_sec + msg_nsec * 1e-9);
        double dt = (this->current_time - this->last_time).toSec();
        this->last_time = this->current_time;

        // Update state prediction 
        this->Xk(0) += std::cos(this->Xk(3)) * this->v * dt;
        this->Xk(1) += std::sin(this->Xk(3)) * this->v * dt;
        this->Xk(2) = this->rz;
        this->Xk(3) += this->w * dt;

        this->Xk(3) = wrap_angle(this->Xk(3)); // Assuming wrap_angle function handles angle wrapping

        // State transition matrix F1
        Eigen::MatrixXd Ak(10, 10);
        Ak << 1, 0, 0, -std::sin(this->Xk(3)) * this->v * dt, 0, 0, 0, 0, 0, 0,
            0, 1, 0, std::cos(this->Xk(3)) * this->v * dt, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

        // Control input matrix F2
        Eigen::MatrixXd Bk(10, 2);
        Bk << std::cos(this->Xk(3)) * dt * 0.5 * this->wheel_radius, std::cos(this->Xk(3)) * dt * 0.5 * this->wheel_radius,
            std::sin(this->Xk(3)) * dt * 0.5 * this->wheel_radius, std::sin(this->Xk(3)) * dt * 0.5 * this->wheel_radius,
            0, 0,
            (dt * this->wheel_radius) / this->wheel_base_distance, -(dt * this->wheel_radius) / this->wheel_base_distance,
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0;

        // Update covariance prediction
        this->Pk = Ak * this->Pk * Ak.transpose() + Bk * this->Qk * Bk.transpose();

        // Publish marker for visualization
        visualization_msgs::MarkerArray ma;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "kobuki/base_link";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.0; // Assuming it should be less than 1.0 for visibility

        marker.id = 945;

        marker.header.stamp = ros::Time::now();
        marker.pose.position.x = this->Xk(4);
        marker.pose.position.y = this->Xk(5);
        marker.pose.position.z = -0.1;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        ma.markers.push_back(marker);

        // Uncertainty visualization
        Eigen::MatrixXd cov_c = this->Pk.block(4, 4, 3, 3);

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver_c(cov_c);
        Eigen::VectorXd eigenvalues_c = eigensolver_c.eigenvalues();
        Eigen::MatrixXd eigenvectors_c = eigensolver_c.eigenvectors();

        Eigen::Vector3d major_axis_eigenvector_c = eigenvectors_c.col(2); // Assuming largest eigenvalue is last

        double roll_m_c = std::atan2(major_axis_eigenvector_c.y(), major_axis_eigenvector_c.x());

        double major_axis_length_c = 2.0 * std::sqrt(eigenvalues_c(2));
        double minor_axis_length_c = 2.0 * std::sqrt(eigenvalues_c(0));

        double orientation_angle_c = std::atan2(major_axis_eigenvector_c.y(), major_axis_eigenvector_c.x());
        double pitch_m_c = std::asin(-major_axis_eigenvector_c.z());
        double yaw_m_c = std::atan2(cov_c(2, 1), cov_c(2, 2));

        geometry_msgs::Quaternion quat_c = tf::createQuaternionMsgFromRollPitchYaw(roll_m_c, pitch_m_c, yaw_m_c);

        visualization_msgs::Marker marker1_c;
        marker1_c.header.frame_id = "kobuki/base_link";
        marker1_c.header.stamp = ros::Time::now();
        marker1_c.ns = "feature_ellipsoid";
        marker1_c.id = 345;
        marker1_c.type = visualization_msgs::Marker::SPHERE;
        marker1_c.action = visualization_msgs::Marker::ADD;
        marker1_c.color.r = 0.0;
        marker1_c.color.g = 0.0;
        marker1_c.color.b = 1.0;
        marker1_c.color.a = 1.0;

        marker1_c.pose.position.x = this->Xk(4);
        marker1_c.pose.position.y = this->Xk(5);
        marker1_c.pose.position.z = -0.1; // Assuming this->Xk(6);

        marker1_c.pose.orientation = quat_c;

        marker1_c.scale.x = major_axis_length_c;
        marker1_c.scale.y = minor_axis_length_c;
        marker1_c.scale.z = 0.01;

        ma.markers.push_back(marker1_c);

        marker_pub.publish(ma);

        return std::make_tuple(this->Xk, this->Pk); 
    } 


    void quad(const Eigen::VectorXd& Xk, const Eigen::MatrixXd& Pk, const ros::Time& current_time, double v, double w) {
        this->Xk = Xk;
        this->Pk = Pk;
        this->v = v;
        this->w = w;
        this->current_time = current_time;

        geometry_msgs::Quaternion q = tf2::toMsg(tf2::Quaternion(0, 0, Xk(3, 0)));

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";
        odom.child_frame_id = "kobuki/base_footprint";

        odom.pose.pose.position.x = Xk(0, 0);
        odom.pose.pose.position.y = Xk(1, 0);
        odom.pose.pose.position.z = Xk(2, 0);

        // Covariance matrix
        Eigen::Matrix<double, 6, 6> covariance;
        covariance << Pk(0, 0), Pk(0, 1), Pk(0, 2), 0.0, 0.0, Pk(0, 3),
                      Pk(1, 0), Pk(1, 1), Pk(1, 2), 0.0, 0.0, Pk(1, 3),
                      Pk(2, 0), Pk(2, 1), Pk(2, 2), 0.0, 0.0, Pk(2, 3),
                      0.0, 0.0, 0.0, 99999, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 99999, 0.0,
                      Pk(3, 0), Pk(3, 1), Pk(3, 2), 0.0, 0.0, Pk(3, 3);

        for (int i = 0; i < 36; ++i)
            odom.pose.covariance[i] = covariance(i);

        odom.pose.pose.orientation.x = q.x;
        odom.pose.pose.orientation.y = q.y;
        odom.pose.pose.orientation.z = q.z;
        odom.pose.pose.orientation.w = q.w;

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        odom_pub.publish(odom);

        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(Xk(0, 0), Xk(1, 0), Xk(2, 0)));
        transform.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));

        tf_br.sendTransform(tf2::StampedTransform(transform, ros::Time::now(), odom.child_frame_id, odom.header.frame_id));
    }

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (msg->name[0] == "kobuki/wheel_left_joint") {
            this->left_wheel_velocity = msg->velocity[0];
            this->left_wheel_received = true;
        } else if (msg->name[0] == "kobuki/wheel_right_joint") {
            this->right_wheel_velocity = msg->velocity[0];
            if (this->left_wheel_received) {
                this->left_wheel_received = false;
                double msg_sec = msg->header.stamp.sec;
                double msg_nsec = msg->header.stamp.nsec;

                std::tie(this->Xk, this->Pk) = this->prediction(this->left_wheel_velocity, this->right_wheel_velocity, msg_sec, msg_nsec);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "differential_drive");
    double sensor_noise = 0.1;
    double sensor_noise_angle = 0.2;
    DifferentialDrive robot(sensor_noise, sensor_noise_angle);
    ros::spin();
    return 0;
}