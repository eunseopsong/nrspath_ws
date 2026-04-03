#include "nrs_io.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

namespace
{
    // rotvec (wx, wy, wz) -> rotation matrix
    Eigen::Matrix3d rotVecToMatrix(double wx, double wy, double wz)
    {
        Eigen::Vector3d w(wx, wy, wz);
        double angle = w.norm();

        if (angle < 1e-12 || std::isnan(angle))
        {
            return Eigen::Matrix3d::Identity();
        }

        Eigen::Vector3d axis = w / angle;
        Eigen::AngleAxisd aa(angle, axis);
        return aa.toRotationMatrix();
    }

    // rotation matrix -> rotvec (wx, wy, wz)
    Eigen::Vector3d matrixToRotVec(const Eigen::Matrix3d &R_in)
    {
        Eigen::AngleAxisd aa(R_in);
        double angle = aa.angle();

        if (std::abs(angle) < 1e-12 || std::isnan(angle))
        {
            return Eigen::Vector3d::Zero();
        }

        return aa.axis() * angle;
    }
}

void nrs_io::clearFile(const std::string &file_path)
{
    std::ofstream file(file_path, std::ofstream::trunc);
    if (!file.is_open())
        std::cerr << "Failed to open file: " << file_path << std::endl;
    else
        file.close();
}

void nrs_io::saveWaypointsToFile(const nrs_path2::msg::Waypoints &final_waypoints,
                                 const std::string &file_path)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << file_path << std::endl;
        return;
    }

    // ------------------------------------------------------------
    // Flat STL 기준 보정용 reference
    //
    // 현재 시스템에서 flat surface를 저장했을 때 나온 raw rotvec:
    //   (-1.20224,  2.90245, 0.0)
    //
    // 사용자가 원하는 flat 기준 export rotvec:
    //   (0.0, 0.0, 1.57)
    //
    // R_fix * R_raw_flat = R_des_flat
    // ------------------------------------------------------------
    constexpr double raw_flat_wx = -1.20224;
    constexpr double raw_flat_wy =  2.90245;
    constexpr double raw_flat_wz =  0.0;

    constexpr double des_flat_wx = 0.0;
    constexpr double des_flat_wy = 0.0;
    constexpr double des_flat_wz = 1.57;

    const Eigen::Matrix3d R_raw_flat = rotVecToMatrix(raw_flat_wx, raw_flat_wy, raw_flat_wz);
    const Eigen::Matrix3d R_des_flat = rotVecToMatrix(des_flat_wx, des_flat_wy, des_flat_wz);

    const Eigen::Matrix3d R_fix = R_des_flat * R_raw_flat.transpose();

    for (size_t i = 0; i < final_waypoints.waypoints.size(); ++i)
    {
        const auto &wp = final_waypoints.waypoints[i];

        // 1) quaternion -> raw rotvec
        double raw_wx = 0.0, raw_wy = 0.0, raw_wz = 0.0;
        n_math.quaternionToRotVec(wp.qx, wp.qy, wp.qz, wp.qw, raw_wx, raw_wy, raw_wz);

        // 2) raw rotvec -> rotation matrix
        Eigen::Matrix3d R_raw = rotVecToMatrix(raw_wx, raw_wy, raw_wz);

        // 3) apply fixed alignment
        Eigen::Matrix3d R_aligned = R_fix * R_raw;

        // 4) aligned rotation matrix -> rotvec
        Eigen::Vector3d w_aligned = matrixToRotVec(R_aligned);

        double wx = w_aligned.x();
        double wy = w_aligned.y();
        double wz = w_aligned.z();

        // meter -> millimeter
        double x_mm = wp.x * 1000.0;
        double y_mm = wp.y * 1000.0;
        double z_mm = wp.z * 1000.0;

        file << x_mm << " " << y_mm << " " << z_mm << " "
             << wx   << " " << wy   << " " << wz   << " "
             << wp.fx << " " << wp.fy << " " << wp.fz << "\n";
    }

    file.close();
    std::cout << "Waypoints saved to " << file_path << std::endl;
}

void nrs_io::sendFile(const std::string &file_path,
                      const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &file_pub,
                      const rclcpp::Node::SharedPtr &node)
//// void nrs_io::sendFile(const std::string &file_path, ros::Publisher &file_pub)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file: %s", file_path.c_str());
        //// ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string file_data = buffer.str();
    file.close();

    std_msgs::msg::String msg;
    msg.data = file_data;

    RCLCPP_INFO(node->get_logger(), "Sending file data...");
    file_pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "File data sent.");
}