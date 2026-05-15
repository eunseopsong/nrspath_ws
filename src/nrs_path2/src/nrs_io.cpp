#include "nrs_io.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <Eigen/Geometry>

namespace
{
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

    Eigen::Matrix3d quaternionToMatrix(double qx, double qy, double qz, double qw)
    {
        const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (norm < 1e-12 || std::isnan(norm))
            return Eigen::Matrix3d::Identity();

        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        return q.toRotationMatrix();
    }

    Eigen::Matrix3d getFlatReferenceMatrix()
    {
        static const Eigen::Matrix3d R_flat = rotVecToMatrix(0.0, 0.0, 1.57);
        return R_flat;
    }

    std::string shellQuote(const std::string &value)
    {
        std::string quoted = "'";
        for (char c : value)
        {
            if (c == '\'')
                quoted += "'\\''";
            else
                quoted += c;
        }
        quoted += "'";
        return quoted;
    }

    bool toForceConPath(const std::string &local_path, std::string &remote_path)
    {
        constexpr const char *local_home = "/home/eunseop/";
        constexpr const char *remote_home = "/home/nrs_forcecon/";

        if (local_path.rfind(local_home, 0) != 0)
            return false;

        remote_path = std::string(remote_home) + local_path.substr(std::string(local_home).size());
        return true;
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

    for (size_t i = 0; i < final_waypoints.waypoints.size(); ++i)
    {
        const auto &wp = final_waypoints.waypoints[i];

        Eigen::Matrix3d R_cur = quaternionToMatrix(wp.qx, wp.qy, wp.qz, wp.qw);
        Eigen::Matrix3d R_flat = getFlatReferenceMatrix();

        Eigen::Vector3d z_cur = R_cur.col(2).normalized();
        Eigen::Vector3d z_flat = R_flat.col(2).normalized();

        Eigen::Quaterniond q_tilt = Eigen::Quaterniond::FromTwoVectors(z_flat, z_cur);
        q_tilt.normalize();

        Eigen::Matrix3d R_export = q_tilt.toRotationMatrix() * R_flat;
        Eigen::Vector3d w_export = matrixToRotVec(R_export);

        double wx = w_export.x();
        double wy = w_export.y();
        double wz = w_export.z();

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

void nrs_io::saveWaypointsToFlatDebugFile(const nrs_path2::msg::Waypoints &final_waypoints,
                                          const std::string &file_path)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open debug file " << file_path << std::endl;
        return;
    }

    const Eigen::Matrix3d R_flat = getFlatReferenceMatrix();
    const Eigen::Vector3d z_flat = R_flat.col(2).normalized();

    for (size_t i = 0; i < final_waypoints.waypoints.size(); ++i)
    {
        const auto &wp = final_waypoints.waypoints[i];

        Eigen::Matrix3d R_cur = quaternionToMatrix(wp.qx, wp.qy, wp.qz, wp.qw);
        Eigen::Vector3d z_cur = R_cur.col(2).normalized();

        // flat 기준 z축 -> 현재 z축 으로 가는 최소 회전(tilt only)
        Eigen::Quaterniond q_tilt = Eigen::Quaterniond::FromTwoVectors(z_flat, z_cur);
        q_tilt.normalize();

        Eigen::Matrix3d R_export = q_tilt.toRotationMatrix() * R_flat;
        Eigen::Vector3d w_export = matrixToRotVec(R_export);

        double wx_dbg = w_export.x();
        double wy_dbg = w_export.y();
        double wz_dbg = w_export.z();

        double x_mm = wp.x * 1000.0;
        double y_mm = wp.y * 1000.0;
        double z_mm = wp.z * 1000.0;

        file << x_mm << " " << y_mm << " " << z_mm << " "
             << wx_dbg << " " << wy_dbg << " " << wz_dbg << " "
             << wp.fx << " " << wp.fy << " " << wp.fz << "\n";
    }

    file.close();
    std::cout << "Debug waypoints saved to " << file_path << std::endl;
}

bool nrs_io::copyFileToForceConSamePath(const std::string &file_path)
{
    constexpr const char *remote = "nrs_forcecon@192.168.0.151";

    std::string remote_file_path;
    if (!toForceConPath(file_path, remote_file_path))
    {
        std::cerr << "Warning: skipping remote copy for unsupported local path: "
                  << file_path << std::endl;
        return false;
    }

    const fs::path remote_path(remote_file_path);
    const std::string remote_dir = remote_path.parent_path().string();

    const std::string mkdir_command =
        "ssh -o BatchMode=yes -o ConnectTimeout=5 " +
        std::string(remote) + " " +
        shellQuote("mkdir -p " + shellQuote(remote_dir));

    int ret = std::system(mkdir_command.c_str());
    if (ret != 0)
    {
        std::cerr << "Warning: failed to create remote directory "
                  << remote << ":" << remote_dir
                  << " (ssh exit code: " << ret << ")" << std::endl;
        return false;
    }

    const std::string copy_command =
        "scp -o BatchMode=yes -o ConnectTimeout=5 " +
        shellQuote(file_path) + " " +
        std::string(remote) + ":" + shellQuote(remote_file_path);

    ret = std::system(copy_command.c_str());
    if (ret != 0)
    {
        std::cerr << "Warning: failed to copy " << file_path
                  << " to " << remote << ":" << remote_file_path
                  << " (scp exit code: " << ret << ")" << std::endl;
        return false;
    }

    std::cout << "Copied " << file_path
              << " to " << remote << ":" << remote_file_path << std::endl;
    return true;
}

void nrs_io::sendFile(const std::string &file_path,
                      const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &file_pub,
                      const rclcpp::Node::SharedPtr &node)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file: %s", file_path.c_str());
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
