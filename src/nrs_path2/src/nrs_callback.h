#ifndef NRS_CALLBACK_H
#define NRS_CALLBACK_H

#include "nrs_io.h"
#include "nrs_geodesic.h"
#include "nrs_interpolation.h"
#include "nrs_visualization.h"

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nrs_path2/msg/waypoint.hpp"
#include "nrs_path2/msg/waypoints.hpp"

class nrs_callback
{
public:
    nrs_callback();
    explicit nrs_callback(rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr spline_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr straight_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr interpolation_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr path_delete_service;

    nrs_io n_io;
    nrs_geodesic n_geodesic;
    nrs_interpolation n_interpolation;
    nrs_visualization n_visualization;

    std::string mesh_file_path;

    rclcpp::Publisher<nrs_path2::msg::Waypoints>::SharedPtr geodesic_waypoints_pub;
    nrs_path2::msg::Waypoints waypoints_msg;
    std::vector<Eigen::Vector3d> selected_points;
    std::string geodesic_waypoints_file_path;

    rclcpp::Publisher<nrs_path2::msg::Waypoints>::SharedPtr interpolated_waypoints_pub;
    nrs_path2::msg::Waypoints geodesic_path;

    double desired_interval, fx, fy, fz;
    std::string interpolated_waypoints_file_path;
    std::string interpolated_waypoints_debug_file_path;

    bool splinePathServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    bool straightPathServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    bool PathInterpolationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    bool pathDeleteCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);
};

#endif // NRS_CALLBACK_H