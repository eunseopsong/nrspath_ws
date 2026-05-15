#include "nrs_callback.h"

nrs_callback::nrs_callback()
{
}

nrs_callback::nrs_callback(rclcpp::Node::SharedPtr node)
: node_(node)
{
    spline_service = node_->create_service<std_srvs::srv::Empty>(
        "spline",
        std::bind(&nrs_callback::splinePathServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    straight_service = node_->create_service<std_srvs::srv::Empty>(
        "straight",
        std::bind(&nrs_callback::straightPathServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    interpolation_service = node_->create_service<std_srvs::srv::Empty>(
        "interpolation",
        std::bind(&nrs_callback::PathInterpolationCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    path_delete_service = node_->create_service<std_srvs::srv::Empty>(
        "delete",
        std::bind(&nrs_callback::pathDeleteCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

bool nrs_callback::splinePathServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    RCLCPP_INFO(node_->get_logger(), "Spline path service called. Generating Hermite Spline Path...");

    std::ifstream input(mesh_file_path, std::ios::binary);
    Triangle_mesh tmesh;
    n_geodesic.load_stl_file(input, tmesh);
    Tree *tree;
    Surface_mesh_shortest_path *shortest_paths;
    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(tmesh);
    waypoints_msg.waypoints.clear();

    if (selected_points.size() > 2)
    {
        nrs_path2::msg::Waypoints path_points = n_geodesic.GenerateHermiteSplinePath(selected_points, tmesh);
        geodesic_waypoints_pub->publish(path_points);

        RCLCPP_INFO(node_->get_logger(), "Published Hermite Spline Path with %zu waypoints", path_points.waypoints.size());

        n_io.clearFile(geodesic_waypoints_file_path);
        n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
        return true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Not enough selected points for Hermite Spline Path generation.");
        return false;
    }
}

bool nrs_callback::straightPathServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    if (!node_) {
        std::cerr << "[FATAL] node_ is null!" << std::endl;
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[straight] Service called!");
    RCLCPP_INFO(node_->get_logger(), "[straight] selected_points size: %zu", selected_points.size());

    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open mesh file: %s", mesh_file_path.c_str());
        return false;
    }

    Triangle_mesh tmesh;
    if (!n_geodesic.load_stl_file(input, tmesh)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh.");
        return false;
    }

    if (selected_points.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "Not enough points.");
        return false;
    }

    auto path_points = n_geodesic.GenerateStraightGeodesicPath(selected_points, tmesh);
    geodesic_waypoints_pub->publish(path_points);
    n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);

    return true;
}

bool nrs_callback::PathInterpolationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open mesh file: %s", mesh_file_path.c_str());
        return false;
    }

    Triangle_mesh tmesh;
    if (!n_geodesic.load_stl_file(input, tmesh))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh from file: %s", mesh_file_path.c_str());
        return false;
    }
    input.close();

    Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

    auto start_time_point = std::chrono::high_resolution_clock::now();

    nrs_path2::msg::Waypoints final_waypoints =
        n_interpolation.interpolateEnd2End(geodesic_path, desired_interval, tmesh, fx, fy, fz);

    if (final_waypoints.waypoints.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Interpolation produced no waypoints.");
        delete tree;
        delete shortest_paths;
        return false;
    }

    interpolated_waypoints_pub->publish(final_waypoints);

    n_io.clearFile(interpolated_waypoints_file_path);
    n_io.saveWaypointsToFile(final_waypoints, interpolated_waypoints_file_path);
    n_io.copyFileToForceConSamePath(interpolated_waypoints_file_path);

    if (!interpolated_waypoints_debug_file_path.empty())
    {
        n_io.clearFile(interpolated_waypoints_debug_file_path);
        n_io.saveWaypointsToFlatDebugFile(final_waypoints, interpolated_waypoints_debug_file_path);
        n_io.copyFileToForceConSamePath(interpolated_waypoints_debug_file_path);
    }

    auto end_time_point = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_point - start_time_point).count();
    std::cout << "Interpolation & Normal smoothing time: " << duration << " s" << std::endl;

    delete tree;
    delete shortest_paths;

    return true;
}

bool nrs_callback::pathDeleteCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    n_visualization.deleteMarkers();
    selected_points.clear();
    geodesic_path.waypoints.clear();

    nrs_path2::msg::Waypoints empty_waypoints;
    empty_waypoints.waypoints.clear();
    interpolated_waypoints_pub->publish(empty_waypoints);

    RCLCPP_INFO(node_->get_logger(), "Path deleted via pathDeleteCallback");
    return true;
}
