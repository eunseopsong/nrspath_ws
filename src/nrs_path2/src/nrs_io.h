#ifndef NRS_IO_H
#define NRS_IO_H

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/filesystem.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

#include "nrs_path2/msg/waypoint.hpp"
#include "nrs_path2/msg/waypoints.hpp"

#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>

#include "nrs_math.h"

namespace fs = boost::filesystem;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Point_set_3<Point_3> Point_set;

class nrs_io
{
public:
    nrs_math n_math;
    std::string file_path_;
    int file_counter = 0;

    void clearFile(const std::string &file_path);

    // 실제 제어용 export: quaternion -> true rotvec
    void saveWaypointsToFile(const nrs_path2::msg::Waypoints &final_waypoints,
                             const std::string &file_path);

    // 디버그용 export: flat yaw 기준에 현재 tool z축 tilt를 합성한 true rotvec
    void saveWaypointsToFlatDebugFile(const nrs_path2::msg::Waypoints &final_waypoints,
                                      const std::string &file_path);

    bool copyFileToForceConSamePath(const std::string &file_path);

    void sendFile(const std::string &file_path,
                  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &file_pub,
                  const rclcpp::Node::SharedPtr &node);
};

#endif // NRS_IO_H
