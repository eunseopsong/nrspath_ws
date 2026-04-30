#include "nrs_interpolation.h"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace
{
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

geometry_msgs::msg::Point toRosPoint(const Eigen::Vector3d &p)
{
    geometry_msgs::msg::Point out;
    out.x = p.x();
    out.y = p.y();
    out.z = p.z();
    return out;
}

double quatDot(const tf2::Quaternion &q1, const tf2::Quaternion &q2)
{
    return q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w();
}

Eigen::Vector3d projectToPlane(const Eigen::Vector3d &v, const Eigen::Vector3d &normal)
{
    return v - v.dot(normal) * normal;
}

Eigen::Vector3d fallbackTangent(const Eigen::Vector3d &z_axis)
{
    Eigen::Vector3d ref(1.0, 0.0, 0.0);
    if (std::abs(z_axis.dot(ref)) > 0.9)
        ref = Eigen::Vector3d(0.0, 1.0, 0.0);

    Eigen::Vector3d x = projectToPlane(ref, z_axis);

    if (x.norm() < 1e-12)
    {
        ref = Eigen::Vector3d(0.0, 0.0, 1.0);
        x = projectToPlane(ref, z_axis);
    }

    if (x.norm() < 1e-12)
        x = Eigen::Vector3d(1.0, 0.0, 0.0);

    return x.normalized();
}

tf2::Quaternion axesToQuaternion(const Eigen::Vector3d &x_axis,
                                 const Eigen::Vector3d &y_axis,
                                 const Eigen::Vector3d &z_axis)
{
    tf2::Matrix3x3 R(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());

    tf2::Quaternion q;
    R.getRotation(q);
    q.normalize();
    return q;
}

geometry_msgs::msg::Point interpolatePointLinear(const geometry_msgs::msg::Point &p0,
                                                 const geometry_msgs::msg::Point &p1,
                                                 double t)
{
    geometry_msgs::msg::Point out;
    out.x = p0.x + t * (p1.x - p0.x);
    out.y = p0.y + t * (p1.y - p0.y);
    out.z = p0.z + t * (p1.z - p0.z);
    return out;
}

Eigen::Vector3d interpolateNormalLinear(const Eigen::Vector3d &n0,
                                        const Eigen::Vector3d &n1,
                                        double t)
{
    Eigen::Vector3d n = (1.0 - t) * n0 + t * n1;
    if (n.norm() < 1e-12)
        return n0;
    return n.normalized();
}

Eigen::Vector3d computeLocalTangent(const std::vector<Eigen::Vector3d> &pts, size_t i)
{
    if (pts.size() < 2)
        return Eigen::Vector3d::Zero();

    if (i == 0)
        return pts[1] - pts[0];
    else if (i == pts.size() - 1)
        return pts[i] - pts[i - 1];
    else
        return pts[i + 1] - pts[i - 1];
}

nrs_path2::msg::Waypoints buildWaypointsFromPointsAndNormals(
    const std::vector<geometry_msgs::msg::Point> &points,
    const std::vector<Eigen::Vector3d> &normals_in,
    double fx, double fy, double fz)
{
    nrs_path2::msg::Waypoints waypoints;
    if (points.empty() || points.size() != normals_in.size())
        return waypoints;

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(points.size());
    for (const auto &p : points)
        pts.push_back(toEigen(p));

    std::vector<Eigen::Vector3d> normals = normals_in;
    for (size_t i = 0; i < normals.size(); ++i)
    {
        if (normals[i].norm() < 1e-12)
        {
            if (i > 0)
                normals[i] = normals[i - 1];
            else
                normals[i] = Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        else
        {
            normals[i].normalize();
        }

        if (i > 0 && normals[i].dot(normals[i - 1]) < 0.0)
            normals[i] = -normals[i];
    }

    Eigen::Vector3d prev_x = fallbackTangent(-normals[0]);
    tf2::Quaternion prev_q(0.0, 0.0, 0.0, 1.0);

    for (size_t i = 0; i < pts.size(); ++i)
    {
        Eigen::Vector3d z_axis = -normals[i];
        if (z_axis.norm() < 1e-12)
            z_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
        z_axis.normalize();

        Eigen::Vector3d local_tangent = computeLocalTangent(pts, i);
        Eigen::Vector3d tangent_proj = projectToPlane(local_tangent, z_axis);

        if (tangent_proj.norm() > 1e-12)
            tangent_proj.normalize();

        Eigen::Vector3d x_axis;
        if (i == 0)
        {
            if (tangent_proj.norm() > 1e-12)
                x_axis = tangent_proj;
            else
                x_axis = fallbackTangent(z_axis);
        }
        else
        {
            x_axis = projectToPlane(prev_x, z_axis);

            if (x_axis.norm() < 1e-12)
            {
                if (tangent_proj.norm() > 1e-12)
                    x_axis = tangent_proj;
                else
                    x_axis = fallbackTangent(z_axis);
            }
            else
            {
                x_axis.normalize();
            }

            if (tangent_proj.norm() > 1e-12 && x_axis.dot(tangent_proj) < 0.0)
                x_axis = -x_axis;
        }

        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        if (y_axis.norm() < 1e-12)
        {
            x_axis = fallbackTangent(z_axis);
            y_axis = z_axis.cross(x_axis);
        }

        y_axis.normalize();
        x_axis = y_axis.cross(z_axis).normalized();

        tf2::Quaternion q = axesToQuaternion(x_axis, y_axis, z_axis);
        if (quatDot(q, prev_q) < 0.0)
            q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());

        nrs_path2::msg::Waypoint wp;
        wp.x = pts[i].x();
        wp.y = pts[i].y();
        wp.z = pts[i].z();
        wp.qw = q.getW();
        wp.qx = q.getX();
        wp.qy = q.getY();
        wp.qz = q.getZ();
        wp.fx = fx;
        wp.fy = fy;
        wp.fz = fz;

        waypoints.waypoints.push_back(wp);

        prev_x = x_axis;
        prev_q = q;
    }

    return waypoints;
}
} // namespace

std::vector<double> nrs_interpolation::computeCumulativeDistances(
    const std::vector<geometry_msgs::msg::Point> &points)
{
    std::vector<double> cumulative_distances(points.size(), 0.0);

    for (size_t i = 1; i < points.size(); ++i)
    {
        Eigen::Vector3d p0(points[i - 1].x, points[i - 1].y, points[i - 1].z);
        Eigen::Vector3d p1(points[i].x, points[i].y, points[i].z);
        cumulative_distances[i] = cumulative_distances[i - 1] + (p1 - p0).norm();
    }

    return cumulative_distances;
}

Eigen::Vector3d nrs_interpolation::getFaceNormal(
    const geometry_msgs::msg::Point &ros_point,
    const Triangle_mesh &mesh)
{
    Point_3 cgal_point(ros_point.x, ros_point.y, ros_point.z);

    face_descriptor face;
    CGAL::cpp11::array<double, 3> location;

    if (!n_geodesic.locate_face_and_point(cgal_point, face, location, mesh))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nrs_interpolation"),
                     "Failed to locate face and point for point: [%f, %f, %f]",
                     ros_point.x, ros_point.y, ros_point.z);
        return Eigen::Vector3d::Zero();
    }

    Triangle_mesh::Halfedge_index h = mesh.halfedge(face);
    vertex_descriptor v0 = mesh.source(h);
    vertex_descriptor v1 = mesh.target(h);
    vertex_descriptor v2 = mesh.target(mesh.next(h));

    Kernel::Vector_3 normal_v0 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v0, mesh);
    Kernel::Vector_3 normal_v1 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v1, mesh);
    Kernel::Vector_3 normal_v2 = CGAL::Polygon_mesh_processing::compute_vertex_normal(v2, mesh);

    Eigen::Vector3d n0(normal_v0.x(), normal_v0.y(), normal_v0.z());
    Eigen::Vector3d n1(normal_v1.x(), normal_v1.y(), normal_v1.z());
    Eigen::Vector3d n2(normal_v2.x(), normal_v2.y(), normal_v2.z());

    Eigen::Vector3d normal =
        location[0] * n0 +
        location[1] * n1 +
        location[2] * n2;

    if (normal.norm() < 1e-12)
        return Eigen::Vector3d::Zero();

    return normal.normalized();
}

nrs_path2::msg::Waypoints nrs_interpolation::setToolVectorOriginal(
    const std::vector<geometry_msgs::msg::Point> &points,
    const Triangle_mesh &mesh,
    double fx, double fy, double fz)
{
    if (points.empty())
        return nrs_path2::msg::Waypoints();

    std::vector<Eigen::Vector3d> normals;
    normals.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        Eigen::Vector3d n = getFaceNormal(points[i], mesh);

        if (n.norm() < 1e-12)
        {
            if (!normals.empty())
                n = normals.back();
            else
                n = Eigen::Vector3d(0.0, 0.0, 1.0);
        }

        if (!normals.empty() && n.dot(normals.back()) < 0.0)
            n = -n;

        normals.push_back(n.normalized());
    }

    return buildWaypointsFromPointsAndNormals(points, normals, fx, fy, fz);
}

nrs_path2::msg::Waypoints nrs_interpolation::interpolateEnd2End(
    const nrs_path2::msg::Waypoints &original_waypoints,
    double desired_interval,
    const Triangle_mesh &mesh,
    double fx, double fy, double fz)
{
    nrs_path2::msg::Waypoints output;

    if (original_waypoints.waypoints.size() < 2)
    {
        RCLCPP_WARN(rclcpp::get_logger("nrs_interpolation"),
                    "original_waypoints size is less than 2. Returning empty waypoints.");
        return output;
    }

    if (desired_interval <= 0.0)
        desired_interval = 0.001;

    std::vector<geometry_msgs::msg::Point> original_points;
    original_points.reserve(original_waypoints.waypoints.size());

    for (const auto &wp : original_waypoints.waypoints)
    {
        geometry_msgs::msg::Point pt;
        pt.x = wp.x;
        pt.y = wp.y;
        pt.z = wp.z;

        if (!original_points.empty())
        {
            Eigen::Vector3d prev = toEigen(original_points.back());
            Eigen::Vector3d cur = toEigen(pt);
            if ((cur - prev).norm() < 1e-12)
                continue;
        }

        original_points.push_back(pt);
    }

    if (original_points.size() < 2)
    {
        RCLCPP_WARN(rclcpp::get_logger("nrs_interpolation"),
                    "Filtered original_points size is less than 2. Returning empty waypoints.");
        return output;
    }

    std::vector<Eigen::Vector3d> original_normals;
    original_normals.reserve(original_points.size());

    for (size_t i = 0; i < original_points.size(); ++i)
    {
        Eigen::Vector3d n = getFaceNormal(original_points[i], mesh);

        if (n.norm() < 1e-12)
        {
            if (!original_normals.empty())
                n = original_normals.back();
            else
                n = Eigen::Vector3d(0.0, 0.0, 1.0);
        }

        if (!original_normals.empty() && n.dot(original_normals.back()) < 0.0)
            n = -n;

        original_normals.push_back(n.normalized());
    }

    std::vector<double> cumulative_distances = computeCumulativeDistances(original_points);
    double total_distance = cumulative_distances.back();

    if (total_distance < 1e-12)
    {
        return buildWaypointsFromPointsAndNormals(original_points, original_normals, fx, fy, fz);
    }

    std::vector<geometry_msgs::msg::Point> resampled_points;
    std::vector<Eigen::Vector3d> resampled_normals;

    resampled_points.reserve(static_cast<size_t>(std::ceil(total_distance / desired_interval)) + 2);
    resampled_normals.reserve(static_cast<size_t>(std::ceil(total_distance / desired_interval)) + 2);

    Eigen::Vector3d prev_resampled_normal = original_normals.front();

    for (double d = 0.0; d <= total_distance; d += desired_interval)
    {
        size_t i = 1;
        while (i < cumulative_distances.size() && cumulative_distances[i] < d)
            ++i;

        if (i >= cumulative_distances.size())
            i = cumulative_distances.size() - 1;

        double seg_len = cumulative_distances[i] - cumulative_distances[i - 1];
        double t = 0.0;
        if (seg_len > 1e-12)
            t = (d - cumulative_distances[i - 1]) / seg_len;

        geometry_msgs::msg::Point p =
            interpolatePointLinear(original_points[i - 1], original_points[i], t);

        Eigen::Vector3d n =
            interpolateNormalLinear(original_normals[i - 1], original_normals[i], t);

        if (n.dot(prev_resampled_normal) < 0.0)
            n = -n;

        if (!resampled_points.empty())
        {
            Eigen::Vector3d prev = toEigen(resampled_points.back());
            Eigen::Vector3d cur = toEigen(p);
            if ((cur - prev).norm() < 1e-12)
                continue;
        }

        resampled_points.push_back(p);
        resampled_normals.push_back(n.normalized());
        prev_resampled_normal = n.normalized();
    }

    if ((toEigen(resampled_points.back()) - toEigen(original_points.back())).norm() > 1e-12)
    {
        resampled_points.push_back(original_points.back());
        Eigen::Vector3d n_last = original_normals.back();
        if (n_last.dot(prev_resampled_normal) < 0.0)
            n_last = -n_last;
        resampled_normals.push_back(n_last.normalized());
    }

    output = buildWaypointsFromPointsAndNormals(resampled_points, resampled_normals, fx, fy, fz);
    return output;
}