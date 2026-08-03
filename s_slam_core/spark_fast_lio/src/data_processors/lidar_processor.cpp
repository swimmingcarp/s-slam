#include "data_processors/lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

LidarProcessor::LidarProcessor()
    : lidar_type(AVIA),
      point_filter_num(1),
      blind(0.01),
      blind_for_human_pilots(1.5),
      feature_enabled(false),
      scan_start_time_(-1.0),
      scan_end_time_(-1.0)
{
    inf_bound         = 10;
    N_SCANS           = 6;
    SCAN_RATE         = 10;
    group_size        = 8;
    disA              = 0.01;
    disB              = 0.1;
    p2l_ratio         = 225;
    limit_maxmid      = 6.25;
    limit_midmin      = 6.25;
    limit_maxmin      = 3.24;
    jump_up_limit     = 170.0;
    jump_down_limit   = 8.0;
    cos160            = 160.0;
    edgea             = 2;
    edgeb             = 0.1;
    smallp_intersect  = 172.5;
    smallp_ratio      = 1.2;
    given_offset_time = false;

    jump_up_limit    = std::cos(jump_up_limit / 180 * M_PI);
    jump_down_limit  = std::cos(jump_down_limit / 180 * M_PI);
    cos160           = std::cos(cos160 / 180 * M_PI);
    smallp_intersect = std::cos(smallp_intersect / 180 * M_PI);
    setTimestampUnit(MS);
}

void LidarProcessor::set(bool is_enabled, int lid_type, double bld, int pfilt_num)
{
    feature_enabled       = is_enabled;
    lidar_type            = lid_type;
    blind                 = bld;
    point_filter_num      = std::max(1, pfilt_num);
}

void LidarProcessor::setTimestampUnit(const TIME_UNIT timestamp_unit)
{
    switch (timestamp_unit)
    {
        case SEC:
            time_unit_scale_ = 1.e3f;
            break;
        case MS:
            time_unit_scale_ = 1.f;
            break;
        case US:
            time_unit_scale_ = 1.e-3f;
            break;
        case NS:
            time_unit_scale_ = 1.e-6f;
            break;
        default:
            throw std::invalid_argument("Unsupported LiDAR timestamp unit");
    }
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
bool LidarProcessor::process(
    const livox_ros_driver2::msg::CustomMsg &msg,
    PointCloudXYZI::Ptr &pcl_out)
{
    handleAviaPointCloud(msg);
    *pcl_out = pl_surf;
    return !pcl_out->empty();
}
#endif

bool LidarProcessor::process(const sensor_msgs::msg::PointCloud2 &msg, PointCloudXYZI::Ptr &pcl_out)
{
    scan_start_time_ = -1.0;
    scan_end_time_   = -1.0;

    switch (lidar_type)
    {
        case OUST64:
            handleOusterPointCloud(msg);
            break;
        case KMOUST64:
            handleKimeraOusterPointCloud(msg);
            break;
        case VELO16:
            handleVelodynePointCloud(msg);
            break;
        case ROBOSENSE:
            handleRoboSensePointCloud(msg, *pcl_out);
            return has_scan_time() && !pcl_out->empty();
        default:
            RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                         "Unsupported LiDAR type: %d",
                         lidar_type);
            pcl_out->clear();
            return false;
    }

    *pcl_out = pl_surf;
    return !pcl_out->empty();
}

void LidarProcessor::resetFrameClouds()
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    pl_from_pilots.clear();
}

void LidarProcessor::prepareFeatureScanLines(const std::size_t point_count)
{
    const auto points_per_scan_line =
        (point_count + static_cast<std::size_t>(N_SCANS) - 1) / static_cast<std::size_t>(N_SCANS);
    for (int scan_line = 0; scan_line < N_SCANS; ++scan_line)
    {
        pl_buff[scan_line].clear();
        pl_buff[scan_line].reserve(points_per_scan_line);
    }
}

void LidarProcessor::populatePointFeatureInfo(
    const PointCloudXYZI &scan_line,
    std::vector<PointFeatureInfo> &point_feature_infos,
    const NeighborDistance neighbor_distance) const
{
    point_feature_infos.clear();
    point_feature_infos.resize(scan_line.size());

    const auto last_index = scan_line.size() - 1;
    for (std::size_t point_index = 0; point_index < last_index; ++point_index)
    {
        const auto &point = scan_line[point_index];
        const auto &next_point = scan_line[point_index + 1];
        point_feature_infos[point_index].range = std::sqrt(point.x * point.x + point.y * point.y);

        const double delta_x = point.x - next_point.x;
        const double delta_y = point.y - next_point.y;
        const double delta_z = point.z - next_point.z;
        const double squared_distance =
            delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
        point_feature_infos[point_index].dista =
            neighbor_distance == NeighborDistance::kEuclidean ? std::sqrt(squared_distance)
                                                              : squared_distance;
    }

    const auto &last_point = scan_line[last_index];
    point_feature_infos[last_index].range =
        std::sqrt(last_point.x * last_point.x + last_point.y * last_point.y);
}

void LidarProcessor::extractFeaturesFromScanLines(
    const NeighborDistance neighbor_distance,
    const std::size_t minimum_scan_line_points)
{
    for (int scan_line = 0; scan_line < N_SCANS; ++scan_line)
    {
        auto &points = pl_buff[scan_line];
        if (points.size() < minimum_scan_line_points)
        {
            continue;
        }

        auto &point_feature_infos = scan_line_feature_infos_[scan_line];
        populatePointFeatureInfo(points, point_feature_infos, neighbor_distance);
        give_feature(points, point_feature_infos);
    }
}

bool LidarProcessor::isFromPilotZone(
    const float point_x,
    const float point_y,
    const PilotZoneOrientation orientation) const
{
    // Kimera-Multi uses opposite forward axes for Ouster and Velodyne.
    const bool is_laterally_centered = point_y > -0.6F && point_y < 0.6F;
    if (orientation == PilotZoneOrientation::kOuster)
    {
        return is_laterally_centered && point_x > 0.0F && point_x < blind_for_human_pilots;
    }
    return is_laterally_centered && point_x < 0.0F && point_x > -blind_for_human_pilots;
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void LidarProcessor::handleAviaPointCloud(const livox_ros_driver2::msg::CustomMsg &msg)
{
    resetFrameClouds();

    const auto point_count = static_cast<std::size_t>(msg.point_num);
    pl_corn.reserve(point_count);
    pl_surf.reserve(point_count);
    pl_full.resize(point_count);

    std::size_t valid_num = 0;

    if (feature_enabled)
    {
        prepareFeatureScanLines(point_count);
        for (std::size_t point_index = 1; point_index < point_count; ++point_index)
        {
            if ((msg.points[point_index].line < N_SCANS) &&
                ((msg.points[point_index].tag & 0x30) == 0x10 ||
                 (msg.points[point_index].tag & 0x30) == 0x00))
            {
                pl_full[point_index].x         = msg.points[point_index].x;
                pl_full[point_index].y         = msg.points[point_index].y;
                pl_full[point_index].z         = msg.points[point_index].z;
                pl_full[point_index].intensity = msg.points[point_index].reflectivity;
                pl_full[point_index].curvature =
                    msg.points[point_index].offset_time /
                    static_cast<float>(1000000);  // use curvature as time of each laser points

                if ((std::abs(pl_full[point_index].x - pl_full[point_index - 1].x) > 1e-7) ||
                    (std::abs(pl_full[point_index].y - pl_full[point_index - 1].y) > 1e-7) ||
                    (std::abs(pl_full[point_index].z - pl_full[point_index - 1].z) > 1e-7))
                {
                    pl_buff[msg.points[point_index].line].push_back(pl_full[point_index]);
                }
            }
        }
        extractFeaturesFromScanLines(NeighborDistance::kEuclidean, 6);
    }
    else
    {
        for (std::size_t point_index = 1; point_index < point_count; ++point_index)
        {
            if ((msg.points[point_index].line < N_SCANS) &&
                ((msg.points[point_index].tag & 0x30) == 0x10 ||
                 (msg.points[point_index].tag & 0x30) == 0x00))
            {
                ++valid_num;
                if (valid_num % point_filter_num == 0)
                {
                    pl_full[point_index].x         = msg.points[point_index].x;
                    pl_full[point_index].y         = msg.points[point_index].y;
                    pl_full[point_index].z         = msg.points[point_index].z;
                    pl_full[point_index].intensity = msg.points[point_index].reflectivity;
                    pl_full[point_index].curvature =
                        msg.points[point_index].offset_time /
                        static_cast<float>(1000000);  // use curvature as time of each laser points,
                                                      // curvature unit: ms

                    const bool has_new_position =
                        std::abs(pl_full[point_index].x - pl_full[point_index - 1].x) > 1e-7 ||
                        std::abs(pl_full[point_index].y - pl_full[point_index - 1].y) > 1e-7 ||
                        std::abs(pl_full[point_index].z - pl_full[point_index - 1].z) > 1e-7;
                    const double range_squared = pl_full[point_index].x * pl_full[point_index].x +
                                                 pl_full[point_index].y * pl_full[point_index].y +
                                                 pl_full[point_index].z * pl_full[point_index].z;
                    if (has_new_position && range_squared > blind * blind)
                    {
                        pl_surf.push_back(pl_full[point_index]);
                    }
                }
            }
        }
    }
}
#endif

void LidarProcessor::handleOusterPointCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
    resetFrameClouds();

    sensor_adapter::InternalScan scan;
    if (!ouster_adapter_.convert(msg, time_unit_scale_, scan))
    {
        return;
    }

    const auto plsize = scan.points.size();
    pl_surf.reserve(plsize);
    if (feature_enabled)
    {
        pl_corn.reserve(plsize);
        prepareFeatureScanLines(plsize);

        for (const auto &src : scan.points)
        {
            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range_squared < blind * blind)
            {
                continue;
            }

            if (src.ring < static_cast<std::uint16_t>(N_SCANS))
            {
                pl_buff[src.ring].push_back(added_pt);
            }
        }

        extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);
    }
    else
    {
        for (const auto &src : scan.points)
        {
            if (src.source_index % static_cast<std::size_t>(point_filter_num) != 0)
            {
                continue;
            }

            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;

            if (range_squared < blind * blind)
            {
                continue;
            }

            pl_surf.points.push_back(added_pt);
        }
    }
}

void LidarProcessor::handleKimeraOusterPointCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
    resetFrameClouds();

    sensor_adapter::InternalScan scan;
    if (!kimera_ouster_adapter_.convert(msg, time_unit_scale_, scan))
    {
        return;
    }

    const auto plsize = scan.points.size();
    pl_surf.reserve(plsize);
    if (feature_enabled)
    {
        pl_corn.reserve(plsize);
        prepareFeatureScanLines(plsize);

        for (const auto &src : scan.points)
        {
            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range_squared < blind * blind ||
                isFromPilotZone(added_pt.x, added_pt.y, PilotZoneOrientation::kOuster))
            {
                continue;
            }

            if (src.ring < static_cast<std::uint16_t>(N_SCANS))
            {
                pl_buff[src.ring].push_back(added_pt);
            }
        }

        extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);
    }
    else
    {
        for (const auto &src : scan.points)
        {
            if (src.source_index % static_cast<std::size_t>(point_filter_num) != 0)
            {
                continue;
            }

            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;

            if (range_squared < blind * blind ||
                isFromPilotZone(added_pt.x, added_pt.y, PilotZoneOrientation::kOuster))
            {
                continue;
            }

            pl_surf.points.push_back(added_pt);
        }
    }
}

void LidarProcessor::handleVelodynePointCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
    resetFrameClouds();

    sensor_adapter::InternalScan scan;
    if (!velodyne_adapter_.convert(
            msg, N_SCANS, SCAN_RATE, time_unit_scale_, given_offset_time, scan))
    {
        return;
    }

    const auto plsize = scan.points.size();
    pl_surf.reserve(plsize);
    if (feature_enabled)
    {
        prepareFeatureScanLines(plsize);

        for (const auto &src : scan.points)
        {
            if (src.ring >= static_cast<std::uint16_t>(N_SCANS))
            {
                continue;
            }

            pl_buff[src.ring].points.push_back(src.point);
        }

        extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);
    }
    else
    {
        for (const auto &src : scan.points)
        {
            if (src.source_index % static_cast<std::size_t>(point_filter_num) == 0)
            {
                const auto &added_pt = src.point;
                if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
                    (blind * blind))
                {
                    if (isFromPilotZone(
                            added_pt.x, added_pt.y, PilotZoneOrientation::kVelodyne))
                    {
                        // Only for visualization
                        pl_from_pilots.push_back(added_pt);
                        continue;
                    }
                    pl_surf.points.push_back(added_pt);
                }
            }
        }
    }
}

void LidarProcessor::handleRoboSensePointCloud(
    const sensor_msgs::msg::PointCloud2 &msg,
    PointCloudXYZI &output)
{
    scan_start_time_ = -1.0;
    scan_end_time_   = -1.0;

    if (!feature_enabled)
    {
        // When !feature_enabled, FAST-LIO needs only the filtered output cloud.
        // Avoid materializing InternalScan/InternalPoint; the feature-enabled path below
        // preserves them for ring grouping and source-index filtering.
        robosense_fairy_adapter_.convertToFilteredCloud(msg,
                                                        output,
                                                        static_cast<std::uint16_t>(N_SCANS),
                                                        point_filter_num,
                                                        blind,
                                                        scan_start_time_,
                                                        scan_end_time_);
        return;
    }

    resetFrameClouds();

    sensor_adapter::InternalScan scan;
    if (!robosense_fairy_adapter_.convert(msg, scan))
    {
        return;
    }

    scan_start_time_ = scan.start_time;
    scan_end_time_   = scan.end_time;
    const auto plsize = scan.points.size();
    pl_surf.reserve(plsize);

    const auto inConfiguredScan = [this](const sensor_adapter::InternalPoint &point) {
        return point.ring < static_cast<std::uint16_t>(N_SCANS);
    };

    prepareFeatureScanLines(plsize);

    for (const auto &src : scan.points)
    {
        if (!inConfiguredScan(src))
        {
            continue;
        }
        pl_buff[src.ring].points.push_back(src.point);
    }

    extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);

    output = pl_surf;
}

void LidarProcessor::give_feature(
    pcl::PointCloud<PointType> &pl,
    std::vector<PointFeatureInfo> &point_feature_infos)
{
    auto plsize = pl.size();
    size_t plsize2;
    if (plsize == 0)
    {
        // ROS_ERROR("something wrong\n");
        return;
    }
    uint head = 0;

    while (head < point_feature_infos.size() && point_feature_infos[head].range < blind)
    {
        ++head;
    }
    if (head >= point_feature_infos.size())
    {
        return;
    }

    // Surf
    plsize2 =
        (plsize > static_cast<size_t>(group_size)) ? (plsize - static_cast<size_t>(group_size)) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex     = 0;
    int last_state = 0;
    int plane_type;

    for (uint i = head; i < plsize2; ++i)
    {
        if (point_feature_infos[i].range < blind)
        {
            continue;
        }

        plane_type = plane_judge(pl, point_feature_infos, i, i_nex, curr_direct);

        if (plane_type == 1)
        {
            for (uint j = i; j <= i_nex; ++j)
            {
                if (j != i && j != i_nex)
                {
                    point_feature_infos[j].ftype = Real_Plane;
                }
                else
                {
                    point_feature_infos[j].ftype = Poss_Plane;
                }
            }

            // if(last_state==1 && fabs(last_direct.sum())>0.5)
            if (last_state == 1 && last_direct.norm() > 0.1)
            {
                double mod = last_direct.transpose() * curr_direct;
                if (mod > -0.707 && mod < 0.707)
                {
                    point_feature_infos[i].ftype = Edge_Plane;
                }
                else
                {
                    point_feature_infos[i].ftype = Real_Plane;
                }
            }

            i = i_nex - 1;
            last_state = 1;
        }
        else
        {
            i = i_nex;
            last_state = 0;
        }

        last_direct = curr_direct;
    }

    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (uint i = head + 3; i < plsize2; ++i)
    {
        if (point_feature_infos[i].range < blind || point_feature_infos[i].ftype >= Real_Plane)
        {
            continue;
        }

        if (point_feature_infos[i - 1].dista < 1e-16 || point_feature_infos[i].dista < 1e-16)
        {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
        Eigen::Vector3d vecs[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        bool has_neighbor[2] = {false, false};

        for (int j = 0; j < 2; ++j)
        {
            int m = -1;
            if (j == 1)
            {
                m = 1;
            }

            if (point_feature_infos[i + m].range < blind)
            {
                if (point_feature_infos[i].range > inf_bound)
                {
                    point_feature_infos[i].edj[j] = Nr_inf;
                }
                else
                {
                    point_feature_infos[i].edj[j] = Nr_blind;
                }
                continue;
            }

            vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z) - vec_a;
            const double neighbor_norm = vecs[j].norm();
            if (neighbor_norm < 1.0e-12)
            {
                point_feature_infos[i].edj[j] = Nr_zero;
                continue;
            }
            has_neighbor[j] = true;

            point_feature_infos[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / neighbor_norm;
            if (point_feature_infos[i].angle[j] < jump_up_limit)
            {
                point_feature_infos[i].edj[j] = Nr_180;
            }
            else if (point_feature_infos[i].angle[j] > jump_down_limit)
            {
                point_feature_infos[i].edj[j] = Nr_zero;
            }
        }

        if (has_neighbor[Prev] && has_neighbor[Next])
        {
            point_feature_infos[i].intersect =
                vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
        }
        else
        {
            point_feature_infos[i].intersect = 0.0;
        }
        if (point_feature_infos[i].edj[Prev] == Nr_nor && point_feature_infos[i].edj[Next] == Nr_zero &&
            point_feature_infos[i].dista > 0.0225 && point_feature_infos[i].dista > 4 * point_feature_infos[i - 1].dista)
        {
            if (point_feature_infos[i].intersect > cos160)
            {
                if (edge_jump_judge(pl, point_feature_infos, i, Prev))
                {
                    point_feature_infos[i].ftype = Edge_Jump;
                }
            }
        }
        else if (point_feature_infos[i].edj[Prev] == Nr_zero && point_feature_infos[i].edj[Next] == Nr_nor &&
                 point_feature_infos[i - 1].dista > 0.0225 && point_feature_infos[i - 1].dista > 4 * point_feature_infos[i].dista)
        {
            if (point_feature_infos[i].intersect > cos160)
            {
                if (edge_jump_judge(pl, point_feature_infos, i, Next))
                {
                    point_feature_infos[i].ftype = Edge_Jump;
                }
            }
        }
        else if (point_feature_infos[i].edj[Prev] == Nr_nor && point_feature_infos[i].edj[Next] == Nr_inf)
        {
            if (edge_jump_judge(pl, point_feature_infos, i, Prev))
            {
                point_feature_infos[i].ftype = Edge_Jump;
            }
        }
        else if (point_feature_infos[i].edj[Prev] == Nr_inf && point_feature_infos[i].edj[Next] == Nr_nor)
        {
            if (edge_jump_judge(pl, point_feature_infos, i, Next))
            {
                point_feature_infos[i].ftype = Edge_Jump;
            }
        }
        else if (point_feature_infos[i].edj[Prev] > Nr_nor && point_feature_infos[i].edj[Next] > Nr_nor)
        {
            if (point_feature_infos[i].ftype == Nor)
            {
                point_feature_infos[i].ftype = Wire;
            }
        }
    }

    plsize2 = plsize - 1;
    double ratio;
    for (uint i = head + 1; i < plsize2; ++i)
    {
        if (point_feature_infos[i].range < blind || point_feature_infos[i - 1].range < blind || point_feature_infos[i + 1].range < blind)
        {
            continue;
        }

        if (point_feature_infos[i - 1].dista < 1e-8 || point_feature_infos[i].dista < 1e-8)
        {
            continue;
        }

        if (point_feature_infos[i].ftype == Nor)
        {
            if (point_feature_infos[i - 1].dista > point_feature_infos[i].dista)
            {
                ratio = point_feature_infos[i - 1].dista / point_feature_infos[i].dista;
            }
            else
            {
                ratio = point_feature_infos[i].dista / point_feature_infos[i - 1].dista;
            }

            if (point_feature_infos[i].intersect < smallp_intersect && ratio < smallp_ratio)
            {
                if (point_feature_infos[i - 1].ftype == Nor)
                {
                    point_feature_infos[i - 1].ftype = Real_Plane;
                }
                if (point_feature_infos[i + 1].ftype == Nor)
                {
                    point_feature_infos[i + 1].ftype = Real_Plane;
                }
                point_feature_infos[i].ftype = Real_Plane;
            }
        }
    }

    int last_surface = -1;
    for (uint j = head; j < plsize; ++j)
    {
        if (point_feature_infos[j].ftype == Poss_Plane || point_feature_infos[j].ftype == Real_Plane)
        {
            if (last_surface == -1)
            {
                last_surface = j;
            }

            if (j == uint(last_surface + point_filter_num - 1))
            {
                PointType ap{};
                ap.x         = pl[j].x;
                ap.y         = pl[j].y;
                ap.z         = pl[j].z;
                ap.intensity = pl[j].intensity;
                ap.curvature = pl[j].curvature;
                pl_surf.push_back(ap);

                last_surface = -1;
            }
        }
        else
        {
            if (point_feature_infos[j].ftype == Edge_Jump || point_feature_infos[j].ftype == Edge_Plane)
            {
                pl_corn.push_back(pl[j]);
            }
            if (last_surface != -1)
            {
                PointType ap{};
                for (uint k = last_surface; k < j; ++k)
                {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.intensity += pl[k].intensity;
                    ap.curvature += pl[k].curvature;
                }
                ap.x /= (j - last_surface);
                ap.y /= (j - last_surface);
                ap.z /= (j - last_surface);
                ap.intensity /= (j - last_surface);
                ap.curvature /= (j - last_surface);
                pl_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

int LidarProcessor::plane_judge(
    const PointCloudXYZI &pl,
    std::vector<PointFeatureInfo> &point_feature_infos,
    uint i_cur,
    uint &i_nex,
    Eigen::Vector3d &curr_direct)
{
    double group_dis = disA * point_feature_infos[i_cur].range + disB;
    group_dis        = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis = 0.0;
    std::vector<double> disarr;
    disarr.reserve(20);
    double direction_x = 0.0;
    double direction_y = 0.0;
    double direction_z = 0.0;

    for (i_nex = i_cur; i_nex < i_cur + group_size; ++i_nex)
    {
        if (point_feature_infos[i_nex].range < blind)
        {
            curr_direct.setZero();
            return 2;
        }
        disarr.push_back(point_feature_infos[i_nex].dista);
    }

    for (;;)
    {
        if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
        {
            break;
        }

        if (point_feature_infos[i_nex].range < blind)
        {
            curr_direct.setZero();
            return 2;
        }
        direction_x = pl[i_nex].x - pl[i_cur].x;
        direction_y = pl[i_nex].y - pl[i_cur].y;
        direction_z = pl[i_nex].z - pl[i_cur].z;
        two_dis = direction_x * direction_x + direction_y * direction_y + direction_z * direction_z;
        if (two_dis >= group_dis)
        {
            break;
        }
        disarr.push_back(point_feature_infos[i_nex].dista);
        ++i_nex;
    }

    double leng_wid = 0;
    double v1[3], v2[3];
    for (uint j = i_cur + 1; j < i_nex; ++j)
    {
        if ((j >= pl.size()) || (i_cur >= pl.size()))
        {
            break;
        }
        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;

        v2[0] = v1[1] * direction_z - direction_y * v1[2];
        v2[1] = v1[2] * direction_x - v1[0] * direction_z;
        v2[2] = v1[0] * direction_y - direction_x * v1[1];

        double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
        if (lw > leng_wid)
        {
            leng_wid = lw;
        }
    }

    if ((two_dis * two_dis / leng_wid) < p2l_ratio)
    {
        curr_direct.setZero();
        return 0;
    }

    const auto disarrsize = disarr.size();
    std::sort(disarr.begin(), disarr.end(), std::greater<double>());

    if (disarr[disarr.size() - 2] < 1e-16)
    {
        curr_direct.setZero();
        return 0;
    }

    if (lidar_type == AVIA)
    {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2];
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

        if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
        {
            curr_direct.setZero();
            return 0;
        }
    }
    else
    {
        double dismax_min = disarr[0] / disarr[disarrsize - 2];
        if (dismax_min >= limit_maxmin)
        {
            curr_direct.setZero();
            return 0;
        }
    }

    curr_direct << direction_x, direction_y, direction_z;
    curr_direct.normalize();
    return 1;
}

bool LidarProcessor::edge_jump_judge(
    const PointCloudXYZI &pl,
    std::vector<PointFeatureInfo> &point_feature_infos,
    uint i,
    Surround nor_dir)
{
    if (nor_dir == 0)
    {
        if (point_feature_infos[i - 1].range < blind || point_feature_infos[i - 2].range < blind)
        {
            return false;
        }
    }
    else if (nor_dir == 1)
    {
        if (point_feature_infos[i + 1].range < blind || point_feature_infos[i + 2].range < blind)
        {
            return false;
        }
    }
    double d1 = point_feature_infos[i + nor_dir - 1].dista;
    double d2 = point_feature_infos[i + 3 * nor_dir - 2].dista;
    double d;

    if (d1 < d2)
    {
        d  = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = sqrt(d1);
    d2 = sqrt(d2);

    if (d1 > edgea * d2 || (d1 - d2) > edgeb)
    {
        return false;
    }

    return true;
}
