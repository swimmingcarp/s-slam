#include "data_processors/lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr int kFeatureGroupSize = 8;
constexpr double kPlaneDistanceScale = 0.01;
constexpr double kPlaneDistanceOffset = 0.1;
constexpr double kInfinitePointRangeM = 10.0;
constexpr double kPlaneToLineRatio = 225.0;
constexpr double kAviaMaxToMidDistanceRatio = 6.25;
constexpr double kAviaMidToMinDistanceRatio = 6.25;
constexpr double kMaxToMinDistanceRatio = 3.24;
constexpr double kEdgeDistanceRatio = 2.0;
constexpr double kEdgeDistanceDifference = 0.1;
constexpr double kSmallPlaneDistanceRatio = 1.2;
constexpr double kPi = 3.14159265358979323846;
constexpr float kPilotZoneHalfWidth = 0.6F;
constexpr double kPlaneDirectionMinimumNorm = 0.1;
constexpr double kMinimumVectorNorm = 1.0e-12;
constexpr double kEdgePlaneDirectionCosineLimit = 0.707;
constexpr double kEdgeJumpMinimumDistance = 0.0225;
constexpr double kEdgeJumpNeighborDistanceRatio = 4.0;
constexpr std::array<int, 2> kNeighborPointOffsets = {-1, 1};
constexpr std::array<int, 2> kEdgeJumpFirstDistanceOffsets = {-1, 0};
constexpr std::array<int, 2> kEdgeJumpSecondDistanceOffsets = {-2, 1};

double cosineFromDegrees(const double degrees)
{
    return std::cos(degrees / 180.0 * kPi);
}

const double kJumpUpCosineThreshold = cosineFromDegrees(170.0);
const double kJumpDownCosineThreshold = cosineFromDegrees(8.0);
const double kEdgeIntersectionCosineThreshold = cosineFromDegrees(160.0);
const double kSmallPlaneIntersectionCosineThreshold = cosineFromDegrees(172.5);

}  // namespace

LidarProcessor::LidarProcessor(const Config &config)
    : lidar_type_(config.lidar_type),
      scan_line_count_(config.scan_line_count),
      scan_rate_hz_(config.scan_rate_hz),
      point_filter_stride_(config.point_filter_stride),
      blind_distance_(config.blind_distance),
      pilot_zone_blind_distance_(config.pilot_zone_blind_distance),
      feature_extraction_enabled_(config.feature_extraction_enabled),
      time_unit_scale_(timestampUnitScale(config.timestamp_unit)),
      scan_start_time_(-1.0),
      scan_end_time_(-1.0)
{
    validateConfig(config);
}

LidarType LidarProcessor::GetLidarType(const int value)
{
    switch (value)
    {
        case static_cast<int>(LidarType::kAvia):
            return LidarType::kAvia;
        case static_cast<int>(LidarType::kVelo16):
            return LidarType::kVelo16;
        case static_cast<int>(LidarType::kOuster64):
            return LidarType::kOuster64;
        case static_cast<int>(LidarType::kKimeraOuster64):
            return LidarType::kKimeraOuster64;
        case static_cast<int>(LidarType::kRoboSense):
            return LidarType::kRoboSense;
        default:
            throw std::invalid_argument("preprocess.lidar_type must be between 1 and 5");
    }
}

TimestampUnit LidarProcessor::GetTimestampUnit(const int value)
{
    switch (value)
    {
        case static_cast<int>(TimestampUnit::kSeconds):
            return TimestampUnit::kSeconds;
        case static_cast<int>(TimestampUnit::kMilliseconds):
            return TimestampUnit::kMilliseconds;
        case static_cast<int>(TimestampUnit::kMicroseconds):
            return TimestampUnit::kMicroseconds;
        case static_cast<int>(TimestampUnit::kNanoseconds):
            return TimestampUnit::kNanoseconds;
        default:
            throw std::invalid_argument("preprocess.timestamp_unit must be between 0 and 3");
    }
}

void LidarProcessor::validateConfig(const Config &config)
{
    if (config.scan_line_count <= 0 || config.scan_line_count > kMaxScanLines)
    {
        throw std::invalid_argument("preprocess.scan_line must be between 1 and " +
                                    std::to_string(kMaxScanLines));
    }
    if (config.scan_rate_hz <= 0)
    {
        throw std::invalid_argument("preprocess.scan_rate must be positive");
    }
    if (config.point_filter_stride <= 0)
    {
        throw std::invalid_argument("point_filter_num_for_preprocessing must be positive");
    }
}

float LidarProcessor::timestampUnitScale(const TimestampUnit timestamp_unit)
{
    switch (timestamp_unit)
    {
        case TimestampUnit::kSeconds:
            return 1.e3f;
        case TimestampUnit::kMilliseconds:
            return 1.f;
        case TimestampUnit::kMicroseconds:
            return 1.e-3f;
        case TimestampUnit::kNanoseconds:
            return 1.e-6f;
    }
    throw std::invalid_argument("Unsupported LiDAR timestamp unit");
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
bool LidarProcessor::process(
    const livox_ros_driver2::msg::CustomMsg &msg,
    PointCloudXYZI::Ptr &pcl_out)
{
    handleAviaPointCloud(msg);
    *pcl_out = surface_cloud_;
    return !pcl_out->empty();
}
#endif

bool LidarProcessor::process(
    const sensor_msgs::msg::PointCloud2 &msg,
    PointCloudXYZI::Ptr &pcl_out)
{
    scan_start_time_ = -1.0;
    scan_end_time_   = -1.0;

    switch (lidar_type_)
    {
        case LidarType::kOuster64:
            handleOusterPointCloud(msg);
            break;
        case LidarType::kKimeraOuster64:
            handleKimeraOusterPointCloud(msg);
            break;
        case LidarType::kVelo16:
            handleVelodynePointCloud(msg);
            break;
        case LidarType::kRoboSense:
            handleRoboSensePointCloud(msg, *pcl_out);
            return hasScanTime() && !pcl_out->empty();
        default:
            RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                         "Unsupported LiDAR type: %d",
                         static_cast<int>(lidar_type_));
            pcl_out->clear();
            return false;
    }

    *pcl_out = surface_cloud_;
    return !pcl_out->empty();
}

void LidarProcessor::resetFrameClouds()
{
    surface_cloud_.clear();
    corner_cloud_.clear();
    full_cloud_.clear();
    pilot_zone_cloud_.clear();
}

void LidarProcessor::prepareFeatureScanLines(const std::size_t point_count)
{
    const auto points_per_scan_line =
        (point_count + static_cast<std::size_t>(scan_line_count_) - 1) /
        static_cast<std::size_t>(scan_line_count_);
    for (int scan_line = 0; scan_line < scan_line_count_; ++scan_line)
    {
        scan_line_clouds_[scan_line].clear();
        scan_line_clouds_[scan_line].reserve(points_per_scan_line);
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
    for (int scan_line = 0; scan_line < scan_line_count_; ++scan_line)
    {
        auto &points = scan_line_clouds_[scan_line];
        if (points.size() < minimum_scan_line_points)
        {
            continue;
        }

        auto &point_feature_infos = scan_line_feature_infos_[scan_line];
        populatePointFeatureInfo(points, point_feature_infos, neighbor_distance);
        extractFeaturesFromScanLine(points, point_feature_infos);
    }
}

bool LidarProcessor::isFromPilotZone(
    const float point_x,
    const float point_y,
    const PilotZoneOrientation orientation) const
{
    // Kimera-Multi uses opposite forward axes for Ouster and Velodyne.
    const bool is_laterally_centered =
        point_y > -kPilotZoneHalfWidth && point_y < kPilotZoneHalfWidth;
    if (orientation == PilotZoneOrientation::kOuster)
    {
        return is_laterally_centered && point_x > 0.0F && point_x < pilot_zone_blind_distance_;
    }
    return is_laterally_centered && point_x < 0.0F && point_x > -pilot_zone_blind_distance_;
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void LidarProcessor::handleAviaPointCloud(const livox_ros_driver2::msg::CustomMsg &msg)
{
    resetFrameClouds();

    const auto point_count = static_cast<std::size_t>(msg.point_num);
    corner_cloud_.reserve(point_count);
    surface_cloud_.reserve(point_count);
    full_cloud_.resize(point_count);

    std::size_t valid_num = 0;

    if (feature_extraction_enabled_)
    {
        prepareFeatureScanLines(point_count);
        for (std::size_t point_index = 1; point_index < point_count; ++point_index)
        {
            if ((msg.points[point_index].line < scan_line_count_) &&
                ((msg.points[point_index].tag & 0x30) == 0x10 ||
                 (msg.points[point_index].tag & 0x30) == 0x00))
            {
                full_cloud_[point_index].x         = msg.points[point_index].x;
                full_cloud_[point_index].y         = msg.points[point_index].y;
                full_cloud_[point_index].z         = msg.points[point_index].z;
                full_cloud_[point_index].intensity = msg.points[point_index].reflectivity;
                full_cloud_[point_index].curvature =
                    msg.points[point_index].offset_time /
                    static_cast<float>(1000000);  // use curvature as time of each laser points

                if ((std::abs(full_cloud_[point_index].x - full_cloud_[point_index - 1].x) > 1e-7) ||
                    (std::abs(full_cloud_[point_index].y - full_cloud_[point_index - 1].y) > 1e-7) ||
                    (std::abs(full_cloud_[point_index].z - full_cloud_[point_index - 1].z) > 1e-7))
                {
                    scan_line_clouds_[msg.points[point_index].line].push_back(full_cloud_[point_index]);
                }
            }
        }
        extractFeaturesFromScanLines(NeighborDistance::kEuclidean, 6);
    }
    else
    {
        for (std::size_t point_index = 1; point_index < point_count; ++point_index)
        {
            if ((msg.points[point_index].line < scan_line_count_) &&
                ((msg.points[point_index].tag & 0x30) == 0x10 ||
                 (msg.points[point_index].tag & 0x30) == 0x00))
            {
                ++valid_num;
                if (valid_num % point_filter_stride_ == 0)
                {
                    full_cloud_[point_index].x         = msg.points[point_index].x;
                    full_cloud_[point_index].y         = msg.points[point_index].y;
                    full_cloud_[point_index].z         = msg.points[point_index].z;
                    full_cloud_[point_index].intensity = msg.points[point_index].reflectivity;
                    full_cloud_[point_index].curvature =
                        msg.points[point_index].offset_time /
                        static_cast<float>(1000000);  // use curvature as time of each laser points,
                                                      // curvature unit: ms

                    const bool has_new_position =
                        std::abs(full_cloud_[point_index].x - full_cloud_[point_index - 1].x) > 1e-7 ||
                        std::abs(full_cloud_[point_index].y - full_cloud_[point_index - 1].y) > 1e-7 ||
                        std::abs(full_cloud_[point_index].z - full_cloud_[point_index - 1].z) > 1e-7;
                    const double range_squared = full_cloud_[point_index].x * full_cloud_[point_index].x +
                                                 full_cloud_[point_index].y * full_cloud_[point_index].y +
                                                 full_cloud_[point_index].z * full_cloud_[point_index].z;
                    if (has_new_position && range_squared > blind_distance_ * blind_distance_)
                    {
                        surface_cloud_.push_back(full_cloud_[point_index]);
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
    surface_cloud_.reserve(plsize);
    if (feature_extraction_enabled_)
    {
        corner_cloud_.reserve(plsize);
        prepareFeatureScanLines(plsize);

        for (const auto &src : scan.points)
        {
            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range_squared < blind_distance_ * blind_distance_)
            {
                continue;
            }

            if (src.ring < static_cast<std::uint16_t>(scan_line_count_))
            {
                scan_line_clouds_[src.ring].push_back(added_pt);
            }
        }

        extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);
    }
    else
    {
        for (const auto &src : scan.points)
        {
            if (src.source_index % static_cast<std::size_t>(point_filter_stride_) != 0)
            {
                continue;
            }

            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;

            if (range_squared < blind_distance_ * blind_distance_)
            {
                continue;
            }

            surface_cloud_.points.push_back(added_pt);
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
    surface_cloud_.reserve(plsize);
    if (feature_extraction_enabled_)
    {
        corner_cloud_.reserve(plsize);
        prepareFeatureScanLines(plsize);

        for (const auto &src : scan.points)
        {
            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range_squared < blind_distance_ * blind_distance_ ||
                isFromPilotZone(added_pt.x, added_pt.y, PilotZoneOrientation::kOuster))
            {
                continue;
            }

            if (src.ring < static_cast<std::uint16_t>(scan_line_count_))
            {
                scan_line_clouds_[src.ring].push_back(added_pt);
            }
        }

        extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);
    }
    else
    {
        for (const auto &src : scan.points)
        {
            if (src.source_index % static_cast<std::size_t>(point_filter_stride_) != 0)
            {
                continue;
            }

            const auto &added_pt = src.point;
            const double range_squared =
                added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;

            if (range_squared < blind_distance_ * blind_distance_ ||
                isFromPilotZone(added_pt.x, added_pt.y, PilotZoneOrientation::kOuster))
            {
                continue;
            }

            surface_cloud_.points.push_back(added_pt);
        }
    }
}

void LidarProcessor::handleVelodynePointCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
    resetFrameClouds();

    sensor_adapter::InternalScan scan;
    if (!velodyne_adapter_.convert(
            msg, scan_line_count_, scan_rate_hz_, time_unit_scale_, has_point_time_offset_, scan))
    {
        return;
    }

    const auto plsize = scan.points.size();
    surface_cloud_.reserve(plsize);
    if (feature_extraction_enabled_)
    {
        prepareFeatureScanLines(plsize);

        for (const auto &src : scan.points)
        {
            if (src.ring >= static_cast<std::uint16_t>(scan_line_count_))
            {
                continue;
            }

            scan_line_clouds_[src.ring].points.push_back(src.point);
        }

        extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);
    }
    else
    {
        for (const auto &src : scan.points)
        {
            if (src.source_index % static_cast<std::size_t>(point_filter_stride_) == 0)
            {
                const auto &added_pt = src.point;
                if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
                    (blind_distance_ * blind_distance_))
                {
                    if (isFromPilotZone(
                            added_pt.x, added_pt.y, PilotZoneOrientation::kVelodyne))
                    {
                        // Only for visualization
                        pilot_zone_cloud_.push_back(added_pt);
                        continue;
                    }
                    surface_cloud_.points.push_back(added_pt);
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

    if (!feature_extraction_enabled_)
    {
        // When !feature_extraction_enabled_, FAST-LIO needs only the filtered output cloud.
        // Avoid materializing InternalScan/InternalPoint; the feature-enabled path below
        // preserves them for ring grouping and source-index filtering.
        robosense_fairy_adapter_.convertToFilteredCloud(
            msg,
            output,
            static_cast<std::uint16_t>(scan_line_count_),
            point_filter_stride_,
            blind_distance_,
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
    surface_cloud_.reserve(plsize);

    const auto inConfiguredScan = [this](const sensor_adapter::InternalPoint &point) {
        return point.ring < static_cast<std::uint16_t>(scan_line_count_);
    };

    prepareFeatureScanLines(plsize);

    for (const auto &src : scan.points)
    {
        if (!inConfiguredScan(src))
        {
            continue;
        }
        scan_line_clouds_[src.ring].points.push_back(src.point);
    }

    extractFeaturesFromScanLines(NeighborDistance::kSquared, 2);

    output = surface_cloud_;
}

void LidarProcessor::extractFeaturesFromScanLine(
    pcl::PointCloud<PointType> &pl,
    std::vector<PointFeatureInfo> &point_feature_infos)
{
    auto plsize = pl.size();
    std::size_t plsize2;
    if (plsize == 0)
    {
        // ROS_ERROR("something wrong\n");
        return;
    }
    std::size_t head = 0;

    while (head < point_feature_infos.size() && point_feature_infos[head].range < blind_distance_)
    {
        ++head;
    }
    if (head >= point_feature_infos.size())
    {
        return;
    }

    // Surf
    plsize2 = plsize > static_cast<std::size_t>(kFeatureGroupSize)
                  ? plsize - static_cast<std::size_t>(kFeatureGroupSize)
                  : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    std::size_t i_next = 0;
    bool has_previous_plane = false;
    PlaneClassification plane_classification;

    for (std::size_t i = head; i < plsize2; ++i)
    {
        if (point_feature_infos[i].range < blind_distance_)
        {
            continue;
        }

        plane_classification =
            classifyPlaneSegment(pl, point_feature_infos, i, i_next, curr_direct);

        if (plane_classification == PlaneClassification::kPlane)
        {
            for (std::size_t j = i; j <= i_next; ++j)
            {
                if (j != i && j != i_next)
                {
                    point_feature_infos[j].ftype = Real_Plane;
                }
                else
                {
                    point_feature_infos[j].ftype = Poss_Plane;
                }
            }

            if (has_previous_plane && last_direct.norm() > kPlaneDirectionMinimumNorm)
            {
                const double direction_dot_product = last_direct.transpose() * curr_direct;
                if (std::abs(direction_dot_product) < kEdgePlaneDirectionCosineLimit)
                {
                    point_feature_infos[i].ftype = Edge_Plane;
                }
                else
                {
                    point_feature_infos[i].ftype = Real_Plane;
                }
            }

            i = i_next - 1;
            has_previous_plane = true;
        }
        else
        {
            i = i_next;
            has_previous_plane = false;
        }

        last_direct = curr_direct;
    }

    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (std::size_t i = head + 3; i < plsize2; ++i)
    {
        if (point_feature_infos[i].range < blind_distance_ || point_feature_infos[i].ftype >= Real_Plane)
        {
            continue;
        }

        if (point_feature_infos[i - 1].dista < 1e-16 || point_feature_infos[i].dista < 1e-16)
        {
            continue;
        }

        Eigen::Vector3d point_vector(pl[i].x, pl[i].y, pl[i].z);
        const double point_norm = point_vector.norm();
        if (!std::isfinite(point_norm) || point_norm < kMinimumVectorNorm)
        {
            point_feature_infos[i].ftype     = ZeroPoint;
            point_feature_infos[i].edj[Prev] = Nr_zero;
            point_feature_infos[i].edj[Next] = Nr_zero;
            continue;
        }
        std::array<Eigen::Vector3d, 2> neighbor_vectors{
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        std::array<bool, 2> has_neighbor{};

        for (std::size_t neighbor_index = 0;
             neighbor_index < kNeighborPointOffsets.size();
             ++neighbor_index)
        {
            const int neighbor_offset = kNeighborPointOffsets[neighbor_index];

            if (point_feature_infos[i + neighbor_offset].range < blind_distance_)
            {
                if (point_feature_infos[i].range > kInfinitePointRangeM)
                {
                    point_feature_infos[i].edj[neighbor_index] = Nr_inf;
                }
                else
                {
                    point_feature_infos[i].edj[neighbor_index] = Nr_blind;
                }
                continue;
            }

            neighbor_vectors[neighbor_index] =
                Eigen::Vector3d(pl[i + neighbor_offset].x,
                                pl[i + neighbor_offset].y,
                                pl[i + neighbor_offset].z) -
                point_vector;
            const double neighbor_norm = neighbor_vectors[neighbor_index].norm();
            if (!std::isfinite(neighbor_norm) || neighbor_norm < kMinimumVectorNorm)
            {
                point_feature_infos[i].edj[neighbor_index] = Nr_zero;
                continue;
            }
            has_neighbor[neighbor_index] = true;

            point_feature_infos[i].angle[neighbor_index] =
                point_vector.dot(neighbor_vectors[neighbor_index]) /
                point_norm / neighbor_norm;
            if (point_feature_infos[i].angle[neighbor_index] < kJumpUpCosineThreshold)
            {
                point_feature_infos[i].edj[neighbor_index] = Nr_180;
            }
            else if (point_feature_infos[i].angle[neighbor_index] > kJumpDownCosineThreshold)
            {
                point_feature_infos[i].edj[neighbor_index] = Nr_zero;
            }
        }

        if (has_neighbor[Prev] && has_neighbor[Next])
        {
            point_feature_infos[i].intersect =
                neighbor_vectors[Prev].dot(neighbor_vectors[Next]) /
                neighbor_vectors[Prev].norm() / neighbor_vectors[Next].norm();
        }
        else
        {
            point_feature_infos[i].intersect = 0.0;
        }
        if (point_feature_infos[i].edj[Prev] == Nr_nor && point_feature_infos[i].edj[Next] == Nr_zero &&
            point_feature_infos[i].dista > kEdgeJumpMinimumDistance &&
            point_feature_infos[i].dista >
                kEdgeJumpNeighborDistanceRatio * point_feature_infos[i - 1].dista)
        {
            if (point_feature_infos[i].intersect > kEdgeIntersectionCosineThreshold)
            {
                if (isEdgeJump(pl, point_feature_infos, i, Prev))
                {
                    point_feature_infos[i].ftype = Edge_Jump;
                }
            }
        }
        else if (point_feature_infos[i].edj[Prev] == Nr_zero && point_feature_infos[i].edj[Next] == Nr_nor &&
                 point_feature_infos[i - 1].dista > kEdgeJumpMinimumDistance &&
                 point_feature_infos[i - 1].dista >
                     kEdgeJumpNeighborDistanceRatio * point_feature_infos[i].dista)
        {
            if (point_feature_infos[i].intersect > kEdgeIntersectionCosineThreshold)
            {
                if (isEdgeJump(pl, point_feature_infos, i, Next))
                {
                    point_feature_infos[i].ftype = Edge_Jump;
                }
            }
        }
        else if (point_feature_infos[i].edj[Prev] == Nr_nor && point_feature_infos[i].edj[Next] == Nr_inf)
        {
            if (isEdgeJump(pl, point_feature_infos, i, Prev))
            {
                point_feature_infos[i].ftype = Edge_Jump;
            }
        }
        else if (point_feature_infos[i].edj[Prev] == Nr_inf && point_feature_infos[i].edj[Next] == Nr_nor)
        {
            if (isEdgeJump(pl, point_feature_infos, i, Next))
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
    for (std::size_t i = head + 1; i < plsize2; ++i)
    {
        if (point_feature_infos[i].range < blind_distance_ || point_feature_infos[i - 1].range < blind_distance_ || point_feature_infos[i + 1].range < blind_distance_)
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

            if (point_feature_infos[i].intersect < kSmallPlaneIntersectionCosineThreshold && ratio < kSmallPlaneDistanceRatio)
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

    std::optional<std::size_t> first_surface_index;
    for (std::size_t j = head; j < plsize; ++j)
    {
        if (point_feature_infos[j].ftype == Poss_Plane || point_feature_infos[j].ftype == Real_Plane)
        {
            if (!first_surface_index)
            {
                first_surface_index = j;
            }

            if (j == *first_surface_index +
                         static_cast<std::size_t>(point_filter_stride_ - 1))
            {
                PointType ap{};
                ap.x         = pl[j].x;
                ap.y         = pl[j].y;
                ap.z         = pl[j].z;
                ap.intensity = pl[j].intensity;
                ap.curvature = pl[j].curvature;
                surface_cloud_.push_back(ap);

                first_surface_index.reset();
            }
        }
        else
        {
            if (point_feature_infos[j].ftype == Edge_Jump || point_feature_infos[j].ftype == Edge_Plane)
            {
                corner_cloud_.push_back(pl[j]);
            }
            if (first_surface_index)
            {
                PointType ap{};
                for (std::size_t k = *first_surface_index; k < j; ++k)
                {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.intensity += pl[k].intensity;
                    ap.curvature += pl[k].curvature;
                }
                const auto point_count = j - *first_surface_index;
                if (point_count == 0)
                {
                    first_surface_index.reset();
                    continue;
                }
                ap.x /= point_count;
                ap.y /= point_count;
                ap.z /= point_count;
                ap.intensity /= point_count;
                ap.curvature /= point_count;
                surface_cloud_.push_back(ap);
            }
            first_surface_index.reset();
        }
    }
}

PlaneClassification LidarProcessor::classifyPlaneSegment(
    const PointCloudXYZI &pl,
    std::vector<PointFeatureInfo> &point_feature_infos,
    std::size_t i_cur,
    std::size_t &i_next,
    Eigen::Vector3d &curr_direct)
{
    double group_dis = kPlaneDistanceScale * point_feature_infos[i_cur].range + kPlaneDistanceOffset;
    group_dis        = group_dis * group_dis;
    // i_next = i_cur;

    double two_dis = 0.0;
    std::vector<double> disarr;
    disarr.reserve(20);
    double direction_x = 0.0;
    double direction_y = 0.0;
    double direction_z = 0.0;

    for (i_next = i_cur; i_next < i_cur + kFeatureGroupSize; ++i_next)
    {
        if (point_feature_infos[i_next].range < blind_distance_)
        {
            curr_direct.setZero();
            return PlaneClassification::kContainsBlindPoint;
        }
        disarr.push_back(point_feature_infos[i_next].dista);
    }

    for (;;)
    {
        if ((i_cur >= pl.size()) || (i_next >= pl.size()))
        {
            break;
        }

        if (point_feature_infos[i_next].range < blind_distance_)
        {
            curr_direct.setZero();
            return PlaneClassification::kContainsBlindPoint;
        }
        direction_x = pl[i_next].x - pl[i_cur].x;
        direction_y = pl[i_next].y - pl[i_cur].y;
        direction_z = pl[i_next].z - pl[i_cur].z;
        two_dis = direction_x * direction_x + direction_y * direction_y + direction_z * direction_z;
        if (two_dis >= group_dis)
        {
            break;
        }
        disarr.push_back(point_feature_infos[i_next].dista);
        ++i_next;
    }

    double leng_wid = 0;
    double v1[3], v2[3];
    for (std::size_t j = i_cur + 1; j < i_next; ++j)
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

    if (!std::isfinite(leng_wid) || leng_wid < kMinimumVectorNorm ||
        (two_dis * two_dis / leng_wid) < kPlaneToLineRatio)
    {
        curr_direct.setZero();
        return PlaneClassification::kNotPlane;
    }

    const auto disarrsize = disarr.size();
    std::sort(disarr.begin(), disarr.end(), std::greater<double>());

    if (disarr[disarr.size() - 2] < 1e-16)
    {
        curr_direct.setZero();
        return PlaneClassification::kNotPlane;
    }

    if (lidar_type_ == LidarType::kAvia)
    {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2];
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

        if (dismax_mid >= kAviaMaxToMidDistanceRatio || dismid_min >= kAviaMidToMinDistanceRatio)
        {
            curr_direct.setZero();
            return PlaneClassification::kNotPlane;
        }
    }
    else
    {
        double dismax_min = disarr[0] / disarr[disarrsize - 2];
        if (dismax_min >= kMaxToMinDistanceRatio)
        {
            curr_direct.setZero();
            return PlaneClassification::kNotPlane;
        }
    }

    curr_direct << direction_x, direction_y, direction_z;
    curr_direct.normalize();
    return PlaneClassification::kPlane;
}

bool LidarProcessor::isEdgeJump(
    const PointCloudXYZI &pl,
    std::vector<PointFeatureInfo> &point_feature_infos,
    std::size_t i,
    Surround neighbor_direction)
{
    if (neighbor_direction == Prev)
    {
        if (point_feature_infos[i - 1].range < blind_distance_ || point_feature_infos[i - 2].range < blind_distance_)
        {
            return false;
        }
    }
    else if (neighbor_direction == Next)
    {
        if (point_feature_infos[i + 1].range < blind_distance_ || point_feature_infos[i + 2].range < blind_distance_)
        {
            return false;
        }
    }
    const auto direction_index = static_cast<std::size_t>(neighbor_direction);
    double first_distance =
        point_feature_infos[i + kEdgeJumpFirstDistanceOffsets[direction_index]].dista;
    double second_distance =
        point_feature_infos[i + kEdgeJumpSecondDistanceOffsets[direction_index]].dista;

    if (first_distance < second_distance)
    {
        std::swap(first_distance, second_distance);
    }

    first_distance = std::sqrt(first_distance);
    second_distance = std::sqrt(second_distance);

    if (first_distance > kEdgeDistanceRatio * second_distance ||
        (first_distance - second_distance) > kEdgeDistanceDifference)
    {
        return false;
    }

    return true;
}
