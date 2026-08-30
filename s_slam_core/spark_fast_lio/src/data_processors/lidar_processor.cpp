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
constexpr double kRoboSenseMaximumScanDurationScale = 1.2;
constexpr double kSeyondMaximumScanDurationScale = 1.2;
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

constexpr std::size_t neighborIndex(const NeighborDirection direction)
{
    return static_cast<std::size_t>(direction);
}

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
        case static_cast<int>(LidarType::kSeyond):
            return LidarType::kSeyond;
        default:
            throw std::invalid_argument("preprocess.lidar_type must be between 1 and 6");
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
        case LidarType::kSeyond:
            handleSeyondPointCloud(msg, *pcl_out);
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
        point_feature_infos[point_index].neighbor_distance =
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
        std::size_t valid_num = 0;
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

    const auto point_count = scan.points.size();
    surface_cloud_.reserve(point_count);
    if (feature_extraction_enabled_)
    {
        corner_cloud_.reserve(point_count);
        prepareFeatureScanLines(point_count);

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

    const auto point_count = scan.points.size();
    surface_cloud_.reserve(point_count);
    if (feature_extraction_enabled_)
    {
        corner_cloud_.reserve(point_count);
        prepareFeatureScanLines(point_count);

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
    scan_start_time_ = scan.start_time;
    scan_end_time_   = scan.end_time;

    const auto point_count = scan.points.size();
    surface_cloud_.reserve(point_count);
    if (feature_extraction_enabled_)
    {
        prepareFeatureScanLines(point_count);

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
    // A RoboSense cloud represents one rotation. Reuse the existing 20%
    // timing tolerance to prevent a single bad point timestamp from extending
    // the synchronized LiDAR frame.
    const double maximum_scan_duration =
        kRoboSenseMaximumScanDurationScale / static_cast<double>(scan_rate_hz_);

    if (!feature_extraction_enabled_)
    {
        // When !feature_extraction_enabled_, FAST-LIO needs only the filtered output cloud.
        // Avoid materializing InternalScan/InternalPoint; the feature-enabled path below
        // preserves them for ring grouping and source-index filtering.
        if (!robosense_fairy_adapter_.convertToFilteredCloud(
                msg,
                output,
                static_cast<std::uint16_t>(scan_line_count_),
                point_filter_stride_,
                blind_distance_,
                maximum_scan_duration,
                scan_start_time_,
                scan_end_time_))
        {
            return;
        }
        return;
    }

    resetFrameClouds();

    sensor_adapter::InternalScan scan;
    if (!robosense_fairy_adapter_.convert(msg, maximum_scan_duration, scan))
    {
        return;
    }

    scan_start_time_ = scan.start_time;
    scan_end_time_   = scan.end_time;
    const auto point_count = scan.points.size();
    surface_cloud_.reserve(point_count);

    const auto inConfiguredScan = [this](const sensor_adapter::InternalPoint &point) {
        return point.ring < static_cast<std::uint16_t>(scan_line_count_);
    };

    prepareFeatureScanLines(point_count);

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

void LidarProcessor::handleSeyondPointCloud(
    const sensor_msgs::msg::PointCloud2 &msg,
    PointCloudXYZI &output)
{
    scan_start_time_ = -1.0;
    scan_end_time_ = -1.0;
    if (feature_extraction_enabled_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LidarProcessor"),
                     "Seyond Falcon feature extraction is unsupported because scan_id is not a "
                     "conventional ring. Disable feature extraction.");
        output.clear();
        return;
    }

    const double maximum_scan_duration =
        kSeyondMaximumScanDurationScale / static_cast<double>(scan_rate_hz_);
    seyond_adapter_.convertToFilteredCloud(msg,
                                            output,
                                            point_filter_stride_,
                                            blind_distance_,
                                            maximum_scan_duration,
                                            scan_start_time_,
                                            scan_end_time_);
}

void LidarProcessor::extractFeaturesFromScanLine(
    PointCloudXYZI &scan_line,
    std::vector<PointFeatureInfo> &point_feature_infos)
{
    const std::size_t point_count = scan_line.size();
    if (point_count == 0)
    {
        return;
    }
    std::size_t first_visible_index = 0;

    while (first_visible_index < point_feature_infos.size() &&
           point_feature_infos[first_visible_index].range < blind_distance_)
    {
        ++first_visible_index;
    }
    if (first_visible_index >= point_feature_infos.size())
    {
        return;
    }

    std::size_t feature_end_index =
        point_count > static_cast<std::size_t>(kFeatureGroupSize)
            ? point_count - static_cast<std::size_t>(kFeatureGroupSize)
            : 0;

    Eigen::Vector3d plane_direction(Eigen::Vector3d::Zero());
    Eigen::Vector3d previous_plane_direction(Eigen::Vector3d::Zero());

    std::size_t i_next = 0;
    bool has_previous_plane = false;
    PlaneClassification plane_classification;

    for (std::size_t i = first_visible_index; i < feature_end_index; ++i)
    {
        if (point_feature_infos[i].range < blind_distance_)
        {
            continue;
        }

        plane_classification =
            classifyPlaneSegment(scan_line, point_feature_infos, i, i_next, plane_direction);

        if (plane_classification == PlaneClassification::kPlane)
        {
            for (std::size_t j = i; j <= i_next; ++j)
            {
                if (j != i && j != i_next)
                {
                    point_feature_infos[j].feature_type = FeatureType::kPlane;
                }
                else
                {
                    point_feature_infos[j].feature_type = FeatureType::kPossiblePlane;
                }
            }

            if (has_previous_plane &&
                previous_plane_direction.norm() > kPlaneDirectionMinimumNorm)
            {
                const double direction_dot_product =
                    previous_plane_direction.transpose() * plane_direction;
                if (std::abs(direction_dot_product) < kEdgePlaneDirectionCosineLimit)
                {
                    point_feature_infos[i].feature_type = FeatureType::kPlaneEdge;
                }
                else
                {
                    point_feature_infos[i].feature_type = FeatureType::kPlane;
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

        previous_plane_direction = plane_direction;
    }

    feature_end_index = point_count > 3 ? point_count - 3 : 0;
    for (std::size_t i = first_visible_index + 3; i < feature_end_index; ++i)
    {
        if (point_feature_infos[i].range < blind_distance_ ||
            point_feature_infos[i].feature_type >= FeatureType::kPlane)
        {
            continue;
        }

        if (point_feature_infos[i - 1].neighbor_distance < 1e-16 ||
            point_feature_infos[i].neighbor_distance < 1e-16)
        {
            continue;
        }

        Eigen::Vector3d point_vector(scan_line[i].x, scan_line[i].y, scan_line[i].z);
        const double point_norm = point_vector.norm();
        if (!std::isfinite(point_norm) || point_norm < kMinimumVectorNorm)
        {
            point_feature_infos[i].feature_type = FeatureType::kZeroPoint;
            point_feature_infos[i].neighbor_state[neighborIndex(NeighborDirection::kPrevious)] =
                NeighborState::kZeroDistance;
            point_feature_infos[i].neighbor_state[neighborIndex(NeighborDirection::kNext)] =
                NeighborState::kZeroDistance;
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
                    point_feature_infos[i].neighbor_state[neighbor_index] =
                        NeighborState::kInfiniteRange;
                }
                else
                {
                    point_feature_infos[i].neighbor_state[neighbor_index] =
                        NeighborState::kBlindRange;
                }
                continue;
            }

            neighbor_vectors[neighbor_index] =
                Eigen::Vector3d(scan_line[i + neighbor_offset].x,
                                scan_line[i + neighbor_offset].y,
                                scan_line[i + neighbor_offset].z) -
                point_vector;
            const double neighbor_norm = neighbor_vectors[neighbor_index].norm();
            if (!std::isfinite(neighbor_norm) || neighbor_norm < kMinimumVectorNorm)
            {
                point_feature_infos[i].neighbor_state[neighbor_index] =
                    NeighborState::kZeroDistance;
                continue;
            }
            has_neighbor[neighbor_index] = true;

            point_feature_infos[i].neighbor_angle[neighbor_index] =
                point_vector.dot(neighbor_vectors[neighbor_index]) /
                point_norm / neighbor_norm;
            if (point_feature_infos[i].neighbor_angle[neighbor_index] < kJumpUpCosineThreshold)
            {
                point_feature_infos[i].neighbor_state[neighbor_index] =
                    NeighborState::kOppositeDirection;
            }
            else if (point_feature_infos[i].neighbor_angle[neighbor_index] >
                     kJumpDownCosineThreshold)
            {
                point_feature_infos[i].neighbor_state[neighbor_index] =
                    NeighborState::kZeroDistance;
            }
        }

        const std::size_t previous_neighbor = neighborIndex(NeighborDirection::kPrevious);
        const std::size_t next_neighbor = neighborIndex(NeighborDirection::kNext);
        if (has_neighbor[previous_neighbor] && has_neighbor[next_neighbor])
        {
            point_feature_infos[i].neighbor_intersection_cosine =
                neighbor_vectors[previous_neighbor].dot(neighbor_vectors[next_neighbor]) /
                neighbor_vectors[previous_neighbor].norm() / neighbor_vectors[next_neighbor].norm();
        }
        else
        {
            point_feature_infos[i].neighbor_intersection_cosine = 0.0;
        }
        if (point_feature_infos[i].neighbor_state[previous_neighbor] == NeighborState::kNormal &&
            point_feature_infos[i].neighbor_state[next_neighbor] == NeighborState::kZeroDistance &&
            point_feature_infos[i].neighbor_distance > kEdgeJumpMinimumDistance &&
            point_feature_infos[i].neighbor_distance >
                kEdgeJumpNeighborDistanceRatio * point_feature_infos[i - 1].neighbor_distance)
        {
            if (point_feature_infos[i].neighbor_intersection_cosine >
                kEdgeIntersectionCosineThreshold)
            {
                if (isEdgeJump(point_feature_infos, i, NeighborDirection::kPrevious))
                {
                    point_feature_infos[i].feature_type = FeatureType::kJumpEdge;
                }
            }
        }
        else if (point_feature_infos[i].neighbor_state[previous_neighbor] ==
                     NeighborState::kZeroDistance &&
                 point_feature_infos[i].neighbor_state[next_neighbor] == NeighborState::kNormal &&
                 point_feature_infos[i - 1].neighbor_distance > kEdgeJumpMinimumDistance &&
                 point_feature_infos[i - 1].neighbor_distance >
                     kEdgeJumpNeighborDistanceRatio * point_feature_infos[i].neighbor_distance)
        {
            if (point_feature_infos[i].neighbor_intersection_cosine >
                kEdgeIntersectionCosineThreshold)
            {
                if (isEdgeJump(point_feature_infos, i, NeighborDirection::kNext))
                {
                    point_feature_infos[i].feature_type = FeatureType::kJumpEdge;
                }
            }
        }
        else if (point_feature_infos[i].neighbor_state[previous_neighbor] == NeighborState::kNormal &&
                 point_feature_infos[i].neighbor_state[next_neighbor] == NeighborState::kInfiniteRange)
        {
            if (isEdgeJump(point_feature_infos, i, NeighborDirection::kPrevious))
            {
                point_feature_infos[i].feature_type = FeatureType::kJumpEdge;
            }
        }
        else if (point_feature_infos[i].neighbor_state[previous_neighbor] ==
                     NeighborState::kInfiniteRange &&
                 point_feature_infos[i].neighbor_state[next_neighbor] == NeighborState::kNormal)
        {
            if (isEdgeJump(point_feature_infos, i, NeighborDirection::kNext))
            {
                point_feature_infos[i].feature_type = FeatureType::kJumpEdge;
            }
        }
        else if (point_feature_infos[i].neighbor_state[previous_neighbor] != NeighborState::kNormal &&
                 point_feature_infos[i].neighbor_state[next_neighbor] != NeighborState::kNormal)
        {
            if (point_feature_infos[i].feature_type == FeatureType::kNormal)
            {
                point_feature_infos[i].feature_type = FeatureType::kWire;
            }
        }
    }

    feature_end_index = point_count - 1;
    double neighbor_distance_ratio;
    for (std::size_t i = first_visible_index + 1; i < feature_end_index; ++i)
    {
        if (point_feature_infos[i].range < blind_distance_ ||
            point_feature_infos[i - 1].range < blind_distance_ ||
            point_feature_infos[i + 1].range < blind_distance_)
        {
            continue;
        }

        if (point_feature_infos[i - 1].neighbor_distance < 1e-8 ||
            point_feature_infos[i].neighbor_distance < 1e-8)
        {
            continue;
        }

        if (point_feature_infos[i].feature_type == FeatureType::kNormal)
        {
            if (point_feature_infos[i - 1].neighbor_distance >
                point_feature_infos[i].neighbor_distance)
            {
                neighbor_distance_ratio = point_feature_infos[i - 1].neighbor_distance /
                                          point_feature_infos[i].neighbor_distance;
            }
            else
            {
                neighbor_distance_ratio = point_feature_infos[i].neighbor_distance /
                                          point_feature_infos[i - 1].neighbor_distance;
            }

            if (point_feature_infos[i].neighbor_intersection_cosine <
                    kSmallPlaneIntersectionCosineThreshold &&
                neighbor_distance_ratio < kSmallPlaneDistanceRatio)
            {
                if (point_feature_infos[i - 1].feature_type == FeatureType::kNormal)
                {
                    point_feature_infos[i - 1].feature_type = FeatureType::kPlane;
                }
                if (point_feature_infos[i + 1].feature_type == FeatureType::kNormal)
                {
                    point_feature_infos[i + 1].feature_type = FeatureType::kPlane;
                }
                point_feature_infos[i].feature_type = FeatureType::kPlane;
            }
        }
    }

    std::optional<std::size_t> first_surface_index;
    for (std::size_t j = first_visible_index; j < point_count; ++j)
    {
        if (point_feature_infos[j].feature_type == FeatureType::kPossiblePlane ||
            point_feature_infos[j].feature_type == FeatureType::kPlane)
        {
            if (!first_surface_index)
            {
                first_surface_index = j;
            }

            if (j == *first_surface_index +
                         static_cast<std::size_t>(point_filter_stride_ - 1))
            {
                PointType surface_point{};
                surface_point.x         = scan_line[j].x;
                surface_point.y         = scan_line[j].y;
                surface_point.z         = scan_line[j].z;
                surface_point.intensity = scan_line[j].intensity;
                surface_point.curvature = scan_line[j].curvature;
                surface_cloud_.push_back(surface_point);

                first_surface_index.reset();
            }
        }
        else
        {
            if (point_feature_infos[j].feature_type == FeatureType::kJumpEdge ||
                point_feature_infos[j].feature_type == FeatureType::kPlaneEdge)
            {
                corner_cloud_.push_back(scan_line[j]);
            }
            if (first_surface_index)
            {
                PointType average_surface_point{};
                for (std::size_t k = *first_surface_index; k < j; ++k)
                {
                    average_surface_point.x += scan_line[k].x;
                    average_surface_point.y += scan_line[k].y;
                    average_surface_point.z += scan_line[k].z;
                    average_surface_point.intensity += scan_line[k].intensity;
                    average_surface_point.curvature += scan_line[k].curvature;
                }
                const auto surface_point_count = j - *first_surface_index;
                average_surface_point.x /= surface_point_count;
                average_surface_point.y /= surface_point_count;
                average_surface_point.z /= surface_point_count;
                average_surface_point.intensity /= surface_point_count;
                average_surface_point.curvature /= surface_point_count;
                surface_cloud_.push_back(average_surface_point);
            }
            first_surface_index.reset();
        }
    }
}

PlaneClassification LidarProcessor::classifyPlaneSegment(
    const PointCloudXYZI &scan_line,
    const std::vector<PointFeatureInfo> &point_feature_infos,
    const std::size_t point_index,
    std::size_t &next_point_index,
    Eigen::Vector3d &plane_direction)
{
    double segment_distance_threshold =
        kPlaneDistanceScale * point_feature_infos[point_index].range + kPlaneDistanceOffset;
    segment_distance_threshold *= segment_distance_threshold;

    double segment_length_squared = 0.0;
    std::vector<double> neighbor_distances;
    neighbor_distances.reserve(20);
    double direction_x = 0.0;
    double direction_y = 0.0;
    double direction_z = 0.0;

    for (next_point_index = point_index;
         next_point_index < point_index + kFeatureGroupSize;
         ++next_point_index)
    {
        if (point_feature_infos[next_point_index].range < blind_distance_)
        {
            plane_direction.setZero();
            return PlaneClassification::kContainsBlindPoint;
        }
        neighbor_distances.push_back(point_feature_infos[next_point_index].neighbor_distance);
    }

    for (;;)
    {
        if (point_index >= scan_line.size() || next_point_index >= scan_line.size())
        {
            break;
        }

        if (point_feature_infos[next_point_index].range < blind_distance_)
        {
            plane_direction.setZero();
            return PlaneClassification::kContainsBlindPoint;
        }
        direction_x = scan_line[next_point_index].x - scan_line[point_index].x;
        direction_y = scan_line[next_point_index].y - scan_line[point_index].y;
        direction_z = scan_line[next_point_index].z - scan_line[point_index].z;
        segment_length_squared =
            direction_x * direction_x + direction_y * direction_y + direction_z * direction_z;
        if (segment_length_squared >= segment_distance_threshold)
        {
            break;
        }
        neighbor_distances.push_back(point_feature_infos[next_point_index].neighbor_distance);
        ++next_point_index;
    }

    double maximum_width_squared = 0.0;
    for (std::size_t j = point_index + 1; j < next_point_index; ++j)
    {
        if (j >= scan_line.size() || point_index >= scan_line.size())
        {
            break;
        }
        const double point_offset_x = scan_line[j].x - scan_line[point_index].x;
        const double point_offset_y = scan_line[j].y - scan_line[point_index].y;
        const double point_offset_z = scan_line[j].z - scan_line[point_index].z;

        const double cross_product_x =
            point_offset_y * direction_z - direction_y * point_offset_z;
        const double cross_product_y =
            point_offset_z * direction_x - point_offset_x * direction_z;
        const double cross_product_z =
            point_offset_x * direction_y - direction_x * point_offset_y;

        const double width_squared = cross_product_x * cross_product_x +
                                     cross_product_y * cross_product_y +
                                     cross_product_z * cross_product_z;
        if (width_squared > maximum_width_squared)
        {
            maximum_width_squared = width_squared;
        }
    }

    if (!std::isfinite(maximum_width_squared) || maximum_width_squared < kMinimumVectorNorm ||
        (segment_length_squared * segment_length_squared / maximum_width_squared) <
            kPlaneToLineRatio)
    {
        plane_direction.setZero();
        return PlaneClassification::kNotPlane;
    }

    const auto neighbor_distance_count = neighbor_distances.size();
    std::sort(neighbor_distances.begin(), neighbor_distances.end(), std::greater<double>());

    if (neighbor_distances[neighbor_distances.size() - 2] < 1e-16)
    {
        plane_direction.setZero();
        return PlaneClassification::kNotPlane;
    }

    if (lidar_type_ == LidarType::kAvia)
    {
        const double max_to_middle_distance_ratio =
            neighbor_distances[0] / neighbor_distances[neighbor_distance_count / 2];
        const double middle_to_min_distance_ratio =
            neighbor_distances[neighbor_distance_count / 2] /
            neighbor_distances[neighbor_distance_count - 2];

        if (max_to_middle_distance_ratio >= kAviaMaxToMidDistanceRatio ||
            middle_to_min_distance_ratio >= kAviaMidToMinDistanceRatio)
        {
            plane_direction.setZero();
            return PlaneClassification::kNotPlane;
        }
    }
    else
    {
        const double max_to_min_distance_ratio =
            neighbor_distances[0] / neighbor_distances[neighbor_distance_count - 2];
        if (max_to_min_distance_ratio >= kMaxToMinDistanceRatio)
        {
            plane_direction.setZero();
            return PlaneClassification::kNotPlane;
        }
    }

    plane_direction << direction_x, direction_y, direction_z;
    plane_direction.normalize();
    return PlaneClassification::kPlane;
}

bool LidarProcessor::isEdgeJump(
    const std::vector<PointFeatureInfo> &point_feature_infos,
    const std::size_t point_index,
    const NeighborDirection neighbor_direction)
{
    if (neighbor_direction == NeighborDirection::kPrevious)
    {
        if (point_feature_infos[point_index - 1].range < blind_distance_ ||
            point_feature_infos[point_index - 2].range < blind_distance_)
        {
            return false;
        }
    }
    else if (neighbor_direction == NeighborDirection::kNext)
    {
        if (point_feature_infos[point_index + 1].range < blind_distance_ ||
            point_feature_infos[point_index + 2].range < blind_distance_)
        {
            return false;
        }
    }
    const std::size_t direction_index = neighborIndex(neighbor_direction);
    double first_distance =
        point_feature_infos[point_index + kEdgeJumpFirstDistanceOffsets[direction_index]].neighbor_distance;
    double second_distance =
        point_feature_infos[point_index + kEdgeJumpSecondDistanceOffsets[direction_index]].neighbor_distance;

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
