#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adapters/kimera_ouster_adapter.hpp"
#include "adapters/ouster_adapter.hpp"
#include "adapters/robosense_fairy_adapter.hpp"
#include "adapters/seyond_adapter.hpp"
#include "adapters/velodyne_adapter.hpp"

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

using PointType      = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

enum class LidarType : int
{
    kAvia           = 1,
    kVelo16         = 2,
    kOuster64       = 3,
    kKimeraOuster64 = 4,
    kRoboSense      = 5,
    kSeyond         = 6,
};

enum class TimestampUnit : int
{
    kSeconds      = 0,
    kMilliseconds = 1,
    kMicroseconds = 2,
    kNanoseconds  = 3,
};
enum class FeatureType
{
    kNormal,
    kPossiblePlane,
    kPlane,
    kJumpEdge,
    kPlaneEdge,
    kWire,
    kZeroPoint,
};

enum class PlaneClassification
{
    kNotPlane,
    kPlane,
    kContainsBlindPoint,
};

enum class NeighborDirection : std::size_t
{
    kPrevious = 0,
    kNext     = 1,
};

enum class NeighborState
{
    kNormal,
    kZeroDistance,
    kOppositeDirection,
    kInfiniteRange,
    kBlindRange,
};

struct PointFeatureInfo
{
    double range = 0.0;
    double neighbor_distance = 0.0;
    std::array<double, 2> neighbor_angle{};
    double neighbor_intersection_cosine = 2.0;
    std::array<NeighborState, 2> neighbor_state{
        NeighborState::kNormal, NeighborState::kNormal};
    FeatureType feature_type = FeatureType::kNormal;
};

class LidarProcessor
{
public:
    struct Config
    {
        LidarType lidar_type = LidarType::kAvia;
        TimestampUnit timestamp_unit = TimestampUnit::kMilliseconds;
        int scan_line_count = 6;
        int scan_rate_hz = 10;
        int point_filter_stride = 1;
        double blind_distance = 0.01;
        double pilot_zone_blind_distance = 1.5;
        bool feature_extraction_enabled = false;
    };

    static constexpr int kMaxScanLines = 128;

    explicit LidarProcessor(const Config &config);

    static LidarType GetLidarType(int value);
    static TimestampUnit GetTimestampUnit(int value);

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    bool process(const livox_ros_driver2::msg::CustomMsg &msg, PointCloudXYZI::Ptr &pcl_out);
#endif
    bool process(const sensor_msgs::msg::PointCloud2 &msg, PointCloudXYZI::Ptr &pcl_out);

    bool hasScanTime() const
    {
        return scan_start_time_ > 0.0 && scan_end_time_ >= scan_start_time_;
    }

    double scanStartTime() const
    {
        return scan_start_time_;
    }

    double scanEndTime() const
    {
        return scan_end_time_;
    }

    int scanRateHz() const
    {
        return scan_rate_hz_;
    }

    int pointFilterStride() const
    {
        return point_filter_stride_;
    }

private:
    static void validateConfig(const Config &config);
    static float timestampUnitScale(TimestampUnit timestamp_unit);

    enum class NeighborDistance
    {
        kEuclidean,
        kSquared,
    };

    enum class PilotZoneOrientation
    {
        kOuster,
        kVelodyne,
    };

    void resetFrameClouds();
    void prepareFeatureScanLines(std::size_t point_count);
    void extractFeaturesFromScanLines(NeighborDistance neighbor_distance,
                                      std::size_t minimum_scan_line_points);
    void populatePointFeatureInfo(const PointCloudXYZI &scan_line,
                                  std::vector<PointFeatureInfo> &point_feature_infos,
                                  NeighborDistance neighbor_distance) const;
    bool isFromPilotZone(float point_x,
                         float point_y,
                         PilotZoneOrientation orientation) const;
#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    void handleAviaPointCloud(const livox_ros_driver2::msg::CustomMsg &msg);
#endif
    void handleOusterPointCloud(const sensor_msgs::msg::PointCloud2 &msg);
    void handleKimeraOusterPointCloud(const sensor_msgs::msg::PointCloud2 &msg);
    void handleVelodynePointCloud(const sensor_msgs::msg::PointCloud2 &msg);
    void handleRoboSensePointCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                   PointCloudXYZI &output);
    void handleSeyondPointCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                PointCloudXYZI &output);
    void extractFeaturesFromScanLine(PointCloudXYZI &scan_line,
                                     std::vector<PointFeatureInfo> &point_feature_infos);
    PlaneClassification classifyPlaneSegment(
        const PointCloudXYZI &scan_line,
        const std::vector<PointFeatureInfo> &point_feature_infos,
        std::size_t point_index,
        std::size_t &next_point_index,
        Eigen::Vector3d &direction);
    bool isEdgeJump(const std::vector<PointFeatureInfo> &point_feature_infos,
                    std::size_t point_index,
                    NeighborDirection neighbor_direction);

    const LidarType lidar_type_;
    const int scan_line_count_;
    const int scan_rate_hz_;
    const int point_filter_stride_;
    const double blind_distance_;
    const double pilot_zone_blind_distance_;
    const bool feature_extraction_enabled_;
    const float time_unit_scale_;

    PointCloudXYZI full_cloud_;
    PointCloudXYZI corner_cloud_;
    PointCloudXYZI surface_cloud_;
    PointCloudXYZI pilot_zone_cloud_;
    std::array<PointCloudXYZI, kMaxScanLines> scan_line_clouds_;
    std::array<std::vector<PointFeatureInfo>, kMaxScanLines> scan_line_feature_infos_;

    bool has_point_time_offset_ = false;
    double scan_start_time_;
    double scan_end_time_;
    sensor_adapter::OusterAdapter ouster_adapter_;
    sensor_adapter::KimeraOusterAdapter kimera_ouster_adapter_;
    sensor_adapter::VelodyneAdapter velodyne_adapter_;
    sensor_adapter::RoboSenseFairyAdapter robosense_fairy_adapter_;
    sensor_adapter::SeyondAdapter seyond_adapter_;
};
