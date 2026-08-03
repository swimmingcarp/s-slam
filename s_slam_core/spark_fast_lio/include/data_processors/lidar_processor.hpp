#pragma once
#include <cstddef>
#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adapters/kimera_ouster_adapter.hpp"
#include "adapters/ouster_adapter.hpp"
#include "adapters/robosense_fairy_adapter.hpp"
#include "adapters/velodyne_adapter.hpp"
#include "rclcpp/time.hpp"

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

using PointType       = pcl::PointXYZINormal;
using PointCloudXYZI  = pcl::PointCloud<PointType>;

enum LID_TYPE
{
    AVIA      = 1,
    VELO16    = 2,
    OUST64    = 3,
    KMOUST64  = 4,
    ROBOSENSE = 5   // RoboSense (rslidar_sdk XYZIRT, e.g. Fairy)
};  // {1, 2, 3, 4, 5}
enum TIME_UNIT
{
    SEC = 0,
    MS  = 1,
    US  = 2,
    NS  = 3
};
enum Feature
{
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround
{
    Prev,
    Next
};
enum E_jump
{
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};

struct PointFeatureInfo
{
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;
    PointFeatureInfo()
    {
        range     = 0;
        dista     = 0;
        angle[Prev] = 0;
        angle[Next] = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype     = Nor;
        intersect = 2;
    }
};

class LidarProcessor
{
public:
    static constexpr int kMaxScanLines = 128;

    LidarProcessor();

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    bool process(const livox_ros_driver2::msg::CustomMsg &msg, PointCloudXYZI::Ptr &pcl_out);
#endif
    bool process(const sensor_msgs::msg::PointCloud2 &msg, PointCloudXYZI::Ptr &pcl_out);
    void set(bool is_enabled, int lid_type, double bld, int pfilt_num);
    // Configure the PointCloud2 per-point timestamp unit once during startup.
    void setTimestampUnit(TIME_UNIT timestamp_unit);

    bool has_scan_time() const
    {
        return scan_start_time_ > 0.0 && scan_end_time_ >= scan_start_time_;
    }

    double scan_start_time() const
    {
        return scan_start_time_;
    }

    double scan_end_time() const
    {
        return scan_end_time_;
    }

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;
    PointCloudXYZI pl_full, pl_corn, pl_surf, pl_from_pilots;
    PointCloudXYZI pl_buff[kMaxScanLines];       // maximum supported LiDAR scan lines
    // Per-scan-line point feature information, up to kMaxScanLines LiDAR lines.
    std::array<std::vector<PointFeatureInfo>, kMaxScanLines> scan_line_feature_infos_;
    int lidar_type, point_filter_num, N_SCANS, SCAN_RATE;
    double blind, blind_for_human_pilots;
    bool feature_enabled, given_offset_time;

private:
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
    void give_feature(PointCloudXYZI &pl,
                      std::vector<PointFeatureInfo> &point_feature_infos);
    int plane_judge(const PointCloudXYZI &pl,
                    std::vector<PointFeatureInfo> &point_feature_infos,
                    uint i,
                    uint &i_nex,
                    Eigen::Vector3d &curr_direct);
    bool edge_jump_judge(const PointCloudXYZI &pl,
                         std::vector<PointFeatureInfo> &point_feature_infos,
                         uint i,
                         Surround nor_dir);

    int group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double scan_start_time_, scan_end_time_;
    float time_unit_scale_;
    sensor_adapter::OusterAdapter ouster_adapter_;
    sensor_adapter::KimeraOusterAdapter kimera_ouster_adapter_;
    sensor_adapter::VelodyneAdapter velodyne_adapter_;
    sensor_adapter::RoboSenseFairyAdapter robosense_fairy_adapter_;
};
