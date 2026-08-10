#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>

#include "common/gravity_alignment.hpp"
#include "spark_fast_lio.h"

namespace spark_fast_lio
{
class SPARKFastLIO2Test : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    static void prepareInitialMap(SPARKFastLIO2 &node)
    {
        constexpr int kPointCount = 6;

        state_ikfom initial_state;
        node.kf_.change_x(initial_state);
        node.latest_state_ = initial_state;
        node.downsampled_point_count_ = kPointCount;
        node.feats_down_body_->resize(kPointCount);
        node.feats_down_world_->resize(kPointCount);
        node.nearest_map_points_.assign(kPointCount, PointVector{});

        for (int i = 0; i < kPointCount; ++i)
        {
            PointType point{};
            point.x = static_cast<float>(i);
            node.feats_down_body_->points[i] = point;
        }

        node.initializeLocalMapIfNeeded();
    }

    static void setAcceptedScan(SPARKFastLIO2 &node)
    {
        constexpr int kPointCount = 6;
        node.downsampled_point_count_ = kPointCount;
        node.feats_down_body_->resize(kPointCount);
        node.feats_down_world_->resize(kPointCount);
        node.nearest_map_points_.assign(kPointCount, PointVector{});

        for (int i = 0; i < kPointCount; ++i)
        {
            PointType point{};
            point.x = 10.0F + static_cast<float>(i);
            node.feats_down_body_->points[i] = point;
        }
    }

    static std::size_t localMapPointCount(SPARKFastLIO2 &node)
    {
        return node.ikd_tree_.size();
    }

    static void setBaseFrame(SPARKFastLIO2 &node)
    {
        node.base_frame_ = "base_link";
    }

    static bool sensorProcessingIsActive(const SPARKFastLIO2 &node)
    {
        return node.sensor_processing_active_ && node.sub_lidar_ && node.sub_imu_ &&
               node.main_loop_timer_;
    }

    static bool sensorProcessingIsInactive(const SPARKFastLIO2 &node)
    {
        return !node.sensor_processing_active_ && !node.sub_lidar_ && !node.sub_imu_ &&
               !node.main_loop_timer_;
    }

    static bool extrinsicsRetryIsScheduled(const SPARKFastLIO2 &node)
    {
        return node.extrinsics_retry_timer_ != nullptr;
    }

    static bool extrinsicsRetryIsStopped(const SPARKFastLIO2 &node)
    {
        return node.extrinsics_retry_timer_ && node.extrinsics_retry_timer_->is_canceled();
    }

    static void setBaseExtrinsics(SPARKFastLIO2 &node)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id  = "lidar";
        transform.transform.rotation.w = 1.0;

        ASSERT_TRUE(node.tf_buffer_->setTransform(transform, "test", true));
        node.retryBaseExtrinsics();
    }

    static bool hasNoPublishedOdometryOrPath(const SPARKFastLIO2 &node)
    {
        return node.path_msg_.poses.empty() && node.odomAftMapped_.header.stamp.sec == 0 &&
               node.odomAftMapped_.header.stamp.nanosec == 0U;
    }

    static void commitAcceptedFrame(SPARKFastLIO2 &node)
    {
        MeasureGroup measures;
        state_ikfom propagated_state = node.latest_state_;
        SPARKFastLIO2::MotionQualityReport quality;
        quality.lidar_time = 1.0;

        node.lidar_end_time_ = quality.lidar_time;
        node.commitOdometryUpdate(measures, propagated_state, quality);
    }
};

namespace
{

TEST(GravityAlignment, PreservesRawStateForMapAndRotatesPublishedState)
{
    state_ikfom raw_state;
    raw_state.pos << 1.0, 2.0, 3.0;
    raw_state.vel << -4.0, 5.0, -6.0;
    raw_state.grav = S2(V3D(0.0, 0.0, -9.81));

    const state_ikfom raw_state_before = raw_state;
    M3D rotation;
    rotation << 1.0, 0.0, 0.0,
                0.0, 0.0, -1.0,
                0.0, 1.0, 0.0;

    const state_ikfom published_state = gravityAlignedState(raw_state, rotation);

    EXPECT_TRUE(raw_state.pos.isApprox(raw_state_before.pos));
    EXPECT_TRUE(raw_state.vel.isApprox(raw_state_before.vel));
    EXPECT_TRUE(raw_state.rot.toRotationMatrix().isApprox(
        raw_state_before.rot.toRotationMatrix()));
    EXPECT_TRUE(V3D(raw_state.grav[0], raw_state.grav[1], raw_state.grav[2]).isApprox(
        V3D(raw_state_before.grav[0], raw_state_before.grav[1], raw_state_before.grav[2])));

    EXPECT_TRUE(published_state.pos.isApprox(rotation * raw_state.pos));
    EXPECT_TRUE(published_state.vel.isApprox(rotation * raw_state.vel));
    EXPECT_TRUE(published_state.rot.toRotationMatrix().isApprox(rotation));
    EXPECT_TRUE(V3D(published_state.grav[0], published_state.grav[1], published_state.grav[2])
                    .isApprox(rotation * V3D(raw_state.grav[0],
                                              raw_state.grav[1],
                                              raw_state.grav[2])));
}

TEST_F(SPARKFastLIO2Test, PreAlignmentAcceptedFrameKeepsMapCurrent)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("gravity_alignment.enable_gravity_alignment", true);
    options.append_parameter_override("common.process_on_callback", true);
    options.append_parameter_override("publish.path_enabled", true);

    auto node = std::make_shared<SPARKFastLIO2>(options);
    setBaseFrame(*node);
    prepareInitialMap(*node);
    const std::size_t initial_map_point_count = localMapPointCount(*node);

    setAcceptedScan(*node);
    commitAcceptedFrame(*node);

    EXPECT_GT(localMapPointCount(*node), initial_map_point_count);
    EXPECT_TRUE(hasNoPublishedOdometryOrPath(*node));
}

TEST_F(SPARKFastLIO2Test, WaitsForBaseExtrinsicsBeforeActivatingSensorProcessing)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("common.base_frame", "base_link");
    options.append_parameter_override("common.lidar_frame", "lidar");
    options.append_parameter_override("common.process_on_callback", false);

    auto node = std::make_shared<SPARKFastLIO2>(options);

    EXPECT_TRUE(sensorProcessingIsInactive(*node));
    ASSERT_TRUE(extrinsicsRetryIsScheduled(*node));

    setBaseExtrinsics(*node);

    EXPECT_TRUE(sensorProcessingIsActive(*node));
    EXPECT_TRUE(extrinsicsRetryIsStopped(*node));
}

}  // namespace
}  // namespace spark_fast_lio
