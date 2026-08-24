#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <limits>
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

    static rclcpp::ReliabilityPolicy lidarSubscriptionReliability(const SPARKFastLIO2 &node)
    {
        return node.sub_lidar_->get_actual_qos().reliability();
    }

    static rclcpp::ReliabilityPolicy imuSubscriptionReliability(const SPARKFastLIO2 &node)
    {
        return node.sub_imu_->get_actual_qos().reliability();
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

    static void feedImu(SPARKFastLIO2 &node, const double timestamp)
    {
        auto imu = std::make_shared<sensor_msgs::msg::Imu>();
        const int64_t nanoseconds = static_cast<int64_t>(timestamp * 1.0e9);
        imu->header.stamp.sec     = static_cast<int32_t>(nanoseconds / 1000000000LL);
        imu->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        node.imuCallback(imu);
    }

    static PointCloudXYZI::Ptr queueLidar(SPARKFastLIO2 &node, const double timestamp)
    {
        auto cloud = std::make_shared<PointCloudXYZI>();
        node.lidar_buffer_.push_back({cloud, timestamp, timestamp + 0.1});
        return cloud;
    }

    static std::size_t lidarBufferSize(const SPARKFastLIO2 &node)
    {
        return node.lidar_buffer_.size();
    }

    static double oldestLidarTimestamp(const SPARKFastLIO2 &node)
    {
        return node.lidar_buffer_.front().begin_time;
    }

    static double newestLidarTimestamp(const SPARKFastLIO2 &node)
    {
        return node.lidar_buffer_.back().begin_time;
    }

    static std::size_t imuBufferSize(const SPARKFastLIO2 &node)
    {
        return node.imu_buffer_.size();
    }

    static double oldestImuTimestamp(const SPARKFastLIO2 &node)
    {
        return rclcpp::Time(node.imu_buffer_.front()->header.stamp).seconds();
    }

    static double newestImuTimestamp(const SPARKFastLIO2 &node)
    {
        return rclcpp::Time(node.imu_buffer_.back()->header.stamp).seconds();
    }

    static bool syncPackages(SPARKFastLIO2 &node, MeasureGroup &measurements)
    {
        return node.syncPackages(measurements, false);
    }

    static bool hasInFlightLidar(const SPARKFastLIO2 &node)
    {
        return node.lidar_pushed_;
    }

    static int imuGapLidarSkipCount(const SPARKFastLIO2 &node)
    {
        return node.imu_gap_lidar_skip_count_;
    }

    static ImuProcessor::Snapshot imuProcessorSnapshot(const SPARKFastLIO2 &node)
    {
        return node.imu_processor_->getSnapshot();
    }

    static void integratePredictedImu(SPARKFastLIO2 &node,
                                      const double timestamp,
                                      const double angular_velocity_z)
    {
        sensor_msgs::msg::Imu imu;
        const int64_t nanoseconds = static_cast<int64_t>(timestamp * 1.0e9);
        imu.header.stamp.sec     = static_cast<int32_t>(nanoseconds / 1000000000LL);
        imu.header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        imu.linear_acceleration.z = G_m_s2;
        imu.angular_velocity.z    = angular_velocity_z;
        node.integrateIMU(node.kf_, imu);
    }

    static std::size_t predictedImuQueueSize(const SPARKFastLIO2 &node)
    {
        return node.imu_integration_queue_.size();
    }

    static double predictedImuQueueFrontTime(const SPARKFastLIO2 &node)
    {
        return rclcpp::Time(node.imu_integration_queue_.front().header.stamp).seconds();
    }

    static PoseStruct lidarPose(const SPARKFastLIO2 &node, const state_ikfom &state)
    {
        return node.transformPoseToLidarFrame(state);
    }

    static PoseStruct basePose(const SPARKFastLIO2 &node, const state_ikfom &state)
    {
        return node.transformPoseToBaseFrame(state);
    }

    static void setLidarPoseInBase(SPARKFastLIO2 &node,
                                   const M3D &rotation_base_lidar,
                                   const V3D &translation_base_lidar)
    {
        node.lidar_rotation_in_base_    = rotation_base_lidar;
        node.lidar_translation_in_base_ = translation_base_lidar;
    }

    static PointType lidarPointInWorld(SPARKFastLIO2 &node,
                                       const PointType &point_in_lidar,
                                       const state_ikfom &state)
    {
        PointType point_in_world{};
        node.pclPointBodyToWorld(&point_in_lidar, &point_in_world, state);
        return point_in_world;
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

    static void initializeWithZeroAcceleration(SPARKFastLIO2 &node)
    {
        MeasureGroup measures;
        measures.lidar_beg_time = 0.0;
        measures.lidar_end_time = 2.1;
        for (int sample_index = 0; sample_index <= kMinImuInitSamples; ++sample_index)
        {
            auto imu = std::make_shared<sensor_msgs::msg::Imu>();
            const int64_t nanoseconds = static_cast<int64_t>(sample_index) * 10000000LL;
            imu->header.stamp.sec     = static_cast<int32_t>(nanoseconds / 1000000000LL);
            imu->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
            measures.imu.push_back(imu);
        }

        auto undistorted_cloud = std::make_shared<PointCloudXYZI>();
        node.imu_processor_->process(measures, node.kf_, undistorted_cloud);
    }

    static void initializeWithStationaryGravity(SPARKFastLIO2 &node)
    {
        MeasureGroup measures;
        measures.lidar_beg_time = 0.0;
        measures.lidar_end_time = 2.1;
        for (int sample_index = 0; sample_index <= kMinImuInitSamples; ++sample_index)
        {
            auto imu = std::make_shared<sensor_msgs::msg::Imu>();
            const int64_t nanoseconds = static_cast<int64_t>(sample_index) * 10000000LL;
            imu->header.stamp.sec     = static_cast<int32_t>(nanoseconds / 1000000000LL);
            imu->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
            imu->linear_acceleration.z = G_m_s2;
            measures.imu.push_back(imu);
        }

        auto undistorted_cloud = std::make_shared<PointCloudXYZI>();
        node.imu_processor_->process(measures, node.kf_, undistorted_cloud);
        ASSERT_TRUE(node.imu_processor_->isInitialized());
    }

    static bool filterStateIsFinite(const SPARKFastLIO2 &node)
    {
        const state_ikfom state = node.kf_.get_x();
        const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
        return state.pos.allFinite() && state.vel.allFinite() && state.bg.allFinite() &&
               state.ba.allFinite() && gravity.allFinite() &&
               state.rot.toRotationMatrix().allFinite();
    }

    static state_ikfom filterState(const SPARKFastLIO2 &node)
    {
        return node.kf_.get_x();
    }

    static bool imuProcessorIsInitialized(const SPARKFastLIO2 &node)
    {
        return node.imu_processor_->isInitialized();
    }

    static PointCloudXYZI::Ptr fullPoints(const SPARKFastLIO2 &node)
    {
        return node.full_points_;
    }

    static void undistortQueuedCloud(SPARKFastLIO2 &node, MeasureGroup &measures)
    {
        node.imu_processor_->process(measures, node.kf_, node.full_points_);
    }

    static state_ikfom warmResetFromPropagatedState(SPARKFastLIO2 &node,
                                                     const state_ikfom &stale_state,
                                                     const state_ikfom &propagated_state)
    {
        state_ikfom stale_state_copy      = stale_state;
        state_ikfom propagated_state_copy = propagated_state;
        node.kf_.change_x(stale_state_copy);
        node.kf_for_preintegration_ = node.kf_;
        node.kf_for_preintegration_->change_x(propagated_state_copy);
        node.resetEstimatorState("test warm recovery", SPARKFastLIO2::ResetMode::kWarmRecovery);

        MeasureGroup measures;
        measures.lidar_beg_time = 2.1;
        measures.lidar_end_time = 2.2;
        measures.lidar          = std::make_shared<PointCloudXYZI>();
        for (const double timestamp : {2.1, 2.2})
        {
            auto imu = std::make_shared<sensor_msgs::msg::Imu>();
            const int64_t nanoseconds = static_cast<int64_t>(timestamp * 1.0e9);
            imu->header.stamp.sec     = static_cast<int32_t>(nanoseconds / 1000000000LL);
            imu->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
            imu->linear_acceleration.z = G_m_s2;
            measures.imu.push_back(imu);
        }

        node.imu_processor_->process(measures, node.kf_, node.full_points_);
        return node.kf_.get_x();
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

TEST(SafetyChecks, EmptyImuGroupHasZeroMeanAcceleration)
{
    MeasureGroup measures;
    EXPECT_TRUE(measures.getMeanAcc().isZero());
}

TEST(SafetyChecks, RejectsDegeneratePlaneNormal)
{
    PointVector points(NUM_MATCH_POINTS);
    Eigen::Matrix<float, 4, 1> plane;
    EXPECT_FALSE(esti_plane(plane, points, 0.1F));
}

TEST_F(SPARKFastLIO2Test, LidarPoseAndRegisteredPointUseSameMapTransform)
{
    auto node = std::make_shared<SPARKFastLIO2>();

    state_ikfom state;
    M3D rotation_world_imu;
    rotation_world_imu << 0.0, -1.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0;
    M3D rotation_imu_lidar;
    rotation_imu_lidar << 1.0, 0.0, 0.0,
                           0.0, 0.0, -1.0,
                           0.0, 1.0, 0.0;
    state.rot          = SO3(rotation_world_imu);
    state.offset_R_L_I = SO3(rotation_imu_lidar);
    state.pos          = V3D(3.0, -2.0, 4.0);
    state.offset_T_L_I = V3D(0.5, -0.25, 1.0);

    const PoseStruct lidar_pose = lidarPose(*node, state);
    const V3D expected_position = state.rot * state.offset_T_L_I + state.pos;
    const M3D expected_rotation =
        (state.rot * state.offset_R_L_I).toRotationMatrix();

    EXPECT_TRUE(lidar_pose.position_.isApprox(expected_position));
    EXPECT_TRUE(lidar_pose.orientation_.toRotationMatrix().isApprox(expected_rotation));

    PointType point_in_lidar{};
    point_in_lidar.x = 1.0F;
    point_in_lidar.y = -3.0F;
    point_in_lidar.z = 2.0F;
    const PointType point_in_world = lidarPointInWorld(*node, point_in_lidar, state);
    const V3D expected_point =
        expected_rotation * V3D(point_in_lidar.x, point_in_lidar.y, point_in_lidar.z) +
        expected_position;

    EXPECT_TRUE(V3D(point_in_world.x, point_in_world.y, point_in_world.z).isApprox(expected_point));

    M3D rotation_base_lidar;
    rotation_base_lidar << 0.0, 0.0, 1.0,
                           0.0, 1.0, 0.0,
                           -1.0, 0.0, 0.0;
    const V3D translation_base_lidar(-0.2, 0.4, 0.1);
    setLidarPoseInBase(*node, rotation_base_lidar, translation_base_lidar);

    const PoseStruct base_pose = basePose(*node, state);
    const M3D rotation_imu_base = rotation_imu_lidar * rotation_base_lidar.inverse();
    const V3D translation_imu_base =
        state.offset_T_L_I - rotation_imu_base * translation_base_lidar;
    const M3D expected_base_rotation = rotation_world_imu * rotation_imu_base;
    const V3D expected_base_position =
        rotation_world_imu * translation_imu_base + state.pos;

    EXPECT_TRUE(base_pose.position_.isApprox(expected_base_position));
    EXPECT_TRUE(base_pose.orientation_.toRotationMatrix().isApprox(expected_base_rotation));
}

TEST_F(SPARKFastLIO2Test, GravityAlignedLidarPoseAndRegisteredPointUseSamePublicMapFrame)
{
    auto node = std::make_shared<SPARKFastLIO2>();

    state_ikfom raw_state;
    raw_state.pos          = V3D(2.0, -3.0, 5.0);
    raw_state.offset_T_L_I = V3D(0.4, -0.2, 0.6);

    M3D rotation_imu_lidar;
    rotation_imu_lidar << 0.0, -1.0, 0.0,
                            1.0, 0.0, 0.0,
                            0.0, 0.0, 1.0;
    raw_state.offset_R_L_I = SO3(rotation_imu_lidar);

    M3D gravity_alignment_rotation;
    gravity_alignment_rotation << 1.0, 0.0, 0.0,
                                  0.0, 0.0, -1.0,
                                  0.0, 1.0, 0.0;
    const state_ikfom output_state =
        gravityAlignedState(raw_state, gravity_alignment_rotation);

    PointType point_in_lidar{};
    point_in_lidar.x = 1.0F;
    point_in_lidar.y = 2.0F;
    point_in_lidar.z = -3.0F;

    const PoseStruct lidar_pose = lidarPose(*node, output_state);
    const PointType registered_point =
        lidarPointInWorld(*node, point_in_lidar, output_state);
    const V3D expected_point =
        lidar_pose.orientation_.toRotationMatrix() *
            V3D(point_in_lidar.x, point_in_lidar.y, point_in_lidar.z) +
        lidar_pose.position_;

    EXPECT_TRUE(V3D(registered_point.x, registered_point.y, registered_point.z)
                    .isApprox(expected_point, 1.0e-5));
}

TEST_F(SPARKFastLIO2Test, RejectsZeroAccelerationDuringImuInitialization)
{
    auto node = std::make_shared<SPARKFastLIO2>();

    initializeWithZeroAcceleration(*node);

    EXPECT_FALSE(imuProcessorIsInitialized(*node));
    EXPECT_TRUE(filterStateIsFinite(*node));
}

TEST_F(SPARKFastLIO2Test, ImuUndistortionTakesQueuedCloudOwnership)
{
    auto node = std::make_shared<SPARKFastLIO2>();
    initializeWithStationaryGravity(*node);

    MeasureGroup measures;
    measures.lidar_beg_time = 2.1;
    measures.lidar_end_time = 2.2;
    measures.lidar = std::make_shared<PointCloudXYZI>();
    measures.lidar->resize(1);
    measures.lidar->points[0].x = 1.0F;
    const PointCloudXYZI::Ptr queued_cloud = measures.lidar;
    const PointCloudXYZI::Ptr previous_working_cloud = fullPoints(*node);

    for (const double timestamp : {2.1, 2.2})
    {
        auto imu = std::make_shared<sensor_msgs::msg::Imu>();
        const int64_t nanoseconds = static_cast<int64_t>(timestamp * 1.0e9);
        imu->header.stamp.sec     = static_cast<int32_t>(nanoseconds / 1000000000LL);
        imu->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        imu->linear_acceleration.z = G_m_s2;
        measures.imu.push_back(imu);
    }

    undistortQueuedCloud(*node, measures);

    EXPECT_EQ(fullPoints(*node), queued_cloud);
    EXPECT_EQ(measures.lidar, previous_working_cloud);
    ASSERT_EQ(fullPoints(*node)->size(), 1U);
    EXPECT_FLOAT_EQ(fullPoints(*node)->points[0].x, 1.0F);
}

TEST_F(SPARKFastLIO2Test, WarmResetUsesCurrentImuPropagationState)
{
    auto node = std::make_shared<SPARKFastLIO2>();
    initializeWithStationaryGravity(*node);

    state_ikfom stale_state = filterState(*node);
    stale_state.vel         = V3D(-3.0, 4.0, -2.0);
    stale_state.bg          = V3D(0.01, -0.02, 0.03);
    stale_state.ba          = V3D(-0.04, 0.05, -0.06);

    state_ikfom propagated_state = stale_state;
    M3D propagated_rotation;
    propagated_rotation << 1.0, 0.0, 0.0,
                           0.0, 0.0, -1.0,
                           0.0, 1.0, 0.0;
    propagated_state.rot = SO3(propagated_rotation);
    propagated_state.vel = V3D(1.0, 2.0, 3.0);
    propagated_state.bg  = V3D(-0.11, 0.12, -0.13);
    propagated_state.ba  = V3D(0.14, -0.15, 0.16);

    const state_ikfom reset_state =
        warmResetFromPropagatedState(*node, stale_state, propagated_state);
    const V3D expected_velocity = propagated_state.rot.conjugate() * propagated_state.vel;
    const V3D expected_gravity =
        propagated_state.rot.conjugate() *
        V3D(propagated_state.grav[0], propagated_state.grav[1], propagated_state.grav[2]);
    const V3D reset_gravity(
        reset_state.grav[0], reset_state.grav[1], reset_state.grav[2]);

    EXPECT_TRUE(reset_state.vel.isApprox(expected_velocity));
    EXPECT_FALSE(reset_state.vel.isApprox(stale_state.vel));
    EXPECT_TRUE(reset_gravity.isApprox(expected_gravity));
    EXPECT_TRUE(reset_state.bg.isApprox(propagated_state.bg));
    EXPECT_TRUE(reset_state.ba.isApprox(propagated_state.ba));
}

TEST_F(SPARKFastLIO2Test, RejectsNonPositiveMapFilterSize)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("filter_size_map", 0.0);

    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(options), std::invalid_argument);
}

TEST_F(SPARKFastLIO2Test, RejectsInvalidImuLidarExtrinsics)
{
    rclcpp::NodeOptions short_translation;
    short_translation.append_parameter_override(
        "mapping.extrinsic_T", std::vector<double>{0.0, 0.0});
    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(short_translation), std::invalid_argument);

    rclcpp::NodeOptions short_rotation;
    short_rotation.append_parameter_override(
        "mapping.extrinsic_R", std::vector<double>{1.0, 0.0, 0.0});
    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(short_rotation), std::invalid_argument);

    rclcpp::NodeOptions nonfinite_translation;
    nonfinite_translation.append_parameter_override(
        "mapping.extrinsic_T",
        std::vector<double>{0.0, std::numeric_limits<double>::quiet_NaN(), 0.0});
    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(nonfinite_translation), std::invalid_argument);

    rclcpp::NodeOptions nonfinite_rotation;
    nonfinite_rotation.append_parameter_override(
        "mapping.extrinsic_R",
        std::vector<double>{1.0,
                            0.0,
                            0.0,
                            0.0,
                            std::numeric_limits<double>::quiet_NaN(),
                            0.0,
                            0.0,
                            0.0,
                            1.0});
    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(nonfinite_rotation), std::invalid_argument);

    rclcpp::NodeOptions reflected_rotation;
    reflected_rotation.append_parameter_override(
        "mapping.extrinsic_R", std::vector<double>{-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(reflected_rotation), std::invalid_argument);

    rclcpp::NodeOptions nonorthogonal_rotation;
    nonorthogonal_rotation.append_parameter_override(
        "mapping.extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, 1.0});
    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(nonorthogonal_rotation), std::invalid_argument);
}

TEST_F(SPARKFastLIO2Test, AcceptsRoundedImuLidarRotation)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override(
        "mapping.extrinsic_T", std::vector<double>{0.0420405650, -0.0738194508, -0.159149569});
    options.append_parameter_override(
        "mapping.extrinsic_R",
        std::vector<double>{-0.0009486970,
                            0.9999950000,
                            0.0029021900,
                            -0.0001855830,
                            0.0029020100,
                            -0.9999960000,
                            -1.0000000000,
                            -0.0009492320,
                            0.0001828290});

    EXPECT_NO_THROW(std::make_shared<SPARKFastLIO2>(options));
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

TEST_F(SPARKFastLIO2Test, ConfiguresInputSubscriptionReliability)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("common.lidar_qos_reliability", "best_effort");
    options.append_parameter_override("common.imu_qos_reliability", "best_effort");

    auto node = std::make_shared<SPARKFastLIO2>(options);

    EXPECT_EQ(lidarSubscriptionReliability(*node), rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(imuSubscriptionReliability(*node), rclcpp::ReliabilityPolicy::BestEffort);
}

TEST_F(SPARKFastLIO2Test, RejectsUnknownInputSubscriptionReliability)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("common.lidar_qos_reliability", "lossless");

    EXPECT_THROW(std::make_shared<SPARKFastLIO2>(options), std::invalid_argument);
}

TEST_F(SPARKFastLIO2Test, InputBuffersRetainNewestDataAtCapacity)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("common.lidar_buffer_capacity", 2);
    options.append_parameter_override("common.imu_buffer_capacity", 2);

    auto node = std::make_shared<SPARKFastLIO2>(options);
    feedImu(*node, 1.0);
    feedImu(*node, 1.01);
    feedImu(*node, 1.02);
    queueLidar(*node, 2.0);
    queueLidar(*node, 2.1);
    queueLidar(*node, 2.2);

    ASSERT_EQ(imuBufferSize(*node), 2U);
    EXPECT_DOUBLE_EQ(oldestImuTimestamp(*node), 1.01);
    EXPECT_DOUBLE_EQ(newestImuTimestamp(*node), 1.02);
    ASSERT_EQ(lidarBufferSize(*node), 2U);
    EXPECT_DOUBLE_EQ(oldestLidarTimestamp(*node), 2.1);
    EXPECT_DOUBLE_EQ(newestLidarTimestamp(*node), 2.2);
}

TEST_F(SPARKFastLIO2Test, InFlightLidarFrameSurvivesBufferWrapping)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("common.lidar_buffer_capacity", 2);
    options.append_parameter_override("common.imu_buffer_capacity", 10);

    auto node = std::make_shared<SPARKFastLIO2>(options);
    const PointCloudXYZI::Ptr first_lidar = queueLidar(*node, 1.0);
    feedImu(*node, 1.0);

    MeasureGroup measurements;
    EXPECT_FALSE(syncPackages(*node, measurements));
    EXPECT_TRUE(hasInFlightLidar(*node));
    EXPECT_EQ(measurements.lidar, first_lidar);

    queueLidar(*node, 1.2);
    queueLidar(*node, 1.3);
    queueLidar(*node, 1.4);
    feedImu(*node, 1.1);

    EXPECT_TRUE(syncPackages(*node, measurements));

    EXPECT_EQ(measurements.lidar, first_lidar);
    EXPECT_FALSE(hasInFlightLidar(*node));
    ASSERT_EQ(lidarBufferSize(*node), 2U);
    EXPECT_DOUBLE_EQ(oldestLidarTimestamp(*node), 1.3);
    EXPECT_DOUBLE_EQ(newestLidarTimestamp(*node), 1.4);
}

TEST_F(SPARKFastLIO2Test, IncludesImuAtLidarEndpoint)
{
    auto node = std::make_shared<SPARKFastLIO2>();
    const PointCloudXYZI::Ptr lidar = queueLidar(*node, 1.0);
    feedImu(*node, 1.1);

    MeasureGroup measurements;
    ASSERT_TRUE(syncPackages(*node, measurements));
    EXPECT_EQ(measurements.lidar, lidar);
    ASSERT_EQ(measurements.imu.size(), 1U);
    EXPECT_DOUBLE_EQ(rclcpp::Time(measurements.imu.back()->header.stamp).seconds(), 1.1);
}

TEST_F(SPARKFastLIO2Test, RejectsLongImuGapAndAdvancesTemporalCursor)
{
    auto node = std::make_shared<SPARKFastLIO2>();
    queueLidar(*node, 1.0);
    feedImu(*node, 1.0);
    feedImu(*node, 1.1);

    MeasureGroup measurements;
    ASSERT_TRUE(syncPackages(*node, measurements));

    queueLidar(*node, 1.5);
    feedImu(*node, 1.6);
    EXPECT_FALSE(syncPackages(*node, measurements));
    EXPECT_EQ(imuGapLidarSkipCount(*node), 1);
    EXPECT_FALSE(hasInFlightLidar(*node));

    const ImuProcessor::Snapshot skipped_snapshot = imuProcessorSnapshot(*node);
    ASSERT_NE(skipped_snapshot.last_imu, nullptr);
    EXPECT_DOUBLE_EQ(rclcpp::Time(skipped_snapshot.last_imu->header.stamp).seconds(), 1.6);
    EXPECT_DOUBLE_EQ(skipped_snapshot.last_lidar_end_time, 1.6);

    queueLidar(*node, 1.6);
    feedImu(*node, 1.605);
    feedImu(*node, 1.61);
    feedImu(*node, 1.7);
    feedImu(*node, 1.701);
    EXPECT_TRUE(syncPackages(*node, measurements));
}

TEST_F(SPARKFastLIO2Test, SkipsPredictedOdometryAcrossLongImuGap)
{
    auto node = std::make_shared<SPARKFastLIO2>();
    initializeWithStationaryGravity(*node);
    const state_ikfom state_before = filterState(*node);

    integratePredictedImu(*node, 3.0, 1.0);
    integratePredictedImu(*node, 3.5, 1.0);

    const state_ikfom state_after = filterState(*node);
    EXPECT_TRUE(state_after.rot.toRotationMatrix().isApprox(
        state_before.rot.toRotationMatrix()));
    ASSERT_EQ(predictedImuQueueSize(*node), 1U);
    EXPECT_DOUBLE_EQ(predictedImuQueueFrontTime(*node), 3.5);
}

TEST_F(SPARKFastLIO2Test, TimestampLoopbackClearsCircularBuffers)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("common.lidar_buffer_capacity", 2);
    options.append_parameter_override("common.imu_buffer_capacity", 3);

    auto node = std::make_shared<SPARKFastLIO2>(options);
    queueLidar(*node, 2.0);
    feedImu(*node, 1.0);
    feedImu(*node, 1.1);
    feedImu(*node, 0.5);

    EXPECT_EQ(lidarBufferSize(*node), 0U);
    ASSERT_EQ(imuBufferSize(*node), 1U);
    EXPECT_DOUBLE_EQ(oldestImuTimestamp(*node), 0.5);
}

}  // namespace
}  // namespace spark_fast_lio
