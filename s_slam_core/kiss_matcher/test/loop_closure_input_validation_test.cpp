#include <gtest/gtest.h>

#include "slam/loop_closure.h"

namespace kiss_matcher
{
TEST(LoopClosure, EmptyCloudsAreRejectedBeforeICP)
{
    LoopClosureConfig config;
    LoopClosure loop_closure(config, rclcpp::get_logger("loop_closure_input_validation_test"));
    pcl::PointCloud<PointType> empty_cloud;

    const RegOutput result = loop_closure.icpAlignment(empty_cloud, empty_cloud);

    EXPECT_FALSE(result.is_valid_);
    EXPECT_FALSE(result.is_converged_);
    EXPECT_DOUBLE_EQ(result.overlapness_, 0.0);
    EXPECT_TRUE(result.pose_.allFinite());
}
}  // namespace kiss_matcher
