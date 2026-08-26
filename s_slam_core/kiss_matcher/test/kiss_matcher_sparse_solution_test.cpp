#include <gtest/gtest.h>

#include <KISSMatcher.hpp>

namespace kiss_matcher
{
TEST(KISSMatcher, SparseCorrespondencesDoNotReusePreviousSolution)
{
    KISSMatcher matcher(KISSMatcherConfig(0.3F));

    Eigen::Matrix<double, 3, Eigen::Dynamic> source(3, 6);
    source << 0.0, 1.0, 0.0, 0.0, 1.0, 2.0,
              0.0, 0.0, 1.0, 0.0, 2.0, 1.0,
              0.0, 0.0, 0.0, 1.0, 1.0, 2.0;
    const Eigen::Vector3d translation(1.0, -2.0, 0.5);
    const Eigen::Matrix<double, 3, Eigen::Dynamic> target =
        source.colwise() + translation;

    const RegistrationSolution first_solution = matcher.solve(source, target);
    ASSERT_TRUE(first_solution.valid);

    Eigen::Matrix<double, 3, Eigen::Dynamic> sparse_source(3, 1);
    Eigen::Matrix<double, 3, Eigen::Dynamic> sparse_target(3, 1);
    sparse_source << 0.0, 0.0, 0.0;
    sparse_target << 1.0, 0.0, 0.0;

    const RegistrationSolution sparse_solution = matcher.solve(sparse_source, sparse_target);
    EXPECT_FALSE(sparse_solution.valid);
    EXPECT_TRUE(sparse_solution.translation.isZero());
    EXPECT_TRUE(sparse_solution.rotation.isIdentity());
}
}  // namespace kiss_matcher
