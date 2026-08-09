#include <gtest/gtest.h>

#include "common/gravity_alignment.hpp"

namespace spark_fast_lio
{
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

}  // namespace
}  // namespace spark_fast_lio
