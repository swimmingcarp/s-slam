#pragma once

#include "common/common_lib.h"
#include "common/use-ikfom.hpp"

namespace spark_fast_lio
{

inline state_ikfom gravityAlignedState(const state_ikfom &state,
                                       const M3D &gravity_alignment_rotation)
{
    state_ikfom aligned_state = state;
    aligned_state.pos = gravity_alignment_rotation * state.pos;
    aligned_state.vel = gravity_alignment_rotation * state.vel;
    aligned_state.rot = gravity_alignment_rotation * state.rot;

    const V3D gravity(state.grav[0], state.grav[1], state.grav[2]);
    aligned_state.grav = S2(gravity_alignment_rotation * gravity);
    return aligned_state;
}

}  // namespace spark_fast_lio
