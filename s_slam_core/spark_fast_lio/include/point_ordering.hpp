#pragma once

#include "spark_fast_lio.h"

namespace spark_fast_lio::point_ordering
{
#ifdef SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER
inline bool lessXYZICurvature(const PointType &lhs, const PointType &rhs)
{
    if (lhs.x != rhs.x)
    {
        return lhs.x < rhs.x;
    }
    if (lhs.y != rhs.y)
    {
        return lhs.y < rhs.y;
    }
    if (lhs.z != rhs.z)
    {
        return lhs.z < rhs.z;
    }
    if (lhs.intensity != rhs.intensity)
    {
        return lhs.intensity < rhs.intensity;
    }
    return lhs.curvature < rhs.curvature;
}
#endif
}  // namespace spark_fast_lio::point_ordering
