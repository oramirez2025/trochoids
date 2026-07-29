#ifndef TROCHOIDS_TEST_UTILS_H
#define TROCHOIDS_TEST_UTILS_H

#include <cmath>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

#include "trochoids/trochoid_utils.h"

namespace trochoids_test
{
inline bool ratio_within(double lhs, double rhs, double tolerance)
{
    if (!std::isfinite(lhs) || !std::isfinite(rhs) || rhs == 0.0)
    {
        return false;
    }
    return std::abs(lhs / rhs - 1.0) < tolerance;
}

inline bool debug_artifacts_enabled(const char *env_name)
{
    const char *value = std::getenv(env_name);
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

inline void expect_path_endpoints_match(const std::vector<trochoids::XYZPsiState> &path,
                                        const trochoids::XYZPsiState &start_state,
                                        const trochoids::XYZPsiState &goal_state,
                                        double position_tolerance = 1e-6,
                                        double heading_tolerance = 1e-6)
{
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.front().x, start_state.x, position_tolerance);
    EXPECT_NEAR(path.front().y, start_state.y, position_tolerance);
    EXPECT_NEAR(path.front().z, start_state.z, position_tolerance);
    EXPECT_NEAR(path.front().psi, start_state.psi, heading_tolerance);

    EXPECT_NEAR(path.back().x, goal_state.x, position_tolerance);
    EXPECT_NEAR(path.back().y, goal_state.y, position_tolerance);
    EXPECT_NEAR(path.back().z, goal_state.z, position_tolerance);
    EXPECT_NEAR(path.back().psi, goal_state.psi, heading_tolerance);
}

inline void expect_monotonic_altitude(const std::vector<trochoids::XYZPsiState> &path,
                                      bool climbing)
{
    ASSERT_FALSE(path.empty());
    for (size_t i = 1; i < path.size(); ++i)
    {
        if (climbing)
        {
            EXPECT_LE(path[i - 1].z, path[i].z + 1e-9);
        }
        else
        {
            EXPECT_GE(path[i - 1].z, path[i].z - 1e-9);
        }
    }
}
}  // namespace trochoids_test

#endif  // TROCHOIDS_TEST_UTILS_H
