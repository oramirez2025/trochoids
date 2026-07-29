/*********************************************************************
    The Clear BSD License

    Copyright (c) 2026, AirLab
    All rights reserved.
*********************************************************************/

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>

#include "trochoids/trochoid_utils.h"
#include "test_utils.h"

namespace
{
std::string get_csv_dir()
{
    const char *env_dir = std::getenv("TROCHOIDS_3D_CSV_DIR");
    if (env_dir != nullptr && env_dir[0] != '\0')
    {
        return std::filesystem::absolute(std::filesystem::path(env_dir)).string();
    }
    // Default to repo_root/csv_files/3d based on this source file path.
    const std::filesystem::path src_file(__FILE__);
    const std::filesystem::path repo_root = src_file.parent_path().parent_path();
    return (repo_root / "csv_files" / "3d").string();
}

void write_path_csv(const std::string &filename, const std::vector<trochoids::XYZPsiState> &path)
{
    if (!trochoids_test::debug_artifacts_enabled("TROCHOIDS_WRITE_3D_CSV"))
    {
        return;
    }

    const std::string csv_dir = get_csv_dir();
    std::filesystem::create_directories(csv_dir);
    const std::filesystem::path out_path = std::filesystem::path(csv_dir) / filename;
    std::ofstream out(out_path);
    if (!out.is_open())
    {
        std::cerr << "[3d-test] Failed to open CSV for writing: " << out_path << std::endl;
        return;
    }
    for (const auto &state : path)
    {
        out << state.x << "," << state.y << "," << state.z << "," << state.psi << "\n";
    }
    out.close();
    std::cerr << "[3d-test] Wrote CSV: " << out_path << std::endl;
}

double wrap_pi(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double path_time_estimate(const std::vector<trochoids::XYZPsiState> &path,
                          const double wind[3],
                          double v)
{
    if (path.size() < 2)
    {
        return 0.0;
    }
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const auto &a = path[i];
        const auto &b = path[i + 1];
        const double ds = std::hypot(b.x - a.x, b.y - a.y);
        if (ds < 1e-9)
        {
            continue;
        }
        const double psi_avg = wrap_pi(0.5 * (a.psi + b.psi));
        const double gx = v * std::cos(psi_avg) + wind[0];
        const double gy = v * std::sin(psi_avg) + wind[1];
        const double gs = std::max(std::hypot(gx, gy), 1e-9);
        total += ds / gs;
    }
    return total;
}

double max_segment_length3d(const std::vector<trochoids::XYZPsiState> &path)
{
    double max_len = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const auto &a = path[i];
        const auto &b = path[i + 1];
        const double d = std::sqrt((b.x - a.x) * (b.x - a.x) +
                                   (b.y - a.y) * (b.y - a.y) +
                                   (b.z - a.z) * (b.z - a.z));
        max_len = std::max(max_len, d);
    }
    return max_len;
}
}  // namespace

TEST(TestTrochoids3D, constrained_vertical_profile_feasible)
{
    double wind[3] = {4.0, -2.0, 0.0};
    const double desired_speed = 40.0;
    const double max_kappa = 0.02;

    trochoids::XYZPsiState start_state = {0.0, 0.0, 100.0, 0.2};
    trochoids::XYZPsiState goal_state = {2200.0, 400.0, 180.0, 0.2};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 2.0;
    constraints.max_descent_rate = 2.0;

    std::vector<trochoids::XYZPsiState> trochoid_path;
    trochoids::VerticalPlanInfo plan_info;
    const bool valid = trochoids::get_trochoid_path_3d(start_state, goal_state, trochoid_path, wind,
                                                        desired_speed, max_kappa, constraints, &plan_info);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(plan_info.valid);
    EXPECT_TRUE(plan_info.vertical_feasible);
    EXPECT_EQ(plan_info.case_used, trochoids::VerticalPlanningCase::DIRECT_PROFILE);
    ASSERT_FALSE(trochoid_path.empty());
    EXPECT_NEAR(trochoid_path.front().z, start_state.z, 1e-6);
    EXPECT_NEAR(trochoid_path.back().z, goal_state.z, 1e-6);
    EXPECT_NEAR(trochoid_path.back().x, goal_state.x, 1e-3);
    EXPECT_NEAR(trochoid_path.back().y, goal_state.y, 1e-3);

    write_path_csv("constrained_vertical_profile_feasible.csv", trochoid_path);
}

TEST(TestTrochoids3D, constrained_vertical_profile_infeasible_reports_loop_estimate)
{
    double wind[3] = {0.0, 0.0, 0.0};
    const double desired_speed = 30.0;
    const double max_kappa = 0.02;

    trochoids::XYZPsiState start_state = {0.0, 0.0, 0.0, 0.0};
    trochoids::XYZPsiState goal_state = {100.0, 0.0, 300.0, 0.0};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 0.5;
    constraints.max_descent_rate = 0.5;

    std::vector<trochoids::XYZPsiState> trochoid_path;
    trochoids::VerticalPlanInfo plan_info;
    const bool valid = trochoids::get_trochoid_path_3d(start_state, goal_state, trochoid_path, wind,
                                                        desired_speed, max_kappa, constraints, &plan_info);

    EXPECT_FALSE(valid);
    EXPECT_FALSE(plan_info.valid);
    EXPECT_FALSE(plan_info.vertical_feasible);
    EXPECT_TRUE(plan_info.required_vertical_time_sec > plan_info.xy_time_sec);
    EXPECT_TRUE(plan_info.estimated_full_loops_needed > 0);

    if (!trochoid_path.empty())
    {
        write_path_csv("constrained_vertical_profile_infeasible_reports_loop_estimate.csv", trochoid_path);
    }
}

TEST(TestTrochoids3D, constrained_vertical_profile_with_full_loop_extension)
{
    double wind[3] = {5.0, 1.0, 0.0};
    const double desired_speed = 20.0;
    const double max_kappa = 0.2;

    trochoids::XYZPsiState start_state = {0.0, 0.0, 0.0, 0.0};
    trochoids::XYZPsiState goal_state = {20.0, 0.0, 25.0, 0.0};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 5.0;
    constraints.max_descent_rate = 5.0;
    constraints.allow_full_loop_extension = true;
    constraints.max_full_loops = 6;

    std::vector<trochoids::XYZPsiState> trochoid_path;
    trochoids::VerticalPlanInfo plan_info;
    const bool valid = trochoids::get_trochoid_path_3d(start_state, goal_state, trochoid_path, wind,
                                                        desired_speed, max_kappa, constraints, &plan_info);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(plan_info.valid);
    EXPECT_TRUE(plan_info.vertical_feasible);
    EXPECT_EQ(plan_info.case_used, trochoids::VerticalPlanningCase::FULL_LOOP_EXTENSION);
    EXPECT_TRUE(plan_info.loops_added_start + plan_info.loops_added_end > 0);
    EXPECT_TRUE(plan_info.added_extension_time_sec > 0.0);

    ASSERT_FALSE(trochoid_path.empty());
    EXPECT_NEAR(trochoid_path.back().x, goal_state.x, 1e-3);
    EXPECT_NEAR(trochoid_path.back().y, goal_state.y, 1e-3);
    EXPECT_NEAR(trochoid_path.back().z, goal_state.z, 1e-6);
    EXPECT_NEAR(trochoid_path.back().psi, goal_state.psi, 1e-6);

    write_path_csv("constrained_vertical_profile_with_full_loop_extension.csv", trochoid_path);
}

TEST(TestTrochoids3D, wind_loop_extension_preserves_terminal_state_regression)
{
    double wind[3] = {6.0, -3.0, 0.0};
    const double desired_speed = 18.0;
    const double max_kappa = 0.2;

    trochoids::XYZPsiState start_state = {10.0, -20.0, 0.0, 0.4};
    trochoids::XYZPsiState goal_state = {60.0, 30.0, 30.0, 0.4};

    // First confirm direct-only mode is infeasible.
    trochoids::VerticalConstraints no_extension;
    no_extension.max_climb_rate = 3.0;
    no_extension.max_descent_rate = 3.0;

    std::vector<trochoids::XYZPsiState> no_extension_path;
    trochoids::VerticalPlanInfo no_extension_info;
    const bool no_extension_valid = trochoids::get_trochoid_path_3d(
        start_state, goal_state, no_extension_path, wind, desired_speed, max_kappa, no_extension, &no_extension_info);
    EXPECT_FALSE(no_extension_valid);
    EXPECT_FALSE(no_extension_info.valid);
    EXPECT_FALSE(no_extension_info.vertical_feasible);

    // Then enable loop extension and verify exact terminal state recovery.
    trochoids::VerticalConstraints with_extension = no_extension;
    with_extension.allow_full_loop_extension = true;
    with_extension.max_full_loops = 8;

    std::vector<trochoids::XYZPsiState> extension_path;
    trochoids::VerticalPlanInfo extension_info;
    const bool extension_valid = trochoids::get_trochoid_path_3d(
        start_state, goal_state, extension_path, wind, desired_speed, max_kappa, with_extension, &extension_info);

    EXPECT_TRUE(extension_valid);
    EXPECT_TRUE(extension_info.valid);
    EXPECT_TRUE(extension_info.vertical_feasible);
    EXPECT_EQ(extension_info.case_used, trochoids::VerticalPlanningCase::FULL_LOOP_EXTENSION);
    EXPECT_TRUE(extension_info.loops_added_start + extension_info.loops_added_end > 0);
    ASSERT_FALSE(extension_path.empty());
    EXPECT_NEAR(extension_path.back().x, goal_state.x, 1e-6);
    EXPECT_NEAR(extension_path.back().y, goal_state.y, 1e-6);
    EXPECT_NEAR(extension_path.back().z, goal_state.z, 1e-6);
    EXPECT_NEAR(extension_path.back().psi, goal_state.psi, 1e-6);

    write_path_csv("wind_loop_extension_preserves_terminal_state_regression.csv", extension_path);
}

TEST(TestTrochoids3D, direct_profile_preferred_when_feasible_even_if_loops_enabled)
{
    double wind[3] = {2.0, 1.0, 0.0};
    const double desired_speed = 30.0;
    const double max_kappa = 0.08;
    trochoids::XYZPsiState start_state = {0.0, 0.0, 100.0, 0.1};
    trochoids::XYZPsiState goal_state = {3000.0, 800.0, 130.0, 0.2};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 4.0;
    constraints.max_descent_rate = 4.0;
    constraints.allow_full_loop_extension = true;
    constraints.max_full_loops = 10;

    std::vector<trochoids::XYZPsiState> path;
    trochoids::VerticalPlanInfo info;
    const bool valid = trochoids::get_trochoid_path_3d(start_state, goal_state, path, wind,
                                                        desired_speed, max_kappa, constraints, &info);
    EXPECT_TRUE(valid);
    EXPECT_TRUE(info.valid);
    EXPECT_EQ(info.case_used, trochoids::VerticalPlanningCase::DIRECT_PROFILE);
    EXPECT_EQ(info.loops_added_start + info.loops_added_end, 0);
}

TEST(TestTrochoids3D, fails_when_full_loop_budget_insufficient)
{
    double wind[3] = {4.0, -1.0, 0.0};
    const double desired_speed = 20.0;
    const double max_kappa = 0.15;
    trochoids::XYZPsiState start_state = {0.0, 0.0, 0.0, 0.0};
    trochoids::XYZPsiState goal_state = {20.0, 0.0, 120.0, 0.0};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 2.0;
    constraints.max_descent_rate = 2.0;
    constraints.allow_full_loop_extension = true;
    constraints.max_full_loops = 1;

    std::vector<trochoids::XYZPsiState> path;
    trochoids::VerticalPlanInfo info;
    const bool valid = trochoids::get_trochoid_path_3d(start_state, goal_state, path, wind,
                                                        desired_speed, max_kappa, constraints, &info);
    EXPECT_FALSE(valid);
    EXPECT_FALSE(info.valid);
    EXPECT_TRUE(info.estimated_full_loops_needed > constraints.max_full_loops);
}

TEST(TestTrochoids3D, flight_path_angle_limit_can_block_otherwise_feasible_profile)
{
    double wind[3] = {0.0, 0.0, 0.0};
    const double desired_speed = 20.0;
    const double max_kappa = 0.05;
    trochoids::XYZPsiState start_state = {0.0, 0.0, 0.0, 0.0};
    trochoids::XYZPsiState goal_state = {200.0, 0.0, 80.0, 0.0};

    trochoids::VerticalConstraints unconstrained;
    unconstrained.max_climb_rate = 100.0;
    unconstrained.max_descent_rate = 100.0;

    std::vector<trochoids::XYZPsiState> direct_path;
    trochoids::VerticalPlanInfo direct_info;
    const bool direct_valid = trochoids::get_trochoid_path_3d(
        start_state, goal_state, direct_path, wind, desired_speed, max_kappa, unconstrained, &direct_info);
    EXPECT_TRUE(direct_valid);
    EXPECT_TRUE(direct_info.valid);

    trochoids::VerticalConstraints constrained = unconstrained;
    constrained.enforce_flight_path_angle = true;
    constrained.max_flight_path_angle_rad = 10.0 * M_PI / 180.0;

    std::vector<trochoids::XYZPsiState> constrained_path;
    trochoids::VerticalPlanInfo constrained_info;
    const bool constrained_valid = trochoids::get_trochoid_path_3d(
        start_state, goal_state, constrained_path, wind, desired_speed, max_kappa, constrained, &constrained_info);

    EXPECT_FALSE(constrained_valid);
    EXPECT_FALSE(constrained_info.valid);
    EXPECT_FALSE(constrained_info.vertical_feasible);
    EXPECT_GT(constrained_info.required_vertical_time_sec, constrained_info.xy_time_sec);
    EXPECT_NEAR(constrained_info.required_vertical_time_sec,
                (goal_state.z - start_state.z) /
                    (desired_speed * std::tan(constrained.max_flight_path_angle_rad)),
                1e-6);
}

TEST(TestTrochoids3D, flight_path_angle_limit_can_be_satisfied_with_loop_extension)
{
    double wind[3] = {0.0, 0.0, 0.0};
    const double desired_speed = 20.0;
    const double max_kappa = 0.2;
    trochoids::XYZPsiState start_state = {0.0, 0.0, 0.0, 0.0};
    trochoids::XYZPsiState goal_state = {20.0, 0.0, 30.0, 0.0};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 100.0;
    constraints.max_descent_rate = 100.0;
    constraints.enforce_flight_path_angle = true;
    constraints.max_flight_path_angle_rad = 5.0 * M_PI / 180.0;
    constraints.allow_full_loop_extension = true;
    constraints.max_full_loops = 20;

    std::vector<trochoids::XYZPsiState> path;
    trochoids::VerticalPlanInfo info;
    const bool valid = trochoids::get_trochoid_path_3d(
        start_state, goal_state, path, wind, desired_speed, max_kappa, constraints, &info);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.vertical_feasible);
    EXPECT_EQ(info.case_used, trochoids::VerticalPlanningCase::FULL_LOOP_EXTENSION);
    EXPECT_GT(info.loops_added_start + info.loops_added_end, 0);
    trochoids_test::expect_path_endpoints_match(path, start_state, goal_state, 1e-6, 1e-6);
    trochoids_test::expect_monotonic_altitude(path, true);
}

TEST(TestTrochoids3D, flight_path_angle_limit_uses_more_restrictive_climb_rate)
{
    double wind[3] = {0.0, 0.0, 0.0};
    const double desired_speed = 25.0;
    const double max_kappa = 0.08;
    trochoids::XYZPsiState start_state = {0.0, 0.0, 0.0, 0.1};
    trochoids::XYZPsiState goal_state = {300.0, 100.0, 60.0, 0.1};

    trochoids::VerticalConstraints constraints;
    constraints.max_climb_rate = 1.0;
    constraints.max_descent_rate = 100.0;
    constraints.enforce_flight_path_angle = true;
    constraints.max_flight_path_angle_rad = 25.0 * M_PI / 180.0;

    std::vector<trochoids::XYZPsiState> path;
    trochoids::VerticalPlanInfo info;
    const bool valid = trochoids::get_trochoid_path_3d(
        start_state, goal_state, path, wind, desired_speed, max_kappa, constraints, &info);

    EXPECT_FALSE(valid);
    EXPECT_FALSE(info.valid);
    EXPECT_NEAR(info.required_vertical_time_sec, 60.0, 1e-6);
}

TEST(TestTrochoids3D, DISABLED_skeptic_randomized_regression_terminal_and_continuity)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dis_pos(-250.0, 250.0);
    std::uniform_real_distribution<double> dis_wind(-8.0, 8.0);
    std::uniform_real_distribution<double> dis_psi(-M_PI, M_PI);
    std::uniform_real_distribution<double> dis_speed(15.0, 35.0);
    std::uniform_real_distribution<double> dis_kappa(0.05, 0.2);
    std::uniform_real_distribution<double> dis_zdelta(-70.0, 70.0);

    for (int i = 0; i < 120; ++i)
    {
        double wind[3] = {dis_wind(rng), dis_wind(rng), 0.0};
        const double v = dis_speed(rng);
        const double kappa = dis_kappa(rng);

        trochoids::XYZPsiState start_state = {dis_pos(rng), dis_pos(rng), 100.0, dis_psi(rng)};
        trochoids::XYZPsiState goal_state = {dis_pos(rng), dis_pos(rng), 100.0 + dis_zdelta(rng), dis_psi(rng)};

        trochoids::VerticalConstraints constraints;
        constraints.max_climb_rate = 2.5;
        constraints.max_descent_rate = 2.5;
        constraints.allow_full_loop_extension = true;
        constraints.max_full_loops = 10;

        std::vector<trochoids::XYZPsiState> path;
        trochoids::VerticalPlanInfo info;
        const bool valid = trochoids::get_trochoid_path_3d(start_state, goal_state, path, wind, v, kappa, constraints, &info, 1.0);

        if (!valid)
        {
            // Expected for some hard random instances. If invalid, it should not claim validity.
            EXPECT_FALSE(info.valid);
            continue;
        }

        ASSERT_FALSE(path.empty());
        EXPECT_NEAR(path.back().x, goal_state.x, 1e-6);
        EXPECT_NEAR(path.back().y, goal_state.y, 1e-6);
        EXPECT_NEAR(path.back().z, goal_state.z, 1e-6);
        EXPECT_NEAR(path.back().psi, goal_state.psi, 1e-6);

        // Catch suspicious discontinuities caused by stitching/overwrite logic.
        const double max_step = max_segment_length3d(path);
        EXPECT_TRUE(std::isfinite(max_step));
        EXPECT_LT(max_step, 2.0);

        // Verify produced total time is consistent with requested vertical feasibility.
        const double t = path_time_estimate(path, wind, v);
        EXPECT_GE(t + 1e-6, info.required_vertical_time_sec);
    }
}
