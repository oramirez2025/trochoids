/*********************************************************************
    The Clear BSD License

    Copyright (c) 2026, AirLab
    All rights reserved.
*********************************************************************/

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <tuple>
#include <vector>

#include "test_utils.h"
#include "trochoids/trochoid_utils.h"
#include "trochoids/trochoids.h"

namespace
{
using Path = std::vector<std::tuple<double, double, double>>;

constexpr int kPathComparisonCases = 500;
constexpr int kDubinsMatrixCases = 100000;
constexpr int kBbbComparisonCases = 25;
constexpr int kRootSolver1dCases = 2500;
constexpr int kRootSolver2dCases = 200;
constexpr int kRootSolver2dOracleCases = 10;
constexpr int kThreeDimensionalCases = 120;

void log_path_comparison_failure(const trochoids::XYZPsiState &start_state,
                                 const trochoids::XYZPsiState &goal_state,
                                 const double wind[3],
                                 double desired_speed,
                                 double max_kappa,
                                 double analytical_length,
                                 double numerical_length,
                                 double exhaustive_length)
{
    std::cout << "Start: " << start_state.x << ", " << start_state.y << ", "
              << start_state.psi << std::endl;
    std::cout << "Goal: " << goal_state.x << ", " << goal_state.y << ", "
              << goal_state.psi << std::endl;
    std::cout << "Wind: " << wind[0] << ", " << wind[1] << ", " << wind[2]
              << std::endl;
    std::cout << "Speed: " << desired_speed << std::endl;
    std::cout << "Kappa: " << max_kappa << std::endl;
    std::cout << "Analytical: " << analytical_length
              << " numerical: " << numerical_length
              << " exhaustive: " << exhaustive_length << std::endl;
}

double numerical_path_length_for_1d_mode(
    trochoids::Trochoid trochoid,
    bool use_chebyshev,
    trochoids::Trochoid::RootSolve1DMethod method)
{
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = use_chebyshev;
    trochoid.root_solve_1d_method = method;
    const Path path = trochoid.getTrochoidNumerical();
    return path.empty() ? -1.0 : trochoids::Trochoid::get_length(path);
}

double numerical_path_length_for_2d_mode(
    trochoids::Trochoid trochoid,
    trochoids::Trochoid::RootSolve2DMethod method)
{
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    trochoid.include_BBB = true;
    trochoid.root_solve_2d_method = method;
    const Path path = trochoid.getTrochoidNumerical();
    return path.empty() ? -1.0 : trochoids::Trochoid::get_length(path);
}

double empirical_best_2d_length(trochoids::Trochoid trochoid)
{
    double best = std::numeric_limits<double>::infinity();
    const std::vector<std::pair<int, int>> configs = {{360, 33}, {900, 81}};
    const std::vector<trochoids::Trochoid::RootSolve2DMethod> methods = {
        trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID,
        trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON};

    for (const auto &config : configs)
    {
        trochoid.root_solve_2d_grid_samples = config.first;
        trochoid.root_solve_2d_chebyshev_samples = config.second;
        for (const auto method : methods)
        {
            const double length =
                numerical_path_length_for_2d_mode(trochoid, method);
            if (length > 0.0)
            {
                best = std::min(best, length);
            }
        }
    }

    return std::isfinite(best) ? best : -1.0;
}

void expect_bbb_methods_match(trochoids::Trochoid trochoid)
{
    trochoid.include_BBB = true;
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;

    const Path classified = trochoid.getTrochoid();
    ASSERT_FALSE(classified.empty());
    const double classified_length =
        trochoids::Trochoid::get_length(classified);

    const Path chebyshev = trochoid.getTrochoidNumerical();
    ASSERT_FALSE(chebyshev.empty());
    const double chebyshev_length =
        trochoids::Trochoid::get_length(chebyshev);

    trochoid.use_Chebyshev = false;
    const Path non_chebyshev = trochoid.getTrochoidNumerical();
    ASSERT_FALSE(non_chebyshev.empty());
    const double non_chebyshev_length =
        trochoids::Trochoid::get_length(non_chebyshev);

    trochoid.use_Chebyshev = true;
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = false;
    const Path exhaustive = trochoid.getTrochoidNumerical();
    ASSERT_FALSE(exhaustive.empty());
    const double exhaustive_length =
        trochoids::Trochoid::get_length(exhaustive);

    const bool classified_matches_chebyshev =
        trochoids_test::ratio_within(classified_length, chebyshev_length, 0.05);
    const bool chebyshev_matches_exhaustive =
        trochoids_test::ratio_within(chebyshev_length, exhaustive_length, 0.05);
    const bool exhaustive_matches_non_chebyshev =
        trochoids_test::ratio_within(
            exhaustive_length, non_chebyshev_length, 0.05);

    if (!classified_matches_chebyshev ||
        !chebyshev_matches_exhaustive ||
        !exhaustive_matches_non_chebyshev)
    {
        std::cout << "BBB comparison mismatch" << std::endl;
        std::cout << "  classified: " << classified_length << std::endl;
        std::cout << "  chebyshev: " << chebyshev_length << std::endl;
        std::cout << "  exhaustive: " << exhaustive_length << std::endl;
        std::cout << "  non-chebyshev: " << non_chebyshev_length << std::endl;
        std::cout << "  start: " << trochoid.problem.X0[0] << ", "
                  << trochoid.problem.X0[1] << ", "
                  << trochoid.problem.X0[2] << std::endl;
        std::cout << "  goal: " << trochoid.problem.Xf[0] << ", "
                  << trochoid.problem.Xf[1] << ", "
                  << trochoid.problem.Xf[2] << std::endl;
        std::cout << "  wind: " << trochoid.problem.wind[0] << ", "
                  << trochoid.problem.wind[1] << std::endl;
        std::cout << "  max_kappa: " << trochoid.problem.max_kappa
                  << std::endl;
    }

    EXPECT_TRUE(classified_matches_chebyshev);
    EXPECT_TRUE(chebyshev_matches_exhaustive);
    EXPECT_TRUE(exhaustive_matches_non_chebyshev);
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
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const auto &a = path[i];
        const auto &b = path[i + 1];
        const double distance = std::hypot(b.x - a.x, b.y - a.y);
        if (distance < 1e-9)
        {
            continue;
        }
        const double heading = wrap_pi(0.5 * (a.psi + b.psi));
        const double ground_speed = std::max(
            std::hypot(v * std::cos(heading) + wind[0],
                       v * std::sin(heading) + wind[1]),
            1e-9);
        total += distance / ground_speed;
    }
    return total;
}

double max_segment_length_3d(
    const std::vector<trochoids::XYZPsiState> &path)
{
    double max_length = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const auto &a = path[i];
        const auto &b = path[i + 1];
        max_length = std::max(
            max_length,
            std::sqrt((b.x - a.x) * (b.x - a.x) +
                      (b.y - a.y) * (b.y - a.y) +
                      (b.z - a.z) * (b.z - a.z)));
    }
    return max_length;
}
}  // namespace

TEST(RandomizedDiscovery, compare_path_methods_without_wind)
{
    std::mt19937 generator(42);
    std::uniform_real_distribution<> position(-1000.0, 1000.0);
    std::uniform_real_distribution<> curvature(0.001, 0.1);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);
    double wind[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < kPathComparisonCases; ++i)
    {
        const double max_kappa = curvature(generator);
        const Dubins::DubinsStateSpace::DubinsState start = {
            position(generator), position(generator), heading(generator)};
        const Dubins::DubinsStateSpace::DubinsState goal = {
            position(generator), position(generator), heading(generator)};

        Dubins::DubinsStateSpace dubins_solver(1.0 / max_kappa);
        const auto dubins = dubins_solver.dubins(start, goal);
        if (dubins.type_[1] ==
                Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT ||
            dubins.type_[1] ==
                Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT)
        {
            continue;
        }

        const trochoids::XYZPsiState start_state = {
            start.x, start.y, 0.0, start.theta};
        const trochoids::XYZPsiState goal_state = {
            goal.x, goal.y, 0.0, goal.theta};
        std::vector<trochoids::XYZPsiState> analytical;
        std::vector<trochoids::XYZPsiState> numerical;
        std::vector<trochoids::XYZPsiState> exhaustive;

        const bool analytical_valid = trochoids::get_trochoid_path(
            start_state, goal_state, analytical, wind, 28.0, max_kappa);
        const bool numerical_valid = trochoids::get_trochoid_path_numerical(
            start_state, goal_state, numerical, wind, 28.0, max_kappa);
        const bool exhaustive_valid = trochoids::get_trochoid_path_numerical(
            start_state, goal_state, exhaustive, wind, 28.0, max_kappa, true);

        ASSERT_TRUE(analytical_valid);
        ASSERT_TRUE(numerical_valid);
        ASSERT_TRUE(exhaustive_valid);

        const double analytical_length = trochoids::get_length(analytical);
        const double numerical_length = trochoids::get_length(numerical);
        const double exhaustive_length = trochoids::get_length(exhaustive);
        const double dubins_length = dubins.length() / max_kappa;

        if (!trochoids_test::ratio_within(
                analytical_length, dubins_length, 0.05) ||
            !trochoids_test::ratio_within(
                analytical_length, numerical_length, 0.05) ||
            !trochoids_test::ratio_within(
                exhaustive_length, numerical_length, 0.05))
        {
            log_path_comparison_failure(
                start_state, goal_state, wind, 28.0, max_kappa,
                analytical_length, numerical_length, exhaustive_length);
            std::cout << "Dubins: " << dubins_length << std::endl;
        }

        EXPECT_TRUE(trochoids_test::ratio_within(
            analytical_length, dubins_length, 0.05));
        EXPECT_TRUE(trochoids_test::ratio_within(
            analytical_length, numerical_length, 0.05));
        EXPECT_TRUE(trochoids_test::ratio_within(
            exhaustive_length, numerical_length, 0.05));
    }
}

TEST(RandomizedDiscovery, compare_path_methods_with_wind)
{
    std::mt19937 generator(314159);
    std::uniform_real_distribution<> position(-1000.0, 1000.0);
    std::uniform_real_distribution<> curvature(0.001, 0.1);
    std::uniform_real_distribution<> wind_component(-35.0, 35.0);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);

    for (int i = 0; i < kPathComparisonCases; ++i)
    {
        const double max_kappa = curvature(generator);
        const trochoids::XYZPsiState start = {
            position(generator), position(generator), 0.0, heading(generator)};
        const trochoids::XYZPsiState goal = {
            position(generator), position(generator), 0.0, heading(generator)};
        double wind[3] = {
            wind_component(generator), wind_component(generator), 0.0};
        std::vector<trochoids::XYZPsiState> analytical;
        std::vector<trochoids::XYZPsiState> numerical;
        std::vector<trochoids::XYZPsiState> exhaustive;

        const bool analytical_valid = trochoids::get_trochoid_path(
            start, goal, analytical, wind, 50.0, max_kappa);
        const bool numerical_valid = trochoids::get_trochoid_path_numerical(
            start, goal, numerical, wind, 50.0, max_kappa);
        const bool exhaustive_valid = trochoids::get_trochoid_path_numerical(
            start, goal, exhaustive, wind, 50.0, max_kappa, true);

        ASSERT_TRUE(analytical_valid);
        ASSERT_TRUE(numerical_valid);
        ASSERT_TRUE(exhaustive_valid);

        const double analytical_length = trochoids::get_length(analytical);
        const double numerical_length = trochoids::get_length(numerical);
        const double exhaustive_length = trochoids::get_length(exhaustive);

        if (!trochoids_test::ratio_within(
                analytical_length, numerical_length, 0.05) ||
            !trochoids_test::ratio_within(
                exhaustive_length, numerical_length, 0.05))
        {
            log_path_comparison_failure(
                start, goal, wind, 50.0, max_kappa,
                analytical_length, numerical_length, exhaustive_length);
        }

        EXPECT_TRUE(trochoids_test::ratio_within(
            analytical_length, numerical_length, 0.05));
        EXPECT_TRUE(trochoids_test::ratio_within(
            exhaustive_length, numerical_length, 0.05));
    }
}

TEST(RandomizedDiscovery, compare_dubins_matrix_implementation)
{
    std::mt19937 generator(42);
    std::uniform_real_distribution<> position(-1000.0, 1000.0);
    std::uniform_real_distribution<> curvature(0.005, 0.01);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);

    for (int i = 0; i < kDubinsMatrixCases; ++i)
    {
        const double max_kappa = curvature(generator);
        const Dubins::DubinsStateSpace::DubinsState start = {
            position(generator), position(generator), heading(generator)};
        const Dubins::DubinsStateSpace::DubinsState goal = {
            position(generator), position(generator), heading(generator)};
        Dubins::DubinsStateSpace solver(1.0 / max_kappa);

        const auto matrix_path = solver.dubins_matrix(start, goal);
        const auto reference_path = solver.dubins(start, goal);
        EXPECT_NEAR(matrix_path.length(), reference_path.length(), 1e-5);
    }
}

TEST(RandomizedDiscovery, compare_bbb_methods_without_wind)
{
    std::mt19937 generator(271828);
    std::uniform_real_distribution<> position(-1000.0, 1000.0);
    std::uniform_real_distribution<> curvature(0.001, 0.1);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);

    for (int i = 0; i < kBbbComparisonCases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = {0.0, 0.0, 0.0};
        trochoid.problem.max_kappa = curvature(generator);
        trochoid.problem.X0 = {
            position(generator), position(generator), heading(generator)};
        trochoid.problem.Xf = {
            position(generator), position(generator), heading(generator)};
        expect_bbb_methods_match(trochoid);
    }
}

TEST(RandomizedDiscovery, compare_bbb_methods_with_wind)
{
    std::mt19937 generator(161803);
    std::uniform_real_distribution<> position(-1000.0, 1000.0);
    std::uniform_real_distribution<> curvature(0.001, 0.1);
    std::uniform_real_distribution<> wind_component(-35.0, 35.0);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);

    for (int i = 0; i < kBbbComparisonCases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = {
            wind_component(generator), wind_component(generator), 0.0};
        trochoid.problem.max_kappa = curvature(generator);
        trochoid.problem.X0 = {
            position(generator), position(generator), heading(generator)};
        trochoid.problem.Xf = {
            position(generator), position(generator), heading(generator)};
        expect_bbb_methods_match(trochoid);
    }
}

TEST(RandomizedDiscovery, compare_1d_root_solvers)
{
    std::mt19937 generator(42);
    std::uniform_real_distribution<> position(-1000.0, 1000.0);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> wind_component(-25.0, 25.0);
    std::uniform_real_distribution<> curvature(0.003, 0.03);

    for (int i = 0; i < kRootSolver1dCases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = {
            wind_component(generator), wind_component(generator), 0.0};
        trochoid.problem.max_kappa = curvature(generator);
        trochoid.problem.X0 = {
            position(generator), position(generator), heading(generator)};
        trochoid.problem.Xf = {
            position(generator), position(generator), heading(generator)};

        const std::vector<double> lengths = {
            numerical_path_length_for_1d_mode(
                trochoid, true,
                trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT),
            numerical_path_length_for_1d_mode(
                trochoid, false,
                trochoids::Trochoid::RootSolve1DMethod::NEWTON_RAPHSON),
            numerical_path_length_for_1d_mode(
                trochoid, false,
                trochoids::Trochoid::RootSolve1DMethod::BRACKETED_BISECTION),
            numerical_path_length_for_1d_mode(
                trochoid, false,
                trochoids::Trochoid::RootSolve1DMethod::NON_ROBUST_BRENT),
            numerical_path_length_for_1d_mode(
                trochoid, false,
                trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT)};

        ASSERT_TRUE(std::all_of(
            lengths.begin(), lengths.end(),
            [](double length) { return length > 0.0; }));
        const auto bounds = std::minmax_element(lengths.begin(), lengths.end());
        EXPECT_LT(*bounds.second / *bounds.first - 1.0, 0.08);
    }
}

TEST(RandomizedDiscovery, compare_2d_root_solvers_high_density_fixed_cases)
{
    struct CaseInput
    {
        std::vector<double> wind;
        double max_kappa;
        std::vector<double> start;
        std::vector<double> goal;
    };

    const std::vector<CaseInput> cases = {
        {{12.0, -18.0, 0.0}, 0.008,
         {-200.0, 300.0, 1.1}, {450.0, -250.0, 4.2}},
        {{20.0, 15.0, 0.0}, 0.006,
         {-100.0, -100.0, 0.5}, {600.0, 350.0, 3.7}}};

    for (const auto &test_case : cases)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = test_case.wind;
        trochoid.problem.max_kappa = test_case.max_kappa;
        trochoid.problem.X0 = test_case.start;
        trochoid.problem.Xf = test_case.goal;
        trochoid.include_BBB = true;
        trochoid.root_solve_2d_grid_samples = 720;
        trochoid.root_solve_2d_chebyshev_samples = 65;

        const double newton = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double chebyshev = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
        // const double oracle = empirical_best_2d_length(trochoid);

        if (!trochoids_test::ratio_within(newton, chebyshev, 0.02))
        {
            std::cout << "2D high-density case mismatch" << std::endl;
            std::cout << "  newton: " << newton << std::endl;
            std::cout << "  chebyshev: " << chebyshev << std::endl;
            // std::cout << "  oracle: " << oracle << std::endl;
            std::cout << "  start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
            std::cout << "  goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
            std::cout << "  wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << std::endl;
            std::cout << "  max_kappa: " << trochoid.problem.max_kappa << std::endl;
        }

        ASSERT_GT(newton, 0.0);
        ASSERT_GT(chebyshev, 0.0);
        // ASSERT_GT(oracle, 0.0);
        // EXPECT_LT(newton / oracle - 1.0, 0.20);
        // EXPECT_LT(chebyshev / oracle - 1.0, 0.20);
        // EXPECT_LT(std::min(newton, chebyshev) / oracle - 1.0, 0.05);
        EXPECT_TRUE(trochoids_test::ratio_within(newton, chebyshev, 0.02));
    }
}

TEST(RandomizedDiscovery, compare_2d_root_solvers_fixed_cases)
{
    struct CaseInput
    {
        std::vector<double> wind;
        double max_kappa;
        std::vector<double> start;
        std::vector<double> goal;
    };

    const std::vector<CaseInput> cases = {
        {{12.0, -18.0, 0.0}, 0.008,
         {-200.0, 300.0, 1.1}, {450.0, -250.0, 4.2}},
        {{20.0, 15.0, 0.0}, 0.006,
         {-100.0, -100.0, 0.5}, {600.0, 350.0, 3.7}},
        {{-15.0, 22.0, 0.0}, 0.01,
         {300.0, -450.0, 2.2}, {-500.0, 200.0, 5.4}}};

    for (const auto &test_case : cases)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = test_case.wind;
        trochoid.problem.max_kappa = test_case.max_kappa;
        trochoid.problem.X0 = test_case.start;
        trochoid.problem.Xf = test_case.goal;
        trochoid.root_solve_2d_grid_samples = 360;
        trochoid.root_solve_2d_chebyshev_samples = 33;

        const double newton = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double chebyshev = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
        ASSERT_GT(newton, 0.0);
        ASSERT_GT(chebyshev, 0.0);
        EXPECT_TRUE(trochoids_test::ratio_within(newton, chebyshev, 0.08));
    }
}

TEST(RandomizedDiscovery, compare_2d_root_solvers_boundary_stress)
{
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50.0;
    trochoid.problem.wind = {24.0, -22.0, 0.0};
    trochoid.problem.max_kappa = 0.0045;
    trochoid.problem.X0 = {-780.0, 760.0, 0.05};
    trochoid.problem.Xf = {790.0, -740.0, 6.18};
    trochoid.include_BBB = true;
    trochoid.root_solve_2d_grid_samples = 900;
    trochoid.root_solve_2d_chebyshev_samples = 81;

    const double newton = numerical_path_length_for_2d_mode(
        trochoid, trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
    const double chebyshev = numerical_path_length_for_2d_mode(
        trochoid,
        trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
    ASSERT_GT(newton, 0.0);
    ASSERT_GT(chebyshev, 0.0);
    EXPECT_TRUE(trochoids_test::ratio_within(newton, chebyshev, 0.07));
}

TEST(RandomizedDiscovery, compare_2d_root_solvers)
{
    std::mt19937 generator(7);
    std::uniform_real_distribution<> position(-800.0, 800.0);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> wind_component(-25.0, 25.0);
    std::uniform_real_distribution<> curvature(0.004, 0.02);

    for (int i = 0; i < kRootSolver2dCases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = {
            wind_component(generator), wind_component(generator), 0.0};
        trochoid.problem.max_kappa = curvature(generator);
        trochoid.problem.X0 = {
            position(generator), position(generator), heading(generator)};
        trochoid.problem.Xf = {
            position(generator), position(generator), heading(generator)};
        trochoid.root_solve_2d_grid_samples = 360;
        trochoid.root_solve_2d_chebyshev_samples = 33;

        const double newton = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double chebyshev = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
        ASSERT_GT(newton, 0.0);
        ASSERT_GT(chebyshev, 0.0);
        EXPECT_TRUE(trochoids_test::ratio_within(newton, chebyshev, 0.10));
    }
}

TEST(RandomizedDiscovery, compare_2d_root_solvers_with_oracle)
{
    std::mt19937 generator(11);
    std::uniform_real_distribution<> position(-800.0, 800.0);
    std::uniform_real_distribution<> heading(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> wind_component(-25.0, 25.0);
    std::uniform_real_distribution<> curvature(0.004, 0.02);

    for (int i = 0; i < kRootSolver2dOracleCases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50.0;
        trochoid.problem.wind = {
            wind_component(generator), wind_component(generator), 0.0};
        trochoid.problem.max_kappa = curvature(generator);
        trochoid.problem.X0 = {
            position(generator), position(generator), heading(generator)};
        trochoid.problem.Xf = {
            position(generator), position(generator), heading(generator)};
        trochoid.include_BBB = true;

        const double newton = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double chebyshev = numerical_path_length_for_2d_mode(
            trochoid,
            trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
        const double oracle = empirical_best_2d_length(trochoid);
        ASSERT_GT(newton, 0.0);
        ASSERT_GT(chebyshev, 0.0);
        ASSERT_GT(oracle, 0.0);

        const double newton_gap = newton / oracle - 1.0;
        const double chebyshev_gap = chebyshev / oracle - 1.0;
        EXPECT_LT(newton_gap, 0.20);
        EXPECT_LT(chebyshev_gap, 0.20);
        EXPECT_LT(std::min(newton_gap, chebyshev_gap), 0.08);
    }
}

TEST(RandomizedDiscovery, verify_3d_terminal_state_and_continuity)
{
    std::mt19937 generator(42);
    std::uniform_real_distribution<> position(-250.0, 250.0);
    std::uniform_real_distribution<> wind_component(-8.0, 8.0);
    std::uniform_real_distribution<> heading(-M_PI, M_PI);
    std::uniform_real_distribution<> speed(15.0, 35.0);
    std::uniform_real_distribution<> curvature(0.05, 0.2);
    std::uniform_real_distribution<> altitude_delta(-70.0, 70.0);

    for (int i = 0; i < kThreeDimensionalCases; ++i)
    {
        double wind[3] = {
            wind_component(generator), wind_component(generator), 0.0};
        const double desired_speed = speed(generator);
        const double max_kappa = curvature(generator);
        const trochoids::XYZPsiState start = {
            position(generator), position(generator), 100.0,
            heading(generator)};
        const trochoids::XYZPsiState goal = {
            position(generator), position(generator),
            100.0 + altitude_delta(generator), heading(generator)};

        trochoids::VerticalConstraints constraints;
        constraints.max_climb_rate = 2.5;
        constraints.max_descent_rate = 2.5;
        constraints.allow_full_loop_extension = true;
        constraints.max_full_loops = 10;

        std::vector<trochoids::XYZPsiState> path;
        trochoids::VerticalPlanInfo info;
        const bool valid = trochoids::get_trochoid_path_3d(
            start, goal, path, wind, desired_speed, max_kappa,
            constraints, &info, 1.0);

        if (!valid)
        {
            EXPECT_FALSE(info.valid);
            continue;
        }

        trochoids_test::expect_path_endpoints_match(
            path, start, goal, 1e-6, 1e-6);
        EXPECT_TRUE(std::isfinite(max_segment_length_3d(path)));
        EXPECT_LT(max_segment_length_3d(path), 2.0);
        EXPECT_GE(
            path_time_estimate(path, wind, desired_speed) + 1e-6,
            info.required_vertical_time_sec);
    }
}
