/*********************************************************************
    The Clear BSD License

    Copyright (c) 2025, AirLab
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted (subject to the limitations in the disclaimer
    below) provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

        * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from this
        software without specific prior written permission.

    NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
    THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
    IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Brady Moon*/

#include <gtest/gtest.h>
#include <chrono>
#include <random>
#include <iostream>
#include "trochoids/trochoid_utils.h"
#include "trochoids/trochoids.h"
#include <fstream>

// Straight Line test
// TEST(TestChebyshev, trochoid_analytical_straight)
// {
//     double wind[3] = {0, 0, 0};
//     double desired_speed = 50;
//     double max_kappa = .015;

//     trochoids::XYZPsiState start_state = {0, 0, 110, 0};
//     trochoids::XYZPsiState goal_state = {1000, 0, 110, 0};

//     std::vector<trochoids::XYZPsiState> trochoid_path;
//     bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
//     double dist = trochoids::get_length(trochoid_path);

//     EXPECT_TRUE(valid);
//     EXPECT_TRUE(abs(dist - 1000) < 11);
// }


TEST(TestChebyshev, DISABLED_trochoid_compare_methods_random_wind_varkappa)
{
    
    double desired_speed = 20;
    double max_kappa = 0.01;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> kappaRange(0.01, 0.1);

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    double wind[3] = {5, -5, 0};

    trochoids::Trochoid trochoid;

    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    trochoid.include_BBB = true;
    trochoid.use_dubins_if_low_wind = true;

    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {wind[0], wind[1]};
    double ang_rate = desired_speed/(1.0/max_kappa);

    trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
    trochoid.problem.Xf = {goal_state.x , goal_state.y, goal_state.psi};

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++)
    {   
        // if(i % 1000 == 0)
        //     std::cout << "Iteration number: " << i << std::endl;
        
        max_kappa = kappaRange(gen);
        trochoid.problem.max_kappa = max_kappa;
        Path path = trochoid.getTrochoid();
    }
    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
    // std::cout << "Total time: " << duration.count() << " ms" << std::endl;
}


// This was a case where numerical error of sin(x) was causing issues with the dubins assert
TEST(TestChebyshev, unit_test_edge_cases1){
    double desired_speed = 50;
    double max_kappa = 0.0033911;

    trochoids::Trochoid trochoid;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {0, 0, 0};


    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {-521.029, 364.036, 0};
    trochoid.problem.Xf = {-407, -340, 0};

    // This one is the actual issue
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;
    Path path_no_dubins = trochoid.getTrochoid();
    EXPECT_TRUE(path_no_dubins.size() != 0);
    double path_length_no_dubins = trochoids::Trochoid::get_length(path_no_dubins);
}

// This was a case where numerical error of sin(x) was causing issues with the dubins assert
TEST(TestChebyshev, unit_test_edge_cases2){
    double desired_speed = 50;
    double max_kappa = 0.025151422343509689;

    trochoids::Trochoid trochoid;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {0, 0, 0};


    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {773.62942234084744, -699.73428747120681, 0};
    trochoid.problem.Xf = {90.80313332531523, 656.94323838141622, 0};

    // This one is the actual issue
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;
    Path path_no_dubins = trochoid.getTrochoid();
    EXPECT_TRUE(path_no_dubins.size() != 0);
    double path_length_no_dubins = trochoids::Trochoid::get_length(path_no_dubins);

}

// This is the case when turning off chebyshev yields a different path length
TEST(TestChebyshev, unit_test_edge_cases3){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 158.979
    // Path length numerical: 158.979
    // Path length numerical no dubins: 158.979
    // Path length chebyshev: 104.138
    // Start: 933.617, -965.429, 0.810681
    // Goal: 950.664, -971.378, 3.27622
    // Max Kappa: 0.0770868
    // Wind: 11.4881, 10.9678
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {11.4881, 10.9678, 0};


    trochoid.problem.max_kappa = 0.0770868;
    trochoid.problem.X0 = {933.617, -965.429, 0.810681};
    trochoid.problem.Xf = {950.664, -971.378, 3.27622};

    // without chebyshev
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;
    Path path_no_chebyshev = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_no_chebyshev.size() != 0);
    double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

    // with chebyshev
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    Path path_chebyshev = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_chebyshev.size() != 0);
    double path_length_chebyshev = trochoids::Trochoid::get_length(path_chebyshev);

    bool lengths_match = (abs(path_length_no_chebyshev - path_length_chebyshev) < 0.05);
    if (!lengths_match)
    {
        std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
        std::cout << "Path length chebyshev: " << path_length_chebyshev << std::endl;
    }
    EXPECT_TRUE(lengths_match);
}

// This is the case when turning off chebyshev yields a different path length
TEST(TestChebyshev, unit_test_edge_cases4){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 3530.22
    // Path length numerical: 3530.22
    // Path length numerical no dubins: 3530.22
    // Path length chebyshev: 2871.01
    // Start: -395.692, -707.249, 2.00815
    // Goal: 804.381, 18.0105, 0.261994
    // Max Kappa: 0.00215523
    // Wind: -1.13017, 24.8283
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {-1.13017, 24.8283, 0};


    trochoid.problem.max_kappa = 0.00215523;
    trochoid.problem.X0 = {-395.692, -707.249, 2.00815};
    trochoid.problem.Xf = {804.381, 18.0105, 0.261994};

    // without chebyshev
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;
    Path path_no_chebyshev = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_no_chebyshev.size() != 0);
    double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

    // with chebyshev
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    Path path_chebyshev = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_chebyshev.size() != 0);
    double path_length_chebyshev = trochoids::Trochoid::get_length(path_chebyshev);

    bool lengths_match = (abs(path_length_no_chebyshev - path_length_chebyshev) < 0.05);
    if (!lengths_match)
    {
        std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
        std::cout << "Path length chebyshev: " << path_length_chebyshev << std::endl;
    }
    EXPECT_TRUE(lengths_match);
}



