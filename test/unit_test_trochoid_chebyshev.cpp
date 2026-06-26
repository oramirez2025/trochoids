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
#include <limits>
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

TEST(TestChebyshev, random_BBB_wind){
    // ./devel/lib/trochoids/trochoids-test --gtest_filter="*Che*"
    double desired_speed = 50;
    double max_kappa = .1;

    trochoids::Trochoid trochoid;
    trochoid.problem.v = desired_speed;

    

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-10, 10);
    std::uniform_real_distribution<> kappaRange(0.001, max_kappa);
    std::uniform_real_distribution<> disWind(-35, 35);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    trochoid.include_BBB = true;
    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    for (int i = 0; i < 2; i++)
    {   
        if(i % 1 == 0 && i != 0)
            std::cout << "Iteration number: " << i << std::endl;

        trochoid.problem.wind = {disWind(gen), disWind(gen)};
        trochoid.problem.max_kappa = kappaRange(gen);
        trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};

        trochoid.use_dubins_if_low_wind = true;
        trochoid.use_trochoid_classification = true;
        trochoid.use_Chebyshev = true;
        Path path = trochoid.getTrochoid();
        EXPECT_TRUE(path.size() != 0);
        double path_length = trochoids::Trochoid::get_length(path);

        // Check Chebyshev
        trochoid.use_dubins_if_low_wind = true;
        trochoid.use_trochoid_classification = true;
        trochoid.use_Chebyshev = false;
        Path path_no_chebyshev = trochoid.getTrochoidNumerical();
        EXPECT_TRUE(path_no_chebyshev.size() != 0);
        double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

        // Check without using any classification or analytical methods
        trochoid.use_dubins_if_low_wind = true;
        trochoid.use_Chebyshev = true;
        Path path_numerical = trochoid.getTrochoidNumerical();
        EXPECT_TRUE(path_numerical.size() != 0);
        double path_length_numerical = trochoids::Trochoid::get_length(path_numerical);

        // In low wind won't use dubins (but all of these have wind)
        trochoid.use_dubins_if_low_wind = false;
        trochoid.use_Chebyshev = true;
        Path path_numerical_no_dubins = trochoid.getTrochoidNumerical();
        EXPECT_TRUE(path_numerical_no_dubins.size() != 0);
        double path_length_numerical_no_dubins = trochoids::Trochoid::get_length(path_numerical_no_dubins);

        bool one_and_two_match = (abs(path_length/path_length_numerical - 1.0) < 0.05);
        bool two_and_three_match = (abs(path_length_numerical/path_length_numerical_no_dubins - 1.0) < 0.05);
        bool three_and_four_match = (abs(path_length_numerical_no_dubins/path_length_no_chebyshev - 1.0) < 0.05);

        EXPECT_TRUE(one_and_two_match);
        EXPECT_TRUE(two_and_three_match);
        EXPECT_TRUE(three_and_four_match);
        if (!one_and_two_match || !two_and_three_match || !three_and_four_match)
        {
            std::cout << "Path length: " << path_length << std::endl;
            std::cout << "Path length numerical: " << path_length_numerical << std::endl;
            std::cout << "Path length numerical no dubins: " << path_length_numerical_no_dubins << std::endl;
            std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
            std::cout << "Start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
            std::cout << "Goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
            std::cout << "Max Kappa: " << trochoid.problem.max_kappa << std::endl;
            std::cout << "Wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << std::endl;
        }
    }
}

// TEST(TestChebyshev, compare_with_BBB_no_wind){
//     double desired_speed = 50;
//     double max_kappa = .1;

//     trochoids::Trochoid trochoid;
//     trochoid.problem.v = desired_speed;
//     trochoid.problem.wind = {0, 0, 0};

    

//     std::random_device rd;
//     std::mt19937 gen = std::mt19937(rd());
//     std::uniform_real_distribution<> disRange(-1000, 1000);
//     std::uniform_real_distribution<> kappaRange(0.001, max_kappa);
//     std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
//     // auto start_time = ompl::time::now();

//     // double old_method_time = 0;
//     // double new_method_time = 0;

//     for (int i = 0; i < 100000; i++)
//     {   
//         if(i % 1000 == 0 && i != 0)
//             std::cout << "Iteration number: " << i << std::endl;

//         trochoid.problem.max_kappa = kappaRange(gen);
//         trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
//         trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};
//         // trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
//         // trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};

//         trochoid.use_dubins_if_low_wind = true;
//         trochoid.use_trochoid_classification = true;
//         trochoid.use_Chebyshev = true;
//         Path path = trochoid.getTrochoid();
//         EXPECT_TRUE(path.size() != 0);
//         double path_length = trochoids::Trochoid::get_length(path);

//         // Check without Chebyshev (this just uses dubins in no wind)
//         trochoid.use_Chebyshev = false;
//         Path path_no_chebyshev = trochoid.getTrochoidNumerical();
//         EXPECT_TRUE(path_no_chebyshev.size() != 0);
//         double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

//         // Check without using any classification or analytical methods (this just uses dubins in no wind)
//         trochoid.use_Chebyshev = true;
//         trochoid.use_dubins_if_low_wind = true;
//         Path path_numerical = trochoid.getTrochoidNumerical();
//         EXPECT_TRUE(path_numerical.size() != 0);
//         double path_length_numerical = trochoids::Trochoid::get_length(path_numerical);

//         // In low wind won't use dubins 
//         trochoid.use_dubins_if_low_wind = false;
//         trochoid.use_trochoid_classification = false;
//         trochoid.use_Chebyshev = true;
//         Path path_numerical_no_dubins = trochoid.getTrochoidNumerical();
//         EXPECT_TRUE(path_numerical_no_dubins.size() != 0);
//         double path_length_numerical_no_dubins = trochoids::Trochoid::get_length(path_numerical_no_dubins);

//         // It is using no dubins like above, but uses classification and analytical methods
//         trochoid.use_dubins_if_low_wind = false;
//         trochoid.use_trochoid_classification = true;
//         trochoid.use_Chebyshev = false;
//         Path path_no_dubins = trochoid.getTrochoid();
//         EXPECT_TRUE(path_no_dubins.size() != 0);
//         double path_length_no_dubins = trochoids::Trochoid::get_length(path_no_dubins);

//         bool one_and_two_match = (abs(path_length/path_length_numerical - 1.0) < 0.05);
//         bool two_and_three_match = (abs(path_length_numerical/path_length_numerical_no_dubins - 1.0) < 0.05);
//         bool three_and_four_match = (abs(path_length_numerical_no_dubins/path_length_no_chebyshev - 1.0) < 0.05);
//         bool four_and_five_match = (abs(path_length_no_chebyshev/path_length_no_dubins - 1.0) < 0.05);

//         EXPECT_TRUE(one_and_two_match);
//         EXPECT_TRUE(two_and_three_match);
//         EXPECT_TRUE(three_and_four_match);
//         EXPECT_TRUE(four_and_five_match);
//         if (!one_and_two_match || !two_and_three_match || !three_and_four_match || !four_and_five_match)
//         {
//             std::cout << "Path length: " << path_length << std::endl;
//             std::cout << "Path length numerical: " << path_length_numerical << std::endl;
//             std::cout << "Path length numerical no dubins: " << path_length_numerical_no_dubins << std::endl;
//             std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
//             std::cout << "Path length no dubins: " << path_length_no_dubins << std::endl;
//             std::cout << "Start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
//             std::cout << "Goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
//             std::cout << "Max Kappa: " << trochoid.problem.max_kappa << std::endl;
//             std::cout << "Wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << std::endl;
//         }
//     }
// }

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
    // Path length no chebyshev: 104.138
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
    // Path length no chebyshev: 2871.01
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


TEST(TestChebyshev, DISABLED_unit_test_edge_cases5){
    double desired_speed = 50;
    double max_kappa = 0.00144849;

    trochoids::Trochoid trochoid;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {0, 0, 0};


    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    trochoid.problem.max_kappa = 0.00144849;
    trochoid.problem.X0 = {0, 0, 0};
    trochoid.problem.Xf = {0, 200, 2.63217};

    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    trochoid.include_BBB = true;
    Path path = trochoid.getTrochoid();
    EXPECT_TRUE(path.size() != 0);
    double path_length = trochoids::Trochoid::get_length(path);

    std::ofstream myfile2;

    myfile2.open("/ws/src/trochoids/test/data/finish2.csv");
    myfile2 << "x,y,z" << std::endl; 
    for (int i = 0; i < path.size(); i++) {
        myfile2 << std::get<0>(path[i]) << ","
            << std::get<1>(path[i]) << ","
            << std::get<2>(path[i]) << std::endl;
    }
    myfile2.close();

    // Check without Chebyshev (this just uses dubins in no wind)
    trochoid.use_Chebyshev = false;
    Path path_no_chebyshev = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_no_chebyshev.size() != 0);
    double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

    // Check without using any classification or analytical methods (this just uses dubins in no wind)
    trochoid.use_Chebyshev = true;
    trochoid.use_dubins_if_low_wind = true;
    Path path_numerical = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_numerical.size() != 0);
    double path_length_numerical = trochoids::Trochoid::get_length(path_numerical);

    // In low wind won't use dubins 
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = false;
    trochoid.use_Chebyshev = true;
    Path path_numerical_no_dubins = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_numerical_no_dubins.size() != 0);
    double path_length_numerical_no_dubins = trochoids::Trochoid::get_length(path_numerical_no_dubins);

    // It is using no dubins like above, but uses classification and analytical methods
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;
    trochoid.include_BBB = true;
    Path path_no_dubins = trochoid.getTrochoid();
    EXPECT_TRUE(path_no_dubins.size() != 0);
    double path_length_no_dubins = trochoids::Trochoid::get_length(path_no_dubins);


    std::ofstream myfile;

    myfile.open("/ws/src/trochoids/test/data/finish.csv");
    myfile << "x,y,z" << std::endl; 
    for (int i = 0; i < path_no_dubins.size(); i++) {
        myfile << std::get<0>(path_no_dubins[i]) << ","
            << std::get<1>(path_no_dubins[i]) << ","
            << std::get<2>(path_no_dubins[i]) << std::endl;
    }
    myfile.close();



    bool one_and_two_match = (abs(path_length/path_length_numerical - 1.0) < 0.05);
    bool two_and_three_match = (abs(path_length_numerical/path_length_numerical_no_dubins - 1.0) < 0.05);
    bool three_and_four_match = (abs(path_length_numerical_no_dubins/path_length_no_chebyshev - 1.0) < 0.05);
    bool four_and_five_match = (abs(path_length_no_chebyshev/path_length_no_dubins - 1.0) < 0.05);

    EXPECT_TRUE(one_and_two_match);
    EXPECT_TRUE(two_and_three_match);
    EXPECT_TRUE(three_and_four_match);
    EXPECT_TRUE(four_and_five_match);
    if (!one_and_two_match || !two_and_three_match || !three_and_four_match || !four_and_five_match)
    {
        std::cout << "Path length: " << path_length << std::endl;
        std::cout << "Path length numerical: " << path_length_numerical << std::endl;
        std::cout << "Path length numerical no dubins: " << path_length_numerical_no_dubins << std::endl;
        std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
        std::cout << "Path length no dubins: " << path_length_no_dubins << std::endl;
        std::cout << "Start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
        std::cout << "Goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
        std::cout << "Max Kappa: " << trochoid.problem.max_kappa << std::endl;
        std::cout << "Wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << ", " << trochoid.problem.wind[2] << std::endl;
    }
}

TEST(TestChebyshev, DISABLED_unit_test_edge_cases6){
    double desired_speed = 20;
    double max_kappa = .1;

    trochoids::Trochoid trochoid;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {0, 0, 0};


    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    trochoid.problem.max_kappa = 0.2832 / 20;
    trochoid.problem.X0 = {0, -200, 0};
    trochoid.problem.Xf = {0, -180, M_PI};

    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    trochoid.include_BBB = true;
    Path path = trochoid.getTrochoid();
    EXPECT_TRUE(path.size() != 0);
    double path_length = trochoids::Trochoid::get_length(path);
    std::ofstream myfile2;

    myfile2.open("/ws/src/trochoids/test/data/finish2.csv");
    myfile2 << "x,y,z" << std::endl; 
    for (int i = 0; i < path.size(); ++i) {
        myfile2 << std::get<0>(path[i]) << ","
            << std::get<1>(path[i]) << ","
            << std::get<2>(path[i]) << std::endl;
    }
    myfile2.close();



    // Check without Chebyshev (this just uses dubins in no wind)
    trochoid.use_Chebyshev = false;
    Path path_no_chebyshev = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_no_chebyshev.size() != 0);
    double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

    // Check without using any classification or analytical methods (this just uses dubins in no wind)
    trochoid.use_Chebyshev = true;
    trochoid.use_dubins_if_low_wind = true;
    Path path_numerical = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_numerical.size() != 0);
    double path_length_numerical = trochoids::Trochoid::get_length(path_numerical);
    
    // In low wind won't use dubins 
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = false;
    trochoid.use_Chebyshev = true;
    trochoid.include_BBB = true;
    Path path_numerical_no_dubins = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_numerical_no_dubins.size() != 0);
    double path_length_numerical_no_dubins = trochoids::Trochoid::get_length(path_numerical_no_dubins);

    // It is using no dubins like above, but uses classification and analytical methods
    trochoid.use_dubins_if_low_wind = false;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;
    trochoid.include_BBB = true;
    Path path_no_dubins = trochoid.getTrochoid();
    EXPECT_TRUE(path_no_dubins.size() != 0);
    double path_length_no_dubins = trochoids::Trochoid::get_length(path_no_dubins);


    std::ofstream myfile;

    myfile.open("/ws/src/trochoids/test/data/finish.csv");
    myfile << "x,y,z" << std::endl; 
    for (int i = 0; i < path_no_dubins.size(); i++) {
        myfile << std::get<0>(path_no_dubins[i]) << ","
            << std::get<1>(path_no_dubins[i]) << ","
            << std::get<2>(path_no_dubins[i]) << std::endl;
    }
    myfile.close();



    bool one_and_two_match = (abs(path_length/path_length_numerical - 1.0) < 0.05);
    bool two_and_three_match = (abs(path_length_numerical/path_length_numerical_no_dubins - 1.0) < 0.05);
    bool three_and_four_match = (abs(path_length_numerical_no_dubins/path_length_no_chebyshev - 1.0) < 0.05);
    bool four_and_five_match = (abs(path_length_no_chebyshev/path_length_no_dubins - 1.0) < 0.05);

    EXPECT_TRUE(one_and_two_match);
    EXPECT_TRUE(two_and_three_match);
    EXPECT_TRUE(three_and_four_match);
    EXPECT_TRUE(four_and_five_match);
    if (!one_and_two_match || !two_and_three_match || !three_and_four_match || !four_and_five_match)
    {
        std::cout << "Path length: " << path_length << std::endl;
        std::cout << "Path length numerical: " << path_length_numerical << std::endl;
        std::cout << "Path length numerical no dubins: " << path_length_numerical_no_dubins << std::endl;
        std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
        std::cout << "Path length no dubins: " << path_length_no_dubins << std::endl;
        std::cout << "Start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
        std::cout << "Goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
        std::cout << "Max Kappa: " << trochoid.problem.max_kappa << std::endl;
        std::cout << "Wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << ", " << trochoid.problem.wind[2] << std::endl;
    }
}


TEST(TestChebyshev, compare_with_BBB_no_wind){
    double desired_speed = 50;
    double max_kappa = .1;

    trochoids::Trochoid trochoid;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {0, 0, 0};

    

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-100, 100);
    std::uniform_real_distribution<> kappaRange(0.001, max_kappa);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    for (int i = 0; i < 1; i++)
    {   
        if(i % 1000 == 0 && i != 0)
            std::cout << "Iteration number: " << i << std::endl;

        trochoid.problem.max_kappa = kappaRange(gen);
        trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};
        // trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
        // trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};

        trochoid.use_dubins_if_low_wind = true;
        trochoid.use_trochoid_classification = true;
        trochoid.use_Chebyshev = true;
        trochoid.include_BBB = true;
        Path path = trochoid.getTrochoid();
        EXPECT_TRUE(path.size() != 0);
        double path_length = trochoids::Trochoid::get_length(path);
        // std::ofstream myfile2;

        // myfile2.open("/ws/src/trochoids/test/data/finish2.csv");
        // myfile2 << "x,y,z" << std::endl; 
        // for (int i = 0; i < path.size(); i++) {
        //     myfile2 << std::get<0>(path[i]) << ","
        //         << std::get<1>(path[i]) << ","
        //         << std::get<2>(path[i]) << std::endl;
        // }
        // myfile2.close();

        // Check without Chebyshev (this just uses dubins in no wind)
        trochoid.use_Chebyshev = false;
        Path path_no_chebyshev = trochoid.getTrochoidNumerical();
        EXPECT_TRUE(path_no_chebyshev.size() != 0);
        double path_length_no_chebyshev = trochoids::Trochoid::get_length(path_no_chebyshev);

        // Check without using any classification or analytical methods (this just uses dubins in no wind)
        trochoid.use_Chebyshev = true;
        trochoid.use_dubins_if_low_wind = true;
        Path path_numerical = trochoid.getTrochoidNumerical();
        EXPECT_TRUE(path_numerical.size() != 0);
        double path_length_numerical = trochoids::Trochoid::get_length(path_numerical);

        // In low wind won't use dubins 
        trochoid.use_dubins_if_low_wind = false;
        trochoid.use_trochoid_classification = false;
        trochoid.use_Chebyshev = true;
        Path path_numerical_no_dubins = trochoid.getTrochoidNumerical();
        EXPECT_TRUE(path_numerical_no_dubins.size() != 0);
        double path_length_numerical_no_dubins = trochoids::Trochoid::get_length(path_numerical_no_dubins);

        // It is using no dubins like above, but uses classification and analytical methods
        trochoid.use_dubins_if_low_wind = false;
        trochoid.use_trochoid_classification = true;
        trochoid.use_Chebyshev = false;
        trochoid.include_BBB = true;
        Path path_no_dubins = trochoid.getTrochoid();
        EXPECT_TRUE(path_no_dubins.size() != 0);
        double path_length_no_dubins = trochoids::Trochoid::get_length(path_no_dubins);
        
        // std::ofstream myfile;

        // myfile.open("/ws/src/trochoids/test/data/finish.csv");
        // myfile << "x,y,z" << std::endl; 
        // for (int i = 0; i < path_no_dubins.size(); i++) {
        //     myfile << std::get<0>(path_no_dubins[i]) << ","
        //         << std::get<1>(path_no_dubins[i]) << ","
        //         << std::get<2>(path_no_dubins[i]) << std::endl;
        // }
        // myfile.close();

        bool one_and_two_match = (abs(path_length/path_length_numerical - 1.0) < 0.05);
        bool two_and_three_match = (abs(path_length_numerical/path_length_numerical_no_dubins - 1.0) < 0.05);
        bool three_and_four_match = (abs(path_length_numerical_no_dubins/path_length_no_chebyshev - 1.0) < 0.05);
        bool four_and_five_match = (abs(path_length_no_chebyshev/path_length_no_dubins - 1.0) < 0.05);

        EXPECT_TRUE(one_and_two_match);
        EXPECT_TRUE(two_and_three_match);
        EXPECT_TRUE(three_and_four_match);
        EXPECT_TRUE(four_and_five_match);
        if (!one_and_two_match || !two_and_three_match || !three_and_four_match || !four_and_five_match)
        {
            std::cout << "Path length: " << path_length << std::endl;
            std::cout << "Path length numerical: " << path_length_numerical << std::endl;
            std::cout << "Path length numerical no dubins: " << path_length_numerical_no_dubins << std::endl;
            std::cout << "Path length no chebyshev: " << path_length_no_chebyshev << std::endl;
            std::cout << "Path length no dubins: " << path_length_no_dubins << std::endl;
            std::cout << "Start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
            std::cout << "Goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
            std::cout << "Max Kappa: " << trochoid.problem.max_kappa << std::endl;
            std::cout << "Wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << std::endl;
        }
    }
}

TEST(TestChebyshev, unit_test_edge_cases8){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 6841.2
    // Path length numerical: 6841.2
    // Path length numerical no dubins: 6841.2
    // Path length no chebyshev: 2408.94
    // Start: 971.444, -980.626, 0.896657
    // Goal: 291.643, 761.934, 0.481008
    // Max Kappa: 0.00146197
    // Wind: -7.27171, -9.64916
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {-7.27171, -9.64916, 0};


    trochoid.problem.max_kappa = 0.00146197;
    trochoid.problem.X0 = {971.444, -980.626, 0.896657};
    trochoid.problem.Xf = {291.643, 761.934, 0.481008};

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

TEST(TestChebyshev, unit_test_edge_cases9){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 3449.61
    // Path length numerical: 3449.61
    // Path length numerical no dubins: 3449.61
    // Path length chebyshev: 1171.93
    // Start: -941.678, 707.667, 0.296634
    // Goal: -575.211, -226.426, 0.328045
    // Max Kappa: 0.00302947
    // Wind: -19.4511, -10.5802
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {-19.4511, -10.5802, 0};
    trochoid.problem.max_kappa = 0.00302947;
    trochoid.problem.X0 = {-941.678, 707.667, 0.296634};
    trochoid.problem.Xf = {-575.211, -226.426, 0.328045};

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

TEST(TestChebyshev, unit_test_edge_cases10){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 3624.59
    // Path length numerical: 3624.59
    // Path length numerical no dubins: 3624.59
    // Path length no chebyshev: 1718.07
    // Start: -844.459, -552.431, 1.41651
    // Goal: -553.007, 274.343, 3.1377
    // Max Kappa: 0.00352732
    // Wind: 10.4918, 31.7216
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {10.4918, 31.7216, 0};
    trochoid.problem.max_kappa = 0.00352732;
    trochoid.problem.X0 = {-844.459, -552.431, 1.41651};
    trochoid.problem.Xf = {-553.007, 274.343, 3.1377};

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

TEST(TestChebyshev, unit_test_edge_cases11){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 3276.86
    // Path length numerical: 3276.86
    // Path length numerical no dubins: 3276.86
    // Path length no chebyshev: 2159.79
    // Start: -563.835, 873.437, 0.0634033
    // Goal: -561.61, 306.445, 3.99108
    // Max Kappa: 0.00222055
    // Wind: 18.505, 4.89311
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {18.505, 4.89311, 0};
    trochoid.problem.max_kappa = 0.00222055;
    trochoid.problem.X0 = {-563.835, 873.437, 0.0634033};
    trochoid.problem.Xf = {-561.61, 306.445, 3.99108};

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

TEST(TestChebyshev, unit_test_edge_cases12){
    // Value of: three_and_four_match
    //   Actual: false
    // Expected: true
    // Path length: 342.005
    // Path length numerical: 342.005
    // Path length numerical no dubins: 342.005
    // Path length no chebyshev: 428.632
    // Start: -959.848, 527.061, 1.31217
    // Goal: -811.3, 670.211, 3.86527
    // Max Kappa: 0.0153616
    // Wind: 19.8186, 34.7972
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {19.8186, 34.7972, 0};
    trochoid.problem.max_kappa = 0.0153616;
    trochoid.problem.X0 = {-959.848, 527.061, 1.31217};
    trochoid.problem.Xf = {-811.3, 670.211, 3.86527};

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

TEST(TestChebyshev, root_solver_1d_methods_match_newton_without_chebyshev){
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {19.8186, 34.7972, 0};
    trochoid.problem.max_kappa = 0.0153616;
    trochoid.problem.X0 = {-959.848, 527.061, 1.31217};
    trochoid.problem.Xf = {-811.3, 670.211, 3.86527};
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = false;

    trochoid.root_solve_1d_method = trochoids::Trochoid::RootSolve1DMethod::NEWTON_RAPHSON;
    Path path_newton = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_newton.size() != 0);
    double path_length_newton = trochoids::Trochoid::get_length(path_newton);

    trochoid.root_solve_1d_method = trochoids::Trochoid::RootSolve1DMethod::BRACKETED_BISECTION;
    Path path_bracketed = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_bracketed.size() != 0);
    double path_length_bracketed = trochoids::Trochoid::get_length(path_bracketed);

    trochoid.root_solve_1d_method = trochoids::Trochoid::RootSolve1DMethod::NON_ROBUST_BRENT;
    Path path_non_robust_brent = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_non_robust_brent.size() != 0);
    double path_length_non_robust_brent = trochoids::Trochoid::get_length(path_non_robust_brent);

    trochoid.root_solve_1d_method = trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT;
    Path path_global_brent = trochoid.getTrochoidNumerical();
    EXPECT_TRUE(path_global_brent.size() != 0);
    double path_length_global_brent = trochoids::Trochoid::get_length(path_global_brent);

    bool bracketed_matches_newton = abs(path_length_newton / path_length_bracketed - 1.0) < 0.05;
    bool non_robust_brent_matches_newton = abs(path_length_newton / path_length_non_robust_brent - 1.0) < 0.05;
    bool global_brent_matches_newton = abs(path_length_newton / path_length_global_brent - 1.0) < 0.05;
    if (!bracketed_matches_newton || !non_robust_brent_matches_newton || !global_brent_matches_newton)
    {
        std::cout << "Path length newton: " << path_length_newton << std::endl;
        std::cout << "Path length bracketed: " << path_length_bracketed << std::endl;
        std::cout << "Path length non robust brent: " << path_length_non_robust_brent << std::endl;
        std::cout << "Path length global brent: " << path_length_global_brent << std::endl;
    }
    EXPECT_TRUE(bracketed_matches_newton);
    EXPECT_TRUE(non_robust_brent_matches_newton);
    EXPECT_TRUE(global_brent_matches_newton);
}

static double get_numerical_path_length_for_mode(trochoids::Trochoid trochoid,
                                                  bool use_chebyshev,
                                                  trochoids::Trochoid::RootSolve1DMethod method)
{
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = use_chebyshev;
    trochoid.root_solve_1d_method = method;
    Path path = trochoid.getTrochoidNumerical();
    if (path.size() == 0)
    {
        return -1.0;
    }
    return trochoids::Trochoid::get_length(path);
}

TEST(TestChebyshev, root_solver_1d_all_methods_match_fixed_cases){
    struct CaseInput {
        double v;
        std::vector<double> wind;
        double max_kappa;
        std::vector<double> x0;
        std::vector<double> xf;
    };

    const std::vector<CaseInput> cases = {
        {50, {19.8186, 34.7972, 0}, 0.0153616, {-959.848, 527.061, 1.31217}, {-811.3, 670.211, 3.86527}},
        {50, {18.505, 4.89311, 0}, 0.00222055, {-563.835, 873.437, 0.0634033}, {-561.61, 306.445, 3.99108}},
        {50, {15.0, -7.0, 0}, 0.01, {0.0, 0.0, 1.5707}, {1000.0, 1000.0, 1.5707}}
    };

    for (size_t i = 0; i < cases.size(); ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = cases[i].v;
        trochoid.problem.wind = cases[i].wind;
        trochoid.problem.max_kappa = cases[i].max_kappa;
        trochoid.problem.X0 = cases[i].x0;
        trochoid.problem.Xf = cases[i].xf;

        const double len_cheb = get_numerical_path_length_for_mode(
            trochoid, true, trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT);
        const double len_newton = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::NEWTON_RAPHSON);
        const double len_bracketed = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::BRACKETED_BISECTION);
        const double len_non_robust_brent = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::NON_ROBUST_BRENT);
        const double len_global_brent = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT);

        EXPECT_GT(len_cheb, 0.0);
        EXPECT_GT(len_newton, 0.0);
        EXPECT_GT(len_bracketed, 0.0);
        EXPECT_GT(len_non_robust_brent, 0.0);
        EXPECT_GT(len_global_brent, 0.0);

        if (len_cheb <= 0.0 || len_newton <= 0.0 || len_bracketed <= 0.0 || len_non_robust_brent <= 0.0 || len_global_brent <= 0.0)
        {
            std::cout << "Case " << i << " failed to produce valid paths in one or more methods." << std::endl;
            continue;
        }

        const double max_len = std::max(std::max(len_cheb, len_newton), std::max(len_bracketed, std::max(len_non_robust_brent, len_global_brent)));
        const double min_len = std::min(std::min(len_cheb, len_newton), std::min(len_bracketed, std::min(len_non_robust_brent, len_global_brent)));
        const bool all_match = (max_len / min_len - 1.0) < 0.05;

        if (!all_match)
        {
            std::cout << "Case " << i << " mismatch:" << std::endl;
            std::cout << "  chebyshev: " << len_cheb << std::endl;
            std::cout << "  newton: " << len_newton << std::endl;
            std::cout << "  bracketed: " << len_bracketed << std::endl;
            std::cout << "  non robust brent: " << len_non_robust_brent << std::endl;
            std::cout << "  global brent: " << len_global_brent << std::endl;
        }
        EXPECT_TRUE(all_match);
    }
}

static void run_1d_seeded_random_comparison(int num_cases, int seed)
{
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> disRange(-1000.0, 1000.0);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> disWind(-25.0, 25.0);
    std::uniform_real_distribution<> disKappa(0.003, 0.03);

    for (int i = 0; i < num_cases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50;
        trochoid.problem.wind = {disWind(gen), disWind(gen), 0};
        trochoid.problem.max_kappa = disKappa(gen);
        trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};

        const double len_cheb = get_numerical_path_length_for_mode(
            trochoid, true, trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT);
        const double len_newton = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::NEWTON_RAPHSON);
        const double len_bracketed = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::BRACKETED_BISECTION);
        const double len_non_robust_brent = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::NON_ROBUST_BRENT);
        const double len_global_brent = get_numerical_path_length_for_mode(
            trochoid, false, trochoids::Trochoid::RootSolve1DMethod::GLOBAL_BRENT);

        EXPECT_GT(len_cheb, 0.0);
        EXPECT_GT(len_newton, 0.0);
        EXPECT_GT(len_bracketed, 0.0);
        EXPECT_GT(len_non_robust_brent, 0.0);
        EXPECT_GT(len_global_brent, 0.0);

        if (len_cheb <= 0.0 || len_newton <= 0.0 || len_bracketed <= 0.0 || len_non_robust_brent <= 0.0 || len_global_brent <= 0.0)
        {
            std::cout << "Random case " << i << " failed to produce valid paths in one or more methods." << std::endl;
            continue;
        }

        const double max_len = std::max(std::max(len_cheb, len_newton), std::max(len_bracketed, std::max(len_non_robust_brent, len_global_brent)));
        const double min_len = std::min(std::min(len_cheb, len_newton), std::min(len_bracketed, std::min(len_non_robust_brent, len_global_brent)));
        const bool all_match = (max_len / min_len - 1.0) < 0.08;

        if (!all_match)
        {
            std::cout << "Random case " << i << " mismatch:" << std::endl;
            std::cout << "  chebyshev: " << len_cheb << std::endl;
            std::cout << "  newton: " << len_newton << std::endl;
            std::cout << "  bracketed: " << len_bracketed << std::endl;
            std::cout << "  non robust brent: " << len_non_robust_brent << std::endl;
            std::cout << "  global brent: " << len_global_brent << std::endl;
            std::cout << "  start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
            std::cout << "  goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
            std::cout << "  wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << std::endl;
            std::cout << "  max_kappa: " << trochoid.problem.max_kappa << std::endl;
        }
        EXPECT_TRUE(all_match);
    }
}

TEST(TestChebyshev, root_solver_1d_all_methods_match_seeded_random_cases){
    run_1d_seeded_random_comparison(100, 42);
}

TEST(TestChebyshev, DISABLED_root_solver_1d_all_methods_match_seeded_random_cases_extended){
    run_1d_seeded_random_comparison(2500, 42);
}

static double get_numerical_path_length_for_2d_mode(trochoids::Trochoid trochoid,
                                                     trochoids::Trochoid::RootSolve2DMethod method);
static double get_empirical_best_2d_length(trochoids::Trochoid trochoid);

TEST(TestChebyshev, root_solver_2d_methods_match_high_density_fixed_cases){
    struct CaseInput {
        double v;
        std::vector<double> wind;
        double max_kappa;
        std::vector<double> x0;
        std::vector<double> xf;
    };

    const std::vector<CaseInput> cases = {
        {50, {12.0, -18.0, 0}, 0.008, {-200.0, 300.0, 1.1}, {450.0, -250.0, 4.2}},
        {50, {20.0, 15.0, 0}, 0.006, {-100.0, -100.0, 0.5}, {600.0, 350.0, 3.7}}
    };

    for (size_t i = 0; i < cases.size(); ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = cases[i].v;
        trochoid.problem.wind = cases[i].wind;
        trochoid.problem.max_kappa = cases[i].max_kappa;
        trochoid.problem.X0 = cases[i].x0;
        trochoid.problem.Xf = cases[i].xf;
        trochoid.include_BBB = true;
        trochoid.root_solve_2d_grid_samples = 720;
        trochoid.root_solve_2d_chebyshev_samples = 65;

        const double len_newton_grid = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double len_cheb_grid = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
        const double len_oracle_best = get_empirical_best_2d_length(trochoid);

        EXPECT_GT(len_newton_grid, 0.0);
        EXPECT_GT(len_cheb_grid, 0.0);
        EXPECT_GT(len_oracle_best, 0.0);
        if (len_newton_grid <= 0.0 || len_cheb_grid <= 0.0)
        {
            std::cout << "2D high-density case " << i << " invalid path:"
                      << " len_newton_grid=" << len_newton_grid
                      << " len_cheb_grid=" << len_cheb_grid << std::endl;
            continue;
        }
        const double newton_gap = len_newton_grid / len_oracle_best - 1.0;
        const double cheb_gap = len_cheb_grid / len_oracle_best - 1.0;

        std::cout << "2D high-density case " << i << std::endl;
        std::cout << "  len_newton_grid: " << len_newton_grid << std::endl;
        std::cout << "  len_cheb_grid: " << len_cheb_grid << std::endl;
        std::cout << "  len_oracle_best: " << len_oracle_best << std::endl;
        std::cout << "  newton_gap_to_oracle: " << newton_gap << std::endl;
        std::cout << "  cheb_gap_to_oracle: " << cheb_gap << std::endl;
        std::cout << "  better_of_two: " << ((len_cheb_grid < len_newton_grid) ? "chebyshev-grid-newton" : "newton-grid") << std::endl;

        EXPECT_TRUE(newton_gap < 0.20);
        EXPECT_TRUE(cheb_gap < 0.20);
        EXPECT_TRUE(std::min(newton_gap, cheb_gap) < 0.05);
    }
}

TEST(TestChebyshev, root_solver_2d_methods_match_boundary_stress){
    trochoids::Trochoid trochoid;
    trochoid.problem.v = 50;
    trochoid.problem.wind = {24.0, -22.0, 0};
    trochoid.problem.max_kappa = 0.0045;
    trochoid.problem.X0 = {-780.0, 760.0, 0.05};
    trochoid.problem.Xf = {790.0, -740.0, 6.18};
    trochoid.include_BBB = true;

    trochoid.root_solve_2d_grid_samples = 900;
    trochoid.root_solve_2d_chebyshev_samples = 81;
    const double len_newton_dense = get_numerical_path_length_for_2d_mode(
        trochoid, trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
    const double len_cheb_dense = get_numerical_path_length_for_2d_mode(
        trochoid, trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);

    EXPECT_GT(len_newton_dense, 0.0);
    EXPECT_GT(len_cheb_dense, 0.0);
    if (len_newton_dense > 0.0 && len_cheb_dense > 0.0)
    {
        const double ratio_error = std::abs(len_newton_dense / len_cheb_dense - 1.0);
        if (!(ratio_error < 0.07))
        {
            std::cout << "2D boundary-stress mismatch" << std::endl;
            std::cout << "  len_newton_dense: " << len_newton_dense << std::endl;
            std::cout << "  len_cheb_dense: " << len_cheb_dense << std::endl;
            std::cout << "  ratio_error: " << ratio_error << std::endl;
            std::cout << "  start: " << trochoid.problem.X0[0] << ", "
                      << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
            std::cout << "  goal: " << trochoid.problem.Xf[0] << ", "
                      << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
            std::cout << "  wind: " << trochoid.problem.wind[0] << ", "
                      << trochoid.problem.wind[1] << std::endl;
            std::cout << "  max_kappa: " << trochoid.problem.max_kappa << std::endl;
            std::cout << "  root_solve_2d_grid_samples: " << trochoid.root_solve_2d_grid_samples << std::endl;
            std::cout << "  root_solve_2d_chebyshev_samples: " << trochoid.root_solve_2d_chebyshev_samples << std::endl;
        }
        EXPECT_TRUE(ratio_error < 0.07);
    }
}

static double get_numerical_path_length_for_2d_mode(trochoids::Trochoid trochoid,
                                                     trochoids::Trochoid::RootSolve2DMethod method)
{
    trochoid.use_dubins_if_low_wind = true;
    trochoid.use_trochoid_classification = true;
    trochoid.use_Chebyshev = true;
    trochoid.include_BBB = true;
    trochoid.root_solve_2d_method = method;
    Path path = trochoid.getTrochoidNumerical();
    if (path.size() == 0)
    {
        return -1.0;
    }
    return trochoids::Trochoid::get_length(path);
}

static double get_empirical_best_2d_length(trochoids::Trochoid trochoid)
{
    double best = std::numeric_limits<double>::infinity();
    // const std::vector<std::pair<int, int>> configs = {{360, 33}, {720, 65}, {900, 81}};
    const std::vector<std::pair<int, int>> configs = {{360, 33}, {900, 81}};
    const std::vector<trochoids::Trochoid::RootSolve2DMethod> methods = {
        trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID,
        trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON
    };

    for (const auto &cfg : configs)
    {
        trochoid.root_solve_2d_grid_samples = cfg.first;
        trochoid.root_solve_2d_chebyshev_samples = cfg.second;
        for (const auto method : methods)
        {
            const double len = get_numerical_path_length_for_2d_mode(trochoid, method);
            if (len > 0.0 && len < best)
            {
                best = len;
            }
        }
    }

    if (!std::isfinite(best))
    {
        return -1.0;
    }
    return best;
}

TEST(TestChebyshev, root_solver_2d_methods_match_fixed_cases){
    struct CaseInput {
        double v;
        std::vector<double> wind;
        double max_kappa;
        std::vector<double> x0;
        std::vector<double> xf;
    };

    const std::vector<CaseInput> cases = {
        {50, {12.0, -18.0, 0}, 0.008, {-200.0, 300.0, 1.1}, {450.0, -250.0, 4.2}},
        {50, {20.0, 15.0, 0}, 0.006, {-100.0, -100.0, 0.5}, {600.0, 350.0, 3.7}},
        {50, {-15.0, 22.0, 0}, 0.01, {300.0, -450.0, 2.2}, {-500.0, 200.0, 5.4}}
    };

    for (size_t i = 0; i < cases.size(); ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = cases[i].v;
        trochoid.problem.wind = cases[i].wind;
        trochoid.problem.max_kappa = cases[i].max_kappa;
        trochoid.problem.X0 = cases[i].x0;
        trochoid.problem.Xf = cases[i].xf;
        trochoid.root_solve_2d_grid_samples = 360;
        trochoid.root_solve_2d_chebyshev_samples = 33;

        const double len_newton_grid = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double len_cheb_grid = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);

        EXPECT_GT(len_newton_grid, 0.0);
        EXPECT_GT(len_cheb_grid, 0.0);
        if (len_newton_grid <= 0.0 || len_cheb_grid <= 0.0)
        {
            std::cout << "2D case " << i << " failed to produce valid path in one or more methods." << std::endl;
            continue;
        }

        const bool lengths_match = std::abs(len_newton_grid / len_cheb_grid - 1.0) < 0.08;
        if (!lengths_match)
        {
            std::cout << "2D case " << i << " mismatch:" << std::endl;
            std::cout << "  newton grid: " << len_newton_grid << std::endl;
            std::cout << "  chebyshev grid + newton: " << len_cheb_grid << std::endl;
            std::cout << "  ratio_error: " << std::abs(len_newton_grid / len_cheb_grid - 1.0) << std::endl;
            std::cout << "  root_solve_2d_grid_samples: " << trochoid.root_solve_2d_grid_samples << std::endl;
            std::cout << "  root_solve_2d_chebyshev_samples: " << trochoid.root_solve_2d_chebyshev_samples << std::endl;
        }
        EXPECT_TRUE(lengths_match);
    }
}

static void run_2d_seeded_random_comparison(int num_cases, int seed, double tol)
{
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> disRange(-800.0, 800.0);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> disWind(-25.0, 25.0);
    std::uniform_real_distribution<> disKappa(0.004, 0.02);

    for (int i = 0; i < num_cases; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50;
        trochoid.problem.wind = {disWind(gen), disWind(gen), 0};
        trochoid.problem.max_kappa = disKappa(gen);
        trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.root_solve_2d_grid_samples = 360;
        trochoid.root_solve_2d_chebyshev_samples = 33;

        const double len_newton_grid = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double len_cheb_grid = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);

        EXPECT_GT(len_newton_grid, 0.0);
        EXPECT_GT(len_cheb_grid, 0.0);
        if (len_newton_grid <= 0.0 || len_cheb_grid <= 0.0)
        {
            std::cout << "2D random case " << i << " failed to produce valid path in one or more methods." << std::endl;
            continue;
        }

        const bool lengths_match = std::abs(len_newton_grid / len_cheb_grid - 1.0) < tol;
        if (!lengths_match)
        {
            std::cout << "2D random case " << i << " mismatch:" << std::endl;
            std::cout << "  newton grid: " << len_newton_grid << std::endl;
            std::cout << "  chebyshev grid + newton: " << len_cheb_grid << std::endl;
            std::cout << "  ratio_error: " << std::abs(len_newton_grid / len_cheb_grid - 1.0) << std::endl;
            std::cout << "  start: " << trochoid.problem.X0[0] << ", " << trochoid.problem.X0[1] << ", " << trochoid.problem.X0[2] << std::endl;
            std::cout << "  goal: " << trochoid.problem.Xf[0] << ", " << trochoid.problem.Xf[1] << ", " << trochoid.problem.Xf[2] << std::endl;
            std::cout << "  wind: " << trochoid.problem.wind[0] << ", " << trochoid.problem.wind[1] << std::endl;
            std::cout << "  max_kappa: " << trochoid.problem.max_kappa << std::endl;
            std::cout << "  root_solve_2d_grid_samples: " << trochoid.root_solve_2d_grid_samples << std::endl;
            std::cout << "  root_solve_2d_chebyshev_samples: " << trochoid.root_solve_2d_chebyshev_samples << std::endl;
        }
        EXPECT_TRUE(lengths_match);
    }
}

TEST(TestChebyshev, root_solver_2d_methods_match_seeded_random_cases){
    run_2d_seeded_random_comparison(1, 7, 0.10);
}

TEST(TestChebyshev, DISABLED_root_solver_2d_methods_match_seeded_random_cases_extended){
    run_2d_seeded_random_comparison(200, 7, 0.10);
}

TEST(TestChebyshev, root_solver_2d_methods_oracle_seeded_random_quick){
    std::mt19937 gen(11);
    std::uniform_real_distribution<> disRange(-800.0, 800.0);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> disWind(-25.0, 25.0);
    std::uniform_real_distribution<> disKappa(0.004, 0.02);

    for (int i = 0; i < 2; ++i)
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = 50;
        trochoid.problem.wind = {disWind(gen), disWind(gen), 0};
        trochoid.problem.max_kappa = disKappa(gen);
        trochoid.problem.X0 = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.problem.Xf = {disRange(gen), disRange(gen), disPhi(gen)};
        trochoid.include_BBB = true;
        // trochoid.root_solve_2d_grid_samples = 720;
        // trochoid.root_solve_2d_chebyshev_samples = 65;

        const double len_newton = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::NEWTON_GRID);
        const double len_cheb = get_numerical_path_length_for_2d_mode(
            trochoid, trochoids::Trochoid::RootSolve2DMethod::CHEBYSHEV_GRID_NEWTON);
        const double len_oracle = get_empirical_best_2d_length(trochoid);

        EXPECT_GT(len_newton, 0.0);
        EXPECT_GT(len_cheb, 0.0);
        EXPECT_GT(len_oracle, 0.0);
        if (len_newton <= 0.0 || len_cheb <= 0.0 || len_oracle <= 0.0)
        {
            continue;
        }

        const double newton_gap = len_newton / len_oracle - 1.0;
        const double cheb_gap = len_cheb / len_oracle - 1.0;
        EXPECT_TRUE(newton_gap < 0.20);
        EXPECT_TRUE(cheb_gap < 0.20);
        EXPECT_TRUE(std::min(newton_gap, cheb_gap) < 0.08);
    }
}
 