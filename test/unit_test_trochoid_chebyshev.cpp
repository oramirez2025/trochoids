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


TEST(TestChebyshev, trochoid_compare_methods_random_wind_varkappa)
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
    trochoid.include_CCC = true;
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
    std::cout << "Total time: " << duration.count() << " ms" << std::endl;
}
