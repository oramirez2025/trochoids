/*********************************************************************
    The Clear BSD License

    Copyright (c) 2023, AirLab
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

/* Authors: Sagar Sachdev, Brady Moon, Jay Patrikar */

#ifndef TROCHOIDS_TROCHOID_UTILS_H
#define TROCHOIDS_TROCHOID_UTILS_H

#include "trochoids/trochoids.h"
#include <limits>

#define M_2PI 2.0*M_PI

namespace trochoids
{
struct XYZPsiState
{
    double x;
    double y;
    double z;
    double psi;
};

enum class VerticalPlanningCase
{
    NONE = 0,
    DIRECT_PROFILE = 1,
    FULL_LOOP_EXTENSION = 2,
    PARTIAL_EXTENSION = 3  // Reserved for future case-3 support.
};

struct VerticalConstraints
{
    double max_climb_rate = std::numeric_limits<double>::infinity();
    double max_descent_rate = std::numeric_limits<double>::infinity();
    bool enforce_flight_path_angle = false;
    double max_flight_path_angle_rad = 0.0;
    bool allow_full_loop_extension = false;  // Enables phase-B loop extension in wind.
    int max_full_loops = 0;
};

struct VerticalPlanInfo
{
    bool valid = false;
    bool vertical_feasible = false;
    VerticalPlanningCase case_used = VerticalPlanningCase::NONE;
    double xy_time_sec = 0.0;
    double required_vertical_time_sec = 0.0;
    double added_extension_time_sec = 0.0;
    int loops_added_start = 0;
    int loops_added_end = 0;
    int estimated_full_loops_needed = 0;
};

double WrapTo2Pi(double a1);

double WrapToPi(double a1);

double get_length(const XYZPsiState &s1,
                    const XYZPsiState &s2,
                    const double wind[],
                    double v,
                    double max_kappa);

double get_length(std::vector<XYZPsiState> &path);

bool get_trochoid_path(const XYZPsiState &s1,
                        const XYZPsiState &s2,
                        std::vector<XYZPsiState> &path,
                        const double wind[],
                        double v,
                        double max_kappa,
                        double waypoint_distance = 0);

bool get_trochoid_path_numerical(const XYZPsiState &s1,
                                    const XYZPsiState &s2,
                                    std::vector<XYZPsiState> &path,
                                    const double wind[],
                                    double v,
                                    double max_kappa,
                                    bool exhaustive_solve_only = false,
                                    double waypoint_distance = 0);

bool get_trochoid_path_3d(const XYZPsiState &s1,
                            const XYZPsiState &s2,
                            std::vector<XYZPsiState> &path,
                            const double wind[],
                            double v,
                            double max_kappa,
                            const VerticalConstraints &vertical_constraints,
                            VerticalPlanInfo *plan_info = nullptr,
                            double waypoint_distance = 0);
// Current support:
// 1) DIRECT_PROFILE (case 1): if base XY time can satisfy vertical constraints.
// 2) FULL_LOOP_EXTENSION (case 2): optional full-loop extension with wind drift.
// PARTIAL_EXTENSION (case 3) is not implemented yet.
}  // namespace trochoids

#endif  // TROCHOIDS_TROCHOID_UTILS_H
