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

#include "trochoids/trochoid_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>

typedef std::vector<std::tuple<double, double, double>> Path;

namespace
{
double safe_norm(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

bool is_finite_state(const trochoids::XYZPsiState &state)
{
    return std::isfinite(state.x) &&
           std::isfinite(state.y) &&
           std::isfinite(state.z) &&
           std::isfinite(state.psi);
}

bool is_valid_xy_request(const trochoids::XYZPsiState &s1,
                         const trochoids::XYZPsiState &s2,
                         const double *wind,
                         double v,
                         double max_kappa,
                         double waypoint_distance)
{
    return wind != nullptr &&
           is_finite_state(s1) &&
           is_finite_state(s2) &&
           std::isfinite(wind[0]) &&
           std::isfinite(wind[1]) &&
           std::isfinite(wind[2]) &&
           std::isfinite(v) &&
           std::isfinite(max_kappa) &&
           std::isfinite(waypoint_distance) &&
           v > EPSILON &&
           max_kappa > EPSILON &&
           waypoint_distance >= 0.0;
}

bool is_valid_vertical_constraints(const trochoids::VerticalConstraints &constraints)
{
    return !std::isnan(constraints.max_climb_rate) &&
           !std::isnan(constraints.max_descent_rate) &&
           std::isfinite(constraints.max_flight_path_angle_rad) &&
           constraints.max_climb_rate >= 0.0 &&
           constraints.max_descent_rate >= 0.0 &&
           constraints.max_flight_path_angle_rad >= 0.0 &&
           constraints.max_full_loops >= 0;
}

double interpolation_alpha(size_t index, size_t total_points)
{
    if (total_points <= 1)
    {
        return 1.0;
    }
    return static_cast<double>(index) /
           static_cast<double>(total_points - 1);
}

double effective_vertical_rate_limit(const trochoids::VerticalConstraints &constraints,
                                     bool climbing,
                                     double v)
{
    double vertical_rate_limit = climbing ? constraints.max_climb_rate : constraints.max_descent_rate;
    if (constraints.enforce_flight_path_angle && constraints.max_flight_path_angle_rad > 0.0)
    {
        const double angle_limited_rate = v * std::tan(constraints.max_flight_path_angle_rad);
        vertical_rate_limit = std::min(vertical_rate_limit, angle_limited_rate);
    }
    return vertical_rate_limit;
}

double estimate_path_time_seconds(const Path &path, const double *wind, double v)
{
    if (path.size() < 2)
    {
        return 0.0;
    }

    double total_time = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const double x0 = std::get<0>(path[i]);
        const double y0 = std::get<1>(path[i]);
        const double x1 = std::get<0>(path[i + 1]);
        const double y1 = std::get<1>(path[i + 1]);
        const double psi0 = std::get<2>(path[i]);
        const double psi1 = std::get<2>(path[i + 1]);

        const double ds = safe_norm(x1 - x0, y1 - y0);
        if (ds < EPSILON)
        {
            continue;
        }

        const double psi_avg = trochoids::WrapToPi(0.5 * (psi0 + psi1));
        const double gx = v * std::cos(psi_avg) + wind[0];
        const double gy = v * std::sin(psi_avg) + wind[1];
        const double ground_speed_norm = std::max(safe_norm(gx, gy), EPSILON);
        total_time += ds / ground_speed_norm;
    }

    return total_time;
}

std::vector<double> estimate_cumulative_time(const Path &path, const double *wind, double v)
{
    std::vector<double> cumulative_time(path.size(), 0.0);
    if (path.size() < 2)
    {
        return cumulative_time;
    }

    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const double x0 = std::get<0>(path[i]);
        const double y0 = std::get<1>(path[i]);
        const double x1 = std::get<0>(path[i + 1]);
        const double y1 = std::get<1>(path[i + 1]);
        const double psi0 = std::get<2>(path[i]);
        const double psi1 = std::get<2>(path[i + 1]);

        const double ds = safe_norm(x1 - x0, y1 - y0);
        const double psi_avg = trochoids::WrapToPi(0.5 * (psi0 + psi1));
        const double gx = v * std::cos(psi_avg) + wind[0];
        const double gy = v * std::sin(psi_avg) + wind[1];
        const double ground_speed_norm = std::max(safe_norm(gx, gy), EPSILON);
        const double dt = ds / ground_speed_norm;
        cumulative_time[i + 1] = cumulative_time[i] + dt;
    }

    return cumulative_time;
}

void append_path(Path &dst, const Path &src, bool skip_first_point)
{
    const size_t begin_idx = (skip_first_point && !src.empty()) ? 1 : 0;
    for (size_t i = begin_idx; i < src.size(); ++i)
    {
        dst.push_back(src[i]);
    }
}

Path generate_full_loop_path(double x0,
                             double y0,
                             double psi0,
                             double duration,
                             const double *wind,
                             double v,
                             double max_kappa,
                             double waypoint_distance)
{
    Path loop_path;
    if (duration <= EPSILON)
    {
        loop_path.push_back(std::make_tuple(x0, y0, trochoids::WrapTo2Pi(psi0)));
        return loop_path;
    }

    const double omega = v * max_kappa;
    if (omega <= EPSILON || v <= EPSILON)
    {
        // Fallback to straight drift if turn-rate is degenerate.
        loop_path.push_back(std::make_tuple(x0, y0, trochoids::WrapTo2Pi(psi0)));
        loop_path.push_back(std::make_tuple(x0 + wind[0] * duration, y0 + wind[1] * duration, trochoids::WrapTo2Pi(psi0)));
        return loop_path;
    }

    double dt = M_PI / (180.0 * omega);
    if (waypoint_distance > 0.01)
    {
        dt = waypoint_distance / v;
    }
    dt = std::max(dt, 1e-3);

    double x = x0;
    double y = y0;
    double t = 0.0;
    loop_path.push_back(std::make_tuple(x, y, trochoids::WrapTo2Pi(psi0)));
    while (t + dt < duration)
    {
        const double psi = trochoids::WrapTo2Pi(psi0 + omega * t);
        x += (v * std::cos(psi) + wind[0]) * dt;
        y += (v * std::sin(psi) + wind[1]) * dt;
        t += dt;
        const double psi_next = trochoids::WrapTo2Pi(psi0 + omega * t);
        loop_path.push_back(std::make_tuple(x, y, psi_next));
    }

    const double dt_last = duration - t;
    if (dt_last > EPSILON)
    {
        const double psi = trochoids::WrapTo2Pi(psi0 + omega * t);
        x += (v * std::cos(psi) + wind[0]) * dt_last;
        y += (v * std::sin(psi) + wind[1]) * dt_last;
    }

    const double psi_final = trochoids::WrapTo2Pi(psi0 + omega * duration);
    const double x_final = x0 + wind[0] * duration;
    const double y_final = y0 + wind[1] * duration;
    loop_path.push_back(std::make_tuple(x_final, y_final, psi_final));
    return loop_path;
}
}  // namespace

double trochoids::WrapTo2Pi(double a1)
{
  return a1 - 2*M_PI * floor(a1 / (2*M_PI));
}

double trochoids::WrapToPi(double a1)
{
  int m = static_cast<int>(a1 / (2*M_PI));
  a1 = a1 - m*2*M_PI;
  if (a1 > M_PI)
    a1 -= 2.0*M_PI;
  else if (a1 < -M_PI)
    a1 +=2.0*M_PI;
  return a1;
}

bool trochoids::get_trochoid_path_numerical(const XYZPsiState &s1,
                                            const XYZPsiState &s2,
                                            std::vector<XYZPsiState> &extended_path_out,
                                            const double *wind,
                                            double v,
                                            double max_kappa,
                                            bool exhaustive_solve_only,
                                            double waypoint_distance)
{
    if (!is_valid_xy_request(s1, s2, wind, v, max_kappa, waypoint_distance))
    {
        return false;
    }

    trochoids::Trochoid trochoid;
    trochoid.problem.v = v;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {s1.x, s1.y, s1.psi};
    trochoid.problem.Xf = {s2.x, s2.y, s2.psi};
    trochoid.use_dubins_if_low_wind = !exhaustive_solve_only;
    Path path = trochoid.getTrochoidNumerical(waypoint_distance);
    if (path.size() == 0)
    {
        return false;
    }
    XYZPsiState new_state;

    for (int i = 0; i < path.size(); i++)
    {
        new_state.x = std::get<0>(path[i]);
        new_state.y = std::get<1>(path[i]);
        new_state.psi = std::get<2>(path[i]);
        new_state.z = s1.z + interpolation_alpha(i, path.size()) * (s2.z - s1.z);

        extended_path_out.push_back(new_state);
    }
    return true;
}

bool trochoids::get_trochoid_path_3d(const XYZPsiState &s1,
                                    const XYZPsiState &s2,
                                    std::vector<XYZPsiState> &extended_path_out,
                                    const double *wind,
                                    double v,
                                    double max_kappa,
                                    const VerticalConstraints &vertical_constraints,
                                    VerticalPlanInfo *plan_info,
                                    double waypoint_distance)
{
    VerticalPlanInfo local_info;
    local_info.valid = false;
    local_info.vertical_feasible = false;
    local_info.case_used = VerticalPlanningCase::NONE;

    if (!is_valid_xy_request(s1, s2, wind, v, max_kappa, waypoint_distance) ||
        !is_valid_vertical_constraints(vertical_constraints))
    {
        if (plan_info != nullptr)
        {
            *plan_info = local_info;
        }
        return false;
    }

    const double dz = s2.z - s1.z;
    const bool climbing = dz >= 0.0;
    const double vertical_rate_limit = effective_vertical_rate_limit(vertical_constraints, climbing, v);
    if (std::abs(dz) > EPSILON && vertical_rate_limit <= EPSILON)
    {
        if (plan_info != nullptr)
        {
            *plan_info = local_info;
        }
        return false;
    }

    const double required_vertical_time = (std::abs(dz) <= EPSILON) ? 0.0 : std::abs(dz) / vertical_rate_limit;
    local_info.required_vertical_time_sec = required_vertical_time;

    const double w = v * max_kappa;
    const double t_loop = (w > EPSILON) ? (2.0 * M_PI / w) : std::numeric_limits<double>::infinity();

    Path best_xy_path;
    double best_total_time = std::numeric_limits<double>::infinity();
    int best_start_loops = 0;
    int best_end_loops = 0;

    auto solve_middle_path = [&](const XYZPsiState &start_state,
                                 const XYZPsiState &goal_state,
                                 Path &middle_path) -> bool
    {
        trochoids::Trochoid trochoid;
        trochoid.problem.v = v;
        trochoid.problem.wind = {wind[0], wind[1]};
        trochoid.problem.max_kappa = max_kappa;
        trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
        trochoid.problem.Xf = {goal_state.x, goal_state.y, goal_state.psi};
        middle_path = trochoid.getTrochoid(waypoint_distance);
        return !middle_path.empty();
    };

    // Case 1 candidate: no loop extension.
    Path base_path;
    if (solve_middle_path(s1, s2, base_path))
    {
        const double base_time = estimate_path_time_seconds(base_path, wind, v);
        local_info.xy_time_sec = base_time;
        if (t_loop < std::numeric_limits<double>::infinity() && base_time + EPSILON < required_vertical_time)
        {
            local_info.estimated_full_loops_needed = static_cast<int>(std::ceil((required_vertical_time - base_time) / t_loop));
        }
        if (base_time + EPSILON >= required_vertical_time)
        {
            best_xy_path = base_path;
            best_total_time = base_time;
            best_start_loops = 0;
            best_end_loops = 0;
        }
    }

    // Phase B: full-loop extension with wind drift.
    if (vertical_constraints.allow_full_loop_extension &&
        t_loop < std::numeric_limits<double>::infinity() &&
        vertical_constraints.max_full_loops > 0)
    {
        const int start_total_loops = std::max(local_info.estimated_full_loops_needed, 1);
        for (int total_loops = start_total_loops; total_loops <= vertical_constraints.max_full_loops; ++total_loops)
        {
            bool found_for_total = false;
            for (int loops_start = 0; loops_start <= total_loops; ++loops_start)
            {
                const int loops_end = total_loops - loops_start;
                const double t_start = loops_start * t_loop;
                const double t_end = loops_end * t_loop;

                XYZPsiState shifted_start = s1;
                shifted_start.x += wind[0] * t_start;
                shifted_start.y += wind[1] * t_start;
                shifted_start.psi = WrapTo2Pi(s1.psi + w * t_start);

                XYZPsiState shifted_goal = s2;
                shifted_goal.x -= wind[0] * t_end;
                shifted_goal.y -= wind[1] * t_end;
                shifted_goal.psi = WrapTo2Pi(s2.psi - w * t_end);

                Path middle_path;
                if (!solve_middle_path(shifted_start, shifted_goal, middle_path))
                {
                    continue;
                }

                const double t_middle = estimate_path_time_seconds(middle_path, wind, v);
                const double t_total = t_start + t_middle + t_end;
                if (t_total + EPSILON < required_vertical_time)
                {
                    continue;
                }

                Path candidate;
                if (loops_start > 0)
                {
                    const Path loop_start_path = generate_full_loop_path(s1.x, s1.y, s1.psi, t_start, wind, v, max_kappa, waypoint_distance);
                    append_path(candidate, loop_start_path, false);
                    append_path(candidate, middle_path, true);
                }
                else
                {
                    append_path(candidate, middle_path, false);
                }

                if (loops_end > 0)
                {
                    const auto end_anchor = candidate.back();
                    const Path loop_end_path = generate_full_loop_path(
                        std::get<0>(end_anchor), std::get<1>(end_anchor), std::get<2>(end_anchor),
                        t_end, wind, v, max_kappa, waypoint_distance);
                    append_path(candidate, loop_end_path, true);
                    if (!candidate.empty())
                    {
                        candidate.back() = std::make_tuple(s2.x, s2.y, WrapTo2Pi(s2.psi));
                    }
                }

                if (candidate.empty())
                {
                    continue;
                }
                if (t_total < best_total_time)
                {
                    best_total_time = t_total;
                    best_xy_path = candidate;
                    best_start_loops = loops_start;
                    best_end_loops = loops_end;
                }
                found_for_total = true;
            }

            // Prefer fewer loops: first loop-count that has any feasible solution.
            if (found_for_total)
            {
                break;
            }
        }
    }

    if (best_xy_path.empty())
    {
        if (plan_info != nullptr)
        {
            *plan_info = local_info;
        }
        return false;
    }

    const std::vector<double> cumulative_time = estimate_cumulative_time(best_xy_path, wind, v);
    const double total_time = cumulative_time.empty() ? 0.0 : cumulative_time.back();
    if (total_time > EPSILON)
    {
        local_info.xy_time_sec = total_time;
    }
    local_info.added_extension_time_sec = (best_start_loops + best_end_loops) * ((t_loop < std::numeric_limits<double>::infinity()) ? t_loop : 0.0);
    local_info.loops_added_start = best_start_loops;
    local_info.loops_added_end = best_end_loops;
    local_info.case_used = (best_start_loops == 0 && best_end_loops == 0) ?
                           VerticalPlanningCase::DIRECT_PROFILE :
                           VerticalPlanningCase::FULL_LOOP_EXTENSION;

    XYZPsiState new_state;
    extended_path_out.reserve(extended_path_out.size() + best_xy_path.size());
    for (size_t i = 0; i < best_xy_path.size(); ++i)
    {
        new_state.x = std::get<0>(best_xy_path[i]);
        new_state.y = std::get<1>(best_xy_path[i]);
        new_state.psi = std::get<2>(best_xy_path[i]);
        if (total_time <= EPSILON)
        {
            new_state.z = s2.z;
        }
        else
        {
            const double alpha = cumulative_time[i] / total_time;
            new_state.z = s1.z + alpha * dz;
        }
        extended_path_out.push_back(new_state);
    }
    if (!extended_path_out.empty())
    {
        extended_path_out.back().x = s2.x;
        extended_path_out.back().y = s2.y;
        extended_path_out.back().z = s2.z;
        extended_path_out.back().psi = s2.psi;
    }

    local_info.valid = true;
    local_info.vertical_feasible = true;
    if (plan_info != nullptr)
    {
        *plan_info = local_info;
    }
    return true;
}

// If waypoint distance is 0, then it will use the default waypoint distance
bool trochoids::get_trochoid_path(const XYZPsiState &s1,
                                const XYZPsiState &s2,
                                std::vector<XYZPsiState> &extended_path_out,
                                const double *wind,
                                double v,
                                double max_kappa,
                                double waypoint_distance)
{
    if (!is_valid_xy_request(s1, s2, wind, v, max_kappa, waypoint_distance))
    {
        return false;
    }

    trochoids::Trochoid trochoid;
    trochoid.problem.v = v;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {s1.x, s1.y, s1.psi};
    trochoid.problem.Xf = {s2.x, s2.y, s2.psi};
    Path path = trochoid.getTrochoid(waypoint_distance);
    if (path.size() == 0)
    {
        return false;
    }
    XYZPsiState new_state;

    for (int i = 0; i < path.size(); i++)
    {
        new_state.x = std::get<0>(path[i]);
        new_state.y = std::get<1>(path[i]);
        new_state.psi = std::get<2>(path[i]);
        new_state.z = s1.z + interpolation_alpha(i, path.size()) * (s2.z - s1.z);

        extended_path_out.push_back(new_state);
    }
    return true;
}

double trochoids::get_length(const XYZPsiState &s1,
                             const XYZPsiState &s2,
                             const double *wind,
                             double v,
                             double max_kappa)
{
    if (!is_valid_xy_request(s1, s2, wind, v, max_kappa, 0.0))
    {
        return 0.0;
    }

    trochoids::Trochoid trochoid;
    trochoid.problem.v = v;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {s1.x, s1.y, s1.psi};
    trochoid.problem.Xf = {s2.x, s2.y, s2.psi};
    Path path = trochoid.getTrochoid();
    double length(0.0);
    //    std::cout<<path.size()<<std::endl;
    if (path.size() == 0)
    {
        //        std::cout<<"pathsizezero"<<std::endl;
        return 0.0;
    }
    for (int i = 0; i < path.size() - 1; i++)
    {
        double x = std::get<0>(path[i]);
        double y = std::get<1>(path[i]);
        double z = s1.z + interpolation_alpha(i, path.size()) * (s2.z - s1.z);
        double x_ = std::get<0>(path[i + 1]);
        double y_ = std::get<1>(path[i + 1]);
        double z_ = s1.z + interpolation_alpha(i + 1, path.size()) * (s2.z - s1.z);

        length += sqrt(pow(x_ - x, 2) + pow(y_ - y, 2) + pow(z_ - z, 2));
    }
    return length;
}

double trochoids::get_length(std::vector<XYZPsiState> &path)
{
    double length = 0;
    for (int i = 0; i < path.size() - 1; i++)
    {
        double x = path[i].x;
        double y = path[i].y;
        double z = path[i].z;
        double x_ = path[i + 1].x;
        double y_ = path[i + 1].y;
        double z_ = path[i + 1].z;

        length += sqrt(pow(x_ - x, 2) + pow(y_ - y, 2) + pow(z_ - z, 2));
    }
    return length;
}
