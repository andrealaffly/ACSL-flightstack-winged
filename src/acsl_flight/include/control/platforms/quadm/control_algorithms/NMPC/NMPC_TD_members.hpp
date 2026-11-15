/***********************************************************************************************************************
 * Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        NMPC_TD_members.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 3, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: NMPC with SO(3) controller class members.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*               _           _      _                      
 $$$$$$\  $$\   $$\  $$$$$$\  $$$$$$$\  $$\      $$\ 
$$  __$$\ $$ |  $$ |$$  __$$\ $$  __$$\ $$$\    $$$ |
$$ /  $$ |$$ |  $$ |$$ /  $$ |$$ |  $$ |$$$$\  $$$$ |
$$ |  $$ |$$ |  $$ |$$$$$$$$ |$$ |  $$ |$$\$$\$$ $$ |
$$ |  $$ |$$ |  $$ |$$  __$$ |$$ |  $$ |$$ \$$$  $$ |
$$ $$\$$ |$$ |  $$ |$$ |  $$ |$$ |  $$ |$$ |\$  /$$ |
\$$$$$$ / \$$$$$$  |$$ |  $$ |$$$$$$$  |$$ | \_/ $$ |
 \___$$$\  \______/ \__|  \__|\_______/ \__|     \__|
     \___|                                           
*/

#ifndef CONTROLLERS_NMPC_TD_MEMBERS_HPP_
#define CONTROLLERS_NMPC_TD_MEMBERS_HPP_

#include "Eigen/Dense"
#include <chrono>
#include <array>                             // Include for std::array (horizon members)
#include "quadm.hpp"                         // Header file for vehicle specific information and some other functions

using namespace Eigen;
namespace _quadm_{
namespace _nmpc_{

// Define the number of steps in the prediction horizon (horizon length [s] = NH * dt_mpc)
constexpr int NH = 30;

// Define the size of the error state [dp, dv, dtheta, domega] and the input [f1, f2, f3, f4]
// NOTE: The full state has 18 numbers (R is 3x3) but the optimizer works on the 12 element error state, where
//       dtheta is a small rotation vector applied on the right (R = R_bar Exp(dtheta)).
constexpr int NX = 12;
constexpr int NU = 4;

// Eigen aliases used by the optimal control problem
using Vec12   = Eigen::Matrix<double, NX, 1>;     // Error state / cost gradient
using Mat12   = Eigen::Matrix<double, NX, NX>;    // State Jacobian A / cost Hessian
using Mat12x4 = Eigen::Matrix<double, NX, NU>;    // Input Jacobian B
using Mat4x12 = Eigen::Matrix<double, NU, NX>;    // Feedback gain K

// Structure for a single quadrotor state. p, v, w are vectors and R lives on SO(3)
struct mpc_state {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();      // Position in the inertial NED frame         [m]
    Eigen::Vector3d v = Eigen::Vector3d::Zero();      // Velocity in the inertial NED frame         [m/s]
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // Rotation from the body to the inertial frame
    Eigen::Vector3d w = Eigen::Vector3d::Zero();      // Angular velocity in the body frame          [rad/s]
};

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    double dt_mpc;                                    // Step length of the prediction model          [s]
    int max_iterations;                               // Max Gauss-Newton iterations per control tick
    int line_search_steps;                            // Max step halvings in the line search
    double regularization;                            // Tikhonov term added to Quu to keep it invertible
    Eigen::Matrix<double, NX, NX> Q;                  // Stage cost on the error state
    Eigen::Matrix<double, NX, NX> Qf;                 // Terminal cost on the error state
    Eigen::Matrix<double, NU, NU> R_in;               // Stage cost on the deviation from the reference motor thrusts
    Eigen::Matrix<double, 3, 3> Ki_tran;              // Integral gain that drives the disturbance estimate
    Eigen::Matrix<double, 3, 1> d_hat_max;            // Saturation on the disturbance estimate        [m/s^2]
    Eigen::Matrix<double, 3, 3> J;                    // Inertia matrix                               [kg m^2]
    Eigen::Matrix<double, 3, 3> J_inv;                // Inverse of the inertia matrix
    Eigen::Matrix<double, 4, 4> mixer_inv;            // Maps the motor thrusts to [thrust; mx; my; mz]
    Eigen::Matrix<double, NU, 1> f_min;               // Min allowed thrust per motor                 [N]
    Eigen::Matrix<double, NU, 1> f_max;               // Max allowed thrust per motor                 [N]
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
    Eigen::Matrix<double, 3, 1> e_tran_pos_I;         // Translational integral error
};

// Structure for all the internal members of the controller
struct controller_internal_members {
    double t;                                                      // Time
    Eigen::Quaterniond q;                                          // Quaternion
    Eigen::Matrix<double, 3, 3> Rji;                               // Rotation from the body to the inertial frame
    Eigen::Matrix<double, 3, 3> Rij;                               // Rotation from the inertial to the body frame
    Eigen::Matrix<double, 3, 1> x_tran_pos;                        // Translational position
    Eigen::Matrix<double, 3, 1> x_tran_vel;                        // Translational velocity
    Eigen::Matrix<double, 3, 1> eta_rot;                           // Euler Angles
    Eigen::Matrix<double, 3, 1> omega;                             // Angular velocities
    Eigen::Matrix<double, 3, 1> r_user;                            // Translational user position command
    Eigen::Matrix<double, 3, 1> r_dot_user;                        // Translational user veloctiy command
    Eigen::Matrix<double, 3, 1> r_ddot_user;                       // Translational user acceleration command
    double psi_user;                                               // User yaw command
    Eigen::Matrix<double, 3, 1> e_tran_pos;                        // Translational error in position
    Eigen::Matrix<double, 3, 1> e_tran_vel;                        // Translational error in velocity
    Eigen::Matrix<double, 3, 1> d_hat;                             // Disturbance acceleration estimate in I [m/s^2]
    mpc_state x0;                                                  // Measured state handed to the NMPC
    std::array<mpc_state, NH + 1> X_ref;                           // Reference state over the horizon
    std::array<Eigen::Matrix<double, NU, 1>, NH> U_ref;            // Reference motor thrusts over the horizon
    std::array<mpc_state, NH + 1> X;                               // Planned state over the horizon (warm start)
    std::array<Eigen::Matrix<double, NU, 1>, NH> U;                // Planned motor thrusts over the horizon
    std::array<mpc_state, NH + 1> X_new;                           // Candidate state during the line search
    std::array<Eigen::Matrix<double, NU, 1>, NH> U_new;            // Candidate motor thrusts during the line search
    std::array<Mat4x12, NH> K_fb;                                  // Feedback gain per step
    std::array<Eigen::Matrix<double, NU, 1>, NH> k_ff;             // Feed-forward correction per step
    double cost_initial;                                           // Cost of the warm start rollout
    double cost;                                                   // Cost after the optimization
    int iterations;                                                // Accepted Gauss-Newton iterations this tick
    double alpha;                                                  // Last accepted line search step
    double shift_timer;                                            // Time elapsed since the last warm start shift (unused)
    Eigen::Matrix<double, 4, 1> u;                                 // [thrust; mx; my; mz]
    Eigen::Matrix<double, 4, 1> Thrust;                            // Newtons
    Eigen::Matrix<double, 4, 1> Sat_Thrust;                        // Saturated thrust
    double alg_duration;                                           // Control execution duration
    std::chrono::high_resolution_clock::time_point alg_start_time; // Algorithm Start timepoint
    std::chrono::high_resolution_clock::time_point alg_end_time;   // Algorithm End timepoint
};


} // namespace _nmpc_
} // namespace _quadm_  


#endif // CONTROLLERS_NMPC_TD_MEMBERS_HPP_