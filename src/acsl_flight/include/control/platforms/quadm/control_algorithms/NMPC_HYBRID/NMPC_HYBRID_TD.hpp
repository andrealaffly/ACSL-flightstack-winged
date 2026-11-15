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
 * File:        NMPC_HYBRID_TD.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 15, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Interconnected NMPC and hybrid MRAC for the medium quadrotor. Inherits controller_base
 *              for the basic functionality that is to be used for all control algorithms.
 *              - Outer loop: translational NMPC baseline + hybrid MRAC (reference resets on the position error norm).
 *              - Inner loop: attitude NMPC baseline on SO(3) + MRAC on the angular velocity.
 *              Ported from the simulator (nmpc-hybrid-quadm.hpp). For reference: G. M. Kumar, X. E. Casanova Pabon,
 *              A. L'Afflitto, "Interconnected Nonlinear MPC and Hybrid MRAC with Applications to Multi-Rotor UAVs".
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

#ifndef CONTROLLERS_NMPC_HYBRID_TD_HPP_
#define CONTROLLERS_NMPC_HYBRID_TD_HPP_

#include "control_algorithm_base.hpp"          // Include for the base class for the basis of control algorithms
#include "NMPC_HYBRID_TD_members.hpp"          // Include for all the members of the NMPC_HYBRID_TD class
#include "NMPC_HYBRID_TD_logger.hpp"           // Include for the logging
#include "adaptive_laws.hpp"                   // Include for the adaptive laws (and the projection / dead-zone operators)
#include "hybrid_functions.hpp"                // Include for the error norm hybrid reset (_adaptive_laws_hybrid_error_norm_)
#include "piecewise_polynomial_trajectory.hpp" // Include for the piecwise polynomial trajectory

using namespace _control_algorithm_base_;
using namespace _piecewise_polynomial_trajectory_;
using namespace _adaptive_laws_;

// Define the number of states in the boost array for integration
//   mu filters (6) + x_tran_ref (6) + K_hat_x_tran (18) + K_hat_r_tran (9) + Theta_hat_tran (21)
//   + omega_ref (3) + K_hat_x_rot (9) + K_hat_r_rot (9) + Theta_hat_rot (21) + e_tran_pos_I (3)
#define NSI 105

namespace _quadm_{
namespace _nmpc_hybrid_{

// =========================================================================================================
// SO(3) helpers
//   - skewSymmetric, veeMap, expSO3, logSO3 live in helper_functions.hpp (namespace _utilities_, pulled in by
//     control_algorithm_base.hpp), ported from the simulator's sim-helpers
//   - box_minus_outer: 6 element error between two outer states [p_a - p_b; v_a - v_b]
//   - box_minus_inner: 6 element error between two inner states [Log(R_b^T R_a); w_a - w_b]
// =========================================================================================================
// Error state of a relative to b for the outer NMPC
inline Vec6 box_minus_outer(const outer_state& a, const outer_state& b)
{
    Vec6 d;
    d << a.p - b.p,
         a.v - b.v;
    return d;
}

// Error state of a relative to b for the inner NMPC. The rotation part is expressed in b's body frame, which matches
// the right perturbation R = R_bar Exp(dtheta) used by the linearization.
inline Vec6 box_minus_inner(const inner_state& a, const inner_state& b)
{
    Vec6 d;
    d << logSO3(b.R.transpose() * a.R),
         a.w - b.w;
    return d;
}


class nmpc_hybrid: public controller_base
{
    public:
        // Constructor
        nmpc_hybrid(flight_params* p, const std::string & controller_log_dir_);

        // Destructor
        ~nmpc_hybrid();

        // Implementing functions from controller_base
        void run(const double time_step_rk4_);
        void update(double time,
                    double x,
                    double y,
                    double z,
                    double vx,
                    double vy,
                    double vz,
                    double q0,
                    double q1,
                    double q2,
                    double q3,
                    double roll,
                    double pitch,
                    double yaw,
                    double w_x,
                    double w_y,
                    double w_z);

        // Implementing getter functions from controller_base
        float get_t1() const;
        float get_t2() const;
        float get_t3() const;
        float get_t4() const;
        float get_t5() const;
        float get_t6() const;
        float get_t7() const;
        float get_t8() const;

    private:
        // Functions from controller_base, called from the constructor
        void read_params(const std::string& jsonFile);
        void init();

        // Trajectory object (piecewise polynomial for flight, lockstep_trajectory in lockstep builds)
        piecewise_polynomial_trajectory ud;

        // Logger object
        nmpc_hybrid_logger logger;

    private:
        // -------------------------------------------------------------------------
        // NON API FUNCTIONS NECESSARY FOR ALL CONTROLLERS - MUST BE IMPLEMENTED
        // -------------------------------------------------------------------------
        // Define the state vector (y) and its derivative (dy) for the rk4 integrator
        rk4_array<double, NSI> y;
        rk4_array<double, NSI> dy;

        // Define the model
        void model(const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, double t);

        // Create a RungeKutta object for integration functionality.
        boost::numeric::odeint::runge_kutta4<rk4_array<double, NSI>> rk4;

        // Define the internal parameter members of the controller
        controller_internal_parameters cip;

        // Define the internal members of the controller
        controller_internal_members cim;

        // Define the internal integrated state members of the controller
        controller_integrated_state_members csm;

        // Define the members of the outer (translational) NMPC
        outer_nmpc_members onm;

        // Define the members of the inner (attitude) NMPC
        inner_nmpc_members inm;

    private:
        // -------------------------------------------------------------------------
        // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
        // -------------------------------------------------------------------------
        // Function to initialize the reference models and the filters at the measured state (first tick only)
        void initialize_from_measured_state();

        // Function to reset the outer reference model with the hybrid algorithm (error norm trigger)
        void compute_hybrid_reference_reset();

        // Function to compute the disturbance estimate injected in the outer NMPC model
        void compute_disturbance_estimate();

        // ---------------------------------- Outer (translational) NMPC ---------------------------------- //
        // Function to sample, solve and hold the outer NMPC and to compute the reference command from its plan
        void compute_outer_nmpc();

        // Function to sample the user trajectory over the outer horizon
        void compute_outer_reference_horizon();

        // Function to propagate the outer nominal model by one step
        outer_state outer_predict_step(const outer_state& x, const Eigen::Vector3d& mu) const;

        // Function to linearize the outer nominal model (linear, so independent of the state)
        void outer_linearize(Mat6& A, Mat6x3& B) const;

        // Function to project the force onto the thrust magnitude and tilt limits
        Eigen::Vector3d outer_project_force(const Eigen::Vector3d& mu) const;

        // Function to compute the outer stage cost at step k
        double outer_stage_cost(const int k, const outer_state& x, const Eigen::Vector3d& mu) const;

        // Function to compute the outer terminal cost
        double outer_terminal_cost(const outer_state& x) const;

        // Function to simulate the outer warm start from the measured state and return its cost
        double outer_rollout();

        // Function to compute the outer feedback and feed-forward gains with a Riccati backward pass
        void outer_backward_pass();

        // Function to simulate the outer nominal model with the new gains and return the candidate cost
        double outer_forward_pass(const double alpha);

        // Function to solve the outer optimal control problem
        void outer_solve_ocp();

        // Function to shift the outer warm start by one step after a solve
        void outer_shift_warm_start();

        // ---------------------------------- Outer (translational) MRAC ---------------------------------- //
        // Function to compute the translational control in the inertial frame (hybrid MRAC on top of the outer NMPC)
        void compute_translational_control_in_I();

        // ---------------------------------------- Command generator ---------------------------------------- //
        // Function to observe the rate of the outerloop control input
        void compute_translational_control_rate();

        // Function to compute the thrust (u1), the desired rotation matrix and the desired angular velocity
        void compute_u1_R_d();

        // Function to map a force and a heading to a desired rotation matrix (Eqs. (60)-(62))
        Eigen::Matrix3d compute_R_d_from_force(const Eigen::Vector3d& mu, const double psi) const;

        // Function to propagate one axis of the mu filter (command generator model) by dt with a held input (RK4)
        Eigen::Matrix<double, 2, 1> propagate_mu_filter(const Eigen::Matrix<double, 2, 1>& state,
                                                        const double u,
                                                        const double dt) const;

        // ----------------------------------- Inner (attitude) NMPC ----------------------------------- //
        // Function to sample, solve and hold the inner NMPC
        void compute_inner_nmpc();

        // Function to build the inner preview [R_d; Omega_d] from the outer plan and the held adaptive force
        void compute_inner_reference_horizon();

        // Function to propagate the inner nominal model by one step (Eqs. (82)-(83))
        inner_state inner_predict_step(const inner_state& x, const Eigen::Vector3d& tau) const;

        // Function to linearize the inner nominal model about (x, tau) in the error state
        void inner_linearize(const inner_state& x, Mat6& A, Mat6x3& B) const;

        // Function to compute the inner residual [eta; e_omega] at step k and its Jacobian E (Eq. (84))
        Vec6 inner_residual(const int k, const inner_state& x, Mat6& E) const;

        // Function to compute the inner feedforward torque at step k (Eq. (86))
        Eigen::Vector3d inner_feedforward_torque(const int k, const inner_state& x) const;

        // Function to scale the torque so that every motor stays within its thrust limits
        Eigen::Vector3d inner_clamp_torque(const Eigen::Vector3d& tau) const;

        // Function to compute the inner stage cost at step k (Eq. (85))
        double inner_stage_cost(const int k, const inner_state& x, const Eigen::Vector3d& tau) const;

        // Function to compute the inner terminal cost
        double inner_terminal_cost(const inner_state& x) const;

        // Function to simulate the inner warm start from the measured state and return its cost
        double inner_rollout();

        // Function to compute the inner feedback and feed-forward gains with a Riccati backward pass
        void inner_backward_pass();

        // Function to simulate the inner nominal model with the new gains and return the candidate cost
        double inner_forward_pass(const double alpha);

        // Function to solve the inner optimal control problem
        void inner_solve_ocp();

        // Function to shift the inner warm start by one step after a solve
        void inner_shift_warm_start();

        // Function to compute the infinite horizon LQR cost-to-go (discrete Riccati) used as the NMPC terminal cost
        Mat6 compute_lqr_terminal_cost(const Mat6& A, const Mat6x3& B, const Mat6& Q, const Eigen::Matrix3d& R) const;

        // ----------------------------------- Inner (rotational) MRAC ----------------------------------- //
        // Function to compute the rotational control input (MRAC on omega on top of the inner NMPC)
        void compute_rotational_control();

        // Function to compute the normalized thrust
        void compute_normalized_thrusts();

        // Assign the values from rk4 to controller internal members
        void assign_from_rk4();
};

}   // namespace _nmpc_hybrid_
}   // namespace _quadm_

#endif  // CONTROLLERS_NMPC_HYBRID_TD_HPP_
