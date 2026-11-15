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
 * File:        NMPC_TD.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 3, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: NMPC with SO(3) for the medium quadrotor. Inherits 
 *              controller_base class for the basic functionality 
 *              that is to be used for all control algorithms.
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

#ifndef CONTROLLERS_NMPC_TD_HPP_
#define CONTROLLERS_NMPC_TD_HPP_

#include "control_algorithm_base.hpp"        // Include for the base class for the basis of control algorithms
#include "NMPC_TD_members.hpp"                // Include for all the members of the NMPC_TD class
#include "NMPC_TD_logger.hpp"                 // Include for the logging

#include "piecewise_polynomial_trajectory.hpp" // Include for the piecwise polynomial trajectory

using namespace _control_algorithm_base_;
using namespace _piecewise_polynomial_trajectory_;


// Define the number of states in the boost array for integration
#define NSI 3

namespace _quadm_{
namespace _nmpc_{

// =========================================================================================================
// SO(3) helpers
//   - skewSymmetric, expSO3, logSO3 live in helper_functions.hpp (namespace _utilities_, pulled in by
//     control_algorithm_base.hpp), ported from the simulator's sim-helpers
//   - box_minus:     12 element error between two states [p_a - p_b; v_a - v_b; Log(R_b^T R_a); w_a - w_b]
// =========================================================================================================

// Error state of a relative to b. The rotation part is expressed in b's body frame, which matches the right
// perturbation R = R_bar Exp(dtheta) used by the linearization.
inline Vec12 box_minus(const mpc_state& a, const mpc_state& b)
{
    Vec12 d;
    d << a.p - b.p,
         a.v - b.v,
         logSO3(b.R.transpose() * a.R),
         a.w - b.w;
    return d;
}


class nmpc: public controller_base
{
    public:
        // COnstructor
        nmpc(flight_params* p, const std::string & controller_log_dir_);

        // Destructor
        ~nmpc();

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
        nmpc_logger logger;

    private:
        // -------------------------------------------------------------------------
        // NON API FUNCTIONS NECESSARY FOR ALL CONTROLLERS - MUST BE IMPLEMENTED
        // -------------------------------------------------------------------------
        // Define the state vector (y) and its derivative (dy) for the rk4 integrator
        rk4_array<double, NSI> y;
        rk4_array<double, NSI> dy;

        // Define the model (dy = position error, so y holds its integral)
        void model(const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, double t);

        // Create a RungeKutta object for integration functionality.
        boost::numeric::odeint::runge_kutta4<rk4_array<double, NSI>> rk4;

        // Define the internal parameter members of the controller
        controller_internal_parameters cip;

        // Define the internal members of the controller
        controller_internal_members cim;

        // Define the internal integrated state members of the controller
        controller_integrated_state_members csm;

    private:
        // -------------------------------------------------------------------------
        // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
        // -------------------------------------------------------------------------
        // Function to compute the disturbance estimate from the integral of the position error
        void compute_disturbance_estimate();

        // Function to sample the user trajectory over the horizon and build the reference states and inputs
        void compute_reference_horizon();

        // Function to propagate the nonlinear prediction model by one step
        mpc_state predict_step(const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f) const;

        // Function to linearize the prediction model about (x, f) in the error state
        void linearize(const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f, Mat12& A, Mat12x4& B) const;

        // Function to compute the stage cost at step k
        double stage_cost(const int k, const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f) const;

        // Function to compute the terminal cost
        double terminal_cost(const mpc_state& x) const;

        // Function to simulate the warm start inputs from the measured state and return its cost
        double rollout();

        // Function to compute the feedback and feed-forward gains with a Riccati backward pass
        void backward_pass();

        // Function to simulate the nonlinear model with the new gains and return the candidate cost
        double forward_pass(const double alpha);

        // Function to solve the optimal control problem (rollout + Gauss-Newton iterations)
        void solve_ocp();

        // Function to shift the warm start every time a full dt_mpc has elapsed
        void shift_warm_start(const double time_step);

        // Function to compute the normalized thrust
        void compute_normalized_thrusts();

        // Assign the values from rk4 to controller internal members
        void assign_from_rk4();
};


}   // namespace _nmpc_
}   // namespace _quadm_


#endif  // CONTROLLERS_NMPC_TD_HPP_