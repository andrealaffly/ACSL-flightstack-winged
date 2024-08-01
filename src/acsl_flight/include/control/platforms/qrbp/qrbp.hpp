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
 * File:        qrbp.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        May 29, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Vehicle Information of the QRBP with the NACA 0012 wings 
 *              and the Velox V2808 kv1300 motors used in all the control
 *              algorithms
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

/*
________/\\\__________/\\\\\\\\\______/\\\\\\\\\\\\\____/\\\\\\\\\\\\\___        
 _____/\\\\/\\\\_____/\\\///////\\\___\/\\\/////////\\\_\/\\\/////////\\\_       
  ___/\\\//\////\\\__\/\\\_____\/\\\___\/\\\_______\/\\\_\/\\\_______\/\\\_      
   __/\\\______\//\\\_\/\\\\\\\\\\\/____\/\\\\\\\\\\\\\\__\/\\\\\\\\\\\\\/__     
    _\//\\\______/\\\__\/\\\//////\\\____\/\\\/////////\\\_\/\\\/////////____    
     __\///\\\\/\\\\/___\/\\\____\//\\\___\/\\\_______\/\\\_\/\\\_____________   
      ____\////\\\//_____\/\\\_____\//\\\__\/\\\_______\/\\\_\/\\\_____________  
       _______\///\\\\\\__\/\\\______\//\\\_\/\\\\\\\\\\\\\/__\/\\\_____________ 
        _________\//////___\///________\///__\/////////////____\///______________
*/

#ifndef QRBP_HPP_
#define QRBP_HPP_

#include <Eigen/Dense>             // Include the Eigen library
#include <cmath>
#include <math.h>
#include "helper_functions.hpp"  // Include the helper functions for using the rotation matrices

namespace _qrbp_{
        
    // Constants
    inline constexpr double G = 9.81;
    inline constexpr double RHO_HAT = 1.225;
    inline constexpr double PI = 3.1415;
    inline constexpr double PIHALF = (PI/2);
    inline constexpr double DEG2RAD = (PI/180);
    inline constexpr double RAD2DEG = (180/PI);
    inline constexpr double SQRT2_OVER_2 = 0.7071067;

    // Vehicle and Environment Defines
    inline constexpr double MASS = 1.95056951;                                    // mass of vehicle                [Kg]                                      
    inline constexpr double LX = 0.097509;                                        // dist to motor along x^J         [m]
    inline constexpr double LY = 0.110688;                                        // dist to motor along y^J         [m]
    inline constexpr double LZ_S = 0.038779;                                      // dist to aero center of stabs    [m]
    inline constexpr double SPAN_W = 0.50;                                        // span of wings                   [m]
    inline constexpr double CHORD_W = 0.12;                                       // chord of wings                  [m]
    inline constexpr double SPAN_S = 0.175;                                       // span of stabs                   [m]
    inline constexpr double CHORD_S = 0.02;                                       // chord of stabs                  [m]
    inline constexpr double PLANFORM_AREA_W = (2.0 * SPAN_W * CHORD_S);           // planform area of wings        [m^2]
    inline constexpr double PLANFORM_AREA_S = (2.0 * SPAN_S * CHORD_S);           // planform area of stabs        [m^2]
    
    // Aerodynamic Dynamic Coefficient Defines
    inline constexpr double DYN_PRESS_COEFF_W = (0.5 * RHO_HAT * PLANFORM_AREA_W);     
    inline constexpr double DYN_PRESS_COEFF_S = (0.5 * RHO_HAT * PLANFORM_AREA_S);

    // Switch over angle of QRBP
    inline constexpr double AoS = 70*DEG2RAD;

    // vairbales for switching mechanism
    inline bool is_biplane;        // Boolean to tell the controller if it is in biplane mode
    inline bool momentary_button;  // Boolean to tell the controller to reset conditions

    // Thrust Normalizeing polynomials are hardcoded!
    // Take care coding this from matlab. check the documentation for the polyval function.
    // The polynomails are reversed!.
    inline constexpr double CT_MOTOR =  0.1017;                                   // motor torque coeff              [-]
    inline constexpr double MAX_THRUST = 18;                                      // max allowed thrust per motor    [N]
    inline constexpr double MIN_THRUST = 0.3;                                     // min allowed thrust per motor    [N]


    // Roll Rate Filter ------------------------------------------------------------------------------------------------
    // A matrix of the roll_ref filter
    const Eigen::Matrix2d A_filter_roll_ref = (Eigen::Matrix2d() << 
                                                    -72.0, -1600.0, 
                                                      1.0,    0.0 
                                                ).finished(); 

    // B matrix of the roll_ref filter
    const Eigen::Vector2d B_filter_roll_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the roll_ref filter
    const Eigen::RowVector2d C_filter_roll_ref = Eigen::RowVector2d(1600.0, 0); 

    // D matrix of the roll_ref filter
    const double D_filter_roll_ref = 0.0; 

    // Pitch Rate Filter -----------------------------------------------------------------------------------------------
    // A matrix of the pitch_ref filter
    const Eigen::Matrix2d A_filter_pitch_ref = (Eigen::Matrix2d() << 
                                                    -72.0, -1600.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the pitch_ref filter
    const Eigen::Vector2d B_filter_pitch_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the pitch_ref filter
    const Eigen::RowVector2d C_filter_pitch_ref = Eigen::RowVector2d(1600.0, 0); 

    // D matrix of the pitch_ref filter
    const double D_filter_pitch_ref = 0.0; 
    
    // Yaw Rate Filter -------------------------------------------------------------------------------------------------
    // A matrix of the yaw_ref filter
    const Eigen::Matrix2d A_filter_yaw_ref = (Eigen::Matrix2d() << 
                                                    -72.0, -1600.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the yaw_ref filter
    const Eigen::Vector2d B_filter_yaw_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the yaw_ref filter
    const Eigen::RowVector2d C_filter_yaw_ref = Eigen::RowVector2d(1600.0, 0); 

    // D matrix of the yaw_ref filter
    const double D_filter_yaw_ref = 0.0; 

    // Roll Acceleration Filter ----------------------------------------------------------------------------------------
    // A matrix of the roll_dot_ref filter
    const Eigen::Matrix2d A_filter_roll_dot_ref = (Eigen::Matrix2d() << 
                                                    -50.0, -2500.0, 
                                                      1.0,    0.0 
                                                ).finished(); 

    // B matrix of the roll_dot_ref filter
    const Eigen::Vector2d B_filter_roll_dot_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the roll_dot_ref filter
    const Eigen::RowVector2d C_filter_roll_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

    // D matrix of the roll_dot_ref filter
    const double D_filter_roll_dot_ref = 0.0; 

    // Pitch Acceleration Filter ---------------------------------------------------------------------------------------
    // A matrix of the pitch_ref filter
    const Eigen::Matrix2d A_filter_pitch_dot_ref = (Eigen::Matrix2d() << 
                                                    -50.0, -2500.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the pitch_ref filter
    const Eigen::Vector2d B_filter_pitch_dot_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the pitch_ref filter
    const Eigen::RowVector2d C_filter_pitch_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

    // D matrix of the pitch_ref filter
    const double D_filter_pitch_dot_ref = 0.0; 

    // Yaw Acceleration Filter -----------------------------------------------------------------------------------------
    // A matrix of the yaw_ref filter
    const Eigen::Matrix2d A_filter_yaw_dot_ref = (Eigen::Matrix2d() << 
                                                    -50.0, -2500.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the yaw_ref filter
    const Eigen::Vector2d B_filter_yaw_dot_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the yaw_ref filter
    const Eigen::RowVector2d C_filter_yaw_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

    // D matrix of the yaw_ref filter
    const double D_filter_yaw_dot_ref = 0.0; 
    
    // Thrust Polynomial -----------------------------------------------------------------------------------------------
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor F35A - Velox V2808 Kv1300
    const Eigen::VectorXd thrust_polynomial_coeff_qrbp = (Eigen::VectorXd(8) << 
                                                            0.000000030316084940690649384320061064321,
                                                           -0.000001814468657920738862350290739045,
                                                            0.000042434512948142268497307011410058,
                                                           -0.00050487431926448940282259325584846,
                                                            0.0035249290604740459061094970394379,
                                                           -0.017552333963129614080589391278409,
                                                            0.1149654034760465154407782506496,
                                                            0.011001286689392602777259888569006
                                                        ).finished();

    // Weight vector of the qrbp
    static inline const Eigen::Vector3d e3_basis = Eigen::Vector3d(0.0, 0.0, 1.0);

    
    // Mixer Matrix - This captures the necessary constants within the lambda function and initializes the
    // mixer_matrix_quadcopter with the resulting matrix.
    // [1/4, -1/(4*l_y),  1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y), -1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y),  1/(4*l_x), -1/(4*c_t)]
    // [1/4, -1/(4*l_y), -1/(4*l_x), -1/(4*c_t)]
    const Eigen::Matrix4d mixer_matrix_qrbp = []() {
        Eigen::Matrix4d mat; // 4 rows, 4 columns
            
        // Assign values element-by-element
        mat(0, 0) =  1.0 / 4.0;
        mat(0, 1) = -1.0 / (4.0 * LY);
        mat(0, 2) =  1.0 / (4.0 * LX);
        mat(0, 3) =  1.0 / (4.0 * CT_MOTOR); 

        mat(1, 0) =  1.0 / 4.0;
        mat(1, 1) =  1.0 / (4.0 * LY);
        mat(1, 2) = -1.0 / (4.0 * LX);
        mat(1, 3) =  1.0 / (4.0 * CT_MOTOR); 

        mat(2, 0) =  1.0 / 4.0;
        mat(2, 1) =  1.0 / (4.0 * LY);
        mat(2, 2) =  1.0 / (4.0 * LX);
        mat(2, 3) = -1.0 / (4.0 * CT_MOTOR); 

        mat(3, 0) =  1.0 / 4.0;
        mat(3, 1) = -1.0 / (4.0 * LY);
        mat(3, 2) = -1.0 / (4.0 * LX);
        mat(3, 3) = -1.0 / (4.0 * CT_MOTOR); 

        return mat; 
    }();



    // Matrix of inertia of the quadcopter frame
    // [kg*m^2] inertia matrix of the vehicle system (drone frame + box + propellers) expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() << 
                                                 0.03093953,  -0.00000662,  0.00005560,
                                                -0.00000662,   0.01964797, -0.00000339,
                                                 0.00005560,  -0.00000339,  0.03676930
                                             ).finished(); 

    // Matrix of inertia of the biplane frame
    // [kg*m^2] inertia matrix of the vehicle system (drone frame + box + propellers) expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_b = (Eigen::Matrix3d() << 
                                                 0.03676930,  0.00000339, -0.00005560,
                                                 0.00000339,  0.01964797, -0.00000662,
                                                -0.00005560, -0.00000662,  0.03093953
                                             ).finished();      

    // Matrix that rotates from quadcopter to biplane mode
    const Eigen::Matrix3d R_Jq_Jb = (Eigen::Matrix3d() << 
                                        0.0, 0.0, 1.0,
                                        0.0, 1.0, 0.0,
                                       -1.0, 0.0, 0.0).finished();

    // Matrix that rotates from biplane to quadcopter mode
    const Eigen::Matrix3d R_Jb_Jq = (Eigen::Matrix3d() <<
                                        0.0, 0.0, -1.0,
                                        0.0, 1.0,  0.0,
                                        1.0, 0.0,  0.0).finished();

    // Angles for the wings and stabilizers
    struct AeroAngles {
        double alpha;     // Angle of attack at the center of mass
        double alpha_up;  // Angle of attack of the upper wing
        double alpha_lw;  // Angle of attack of the lower wing
        double beta;      // Side slip angle at the center of mass
        double beta_lt;   // Side slip angle for the left stabilizer
        double beta_rt;   // Side slip angle for the right stabilizer
    };



    // Inline function to compute the Aerodynamic angles for the NACA-0012 airfoil
    inline AeroAngles computeAeroAnglesNACA0012Quad(const double u, const double v, const double w, 
                                                    const double w_x, const double w_y, const double w_z)
    {
        AeroAngles angles;
        
        // Precompute common expressions
        double u_sq = u * u;
        double w_sq = w * w;
        double LY_wz = LY * w_z;
        double LY_wx = LY * w_x;

        // Compute intermediate values
        double c2 = -(w - LX * w_y);
        double c3 = -(w + LX * w_y);
        double c4 = sqrt(u_sq + w_sq);
        double c5 = sqrt((u + LY_wz) * (u + LY_wz) + (w - LY_wx) * (w - LY_wx));
        double c6 = sqrt((u - LY_wz) * (u - LY_wz) + (w + LY_wx) * (w + LY_wx));

        double n1 = c4;
        double n2 = sqrt(u_sq + c2 * c2);
        double n3 = sqrt(u_sq + c3 * c3);
        double n4 = sqrt(v * v + c4 * c4);
        double n5 = sqrt(v * v + c5 * c5);
        double n6 = sqrt(v * v + c6 * c6);

        // Compute the angle of attack at the center of mass
        angles.alpha = (n1 <= 0.01f) ? 0.0f : atan2(u, -w);

        // Compute the angle of attack of the upper wing
        angles.alpha_up = (n2 <= 0.01f) ? 0.0f : atan2(u, c2);

        // Compute the angle of attack of the lower wing
        angles.alpha_lw = (n3 <= 0.01f) ? 0.0f : atan2(u, c3);

        // Compute the side slip angle at the center of mass
        angles.beta = (n4 <= 0.01f) ? 0.0f : atan2(v, c4);

        // Compute the side slip angle for the left stabilizer
        angles.beta_lt = (n5 <= 0.01f) ? 0.0f : atan2(v, c5);

        // Compute the side slip angle for the right stabilizer
        angles.beta_rt = (n6 <= 0.01f) ? 0.0f : atan2(v, c6);

        return angles;
    }

    // Inline function to compute the coefficient of lift for the NACA-0012 airfoil using polynomial approximations
    inline double compute_cl(double angle)
    {
        using Eigen::Matrix;

        static const struct {
            double min_angle;
            double max_angle;
            Matrix<double, 3, 1> coeffs;
        } angle_coeff_map[] = {
            {-18 * DEG2RAD, -15 * DEG2RAD, Matrix<double, 3, 1>(-2.016727473, -0.040687912, 0.0)},
            {-15 * DEG2RAD,  -7 * DEG2RAD, Matrix<double, 3, 1>(-0.339043494,  0.073185027, 0.0)},
            { -7 * DEG2RAD,   7 * DEG2RAD, Matrix<double, 3, 1>( 0.0,          0.113305266, 0.0)},
            {  7 * DEG2RAD,  15 * DEG2RAD, Matrix<double, 3, 1>( 0.344930022,  0.073338904, 0.0)},
            { 15 * DEG2RAD,  17 * DEG2RAD, Matrix<double, 3, 1>( 2.007250549, -0.39995604,  0.0)},
            { 17 * DEG2RAD,  27 * DEG2RAD, Matrix<double, 3, 1>( 4.98,        -0.378,       0.0087)},
            { 27 * DEG2RAD,  95 * DEG2RAD, Matrix<double, 3, 1>( 0.43,         0.0303,     -0.000382)},
            { 95 * DEG2RAD, 155 * DEG2RAD, Matrix<double, 3, 1>( 8.91,        -0.143,       0.000519)},
            {155 * DEG2RAD, 180 * DEG2RAD, Matrix<double, 3, 1>(139,          -1.67,        0.00499)},
            {-27 * DEG2RAD, -18 * DEG2RAD, Matrix<double, 3, 1>(-4.98,        -0.378,      -0.0087)},
            {-95 * DEG2RAD, -27 * DEG2RAD, Matrix<double, 3, 1>(-0.43,         0.0303,      0.000382)},
            {-155* DEG2RAD, -95 * DEG2RAD, Matrix<double, 3, 1>(-8.91,        -0.143,      -0.000519)},
            {-180* DEG2RAD,-155* DEG2RAD, Matrix<double, 3, 1>(-139,         -1.67,       -0.00499)},
        };

        for (const auto& entry : angle_coeff_map) {
            if (angle >= entry.min_angle && angle < entry.max_angle) {
                return entry.coeffs(0) + entry.coeffs(1) * angle + entry.coeffs(2) * angle * angle;
            }
        }

        return 0.0; // Default return if no range matches
    }

    // Inline function to compute the coefficient of drag for the NACA-0012 airfoil using polynomial approximations
    inline double compute_cd(double angle)
    {
        using Eigen::Matrix;

        static const struct {
            double min_angle;
            double max_angle;
            Matrix<double, 3, 1> coeffs;
        } angle_coeff_map[] = {
            {-18 * DEG2RAD,  18 * DEG2RAD, Matrix<double, 3, 1>( 0.004705434,  0.0,        0.000199101)},
            { 18 * DEG2RAD, 180 * DEG2RAD, Matrix<double, 3, 1>(-0.82,         0.0574,    -0.000319)},
            {-180* DEG2RAD, -18 * DEG2RAD, Matrix<double, 3, 1>(-0.82,        -0.0574,    -0.000319)},
        };

        for (const auto& entry : angle_coeff_map) {
            if (angle >= entry.min_angle && angle < entry.max_angle) {
                return entry.coeffs(0) + entry.coeffs(1) * angle + entry.coeffs(2) * angle * angle;
            }
        }

        return 0.0; // Default return if no range matches
    }

    // Inline function to compute the coefficient of aerodynamic moment for the NACA-0012 airfoil using polynomial 
    // approximations
    inline double compute_cm(double angle)
    {
        using Eigen::Matrix;

        static const struct {
            double min_angle;
            double max_angle;
            Matrix<double, 2, 1> coeffs;
        } angle_coeff_map[] = {
            {-18 * DEG2RAD, -15 * DEG2RAD, Matrix<double, 2, 1>(-0.154623077, -0.007923077)},
            {-15 * DEG2RAD,  -6.75*DEG2RAD, Matrix<double, 2, 1>( 0.043910573,  0.004934576)},
            { -6.75*DEG2RAD,  -4 * DEG2RAD, Matrix<double, 2, 1>(-0.029656624, -0.005647643)},
            { -4 * DEG2RAD,   4 * DEG2RAD, Matrix<double, 2, 1>( 0.0,          0.001556935)},
            {  4 * DEG2RAD,   7 * DEG2RAD, Matrix<double, 2, 1>( 0.027566825, -0.005293839)},
            {  7 * DEG2RAD,  15 * DEG2RAD, Matrix<double, 2, 1>(-0.044215508,  0.004953476)},
            { 15 * DEG2RAD,  18 * DEG2RAD, Matrix<double, 2, 1>( 0.168665618, -0.008760839)},
            {-180* DEG2RAD, -18 * DEG2RAD, Matrix<double, 2, 1>(-0.3,          0.0)}, // Default case
            { 18 * DEG2RAD,  180* DEG2RAD, Matrix<double, 2, 1>(-0.3,          0.0)}, // Default case
        };

        for (const auto& entry : angle_coeff_map) {
            if (angle >= entry.min_angle && angle < entry.max_angle) {
                return entry.coeffs(0) + entry.coeffs(1) * angle;
            }
        }

        return 0.0; // Default return if no range matches
    }

    // Inline function to compute the switch from Quadcopter mode to Biplane Mode and vice versa
    inline void QRBPswitchFun(double pitch, bool& is_biplane, bool& mnt_btn)
    {

        // Code to switch in both directions of flight.
        // double abs_pitch = abs(pitch);

        // if (abs_pitch >= AoS && !is_biplane)
        // {
        //     is_biplane = true;
        //     mnt_btn = true;
        // }
        // else if (abs_pitch < AoS && is_biplane)
        // {
        //     is_biplane = false;
        //     mnt_btn = true;
        // }
        // else
        // {
        //     mnt_btn = false;
        // }

        // For debugging. we only switch in the forward direction
        if (pitch <= -AoS && !is_biplane)
        {
            is_biplane = true;
            mnt_btn = true;
        }
        else if (pitch > -AoS && is_biplane)
        {
            is_biplane = false;
            mnt_btn = true;
        }
        else
        {
            mnt_btn = false;
        }
    }

    // Inline Function to update the biplane mode states - Goes in the inner loop bridge.
    // Need to modify. This function is written assuming that we are only going in forward flight in a straight line!!!
    inline void update_states_to_biplane_mode(const Vector3d eta,const Vector3d omega, 
                                              Vector3d& eta_b, Vector3d& omega_b) 
    {

        // Compute the right hand side of the equation
        Eigen::Matrix<double, 3, 3>  RHS = _utilities_::rotationMatrix321LocalToGlobal(eta(0), eta(1), eta(2))*R_Jq_Jb;

        // Matlab code to follow
        // if RHS(3,1) == 1
        //     theta_B = -pi/2;
        //     psi_B = atan2(-RHS(2,3),-RHS(1,3));
        //     phi_B = 0;
        // elseif RHS(3,1) == -1
        //     theta_B = pi/2;
        //     psi_B = atan2(RHS(1,2),RHS(2,2));
        //     phi_B = 0;
        // else
        //     theta_B(2,1) = atan2(-RHS(3,1),sqrt(1-RHS(3,1)^2));
        //     theta_B(1,1) = atan2(-RHS(3,1),-sqrt(1-RHS(3,1)^2));
            
        //     phi_B(2,1) = atan2(RHS(3,2),RHS(3,3));
        //     phi_B(1,1) = atan2(-RHS(3,2),-RHS(3,3));
            
        //     psi_B(2,1) = atan2(RHS(2,1),RHS(1,1));
        //     psi_B(1,1) = atan2(-RHS(2,1),-RHS(1,1));
        // end

        // Some vectors and temprory variables
        Vector2d phi_b, theta_b, psi_b;             // Resultant angle pair for biplane
        
        double abs0, abs1;                          // Temp variables for caluclating the smallest rotation
        double rhs20 = RHS(2, 0);                   // caching variable
        double rhs20_sqrt = sqrt(1 - rhs20*rhs20);  // caching variable
        double rhs21 = RHS(2,1);                    // caching variable
        double rhs22 = RHS(2,2);                    // caching variable
        double rhs10 = RHS(1,0);                    // caching variable
        double rhs00 = RHS(0,0);                    // caching variable

        if (abs(rhs20) != 1)
        {
            /// The state we usually will be in
            // Find the pitch for biplane
            theta_b(0)  = atan2( -rhs20,  rhs20_sqrt );
            theta_b(1)  = atan2( -rhs20, -rhs20_sqrt );
            abs0 = std::fabs(theta_b(0));
            abs1 = std::fabs(theta_b(1));
            eta_b(1) = (abs0 < abs1) ? theta_b(0) : theta_b(1);

            // Find the roll for biplane
            phi_b(0) = atan2(  rhs21,  rhs22 );
            phi_b(1) = atan2( -rhs21, -rhs22 );
            abs0 = std::fabs(phi_b(0));
            abs1 = std::fabs(phi_b(1));
            eta_b(0) = (abs0 < abs1) ? phi_b(0) : phi_b(1);

            // Find the yaw for biplane
            psi_b(0) = atan2(  rhs10,  rhs00 );
            psi_b(1) = atan2( -rhs10, -rhs00 );

            abs0 = std::fabs(psi_b(0));
            abs1 = std::fabs(psi_b(1));

            eta_b(2) = (abs0 < abs1) ? psi_b(0) : psi_b(1);
        }
        // Conditions for if we are hovering and if we are pointing the propeller's axis to the ground. 
        // This is not going to happen and therfore the if statements are put under another if statement 
        // To save compute
        else
        {
            eta_b(1) = (rhs20 == 1) ? -PIHALF : PIHALF;
            eta_b(2) = (rhs20 == 1) ? atan2(-RHS(1,2), -RHS(0,2)) : atan2(RHS(0,1), RHS(1,1));
            eta_b(0) = 0.0;
        }

        // Compute the angular velocities in the biplane frame
        omega_b = R_Jq_Jb * omega;
    }

    inline void update_states_to_biplane_quaternions(const Eigen::Quaterniond q_q, const Vector3d eta_q, const Vector3d omega, 
                                                     Vector3d& eta_b, Vector3d& omega_b)
    {
        // Modified from pixhawk code that constructs a dcm matrix from quaternion     
        const double a = q_q.w();
        const double b = q_q.x();
        const double c = q_q.y();
        const double d = q_q.z();
        const double aa = a * a;
        const double ab = a * b;
        const double ac = a * c;
        const double ad = a * d;
        const double bb = b * b;
        const double bc = b * c;
        const double bd = b * d;
        const double cc = c * c;
        const double cd = c * d;
        const double dd = d * d;

        // Construct the dcm rotation matrix according to the pixhawk code.
        Eigen::Matrix<double, 3, 3> dcm;
        dcm(0, 0) = aa + bb - cc - dd;
        dcm(0, 1) = 2 * (bc - ad);
        dcm(0, 2) = 2 * (ac + bd);
        dcm(1, 0) = 2 * (bc + ad);
        dcm(1, 1) = aa - bb + cc - dd;
        dcm(1, 2) = 2 * (cd - ab);
        dcm(2, 0) = 2 * (bd - ac);
        dcm(2, 1) = 2 * (ab + cd);
        dcm(2, 2) = aa - bb - cc + dd;

        // Vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
        // Since the VTOL airframe is initialized as a multicopter we need to modify the 
        // estimated attitude fro the fixed wing operation. Since the neutral position of
        // the vehicle in fixed wing mode is -90 degrees rotated around the pitch axis 
        // compared to the neutral position of the vecicle in multicopter mode we need to 
        // swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
        // Additionally, in order to get the correct sign of the pitch, we need to multiply
        // the new x axis of the rotation matrix with -1. 
        
        /*
        * original:			modified:
        *
        * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
        * Rxy  Ryy  Rzy	    -Rzy  Ryy  Rxy
        * Rxz  Ryz  Rzz	    -Rzz  Ryz  Rxz
        * */

        Eigen::Matrix<double, 3,3> dcm_adapted = dcm;        // modified rotation matrix
        /* move z to x */
        dcm_adapted(0, 0) = dcm(0, 2);
        dcm_adapted(1, 0) = dcm(1, 2);
        dcm_adapted(2, 0) = dcm(2, 2);

        /* move x to z */
        dcm_adapted(0, 2) = dcm(0, 0);
        dcm_adapted(1, 2) = dcm(1, 0);
        dcm_adapted(2, 2) = dcm(2, 0);

        /* change direction of pitch (convert to right handed system) */
        dcm_adapted(0, 0) = -dcm_adapted(0, 0);
        dcm_adapted(1, 0) = -dcm_adapted(1, 0);
        dcm_adapted(2, 0) = -dcm_adapted(2, 0);

        // Now reconstruct the euler angles from the dcm_adapted matrix.
        
        // Theta
        double theta = std::asin(-dcm_adapted(2,0));
        double roll;
        double yaw;

        if ((std::fabs(theta - M_PI_2)) < 1.0e-3)
        {
            roll = 0.0;
            yaw = std::atan2(dcm_adapted(1,2), dcm_adapted(0,2));
        }
        else if ((std::fabs(theta + M_PI_2)) < 1.0e-3)
        {
            roll = 0.0;
            yaw = std::atan2(-dcm_adapted(1,2), -dcm_adapted(0,2));
        } 
        else
        {
            roll = std::atan2(dcm_adapted(2,1), dcm_adapted(2,2));
            yaw = std::atan2(dcm_adapted(1,0), dcm_adapted(0,0));
        }
        

        // Assign pitch
        eta_b(1) = theta;

        // Assign roll
        if (roll > M_PI_2) { eta_b(0) = roll - M_PI; }
        else if (roll < -M_PI_2) { eta_b(0) = roll + M_PI; }
        else { eta_b(0) = roll; }
        
        // Assign yaw
        if (yaw > M_PI_2) { eta_b(2) = yaw - M_PI; }
        else if (yaw < -M_PI_2) { eta_b(2) = yaw + M_PI; }
        else { eta_b(2) = yaw; }

        // Compute the angular velocities in the biplane frame
        omega_b = R_Jq_Jb * omega;

    }                                                     


} // namespace _qrbp_

#endif  // QRBP_HPP_
