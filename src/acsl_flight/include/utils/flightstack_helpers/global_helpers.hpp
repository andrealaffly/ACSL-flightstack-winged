///@cond 
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
///@endcond 
/***********************************************************************************************************************
 * File:        global_helpers.hpp \n 
 * Author:      Giri Mugundan Kumar \n 
 * Date:        September 12, 2024 \n 
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Inline functions for all the classes of the flightstack to use.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef GLOBAL_HELPERS_HPP_
#define GLOBAL_HELPERS_HPP_

/**
 * @file global_helpers.hpp
 * @brief Inline functions for all the classes of the flighstack to use.
 */

#include <iostream>     // For std::cout, std::cerr
#include <string>       // For std::string
#include <sstream>      // For std::ostringstream if you need to format types into strings
#include <type_traits>  // For type traits (optional, for advanced type checking or static assertions)
#include <Eigen/Dense>  // For Eigen library support

// ANSI color codes
#define COLOR_BLUE "\033[1;34m"       // Blue ANSI Code
#define COLOR_GREEN "\033[1;32m"      // Green ANSI Code
#define COLOR_RED "\033[1;31m"        // Red ANSI Code
#define COLOR_ORANGE "\033[38;5;214m" // Orange color using the 256-color palette
#define COLOR_RESET "\033[0m"         // Reset to default color

namespace _flightstack_
{

// Function to print a single message of any type
template <typename T>
/**
 * @brief Function to print a single message of any type
 * @param msg 
 */
inline void FLIGHTSTACK_INFO(const T& msg)
{
  std::cout << "[INFO] [--ACSL FLIGHTSTACK--] " << msg << " " << std::endl;
}

// Function to print a string and another data type together
template <typename T1, typename T2>
/**
 * @brief Function to print a string and another data type together
 * @param str 
 * @param value 
 */
inline void FLIGHTSTACK_INFO(const T1& str, const T2& value)
{
  std::cout << "[INFO] [--ACSL FLIGHTSTACK--] " << str << " " << value << " " << std::endl;
}

// Throttled single message
template <typename T>
/**
 * @brief Throttled a single message
 * @param msg 
 * @param throttle_interval_ms 
 */
inline void FLIGHTSTACK_INFO(const T& msg, int throttle_interval_ms)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (duration >= throttle_interval_ms)
    {
        std::cout << "[INFO] [--ACSL FLIGHTSTACK--] " << msg << " " << std::endl;
        last_time = now;
    }
}

// Throttled string and another data type
template <typename T1, typename T2>
/**
 * @brief Throttled string and another data type
 * @param str 
 * @param value 
 * @param throttle_interval_ms 
 */
inline void FLIGHTSTACK_INFO(const T1& str, const T2& value, int throttle_interval_ms)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (duration >= throttle_interval_ms)
    {
        std::cout << "[INFO] [--ACSL FLIGHTSTACK--] " << str << " " << value << " " << std::endl;
        last_time = now;
    }
}

// Function to print a string and an Eigen matrix
template <typename T, int Rows, int Cols>
/**
 * @brief Function to print a string and an Eigen matrix
 * @param str 
 * @param mat 
 */
inline void FLIGHTSTACK_INFO_MATRIX(const std::string& str, const Eigen::Matrix<T, Rows, Cols>& mat)
{
    std::cout << "[INFO] [--ACSL FLIGHTSTACK--] \n" << str << ":\n" << mat << "\n" << std::endl;
}

// Function to print a single message of any type with carriage return
template <typename T>
/**
 * @brief Function to print a single message of any type with carriage return
 * @param msg 
 */
inline void FLIGHTSTACK_INFO_STREAM(const T& msg)
{
  std::cout << COLOR_GREEN << "\r" << "[INFO] [--ACSL FLIGHTSTACK--] " << msg << " " << COLOR_RESET << std::flush;
}

// Function to print a single message of any type with carriage return - NO TAG
template <typename T>
/**
 * @brief Function to print a single message of any type with carriage return - NO TAG
 * @param msg 
 */
inline void FLIGHTSTACK_INFO_STREAM_NO_TAG(const T& msg)
{
  std::cout << COLOR_GREEN << "\r" << msg << " " << COLOR_RESET << std::flush;
}


// Function to print a string and another data type with carriage return
template <typename T1, typename T2>
/**
 * @brief Function to print a string and another data type with carriage return
 * @param str 
 * @param value 
 */
inline void FLIGHTSTACK_INFO_STREAM(const T1& str, const T2& value)
{
  std::cout << COLOR_GREEN << "\r" << "[INFO] [--ACSL FLIGHTSTACK--] " << str << " " << value << " " << COLOR_RESET << std::flush;
}

// Function to print a string and another data type with carriage return - NO TAG
template <typename T1, typename T2>
/**
 * @brief Function to print a string and another data type with carriage return - NO TAG
 * @param str 
 * @param value 
 */
inline void FLIGHTSTACK_INFO_STREAM_NO_TAG(const T1& str, const T2& value)
{
  std::cout << COLOR_GREEN << "\r" << str << " " << value << " " << COLOR_RESET << std::flush;
}

// Throttled single message with carriage return
template <typename T>
/**
 * @brief Throttled single message with carriage return
 * @param msg 
 * @param throttle_interval_ms 
 */
inline void FLIGHTSTACK_INFO_STREAM(const T& msg, int throttle_interval_ms)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (duration >= throttle_interval_ms)
    {
        std::cout << COLOR_GREEN << "\r" << "[INFO] [--ACSL FLIGHTSTACK--] " << msg << " " << COLOR_RESET << std::flush;
        last_time = now;
    }
}

// Throttled single message with carriage return - NO TAG
template <typename T>
/**
 * @brief Throttled single message with carriage return - NO TAG
 * @param msg 
 * @param throttle_interval_ms 
 */
inline void FLIGHTSTACK_INFO_STREAM_NO_TAG(const T& msg, int throttle_interval_ms)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (duration >= throttle_interval_ms)
    {
        std::cout << COLOR_GREEN << "\r" << msg << " " << COLOR_RESET << std::flush;
        last_time = now;
    }
}

// Throttled string and another data type with carriage return
template <typename T1, typename T2>
/**
 * @brief Throttled string and another data type with carriage return
 * @param str 
 * @param value 
 * @param throttle_interval_ms 
 */
inline void FLIGHTSTACK_INFO_STREAM(const T1& str, const T2& value, int throttle_interval_ms)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (duration >= throttle_interval_ms)
    {
        std::cout << COLOR_GREEN << "\r" << "[INFO] [--ACSL FLIGHTSTACK--] " << str << " " << value << " " << COLOR_RESET << std::flush;
        last_time = now;
    }
}

// Throttled string and another data type with carriage return - NO TAG
template <typename T1, typename T2>
/**
 * @brief Throttled string and another data type with carriage return - NO TAG
 * @param str 
 * @param value 
 * @param throttle_interval_ms 
 */
inline void FLIGHTSTACK_INFO_STREAM_NO_TAG(const T1& str, const T2& value, int throttle_interval_ms)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (duration >= throttle_interval_ms)
    {
        std::cout << COLOR_GREEN << "\r" << str << " " << value << " " << COLOR_RESET << std::flush;
        last_time = now;
    }
}

// Function to print an error message of any type and terminate the program
template <typename T>
/**
 * @brief Function to print an error message of any type and terminate the program
 * @param msg 
 */
inline void FLIGHTSTACK_ERROR(const T& msg)
{
    std::cerr << COLOR_RED << "[ERROR] [--ACSL FLIGHTSTACK--] " << msg << " " << COLOR_RESET << std::endl;
    std::exit(EXIT_FAILURE);  // Exit the program with a failure status
}

// Function to print a string and another data type as an error and terminate the program
template <typename T1, typename T2>
/**
 * @brief Function to print a string and another data type as an error and terminate the program
 * @param str 
 * @param value 
 */
inline void FLIGHTSTACK_ERROR(const T1& str, const T2& value)
{
    std::cerr << COLOR_RED << "[ERROR] [--ACSL FLIGHTSTACK--] " << str << " " << value << " " << COLOR_RESET << std::endl;
    std::exit(EXIT_FAILURE);  // Exit the program with a failure status
}

// Function to print a warning message of any type
template <typename T>
/**
 * @brief Function to print a warning message of any type
 * @param msg 
 */
inline void FLIGHTSTACK_WARNING(const T& msg)
{
  std::cout << COLOR_ORANGE << "[WARN] [--ACSL FLIGHTSTACK--] " << msg << " " << COLOR_RESET << std::endl;
}

// Function to print a string and another data type as a warning
template <typename T1, typename T2>
/**
 * @brief Function to print a string and another data type as a warning
 * @param str 
 * @param value 
 */
inline void FLIGHTSTACK_WARNING(const T1& str, const T2& value)
{
  std::cout << COLOR_ORANGE << "[WARN] [--ACSL FLIGHTSTACK--] " << str << " " << value << " " << COLOR_RESET << std::endl;
}

} // namespace _flightstack_


#endif  // GLOBAL_HELPER_HPP_