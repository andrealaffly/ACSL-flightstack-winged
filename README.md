# ACSL Flight Stack - Winged
[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE.txt)

To clone this repo with all the needed submodules you can run the command:

```bash
git clone --recurse-submodules https://github.com/andrealaffly/ACSL-flightstack-aero.git
```

## Introduction

The **Advanced Control Systems Lab (ACSL) Flight Stack - Winged** is a PX4-comaptible offboard flight stack for VTOL (vertical take-off and landing) UAVs (uncrewed aerial vehicles). The scope of this software is to support the UAV control community with a freeware, open-source solution that can serve as a shared platform and hence, facilitate comparison of research results.

We strongly encourage control practitioners, researchers, and UAV enthusiasts to:

- **Use this code**
- **Improve it**
- **Provide constructive feedback**
- **Propose edits through [GitHub](https://github.com/andrealaffly/ACSL_flightstack-aero.git)**

At the moment, this code is designed for quad-biplanes (QRBPs), which are quadcopters with two parallel wings. By adjusting the mixer matrix, which determines the relationship between total thrust, thrust moment, and thrust from each motor, this software can be expanded to support other types of UAV configurations in future versions. This flight stack is also compatible with autonomous UAVs equipped with collinear propellers, including quadcopters, X8-copters, and hexacopters. It currently supports up to 8 motors.

## Outlook on the Control Architecture

QRBPs are inherently under-actuated. To manage this, the software includes:

- **Inner Loop**: Handles rotational dynamics.
- **Outer Loop**: Manages translational dynamics.

Both loops are governed by nonlinear equations of motion.

### Available Control Solutions

This software currently offers two control solutions for the inner and outer loops:

1. **Continuous-Time Feedback-Linearizing Control Law** combined with a **PID (Proportional-Integral-Derivative) Control Law**.
2. The above control law is augmented by a **Model Reference Adaptive Control (MRAC) System**, incorporating an aerodynamic model. The aerodynamic lift and drag forces are deduced from experimental data points of a NACA0012 airfoil. 

For further details on these control architectures, refer to the following publications:

- M. Gramuglia, G. M. Kumar, and A. L'Afflitto, ["Two-Layer Adaptive Funnel MRAC with Applications to the Control of Multi-Rotor UAVs,"](https://doi.org/10.1109/RoMoCo60539.2024.10604361) *2024 13th International Workshop on Robot Motion and Control (RoMoCo),* Poznań, Poland, 2024, pp. 31-36.
- M. Gramuglia, G. M. Kumar, and A. L’Afflitto, ["A Hybrid Model Reference Adaptive Control System for Multi-Rotor Unmanned Aerial Vehicles,"](https://doi.org/10.2514/6.2024-0755) *AIAA SCITECH 2024 Forum, 2024.*
- E. Lavretsky and K. Wise, *"Robust and Adaptive Control: With Aerospace Applications,"*, London, UK: Springer, 2012.

Future versions of the software will include additional control systems.

## Hardware Requirements

This flight stack is compatible with **[ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html)** or **[Humble](https://docs.ros.org/en/humble/Installation.html)** and has been tested on an **[ODROID M1S](https://www.hardkernel.com/shop/odroid-m1s-with-8gbyte-ram-io-header/)** companion computer interfaced with a **[Pixhawk 6c](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html)**. The companion computer runs **Ubuntu 20.04 Linux**. The firmware version of the flight controller is **[PX4 v1.15.0](https://docs.px4.io/v1.15/en/)**.

### Assumed UAV State

- Provided by a motion capture system over UDP (or, alternatively, GNSS).
- IMU (Inertial Measurement Unit) from the Pixhawk flight controller.

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Mattia Gramuglia**](https://github.com/mattia-gramuglia)
- [**Giri M. Kumar**](https://github.com/girimugundankumar)

For more information, visit [https://lafflitto.com](https://lafflitto.com).

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)

---

This software is distributed under a permissive **3-Clause BSD License**.

