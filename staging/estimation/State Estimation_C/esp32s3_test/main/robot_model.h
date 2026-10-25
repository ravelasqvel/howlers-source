/*
 ============================================================================
 Name        : robot_model.h
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Omnidirectional Robot Model for State Estimation
               State: [x, y, psi, vx_b, vy_b, omega]
               Control: [ax_b, ay_b]
               Measurement: [vx_b, vy_b, omega, psi]
 ============================================================================
*/

#ifndef ROBOT_MODEL_H_
#define ROBOT_MODEL_H_

#include <math.h>
#include <stddef.h>
#include "angle_utils.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ============================================================================
 * ROBOT PARAMETERS
 * ============================================================================ */

#define ROBOT_DT 0.01f  /* Time step in seconds */

/* State dimension */
#define STATE_DIM 6

/* Measurement dimension */
#define MEAS_DIM 4

/* Control dimension */
#define CONTROL_DIM 2

/* ============================================================================
 * STATE TRANSITION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Omnidirectional robot dynamics (continuous-discrete)
 * 
 * State: [x, y, psi, vx_b, vy_b, omega]
 * Control: [ax_b, ay_b]
 * 
 * Equations:
 *   x_k+1 = x_k + (vx_b*cos(psi) - vy_b*sin(psi)) * dt
 *   y_k+1 = y_k + (vx_b*sin(psi) + vy_b*cos(psi)) * dt
 *   psi_k+1 = psi_k + omega * dt
 *   vx_b_k+1 = vx_b_k + ax_b * dt
 *   vy_b_k+1 = vy_b_k + ay_b * dt
 *   omega_k+1 = omega_k
 */
void robot_dynamics(const float *x, const float *u, float *x_out);

/**
 * @brief Measurement function for omnidirectional robot
 * 
 * Measurement: [vx_b, vy_b, omega, psi]
 */
void robot_measurement(const float *x, float *z_out);

/**
 * @brief State Jacobian for EKF
 * 
 * F = ∂f/∂x (6x6 matrix)
 */
void robot_jacobian_F(const float *x, const float *u, float *F);

/**
 * @brief Measurement Jacobian for EKF
 * 
 * H = ∂h/∂x (4x6 matrix)
 */
void robot_jacobian_H(const float *x, float *H);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_MODEL_H_ */
