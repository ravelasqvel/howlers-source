/*
 ============================================================================
 Name        : ekf.h
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Extended Kalman Filter (EKF) Implementation in C
               Based on first-order linearization of nonlinear systems
 ============================================================================

 REFERENCES:
 [1] Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). "Estimation with
     Applications to Tracking and Navigation"
 [2] Simon, D. (2006). "Optimal State Estimation: Kalman, H∞, and Nonlinear
     Approaches"

 ALGORITHM OVERVIEW:

 The EKF linearizes nonlinear systems using first-order Taylor expansion:
 1. Propagate state through nonlinear dynamics: x̄ = f(x)
 2. Compute Jacobian F = ∂f/∂x at current state
 3. Propagate covariance: P̄ = F·P·F^T + Q
 4. Compute predicted measurement: ẑ = h(x̄)
 5. Compute Jacobian H = ∂h/∂x at predicted state
 6. Update with Kalman gain: K = P̄·H^T·(H·P̄·H^T + R)^-1
 7. Correct state and covariance

 Key Differences from UKF:
 - Uses analytical Jacobians (requires derivatives)
 - First-order approximation (less accurate for highly nonlinear systems)
 - Simpler implementation, lower computational cost
 - May diverge if linearization is poor

 ============================================================================
*/

#ifndef EKF_H_
#define EKF_H_

#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "linearalgebra.h"
#include "angle_utils.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ============================================================================
     * TYPE DEFINITIONS FOR USER FUNCTIONS
     * ============================================================================ */

    /**
     * @brief State transition function pointer type
     *
     * This function propagates the state forward in time according to the
     * system dynamics: x_k = f(x_k-1, u_k)
     *
     * @param x     Input state vector [n x 1]
     * @param u     Control input vector [dim_u x 1] (can be NULL if no control)
     * @param x_out Output state vector [n x 1]
     *
     * Example for constant velocity model (no control):
     *   x_out[0] = x[0] + x[1] * dt;  // position (dt embedded in model)
     *   x_out[1] = x[1];               // velocity
     *
     * Example with control input:
     *   x_out[0] = x[0] + x[1];       // position update
     *   x_out[1] = x[1] + u[0];       // velocity with acceleration control
     */
    typedef void (*ekf_state_transition_fn)(const float *x, const float *u, float *x_out);

    /**
     * @brief Measurement function pointer type
     *
     * This function converts state to measurement: z = h(x)
     *
     * @param x     State vector [n x 1]
     * @param z_out Measurement vector [m x 1]
     *
     * Example for position-only measurement:
     *   z_out[0] = x[0];  // measure only position
     */
    typedef void (*ekf_measurement_fn)(const float *x, float *z_out);

    /**
     * @brief State Jacobian function pointer type
     *
     * This function computes the Jacobian of the state transition function
     * F = ∂f/∂x evaluated at the current state
     *
     * @param x   Current state vector [n x 1]
     * @param u   Control input vector [dim_u x 1] (can be NULL if no control)
     * @param F   Output Jacobian matrix [n x n] (row-major)
     *
     * Example for constant velocity model (state = [px, vx, py, vy]):
     *   F = [1  dt  0   0 ]
     *       [0  1   0   0 ]
     *       [0  0   1  dt ]
     *       [0  0   0   1 ]
     */
    typedef void (*ekf_jacobian_f_fn)(const float *x, const float *u, float *F);

    /**
     * @brief Measurement Jacobian function pointer type
     *
     * This function computes the Jacobian of the measurement function
     * H = ∂h/∂x evaluated at the predicted state
     *
     * @param x  Predicted state vector [n x 1]
     * @param H  Output Jacobian matrix [m x n] (row-major)
     *
     * Example for position-only measurement (measure px, py from [px, vx, py, vy]):
     *   H = [1  0  0  0]
     *       [0  0  1  0]
     */
    typedef void (*ekf_jacobian_h_fn)(const float *x, float *H);

    /* ============================================================================
     * DATA STRUCTURES
     * ============================================================================ */

    /**
     * @brief EKF Parameters structure
     *
     * Contains the dimensions for the EKF.
     */
    typedef struct
    {
        size_t n;     /**< State dimension */
        size_t m;     /**< Measurement dimension */
        size_t dim_u; /**< Control input dimension */
    } EKF_Params;

    /**
     * @brief EKF State structure
     *
     * Contains the complete state of the Extended Kalman Filter including
     * the state estimate, covariance matrix, and noise matrices.
     */
    typedef struct
    {
        /* Dimensions */
        size_t n;     /**< State dimension */
        size_t m;     /**< Measurement dimension */
        size_t dim_u; /**< Control input dimension */

        /* State and covariance */
        float *x; /**< State estimate [n x 1] */
        float *P; /**< State covariance matrix [n x n] */

        /* Process and measurement noise */
        float *Q; /**< Process noise covariance [n x n] */
        float *R; /**< Measurement noise covariance [m x m] */

        /* User-defined functions */
        ekf_state_transition_fn f; /**< State transition function */
        ekf_measurement_fn h;      /**< Measurement function */
        ekf_jacobian_f_fn jac_f;   /**< State Jacobian function */
        ekf_jacobian_h_fn jac_h;   /**< Measurement Jacobian function */

        /* Internal working memory */
        float *F;        /**< State Jacobian [n x n] */
        float *H;        /**< Measurement Jacobian [m x n] */
        float *x_pred;   /**< Predicted state [n x 1] */
        float *P_pred;   /**< Predicted covariance [n x n] */
        float *z_pred;   /**< Predicted measurement [m x 1] */
        float *y;        /**< Innovation (measurement residual) [m x 1] */
        float *S;        /**< Innovation covariance [m x m] */
        float *K;        /**< Kalman gain [n x m] */
        float *F_T;      /**< F transpose [n x n] */
        float *H_T;      /**< H transpose [n x m] */
        float *temp_nn;  /**< Temporary [n x n] */
        float *temp_nn2; /**< Temporary [n x n] */
        float *temp_mm;  /**< Temporary [m x m] */
        float *temp_nm;  /**< Temporary [n x m] */
        float *temp_mn;  /**< Temporary [m x n] */
        float *temp_n;   /**< Temporary [n x 1] */
        float *temp_m;   /**< Temporary [m x 1] */
        float *I;        /**< Identity matrix [n x n] */

        /* Angle handling (optional) */
        bool *state_is_angle; /**< Flags for angular state variables [n] (NULL if none) */
        bool *meas_is_angle;  /**< Flags for angular measurements [m] (NULL if none) */

    } EKF_State;

    /* ============================================================================
     * API FUNCTIONS
     * ============================================================================ */

    /**
     * @brief Initialize the Extended Kalman Filter
     *
     * Allocates memory and sets up the EKF with the specified dimensions
     * and user-defined functions.
     *
     * @param ekf    Pointer to EKF_State structure to initialize
     * @param n      State dimension
     * @param m      Measurement dimension
     * @param dim_u  Control input dimension (use 0 if no control)
     * @param f      State transition function
     * @param h      Measurement function
     * @param jac_f  State Jacobian function
     * @param jac_h  Measurement Jacobian function
     *
     * @return true if initialization successful, false otherwise
     *
     * @note User must call ekf_set_state() and ekf_set_noise() after initialization
     */
    bool ekf_init(EKF_State *ekf, size_t n, size_t m, size_t dim_u,
                  ekf_state_transition_fn f, ekf_measurement_fn h,
                  ekf_jacobian_f_fn jac_f, ekf_jacobian_h_fn jac_h);

    /**
     * @brief Free all memory allocated by the EKF
     *
     * @param ekf  Pointer to EKF_State structure to free
     */
    void ekf_free(EKF_State *ekf);

    /**
     * @brief Set the initial state and covariance
     *
     * @param ekf  Pointer to initialized EKF_State
     * @param x    Initial state vector [n x 1]
     * @param P    Initial covariance matrix [n x n] (row-major)
     */
    void ekf_set_state(EKF_State *ekf, const float *x, const float *P);

    /**
     * @brief Set the process and measurement noise covariances
     *
     * @param ekf  Pointer to initialized EKF_State
     * @param Q    Process noise covariance [n x n] (row-major)
     * @param R    Measurement noise covariance [m x m] (row-major)
     */
    void ekf_set_noise(EKF_State *ekf, const float *Q, const float *R);

    /**
     * @brief Perform the EKF prediction step
     *
     * Propagates the state and covariance forward in time:
     * 1. x̄ = f(x, u)
     * 2. F = ∂f/∂x
     * 3. P̄ = F·P·F^T + Q
     *
     * @param ekf  Pointer to initialized EKF_State
     * @param u    Control input vector [dim_u x 1] (can be NULL if no control)
     */
    void ekf_predict(EKF_State *ekf, const float *u);

    /**
     * @brief Perform the EKF update step with a measurement
     *
     * Updates the state and covariance using measurement:
     * 1. ẑ = h(x̄)
     * 2. H = ∂h/∂x
     * 3. y = z - ẑ (innovation)
     * 4. S = H·P̄·H^T + R (innovation covariance)
     * 5. K = P̄·H^T·S^-1 (Kalman gain)
     * 6. x = x̄ + K·y
     * 7. P = (I - K·H)·P̄
     *
     * @param ekf  Pointer to initialized EKF_State
     * @param z    Measurement vector [m x 1]
     */
    void ekf_update(EKF_State *ekf, const float *z);

    /**
     * @brief Get the current state estimate and covariance
     *
     * @param ekf  Pointer to initialized EKF_State
     * @param x    Output state estimate [n x 1] (can be NULL)
     * @param P    Output covariance matrix [n x n] (can be NULL)
     */
    void ekf_get_state(const EKF_State *ekf, float *x, float *P);

    /**
     * @brief Set angle handling flags for state and measurement variables
     *
     * Configures which state and measurement components are angles (in radians).
     * When set, the EKF will automatically:
     * - Use angle_diff() for innovation calculation (measurement residuals)
     * - Normalize angles to [-π, π] after state updates
     *
     * This is CRITICAL for systems with angular states (e.g., robot heading,
     * gimbal angles) to prevent filter divergence and incorrect estimates.
     *
     * @param ekf             Pointer to initialized EKF_State
     * @param state_is_angle  Array [n] of booleans (true if state[i] is angle, NULL to disable)
     * @param meas_is_angle   Array [m] of booleans (true if meas[i] is angle, NULL to disable)
     *
     * Example:
     *   // State: [x, y, theta, v], Measurement: [theta]
     *   bool state_angles[4] = {false, false, true, false};  // only theta
     *   bool meas_angles[1] = {true};  // measurement is angle
     *   ekf_set_angle_states(&ekf, state_angles, meas_angles);
     *
     * @note If you don't have angular variables, simply don't call this function
     *       or pass NULL for both parameters to disable angle handling.
     */
    void ekf_set_angle_states(EKF_State *ekf, const bool *state_is_angle, const bool *meas_is_angle);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H_ */
