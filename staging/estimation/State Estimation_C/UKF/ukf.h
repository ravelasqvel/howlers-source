/*
 ============================================================================
 Name        : ukf.h
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Unscented Kalman Filter (UKF) Implementation in C
               Based on Van der Merwe's Scaled Sigma Point Algorithm
 ============================================================================

 REFERENCES:
 [1] Van der Merwe, R. (2004). "Sigma-Point Kalman Filters for Probabilistic
     Inference in Dynamic State-Space Models" PhD Dissertation
 [2] Julier, S.J. (2002). "The Scaled Unscented Transformation"

 ALGORITHM OVERVIEW:

 The UKF uses the Unscented Transform to handle nonlinear systems by:
 1. Generating sigma points around the current state estimate
 2. Propagating these points through nonlinear functions
 3. Computing the mean and covariance from the transformed points

 Van der Merwe's Scaled Sigma Point Parameters:
 - alpha (α): Spread of sigma points around mean (typically 1e-3 to 1)
 - beta (β):  Incorporates prior knowledge (2 is optimal for Gaussian)
 - kappa (κ): Secondary scaling parameter (typically 0 or 3-n)

 ============================================================================
*/

#ifndef UKF_H_
#define UKF_H_

#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "../LinearAlgebra/linearalgebra.h"
#include "../LinearAlgebra/angle_utils.h"

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
     *   x_out[0] = x[0] + x[1] * dt;  // position (dt is embedded in the model)
     *   x_out[1] = x[1];               // velocity
     *
     * Example with control input:
     *   x_out[0] = x[0] + x[1];       // position update
     *   x_out[1] = x[1] + u[0];       // velocity update with acceleration
     */
    typedef void (*ukf_state_transition_fn)(const float *x, const float *u, float *x_out);

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
    typedef void (*ukf_measurement_fn)(const float *x, float *z_out);

    /* ============================================================================
     * DATA STRUCTURES
     * ============================================================================ */

    /**
     * @brief UKF Parameters structure
     *
     * Contains the dimensions and tuning parameters for the UKF.
     */
    typedef struct
    {
        size_t n;          /**< State dimension */
        size_t m;          /**< Measurement dimension */
        size_t dim_u;      /**< Control input dimension */
        float alpha;       /**< Sigma point spread (1e-3 to 1) */
        float beta;        /**< Prior knowledge parameter (2 for Gaussian) */
        float kappa;       /**< Secondary scaling (0 or 3-n) */
        float lambda;      /**< Composite scaling: λ = α²(n+κ) - n */
        size_t num_sigmas; /**< Number of sigma points: 2n + 1 */
    } UKF_Params;

    /**
     * @brief Sigma Points structure
     *
     * Contains the sigma points and their weights for mean and covariance.
     * Sigma points are stored in row-major format [num_sigmas x n]
     */
    typedef struct
    {
        float *points; /**< Sigma points array [num_sigmas x n] */
        float *Wm;     /**< Weights for mean [num_sigmas] */
        float *Wc;     /**< Weights for covariance [num_sigmas] */
    } UKF_SigmaPoints;

    /**
     * @brief UKF State structure
     *
     * Contains all matrices and vectors needed for UKF operation.
     * All matrices stored in row-major format.
     */
    typedef struct
    {
        /* Core state */
        float *x; /**< State estimate [n x 1] */
        float *P; /**< State covariance [n x n] */

        /* Noise matrices */
        float *Q; /**< Process noise covariance [n x n] */
        float *R; /**< Measurement noise covariance [m x m] */

        /* Predicted state (after predict, before update) */
        float *x_pred; /**< Predicted state [n x 1] */
        float *P_pred; /**< Predicted covariance [n x n] */

        /* Sigma points */
        UKF_SigmaPoints sigmas_f; /**< Sigma points after state transition [num_sigmas x n] */
        UKF_SigmaPoints sigmas_h; /**< Sigma points after measurement [num_sigmas x m] */

        /* Measurement prediction */
        float *z_pred; /**< Predicted measurement [m x 1] */
        float *P_z;    /**< Measurement covariance [m x m] */
        float *P_xz;   /**< Cross-covariance [n x m] */

        /* Kalman gain */
        float *K; /**< Kalman gain [n x m] */

        /* Working memory */
        float *temp_n;  /**< Temporary vector [n x 1] */
        float *temp_m;  /**< Temporary vector [m x 1] */
        float *temp_nn; /**< Temporary matrix [n x n] */
        float *temp_mm; /**< Temporary matrix [m x m] */
        float *temp_nm; /**< Temporary matrix [n x m] */

        /* Parameters and functions */
        UKF_Params params;
        ukf_state_transition_fn fx; /**< State transition function */
        ukf_measurement_fn hx;      /**< Measurement function */

        /* Angle handling (optional) */
        bool *state_is_angle; /**< Array [n] indicating if state component is an angle */
        bool *meas_is_angle;  /**< Array [m] indicating if measurement component is an angle */
    } UKF_State;

    /* ============================================================================
     * CORE UKF FUNCTIONS
     * ============================================================================ */

    /**
     * @brief Initialize UKF with dimensions and parameters
     *
     * Allocates memory for all internal matrices and initializes parameters.
     *
     * @param ukf   Pointer to UKF state structure
     * @param n     State dimension
     * @param m     Measurement dimension
     * @param dim_u Control input dimension (use 0 if no control)
     * @param alpha Sigma point spread parameter (typically 0.001 to 1)
     * @param beta  Prior knowledge parameter (2 for Gaussian)
     * @param kappa Secondary scaling parameter (0 or 3-n)
     * @param fx    State transition function
     * @param hx    Measurement function
     * @return      true if successful, false if allocation failed
     */
    bool ukf_init(UKF_State *ukf, size_t n, size_t m, size_t dim_u,
                  float alpha, float beta, float kappa,
                  ukf_state_transition_fn fx, ukf_measurement_fn hx);

    /**
     * @brief Free all memory allocated for UKF
     *
     * @param ukf Pointer to UKF state structure
     */
    void ukf_free(UKF_State *ukf);

    /**
     * @brief Set initial state estimate
     *
     * @param ukf Pointer to UKF state structure
     * @param x   Initial state vector [n x 1]
     * @param P   Initial covariance matrix [n x n]
     */
    void ukf_set_state(UKF_State *ukf, const float *x, const float *P);

    /**
     * @brief Set process and measurement noise covariances
     *
     * @param ukf Pointer to UKF state structure
     * @param Q   Process noise covariance [n x n]
     * @param R   Measurement noise covariance [m x m]
     */
    void ukf_set_noise(UKF_State *ukf, const float *Q, const float *R);

    /**
     * @brief Get current state estimate
     *
     * @param ukf   Pointer to UKF state structure
     * @param x_out Output state vector [n x 1]
     * @param P_out Output covariance matrix [n x n] (can be NULL)
     */
    void ukf_get_state(const UKF_State *ukf, float *x_out, float *P_out);

    /* ============================================================================
     * SIGMA POINT FUNCTIONS (Van der Merwe's Algorithm)
     * ============================================================================ */

    /**
     * @brief Compute weights for sigma points
     *
     * Implements Van der Merwe's weight computation:
     * - Wm[0] = λ / (n + λ)
     * - Wc[0] = λ / (n + λ) + (1 - α² + β)
     * - Wm[i] = Wc[i] = 1 / (2(n + λ))  for i = 1..2n
     *
     * @param ukf Pointer to UKF state structure
     */
    void ukf_compute_weights(UKF_State *ukf);

    /**
     * @brief Generate sigma points using Van der Merwe's scaled algorithm
     *
     * Generates 2n+1 sigma points from mean x and covariance P:
     * - χ₀ = x
     * - χᵢ = x + [√((n+λ)P)]ᵢ     for i = 1..n
     * - χᵢ = x - [√((n+λ)P)]ᵢ₋ₙ   for i = n+1..2n
     *
     * Uses Cholesky decomposition to compute matrix square root.
     *
     * @param ukf    Pointer to UKF state structure
     * @param x      Mean vector [n x 1]
     * @param P      Covariance matrix [n x n]
     * @param sigmas Output sigma points [num_sigmas x n]
     * @return       true if successful, false if Cholesky failed
     */
    bool ukf_compute_sigma_points(const UKF_State *ukf, const float *x,
                                  const float *P, float *sigmas);

    /**
     * @brief Unscented Transform - compute mean and covariance from sigma points
     *
     * Given sigma points Y and weights, computes:
     * - mean = Σ Wm[i] * Y[i]
     * - cov  = Σ Wc[i] * (Y[i] - mean)(Y[i] - mean)ᵀ + noise_cov
     *
     * @param sigmas      Sigma points [num_sigmas x dim]
     * @param Wm          Weights for mean [num_sigmas]
     * @param Wc          Weights for covariance [num_sigmas]
     * @param num_sigmas  Number of sigma points
     * @param dim         Dimension of each sigma point
     * @param noise_cov   Additive noise covariance [dim x dim] (can be NULL)
     * @param is_angle    Array [dim] indicating if component is angle (NULL = all linear)
     * @param mean_out    Output mean vector [dim x 1]
     * @param cov_out     Output covariance matrix [dim x dim]
     */
    void ukf_unscented_transform(const float *sigmas, const float *Wm, const float *Wc,
                                 size_t num_sigmas, size_t dim, const float *noise_cov,
                                 const bool *is_angle, float *mean_out, float *cov_out);

    /* ============================================================================
     * PREDICT AND UPDATE STEPS
     * ============================================================================ */

    /**
     * @brief UKF Predict Step
     *
     * Propagates the state estimate forward in time:
     * 1. Generate sigma points from current (x, P)
     * 2. Pass each sigma point through f(x, u)
     * 3. Compute predicted mean and covariance using Unscented Transform
     * 4. Add process noise Q
     *
     * Updates: x_pred, P_pred, sigmas_f
     *
     * @param ukf Pointer to UKF state structure
     * @param u   Control input vector [dim_u x 1] (can be NULL if no control)
     * @return    true if successful, false otherwise
     */
    bool ukf_predict(UKF_State *ukf, const float *u);

    /**
     * @brief UKF Update Step
     *
     * Updates the state estimate given a measurement:
     * 1. Transform predicted sigma points through h(x)
     * 2. Compute predicted measurement mean and covariance
     * 3. Compute cross-covariance between state and measurement
     * 4. Compute Kalman gain K = P_xz * P_z⁻¹
     * 5. Update state: x = x_pred + K * (z - z_pred)
     * 6. Update covariance: P = P_pred - K * P_z * Kᵀ
     *
     * @param ukf Pointer to UKF state structure
     * @param z   Measurement vector [m x 1]
     * @return    true if successful, false otherwise
     */
    bool ukf_update(UKF_State *ukf, const float *z);

    /* ============================================================================
     * UTILITY FUNCTIONS
     * ============================================================================ */

    /**
     * @brief Print UKF state for debugging
     *
     * @param ukf Pointer to UKF state structure
     */
    void ukf_print_state(const UKF_State *ukf);

    /**
     * @brief Configure which state and measurement components are angles
     *
     * Sets boolean arrays indicating which components are angular (periodic).
     * This enables automatic angle wrapping and proper circular statistics.
     *
     * CRITICAL FOR UKF: When averaging sigma points in the Unscented Transform,
     * angular states MUST use circular_mean() instead of arithmetic mean.
     * Otherwise, the filter will give completely wrong results (e.g., averaging
     * 179° and -179° gives 0° instead of ±180°).
     *
     * Example for a robot state [x, y, θ, v]:
     * @code
     *   bool state_is_angle[] = {false, false, true, false};  // θ is angle
     *   bool meas_is_angle[] = {true};  // bearing measurement is angle
     *   ukf_set_angle_states(&ukf, state_is_angle, 4, meas_is_angle, 1);
     * @endcode
     *
     * @param ukf               Pointer to UKF state structure
     * @param state_is_angle    Array [n] indicating if state component is angle (NULL disables)
     * @param n                 Number of state components
     * @param meas_is_angle     Array [m] indicating if measurement component is angle (NULL disables)
     * @param m                 Number of measurement components
     * @return                  true if successful, false if memory allocation failed
     */
    bool ukf_set_angle_states(UKF_State *ukf, const bool *state_is_angle, size_t n,
                              const bool *meas_is_angle, size_t m);

#ifdef __cplusplus
}
#endif

#endif /* UKF_H_ */
