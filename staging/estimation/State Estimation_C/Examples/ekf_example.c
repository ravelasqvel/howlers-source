/*
 ============================================================================
 Name        : ekf_example.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : EKF example - Constant Velocity 2D Tracking
 ============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "../EKF/ekf.h"

/* ============================================================================
 * CONSTANT VELOCITY MODEL FUNCTIONS
 * ============================================================================ */

/**
 * @brief State transition function for constant velocity model
 * State: [px, vx, py, vy]
 * 
 * NOTE: dt is embedded in this model function
 */
static void f_cv(const float *x, const float *u, float *x_out)
{
    /* Time step embedded in the model (discretization) */
    const float dt = 1.0f;
    
    x_out[0] = x[0] + x[1] * dt; /* px = px + vx*dt */
    x_out[1] = x[1];             /* vx = vx */
    x_out[2] = x[2] + x[3] * dt; /* py = py + vy*dt */
    x_out[3] = x[3];             /* vy = vy */
}

/**
 * @brief Measurement function - measure position only
 * Measurement: [px, py]
 */
static void h_cv(const float *x, float *z_out)
{
    z_out[0] = x[0]; /* Measure px */
    z_out[1] = x[2]; /* Measure py */
}

/**
 * @brief State Jacobian for constant velocity model
 * F = ∂f/∂x
 *
 * For state [px, vx, py, vy]:
 * F = [1  dt  0   0 ]
 *     [0  1   0   0 ]
 *     [0  0   1  dt ]
 *     [0  0   0   1 ]
 */
static void jacobian_f_cv(const float *x, const float *u, float *F)
{
    /* Time step embedded in the model */
    const float dt = 1.0f;
    
    /* Initialize to zero */
    for (int i = 0; i < 16; i++)
        F[i] = 0.0f;

    /* F is 4x4 matrix (row-major) */
    F[0 * 4 + 0] = 1.0f; /* ∂px/∂px = 1 */
    F[0 * 4 + 1] = dt;   /* ∂px/∂vx = dt */
    F[1 * 4 + 1] = 1.0f; /* ∂vx/∂vx = 1 */
    F[2 * 4 + 2] = 1.0f; /* ∂py/∂py = 1 */
    F[2 * 4 + 3] = dt;   /* ∂py/∂vy = dt */
    F[3 * 4 + 3] = 1.0f; /* ∂vy/∂vy = 1 */
}

/**
 * @brief Measurement Jacobian for position-only measurement
 * H = ∂h/∂x
 *
 * For measurement [px, py] from state [px, vx, py, vy]:
 * H = [1  0  0  0]
 *     [0  0  1  0]
 */
static void jacobian_h_cv(const float *x, float *H)
{
    /* Initialize to zero */
    for (int i = 0; i < 8; i++)
        H[i] = 0.0f;

    /* H is 2x4 matrix (row-major) */
    H[0 * 4 + 0] = 1.0f; /* ∂z1/∂px = 1 (measure px) */
    H[1 * 4 + 2] = 1.0f; /* ∂z2/∂py = 1 (measure py) */
}

/**
 * @brief Generate simulated noisy measurements
 */
static void generate_measurement(float true_px, float true_py, float noise_std, float *z)
{
    float noise_x = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * noise_std;
    float noise_y = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * noise_std;
    z[0] = true_px + noise_x;
    z[1] = true_py + noise_y;
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

/**
 * @brief Run the EKF example - can be called from main menu or standalone
 * @return 0 on success, 1 on failure
 */
int run_ekf_example(void)
{
    printf("========================================\n");
    printf("Extended Kalman Filter (EKF) Example\n");
    printf("Constant Velocity 2D Tracking\n");
    printf("========================================\n\n");

    /* ========================================
     * EKF CONFIGURATION
     * ======================================== */

    const size_t n = 4;     /* State: [px, vx, py, vy] */
    const size_t m = 2;     /* Measurement: [px, py] */
    const size_t dim_u = 0; /* No control input */
    const float dt = 1.0f;  /* For simulation only */

    /* Initialize random number generator */
    srand((unsigned int)time(NULL));

    /* ========================================
     * CREATE AND INITIALIZE EKF
     * ======================================== */

    EKF_State ekf;
    if (!ekf_init(&ekf, n, m, dim_u, f_cv, h_cv, jacobian_f_cv, jacobian_h_cv))
    {
        fprintf(stderr, "[ERROR] Failed to initialize EKF\n");
        return 1;
    }

    /* ========================================
     * SET INITIAL STATE
     * ======================================== */

    /* Initial state: start at origin with velocity [2, 3] */
    float x0[4] = {0.0f, 2.0f, 0.0f, 3.0f}; /* [px, vx, py, vy] */

    /* Initial covariance (uncertainty in initial state) */
    float P0[16] = {
        10.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 5.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 10.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 5.0f};

    ekf_set_state(&ekf, x0, P0);

    /* ========================================
     * SET NOISE PARAMETERS
     * ======================================== */

    /* Process noise covariance Q (model uncertainty) */
    float Q[16] = {
        0.1f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.1f};

    /* Measurement noise covariance R (sensor uncertainty) */
    float R[4] = {
        1.0f, 0.0f,
        0.0f, 1.0f};

    ekf_set_noise(&ekf, Q, R);

    printf("[INFO] EKF initialized with:\n");
    printf("  - State dimension:       %zu\n", n);
    printf("  - Measurement dimension: %zu\n", m);
    printf("  - Control dimension:     %zu\n", dim_u);
    printf("  - Initial state:         [%.2f, %.2f, %.2f, %.2f]\n",
           x0[0], x0[1], x0[2], x0[3]);
    printf("  - Process noise std:     %.2f\n", sqrtf(Q[0]));
    printf("  - Measurement noise std: %.2f\n\n", sqrtf(R[0]));

    /* ========================================
     * SIMULATION LOOP
     * ======================================== */

    const int num_steps = 20;
    float true_state[4]; /* True state for simulation */
    memcpy(true_state, x0, sizeof(true_state));

    printf("Starting tracking simulation...\n");
    printf("========================================\n");
    printf("Step | True Pos      | Measured Pos  | Estimated Pos | Error\n");
    printf("-----|---------------|---------------|---------------|-------\n");

    for (int step = 1; step <= num_steps; step++)
    {
        /* ========================================
         * SIMULATE TRUE STATE EVOLUTION
         * ======================================== */

        /* Propagate true state with some process noise */
        float process_noise = ((float)rand() / RAND_MAX - 0.5f) * 0.2f;
        f_cv(true_state, NULL, true_state);  /* NULL = no control */
        true_state[1] += process_noise; /* Add noise to vx */
        true_state[3] += process_noise; /* Add noise to vy */

        /* ========================================
         * PREDICTION STEP
         * ======================================== */

        ekf_predict(&ekf, NULL);  /* NULL = no control input */

        /* ========================================
         * GENERATE MEASUREMENT
         * ======================================== */

        float z[2];
        generate_measurement(true_state[0], true_state[2], sqrtf(R[0]), z);

        /* ========================================
         * UPDATE STEP
         * ======================================== */

        ekf_update(&ekf, z);

        /* ========================================
         * GET ESTIMATE AND DISPLAY RESULTS
         * ======================================== */

        float x_est[4];
        ekf_get_state(&ekf, x_est, NULL);

        /* Calculate position error */
        float error_x = true_state[0] - x_est[0];
        float error_y = true_state[2] - x_est[2];
        float error = sqrtf(error_x * error_x + error_y * error_y);

        printf("%4d | (%5.1f,%5.1f) | (%5.1f,%5.1f) | (%5.1f,%5.1f) | %.2f\n",
               step,
               true_state[0], true_state[2], /* True position */
               z[0], z[1],                   /* Measured position */
               x_est[0], x_est[2],           /* Estimated position */
               error);                       /* Position error */
    }

    /* ========================================
     * FINAL RESULTS
     * ======================================== */

    float x_final[4], P_final[16];
    ekf_get_state(&ekf, x_final, P_final);

    printf("========================================\n\n");
    printf("Final Results:\n");
    printf("  True State:      [px=%.2f, vx=%.2f, py=%.2f, vy=%.2f]\n",
           true_state[0], true_state[1], true_state[2], true_state[3]);
    printf("  Estimated State: [px=%.2f, vx=%.2f, py=%.2f, vy=%.2f]\n",
           x_final[0], x_final[1], x_final[2], x_final[3]);

    float pos_error_x = true_state[0] - x_final[0];
    float pos_error_y = true_state[2] - x_final[2];
    float pos_error = sqrtf(pos_error_x * pos_error_x + pos_error_y * pos_error_y);

    float vel_error_x = true_state[1] - x_final[1];
    float vel_error_y = true_state[3] - x_final[3];
    float vel_error = sqrtf(vel_error_x * vel_error_x + vel_error_y * vel_error_y);

    printf("\n  Position Error:  %.3f\n", pos_error);
    printf("  Velocity Error:  %.3f\n", vel_error);

    printf("\n  Final Covariance (diagonal):\n");
    printf("    P[px] = %.3f\n", P_final[0]);
    printf("    P[vx] = %.3f\n", P_final[5]);
    printf("    P[py] = %.3f\n", P_final[10]);
    printf("    P[vy] = %.3f\n", P_final[15]);

    /* ========================================
     * CLEANUP
     * ======================================== */

    ekf_free(&ekf);
    printf("\n[SUCCESS] EKF example completed successfully!\n");
    printf("========================================\n\n");

    return 0;
}

/* ============================================================================
 * STANDALONE MAIN (only when compiled without menu)
 * ============================================================================ */

#ifndef BUILD_WITH_MENU
int main(void)
{
    return run_ekf_example();
}
#endif
