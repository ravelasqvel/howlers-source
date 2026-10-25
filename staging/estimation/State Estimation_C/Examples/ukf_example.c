/*
 ============================================================================
 Name        : ukf_example.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : UKF example - Constant Velocity 2D Tracking
 ============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "../UKF/ukf.h"

/* ============================================================================
 * CONSTANT VELOCITY MODEL FUNCTIONS
 * ============================================================================ */

/**
 * @brief State transition function for constant velocity model
 * State: [px, vx, py, vy]
 * 
 * NOTE: dt is embedded in this model function
 */
void f_cv(const float *x, const float *u, float *x_out)
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
void h_cv(const float *x, float *z_out)
{
    z_out[0] = x[0]; /* Measure px */
    z_out[1] = x[2]; /* Measure py */
}

/**
 * @brief Generate simulated noisy measurements
 */
void generate_measurement(float true_px, float true_py, float noise_std, float *z)
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
 * @brief Run the UKF example - can be called from main menu or standalone
 * @return 0 on success, 1 on failure
 */
int run_ukf_example(void)
{
    printf("========================================\n");
    printf("Unscented Kalman Filter (UKF) Example\n");
    printf("Constant Velocity 2D Tracking\n");
    printf("========================================\n\n");

    /* ========================================
     * UKF CONFIGURATION
     * ======================================== */

    const size_t n = 4;     /* State: [px, vx, py, vy] */
    const size_t m = 2;     /* Measurement: [px, py] */
    const size_t dim_u = 0; /* No control input */

    const float alpha = 0.1f;
    const float beta = 2.0f;
    const float kappa = 1.0f;
    const float dt = 1.0f;  /* For simulation only, not passed to filter */

    UKF_State ukf;
    bool init_success = ukf_init(&ukf, n, m, dim_u, alpha, beta, kappa, f_cv, h_cv);

    if (!init_success)
    {
        printf("ERROR: Failed to initialize UKF!\n");
        return 1;
    }

    printf("[OK] UKF initialized successfully\n");
    printf("  State dimension: %zu\n", n);
    printf("  Measurement dimension: %zu\n", m);
    printf("  Control dimension: %zu\n", dim_u);
    printf("  Sigma points: %zu\n", ukf.params.num_sigmas);
    printf("  Parameters: alpha=%.3f, beta=%.1f, kappa=%.1f, lambda=%.3f\n\n",
           alpha, beta, kappa, ukf.params.lambda);

    /* ========================================
     * SET INITIAL STATE
     * ======================================== */

    float x_init[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float P_init[16] = {
        100.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 100.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 100.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 100.0f};

    ukf_set_state(&ukf, x_init, P_init);
    printf("[OK] Initial state set: x = [%.1f, %.1f, %.1f, %.1f]\n\n",
           x_init[0], x_init[1], x_init[2], x_init[3]);

    /* ========================================
     * SET NOISE COVARIANCES
     * ======================================== */

    float Q[16] = {
        0.02f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.02f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.02f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.02f};

    float measurement_std = 0.3f;
    float R[4] = {
        measurement_std * measurement_std, 0.0f,
        0.0f, measurement_std * measurement_std};

    ukf_set_noise(&ukf, Q, R);
    printf("[OK] Noise covariances set\n");
    printf("  Process noise Q: variance = %.3f\n", Q[0]);
    printf("  Measurement noise R: std = %.3f meters\n\n", measurement_std);

    /* ========================================
     * SIMULATION SETUP
     * ======================================== */

    const float true_vx = 1.0f;
    const float true_vy = 0.5f;
    float true_px = 0.0f;
    float true_py = 0.0f;

    const int num_steps = 20;
    srand((unsigned int)time(NULL));

    printf("========================================\n");
    printf("SIMULATION: Tracking object with UKF\n");
    printf("========================================\n");
    printf("True velocity: vx=%.2f m/s, vy=%.2f m/s\n", true_vx, true_vy);
    printf("Time steps: %d (dt=%.1f sec)\n\n", num_steps, dt);

    printf("Step |  True Position  | Measurement Pos | Estimated Position | Est. Velocity\n");
    printf("-----|-----------------|-----------------|-------------------|---------------\n");

    /* ========================================
     * MAIN FILTER LOOP
     * ======================================== */

    for (int step = 0; step < num_steps; step++)
    {
        true_px += true_vx * dt;
        true_py += true_vy * dt;

        float z[2];
        generate_measurement(true_px, true_py, measurement_std, z);

        if (!ukf_predict(&ukf, NULL))  /* NULL = no control input */
        {
            printf("ERROR: Predict step failed at step %d\n", step);
            ukf_free(&ukf);
            return 1;
        }

        if (!ukf_update(&ukf, z))
        {
            printf("ERROR: Update step failed at step %d\n", step);
            ukf_free(&ukf);
            return 1;
        }

        printf(" %2d  | (%5.2f, %5.2f) | (%5.2f, %5.2f)  | (%5.2f, %5.2f)    | (%4.2f, %4.2f)\n",
               step + 1,
               true_px, true_py,
               z[0], z[1],
               ukf.x[0], ukf.x[2],
               ukf.x[1], ukf.x[3]);
    }

    /* ========================================
     * FINAL RESULTS
     * ======================================== */

    printf("\n========================================\n");
    printf("FINAL RESULTS\n");
    printf("========================================\n\n");

    float pos_error_x = fabsf(ukf.x[0] - true_px);
    float pos_error_y = fabsf(ukf.x[2] - true_py);
    float vel_error_x = fabsf(ukf.x[1] - true_vx);
    float vel_error_y = fabsf(ukf.x[3] - true_vy);

    printf("Position Error:\n");
    printf("  x: %.4f meters\n", pos_error_x);
    printf("  y: %.4f meters\n", pos_error_y);
    printf("\nVelocity Error:\n");
    printf("  vx: %.4f m/s\n", vel_error_x);
    printf("  vy: %.4f m/s\n", vel_error_y);

    printf("\nTrue Final State:\n");
    printf("  Position: (%.2f, %.2f)\n", true_px, true_py);
    printf("  Velocity: (%.2f, %.2f)\n\n", true_vx, true_vy);

    printf("Estimated Uncertainties (1-sigma):\n");
    printf("  std_px: %.4f meters\n", sqrtf(ukf.P[0 * 4 + 0]));
    printf("  std_vx: %.4f m/s\n", sqrtf(ukf.P[1 * 4 + 1]));
    printf("  std_py: %.4f meters\n", sqrtf(ukf.P[2 * 4 + 2]));
    printf("  std_vy: %.4f m/s\n\n", sqrtf(ukf.P[3 * 4 + 3]));

    /* ========================================
     * CLEANUP
     * ======================================== */

    ukf_free(&ukf);
    printf("[OK] UKF cleaned up successfully\n\n");
    printf("========================================\n");
    printf("Simulation complete!\n");
    printf("========================================\n");

    return 0;
}

/* ============================================================================
 * STANDALONE MAIN (only used when compiled without menu)
 * ============================================================================ */
#ifndef BUILD_WITH_MENU
int main(void)
{
    return run_ukf_example();
}
#endif
