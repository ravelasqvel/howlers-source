/*
 ============================================================================
 Name        : pf_example.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Particle Filter example - Robot Localization with Landmarks
 ============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../PF/pf.h"

/* Landmarks in the environment (x, y positions) */
#define NUM_LANDMARKS 4
static const float landmarks[NUM_LANDMARKS * 2] = {
    -1.0f, 2.0f,  /* Landmark 0 */
    5.0f, 10.0f,  /* Landmark 1 */
    12.0f, 14.0f, /* Landmark 2 */
    18.0f, 21.0f  /* Landmark 3 */
};

/* ============================================================================
 * STATE TRANSITION FUNCTION
 * ============================================================================ */

/**
 * @brief Robot state transition with embedded time step
 * State: [x, y, heading]
 * Control: [angular_vel, linear_vel]
 * 
 * NOTE: dt is embedded in this model (1.0 second)
 */
void robot_state_transition(const float *particle, const float *u,
                            const float *noise, float *particle_out)
{
    /* Time step embedded in model */
    const float dt = 1.0f;
    
    float x = particle[0];
    float y = particle[1];
    float heading = particle[2];

    heading += u[0] + noise[0];
    float distance = (u[1] * dt) + noise[1];

    x += cosf(heading) * distance;
    y += sinf(heading) * distance;

    particle_out[0] = x;
    particle_out[1] = y;
    particle_out[2] = heading;
}

/* ============================================================================
 * MEASUREMENT LIKELIHOOD FUNCTION
 * ============================================================================ */

/**
 * @brief Compute measurement likelihood for robot localization
 * 
 * user_data should point to a structure containing landmarks information
 */
typedef struct {
    const float *landmarks;  /* Landmark positions [num_landmarks x 2] */
    size_t num_landmarks;    /* Number of landmarks */
} LandmarkData;

float robot_measurement_likelihood(const float *particle,
                                   const float *measurements,
                                   const float *R,
                                   const void *user_data)
{
    /* Extract landmark data */
    const LandmarkData *lm_data = (const LandmarkData *)user_data;
    const float *landmarks_ptr = lm_data->landmarks;
    size_t num_landmarks = lm_data->num_landmarks;
    
    /* R is measurement noise covariance matrix (diagonal assumed) */
    float measurement_std = sqrtf(R[0]);  /* Assuming diagonal R */
    
    float likelihood = 1.0f;
    float two_var = 2.0f * measurement_std * measurement_std;
    float norm_factor = 1.0f / sqrtf(2.0f * M_PI * measurement_std * measurement_std);

    for (size_t i = 0; i < num_landmarks; i++)
    {
        float lx = landmarks_ptr[i * 2];
        float ly = landmarks_ptr[i * 2 + 1];

        float dx = particle[0] - lx;
        float dy = particle[1] - ly;
        float distance = sqrtf(dx * dx + dy * dy);

        float diff = distance - measurements[i];
        float exponent = -(diff * diff) / two_var;
        float prob = norm_factor * expf(exponent);

        likelihood *= prob;
    }

    return likelihood;
}

/* ============================================================================
 * SIMULATION HELPER
 * ============================================================================ */

void simulate_measurements(const float *true_pos, float *measurements,
                           float sensor_std, size_t num_landmarks)
{
    for (size_t i = 0; i < num_landmarks; i++)
    {
        float lx = landmarks[i * 2];
        float ly = landmarks[i * 2 + 1];

        float dx = true_pos[0] - lx;
        float dy = true_pos[1] - ly;
        float true_distance = sqrtf(dx * dx + dy * dy);

        float noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * sensor_std;
        measurements[i] = true_distance + noise;
    }
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

/**
 * @brief Run the Particle Filter example - can be called from main menu or standalone
 * @return 0 on success, 1 on failure
 */
int run_pf_example(void)
{
    printf("========================================\n");
    printf("Particle Filter (PF) Example\n");
    printf("Robot Localization with Landmarks\n");
    printf("========================================\n\n");

    /* Configuration */
    const size_t N = 5000;
    const size_t state_dim = 3;
    const size_t dim_u = 2;  /* Control dimension: [angular_vel, linear_vel] */
    const size_t measurement_dim = NUM_LANDMARKS;
    const float dt = 1.0f;  /* For simulation only, not passed to filter */

    float process_std[3] = {0.2f, 0.05f, 0.0f};
    const float sensor_std_err = 0.1f;
    float u[2] = {0.0f, 1.414f};
    const size_t num_iters = 18;
    
    /* Measurement noise covariance (diagonal matrix) */
    float R[NUM_LANDMARKS * NUM_LANDMARKS];
    for (size_t i = 0; i < NUM_LANDMARKS * NUM_LANDMARKS; i++) R[i] = 0.0f;
    for (size_t i = 0; i < NUM_LANDMARKS; i++) {
        R[i * NUM_LANDMARKS + i] = sensor_std_err * sensor_std_err;
    }
    
    /* Setup landmark data structure for user_data */
    LandmarkData lm_data = {
        .landmarks = landmarks,
        .num_landmarks = NUM_LANDMARKS
    };

    /* Initialize Particle Filter */
    PF_State pf;
    bool success = pf_init(&pf, N, state_dim, dim_u, measurement_dim,
                           robot_state_transition,
                           robot_measurement_likelihood,
                           PF_RESAMPLE_SYSTEMATIC);

    if (!success)
    {
        printf("ERROR: Failed to initialize particle filter\n");
        return 1;
    }

    printf("Initialized particle filter:\n");
    printf("  Particles: %zu\n", N);
    printf("  State dim: %zu (x, y, heading)\n", state_dim);
    printf("  Control dim: %zu\n", dim_u);
    printf("  Resampling: Systematic\n\n");

    pf_set_process_noise(&pf, process_std);

    /* Create initial particles */
    float initial_mean[3] = {0.0f, 0.0f, M_PI / 4.0f};
    float initial_std[3] = {5.0f, 5.0f, M_PI / 4.0f};
    pf_create_gaussian_particles(&pf, initial_mean, initial_std);

    printf("Created %zu particles with Gaussian distribution\n", N);
    printf("  Mean: [%.2f, %.2f, %.2f]\n",
           initial_mean[0], initial_mean[1], initial_mean[2]);
    printf("  Std:  [%.2f, %.2f, %.2f]\n\n",
           initial_std[0], initial_std[1], initial_std[2]);

    /* Display landmarks */
    printf("Landmarks:\n");
    for (size_t i = 0; i < NUM_LANDMARKS; i++)
    {
        printf("  [%zu]: (%.1f, %.1f)\n", i,
               landmarks[i * 2], landmarks[i * 2 + 1]);
    }
    printf("\n");

    printf("Starting simulation...\n");
    printf("----------------------------------------\n");

    /* Simulation */
    float robot_pos[3] = {0.0f, 0.0f, M_PI / 4.0f};
    float measurements[NUM_LANDMARKS];

    for (size_t iter = 0; iter < num_iters; iter++)
    {
        robot_pos[0] += 1.0f;
        robot_pos[1] += 1.0f;

        simulate_measurements(robot_pos, measurements, sensor_std_err, NUM_LANDMARKS);

        pf_predict(&pf, u);
        pf_update(&pf, measurements, R, &lm_data);  /* Pass R matrix and landmark data */
        bool resampled = pf_resample_if_needed(&pf);
        pf_estimate(&pf);

        printf("Iter %2zu: True=[%5.2f, %5.2f] Est=[%5.2f, %5.2f] ",
               iter + 1,
               robot_pos[0], robot_pos[1],
               pf.state_mean[0], pf.state_mean[1]);

        float error_x = robot_pos[0] - pf.state_mean[0];
        float error_y = robot_pos[1] - pf.state_mean[1];
        float error = sqrtf(error_x * error_x + error_y * error_y);

        printf("Err=%.3f %s\n", error, resampled ? "[RESAMPLED]" : "");
    }

    printf("----------------------------------------\n\n");

    /* Final Results */
    printf("Final Results:\n");
    printf("  True Position:     [%.2f, %.2f, %.2f]\n",
           robot_pos[0], robot_pos[1], robot_pos[2]);
    printf("  Estimated Mean:    [%.2f, %.2f, %.2f]\n",
           pf.state_mean[0], pf.state_mean[1], pf.state_mean[2]);
    printf("  Estimated Std Dev: [%.2f, %.2f, %.2f]\n",
           sqrtf(pf.state_variance[0]),
           sqrtf(pf.state_variance[1]),
           sqrtf(pf.state_variance[2]));

    float final_error_x = robot_pos[0] - pf.state_mean[0];
    float final_error_y = robot_pos[1] - pf.state_mean[1];
    float final_error = sqrtf(final_error_x * final_error_x +
                              final_error_y * final_error_y);
    printf("  Final Position Error: %.4f\n", final_error);

    float n_eff = pf_neff(pf.weights, N);
    printf("  Effective N: %.2f (%.1f%% of total)\n\n",
           n_eff, 100.0f * n_eff / (float)N);

    /* Cleanup */
    pf_free(&pf);

    printf("========================================\n");
    printf("Particle Filter example completed!\n");
    printf("========================================\n");

    return 0;
}

/* ============================================================================
 * STANDALONE MAIN (only used when compiled without menu)
 * ============================================================================ */
#ifndef BUILD_WITH_MENU
int main(void)
{
    return run_pf_example();
}
#endif
