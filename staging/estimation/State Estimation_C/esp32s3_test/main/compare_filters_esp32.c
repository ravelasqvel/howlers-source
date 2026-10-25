/*
 ============================================================================
 Name        : compare_filters_esp32.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Compare EKF, UKF, and PF on ESP32S3 with omnidirectional robot
               Measures execution time and RMSE for each filter
 ============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_chip_info.h"

#include "ekf.h"
#include "ukf.h"
#include "pf.h"
#include "robot_model.h"
#include "robot_data.h"

/* ============================================================================
 * WRAPPER FUNCTIONS FOR PARTICLE FILTER
 * ============================================================================ */

/**
 * @brief Wrapper for robot_dynamics to match PF signature
 * 
 * PF expects: (particle, u, noise, particle_out)
 * robot_dynamics provides: (x, u, x_out)
 * This wrapper adds the noise after calling robot_dynamics
 */
static void robot_dynamics_pf(const float *particle, const float *u,
                              const float *noise, float *particle_out)
{
    /* Apply dynamics */
    robot_dynamics(particle, u, particle_out);
    
    /* Add process noise */
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        particle_out[i] += noise[i];
    }
    
    /* Normalize angle state (psi is at index 2) */
    particle_out[2] = normalize_angle(particle_out[2]);
}

/* ============================================================================
 * PERFORMANCE METRICS
 * ============================================================================ */

typedef struct {
    float rmse_total;
    float rmse_x;
    float rmse_y;
    float rmse_psi;
    float rmse_vx;
    float rmse_vy;
    float rmse_omega;
    float avg_time_us;    /* Average time per iteration in microseconds */
    float total_time_us;  /* Total execution time in microseconds */
    float min_time_us;    /* Minimum time per iteration in microseconds */
    float max_time_us;    /* Maximum time per iteration in microseconds */
    float std_time_us;    /* Standard deviation of time in microseconds */
} FilterMetrics;

/**
 * @brief Compute RMSE between estimates and ground truth
 */
void compute_rmse(const float estimates[][STATE_DIM], const float ground_truth[][STATE_DIM],
                  size_t N, FilterMetrics *metrics)
{
    float sum_sq[STATE_DIM] = {0};
    
    for (size_t k = 0; k < N; k++)
    {
        for (size_t i = 0; i < STATE_DIM; i++)
        {
            float error = estimates[k][i] - ground_truth[k][i];
            
            /* Handle angle wrapping for psi (index 2) */
            if (i == 2)
            {
                error = normalize_angle(error);
            }
            
            sum_sq[i] += error * error;
        }
    }
    
    /* Compute RMSE for each state */
    metrics->rmse_x = sqrtf(sum_sq[0] / N);
    metrics->rmse_y = sqrtf(sum_sq[1] / N);
    metrics->rmse_psi = sqrtf(sum_sq[2] / N);
    metrics->rmse_vx = sqrtf(sum_sq[3] / N);
    metrics->rmse_vy = sqrtf(sum_sq[4] / N);
    metrics->rmse_omega = sqrtf(sum_sq[5] / N);
    
    /* Compute total RMSE */
    float total_sum = 0;
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        total_sum += sum_sq[i];
    }
    metrics->rmse_total = sqrtf(total_sum / (N * STATE_DIM));
}

/**
 * @brief Print filter metrics
 */
void print_metrics(const char *filter_name, const FilterMetrics *metrics)
{
    printf("\n%s Performance Metrics:\n", filter_name);
    printf("  RMSE Total:  %.6f m\n", metrics->rmse_total);
    printf("  RMSE X:      %.6f m\n", metrics->rmse_x);
    printf("  RMSE Y:      %.6f m\n", metrics->rmse_y);
    printf("  RMSE Psi:    %.6f rad (%.2f deg)\n", metrics->rmse_psi, 
           metrics->rmse_psi * 180.0f / M_PI);
    printf("  RMSE Vx:     %.6f m/s\n", metrics->rmse_vx);
    printf("  RMSE Vy:     %.6f m/s\n", metrics->rmse_vy);
    printf("  RMSE Omega:  %.6f rad/s\n", metrics->rmse_omega);
    printf("  Timing Statistics:\n");
    printf("    Avg Time:  %.2f us/iter (%.6f ms/iter)\n", 
           metrics->avg_time_us, metrics->avg_time_us / 1000.0f);
    printf("    Total:     %.2f ms (%.6f s)\n",
           metrics->total_time_us / 1000.0f, metrics->total_time_us / 1000000.0f);
    printf("    Min:       %.2f us/iter (%.6f ms/iter)\n",
           metrics->min_time_us, metrics->min_time_us / 1000.0f);
    printf("    Max:       %.2f us/iter (%.6f ms/iter)\n",
           metrics->max_time_us, metrics->max_time_us / 1000.0f);
    printf("    Std Dev:   %.2f us (%.6f ms)\n",
           metrics->std_time_us, metrics->std_time_us / 1000.0f);
}

/* ============================================================================
 * EKF COMPARISON
 * ============================================================================ */

void run_ekf_comparison(FilterMetrics *metrics)
{
    printf("\n========================================\n");
    printf("Running EKF...\n");
    printf("========================================\n");
    
    /* Initialize EKF */
    EKF_State ekf;
    if (!ekf_init(&ekf, STATE_DIM, MEAS_DIM, CONTROL_DIM,
                  robot_dynamics, robot_measurement,
                  robot_jacobian_F, robot_jacobian_H))
    {
        printf("[ERROR] EKF initialization failed\n");
        return;
    }
    
    /* Set initial state (zeros) */
    float x0[STATE_DIM] = {0};
    
    /* Set initial covariance (diagonal) */
    float P0[STATE_DIM * STATE_DIM] = {0};
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        P0[i * STATE_DIM + i] = P0_diagonal[i];
    }
    
    ekf_set_state(&ekf, x0, P0);
    
    /* Set noise covariances (diagonal) */
    float Q[STATE_DIM * STATE_DIM] = {0};
    float R[MEAS_DIM * MEAS_DIM] = {0};
    
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        Q[i * STATE_DIM + i] = Q_diagonal[i];
    }
    
    for (size_t i = 0; i < MEAS_DIM; i++)
    {
        R[i * MEAS_DIM + i] = R_diagonal[i];
    }
    
    ekf_set_noise(&ekf, Q, R);
    
    /* Enable angle handling for psi (state index 2, measurement index 3) */
    bool state_is_angle[STATE_DIM] = {false, false, true, false, false, false};
    bool meas_is_angle[MEAS_DIM] = {false, false, false, true};
    ekf_set_angle_states(&ekf, state_is_angle, meas_is_angle);
    
    /* Allocate estimates storage */
    float (*estimates)[STATE_DIM] = malloc(N_DATA_POINTS * sizeof(*estimates));
    if (estimates == NULL)
    {
        printf("[ERROR] Memory allocation failed\n");
        ekf_free(&ekf);
        return;
    }
    
    /* Store initial state */
    memcpy(estimates[0], ekf.x, STATE_DIM * sizeof(float));
    
    /* Allocate timing array */
    float *times_us = malloc((N_DATA_POINTS - 1) * sizeof(float));
    if (times_us == NULL)
    {
        printf("[ERROR] Timing array allocation failed\n");
        free(estimates);
        ekf_free(&ekf);
        return;
    }
    
    /* Run filter and measure time */
    int64_t total_time = 0;
    float min_time = INFINITY;
    float max_time = 0.0f;
    
    for (size_t k = 0; k < N_DATA_POINTS - 1; k++)
    {
        int64_t start_time = esp_timer_get_time();
        
        /* Predict */
        ekf_predict(&ekf, controls_data[k]);
        
        /* Update */
        ekf_update(&ekf, measurements_data[k + 1]);
        
        int64_t end_time = esp_timer_get_time();
        int64_t iteration_time = end_time - start_time;
        total_time += iteration_time;
        
        /* Store timing */
        times_us[k] = (float)iteration_time;
        if (times_us[k] < min_time) min_time = times_us[k];
        if (times_us[k] > max_time) max_time = times_us[k];
        
        /* Store estimate */
        memcpy(estimates[k + 1], ekf.x, STATE_DIM * sizeof(float));
        
        /* Progress indicator */
        if ((k + 1) % 50 == 0)
        {
            printf("  Progress: %zu/%zu\n", k + 1, N_DATA_POINTS - 1);
        }
    }
    
    /* Compute timing metrics */
    float avg_time = (float)total_time / (N_DATA_POINTS - 1);
    
    /* Compute standard deviation */
    float variance = 0.0f;
    for (size_t k = 0; k < N_DATA_POINTS - 1; k++)
    {
        float diff = times_us[k] - avg_time;
        variance += diff * diff;
    }
    float std_time = sqrtf(variance / (N_DATA_POINTS - 1));
    
    /* Compute RMSE metrics */
    compute_rmse(estimates, ground_truth_data, N_DATA_POINTS, metrics);
    
    /* Store timing metrics */
    metrics->avg_time_us = avg_time;
    metrics->total_time_us = (float)total_time;
    metrics->min_time_us = min_time;
    metrics->max_time_us = max_time;
    metrics->std_time_us = std_time;
    
    /* Cleanup */
    free(times_us);
    free(estimates);
    ekf_free(&ekf);
    
    printf("[OK] EKF completed successfully\n");
}

/* ============================================================================
 * UKF COMPARISON
 * ============================================================================ */

void run_ukf_comparison(FilterMetrics *metrics)
{
    printf("\n========================================\n");
    printf("Running UKF...\n");
    printf("========================================\n");
    
    /* UKF parameters */
    const float alpha = 0.5f;
    const float beta = 2.0f;
    const float kappa = 0.0f;
    
    /* Initialize UKF */
    UKF_State ukf;
    if (!ukf_init(&ukf, STATE_DIM, MEAS_DIM, CONTROL_DIM,
                  alpha, beta, kappa,
                  robot_dynamics, robot_measurement))
    {
        printf("[ERROR] UKF initialization failed\n");
        return;
    }
    
    printf("  Sigma points: %zu\n", ukf.params.num_sigmas);
    printf("  Lambda: %.4f\n", ukf.params.lambda);
    
    /* Set initial state (zeros) */
    float x0[STATE_DIM] = {0};
    
    /* Set initial covariance (diagonal) */
    float P0[STATE_DIM * STATE_DIM] = {0};
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        P0[i * STATE_DIM + i] = P0_diagonal[i];
    }
    
    ukf_set_state(&ukf, x0, P0);
    
    /* Set noise covariances (diagonal) */
    float Q[STATE_DIM * STATE_DIM] = {0};
    float R[MEAS_DIM * MEAS_DIM] = {0};
    
    for (size_t i = 0; i < STATE_DIM; i++)
    {
        Q[i * STATE_DIM + i] = Q_diagonal[i] * 1e4f;  /* Scale Q for UKF */
    }
    
    for (size_t i = 0; i < MEAS_DIM; i++)
    {
        R[i * MEAS_DIM + i] = R_diagonal[i];
    }
    
    ukf_set_noise(&ukf, Q, R);
    
    /* Enable angle handling */
    bool state_is_angle[STATE_DIM] = {false, false, true, false, false, false};
    bool meas_is_angle[MEAS_DIM] = {false, false, false, true};
    ukf_set_angle_states(&ukf, state_is_angle, STATE_DIM, meas_is_angle, MEAS_DIM);
    
    /* Allocate estimates storage */
    float (*estimates)[STATE_DIM] = malloc(N_DATA_POINTS * sizeof(*estimates));
    if (estimates == NULL)
    {
        printf("[ERROR] Memory allocation failed\n");
        ukf_free(&ukf);
        return;
    }
    
    /* Store initial state */
    memcpy(estimates[0], ukf.x, STATE_DIM * sizeof(float));
    
    /* Allocate timing array */
    float *times_us = malloc((N_DATA_POINTS - 1) * sizeof(float));
    if (times_us == NULL)
    {
        printf("[ERROR] Timing array allocation failed\n");
        free(estimates);
        ukf_free(&ukf);
        return;
    }
    
    /* Run filter and measure time */
    int64_t total_time = 0;
    float min_time = INFINITY;
    float max_time = 0.0f;
    
    for (size_t k = 0; k < N_DATA_POINTS - 1; k++)
    {
        int64_t start_time = esp_timer_get_time();
        
        /* Predict */
        ukf_predict(&ukf, controls_data[k]);
        
        /* Update */
        ukf_update(&ukf, measurements_data[k + 1]);
        
        int64_t end_time = esp_timer_get_time();
        int64_t iteration_time = end_time - start_time;
        total_time += iteration_time;
        
        /* Store timing */
        times_us[k] = (float)iteration_time;
        if (times_us[k] < min_time) min_time = times_us[k];
        if (times_us[k] > max_time) max_time = times_us[k];
        
        /* Store estimate */
        memcpy(estimates[k + 1], ukf.x, STATE_DIM * sizeof(float));
        
        /* Progress indicator */
        if ((k + 1) % 50 == 0)
        {
            printf("  Progress: %zu/%zu\n", k + 1, N_DATA_POINTS - 1);
        }
    }
    
    /* Compute timing metrics */
    float avg_time = (float)total_time / (N_DATA_POINTS - 1);
    
    /* Compute standard deviation */
    float variance = 0.0f;
    for (size_t k = 0; k < N_DATA_POINTS - 1; k++)
    {
        float diff = times_us[k] - avg_time;
        variance += diff * diff;
    }
    float std_time = sqrtf(variance / (N_DATA_POINTS - 1));
    
    /* Compute RMSE metrics */
    compute_rmse(estimates, ground_truth_data, N_DATA_POINTS, metrics);
    
    /* Store timing metrics */
    metrics->avg_time_us = avg_time;
    metrics->total_time_us = (float)total_time;
    metrics->min_time_us = min_time;
    metrics->max_time_us = max_time;
    metrics->std_time_us = std_time;
    
    /* Cleanup */
    free(times_us);
    free(estimates);
    ukf_free(&ukf);
    
    printf("[OK] UKF completed successfully\n");
}

/* ============================================================================
 * PF COMPARISON
 * ============================================================================ */

/* Measurement likelihood function for PF */
float robot_likelihood(const float *particle, const float *measurement,
                       const float *R, const void *user_data)
{
    /* Predict measurement from particle state */
    float z_pred[MEAS_DIM];
    robot_measurement(particle, z_pred);
    
    /* Compute Gaussian likelihood */
    float likelihood = 1.0f;
    
    for (size_t i = 0; i < MEAS_DIM; i++)
    {
        float residual = measurement[i] - z_pred[i];
        
        /* Handle angle wrapping for psi (index 3) */
        if (i == 3)
        {
            residual = normalize_angle(residual);
        }
        
        float std = sqrtf(R[i * MEAS_DIM + i] * 100.0f);  /* Scale R for PF */
        float exp_arg = -(residual * residual) / (2.0f * std * std);
        likelihood *= expf(exp_arg) / (sqrtf(2.0f * M_PI) * std);
    }
    
    return likelihood;
}

void run_pf_comparison(FilterMetrics *metrics)
{
    printf("\n========================================\n");
    printf("Running PF...\n");
    printf("========================================\n");
    
    const size_t N_PARTICLES = 2000;
    
    /* Initialize PF */
    PF_State pf;
    if (!pf_init(&pf, N_PARTICLES, STATE_DIM, CONTROL_DIM, MEAS_DIM,
                 robot_dynamics_pf, robot_likelihood, PF_RESAMPLE_SYSTEMATIC))
    {
        printf("[ERROR] PF initialization failed\n");
        return;
    }
    
    printf("  Particles: %zu\n", N_PARTICLES);
    
    /* Enable angle handling */
    bool state_is_angle[STATE_DIM] = {false, false, true, false, false, false};
    pf_set_angle_states(&pf, state_is_angle, STATE_DIM);
    
    /* Initialize particles around initial state */
    float mean[STATE_DIM] = {0};
    float std[STATE_DIM] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
    pf_create_gaussian_particles(&pf, mean, std);
    
    /* Set process noise */
    float process_std[STATE_DIM] = {0.01f, 0.01f, 0.01f, 0.05f, 0.05f, 0.05f};
    pf_set_process_noise(&pf, process_std);
    
    /* Measurement noise */
    float R[MEAS_DIM * MEAS_DIM] = {0};
    for (size_t i = 0; i < MEAS_DIM; i++)
    {
        R[i * MEAS_DIM + i] = R_diagonal[i];
    }
    
    /* Allocate estimates storage */
    float (*estimates)[STATE_DIM] = malloc(N_DATA_POINTS * sizeof(*estimates));
    if (estimates == NULL)
    {
        printf("[ERROR] Memory allocation failed\n");
        pf_free(&pf);
        return;
    }
    
    /* Store initial estimate */
    pf_estimate(&pf);
    memcpy(estimates[0], pf.state_mean, STATE_DIM * sizeof(float));
    
    /* Allocate timing array */
    float *times_us = malloc((N_DATA_POINTS - 1) * sizeof(float));
    if (times_us == NULL)
    {
        printf("[ERROR] Timing array allocation failed\n");
        free(estimates);
        pf_free(&pf);
        return;
    }
    
    /* Run filter and measure time */
    int64_t total_time = 0;
    float min_time = INFINITY;
    float max_time = 0.0f;
    
    for (size_t k = 0; k < N_DATA_POINTS - 1; k++)
    {
        int64_t start_time = esp_timer_get_time();
        
        /* Predict */
        pf_predict(&pf, controls_data[k]);
        
        /* Update */
        pf_update(&pf, measurements_data[k + 1], R, NULL);
        
        /* Resample if needed */
        pf_resample_if_needed(&pf);
        
        /* Estimate */
        pf_estimate(&pf);
        
        int64_t end_time = esp_timer_get_time();
        int64_t iteration_time = end_time - start_time;
        total_time += iteration_time;
        
        /* Store timing */
        times_us[k] = (float)iteration_time;
        if (times_us[k] < min_time) min_time = times_us[k];
        if (times_us[k] > max_time) max_time = times_us[k];
        
        /* Store estimate */
        memcpy(estimates[k + 1], pf.state_mean, STATE_DIM * sizeof(float));
        
        /* Yield to scheduler every iteration to prevent watchdog timeout */
        vTaskDelay(1);  /* 1 tick delay (~10ms) allows IDLE task to run and reset watchdog */
        
        /* Progress indicator */
        if ((k + 1) % 50 == 0)
        {
            float neff = pf_neff(pf.weights, pf.params.N);
            printf("  Progress: %zu/%zu, N_eff: %.1f\n", k + 1, N_DATA_POINTS - 1, neff);
        }
    }
    
    /* Compute timing metrics */
    float avg_time = (float)total_time / (N_DATA_POINTS - 1);
    
    /* Compute standard deviation */
    float variance = 0.0f;
    for (size_t k = 0; k < N_DATA_POINTS - 1; k++)
    {
        float diff = times_us[k] - avg_time;
        variance += diff * diff;
    }
    float std_time = sqrtf(variance / (N_DATA_POINTS - 1));
    
    /* Compute RMSE metrics */
    compute_rmse(estimates, ground_truth_data, N_DATA_POINTS, metrics);
    
    /* Store timing metrics */
    metrics->avg_time_us = avg_time;
    metrics->total_time_us = (float)total_time;
    metrics->min_time_us = min_time;
    metrics->max_time_us = max_time;
    metrics->std_time_us = std_time;
    
    /* Cleanup */
    free(times_us);
    free(estimates);
    pf_free(&pf);
    
    printf("[OK] PF completed successfully\n");
}

/* ============================================================================
 * MAIN APPLICATION
 * ============================================================================ */

void app_main(void)
{
    printf("\n");
    printf("============================================================\n");
    printf("  ESP32S3 State Estimation Filter Comparison\n");
    printf("  Omnidirectional Robot Model\n");
    printf("============================================================\n\n");
    
    /* Print chip info */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("Chip: %s with %d CPU cores, %.1f MHz\n",
           CONFIG_IDF_TARGET, chip_info.cores, 240.0f);
    printf("Free heap: %lu bytes\n\n", esp_get_free_heap_size());
    
    /* Print dataset info */
    printf("Dataset Information:\n");
    printf("  Data points: %d\n", N_DATA_POINTS);
    printf("  Duration: %.2f seconds\n", TOTAL_DURATION);
    printf("  Sample rate: %.0f Hz\n", 1.0f / DT);
    printf("  State dimension: %d\n", STATE_DIM);
    printf("  Measurement dimension: %d\n", MEAS_DIM);
    printf("  Control dimension: %d\n\n", CONTROL_DIM);
    
    /* Filter metrics storage */
    FilterMetrics ekf_metrics, ukf_metrics, pf_metrics;
    
    /* Run comparisons */
    run_ekf_comparison(&ekf_metrics);
    run_ukf_comparison(&ukf_metrics);
    run_pf_comparison(&pf_metrics);
    
    /* Print comparison summary */
    printf("\n");
    printf("============================================================\n");
    printf("  COMPARISON SUMMARY\n");
    printf("============================================================\n");
    
    print_metrics("EKF", &ekf_metrics);
    print_metrics("UKF", &ukf_metrics);
    print_metrics("PF", &pf_metrics);
    
    /* Print comparison table - Accuracy */
    printf("\n");
    printf("Accuracy Comparison:\n");
    printf("==========================================================================\n");
    printf("Filter | RMSE Total [m] | RMSE X [m] | RMSE Y [m] | RMSE Psi [rad]\n");
    printf("--------------------------------------------------------------------------\n");
    printf("EKF    | %.6f       | %.6f   | %.6f   | %.6f\n",
           ekf_metrics.rmse_total, ekf_metrics.rmse_x, ekf_metrics.rmse_y,
           ekf_metrics.rmse_psi);
    printf("UKF    | %.6f       | %.6f   | %.6f   | %.6f\n",
           ukf_metrics.rmse_total, ukf_metrics.rmse_x, ukf_metrics.rmse_y,
           ukf_metrics.rmse_psi);
    printf("PF     | %.6f       | %.6f   | %.6f   | %.6f\n",
           pf_metrics.rmse_total, pf_metrics.rmse_x, pf_metrics.rmse_y,
           pf_metrics.rmse_psi);
    printf("==========================================================================\n");
    
    /* Print comparison table - Timing */
    printf("\n");
    printf("Timing Comparison:\n");
    printf("====================================================================================\n");
    printf("Filter | Avg [ms/it] | Total [s] | Min [ms/it] | Max [ms/it] | Std Dev [ms]\n");
    printf("------------------------------------------------------------------------------------\n");
    printf("EKF    | %.6f    | %.6f  | %.6f    | %.6f    | %.6f\n",
           ekf_metrics.avg_time_us / 1000.0f, ekf_metrics.total_time_us / 1000000.0f,
           ekf_metrics.min_time_us / 1000.0f, ekf_metrics.max_time_us / 1000.0f,
           ekf_metrics.std_time_us / 1000.0f);
    printf("UKF    | %.6f    | %.6f  | %.6f    | %.6f    | %.6f\n",
           ukf_metrics.avg_time_us / 1000.0f, ukf_metrics.total_time_us / 1000000.0f,
           ukf_metrics.min_time_us / 1000.0f, ukf_metrics.max_time_us / 1000.0f,
           ukf_metrics.std_time_us / 1000.0f);
    printf("PF     | %.6f    | %.6f  | %.6f    | %.6f    | %.6f\n",
           pf_metrics.avg_time_us / 1000.0f, pf_metrics.total_time_us / 1000000.0f,
           pf_metrics.min_time_us / 1000.0f, pf_metrics.max_time_us / 1000.0f,
           pf_metrics.std_time_us / 1000.0f);
    printf("====================================================================================\n");
    
    printf("\n");
    printf("============================================================\n");
    printf("  Comparison Complete!\n");
    printf("============================================================\n");
}
