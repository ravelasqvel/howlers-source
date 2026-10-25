/*
 ============================================================================
 Name        : ekf.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Extended Kalman Filter (EKF) Implementation in C
 ============================================================================
*/

#include "ekf.h"
#include "../LinearAlgebra/angle_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================
 * MEMORY ALLOCATION HELPERS
 * ============================================================================ */

/**
 * @brief Allocate a float array with error checking
 */
static float *allocate_array(size_t size, const char *name)
{
    float *arr = (float *)calloc(size, sizeof(float));
    if (arr == NULL && size > 0)
    {
        fprintf(stderr, "[EKF ERROR] Failed to allocate memory for %s\n", name);
    }
    return arr;
}

/* ============================================================================
 * INITIALIZATION AND CLEANUP
 * ============================================================================ */

bool ekf_init(EKF_State *ekf, size_t n, size_t m, size_t dim_u,
              ekf_state_transition_fn f, ekf_measurement_fn h,
              ekf_jacobian_f_fn jac_f, ekf_jacobian_h_fn jac_h)
{
    if (ekf == NULL || n == 0 || m == 0)
    {
        fprintf(stderr, "[EKF ERROR] Invalid initialization parameters\n");
        return false;
    }

    if (f == NULL || h == NULL || jac_f == NULL || jac_h == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Function pointers cannot be NULL\n");
        return false;
    }

    /* Initialize dimensions */
    ekf->n = n;
    ekf->m = m;
    ekf->dim_u = dim_u;

    /* Set function pointers */
    ekf->f = f;
    ekf->h = h;
    ekf->jac_f = jac_f;
    ekf->jac_h = jac_h;

    /* Allocate state and covariance */
    ekf->x = allocate_array(n, "state x");
    ekf->P = allocate_array(n * n, "covariance P");

    /* Allocate noise matrices */
    ekf->Q = allocate_array(n * n, "process noise Q");
    ekf->R = allocate_array(m * m, "measurement noise R");

    /* Allocate Jacobian matrices */
    ekf->F = allocate_array(n * n, "state Jacobian F");
    ekf->H = allocate_array(m * n, "measurement Jacobian H");

    /* Allocate prediction memory */
    ekf->x_pred = allocate_array(n, "predicted state");
    ekf->P_pred = allocate_array(n * n, "predicted covariance");
    ekf->z_pred = allocate_array(m, "predicted measurement");

    /* Allocate update memory */
    ekf->y = allocate_array(m, "innovation");
    ekf->S = allocate_array(m * m, "innovation covariance");
    ekf->K = allocate_array(n * m, "Kalman gain");

    /* Allocate temporary matrices */
    ekf->F_T = allocate_array(n * n, "F transpose");
    ekf->H_T = allocate_array(n * m, "H transpose");
    ekf->temp_nn = allocate_array(n * n, "temp_nn");
    ekf->temp_nn2 = allocate_array(n * n, "temp_nn2");
    ekf->temp_mm = allocate_array(m * m, "temp_mm");
    ekf->temp_nm = allocate_array(n * m, "temp_nm");
    ekf->temp_mn = allocate_array(m * n, "temp_mn");
    ekf->temp_n = allocate_array(n, "temp_n");
    ekf->temp_m = allocate_array(m, "temp_m");
    ekf->I = allocate_array(n * n, "identity matrix");

    /* Check if any allocation failed */
    if (ekf->x == NULL || ekf->P == NULL || ekf->Q == NULL || ekf->R == NULL ||
        ekf->F == NULL || ekf->H == NULL || ekf->x_pred == NULL || ekf->P_pred == NULL ||
        ekf->z_pred == NULL || ekf->y == NULL || ekf->S == NULL || ekf->K == NULL ||
        ekf->F_T == NULL || ekf->H_T == NULL || ekf->temp_nn == NULL || ekf->temp_nn2 == NULL ||
        ekf->temp_mm == NULL || ekf->temp_nm == NULL || ekf->temp_mn == NULL ||
        ekf->temp_n == NULL || ekf->temp_m == NULL || ekf->I == NULL)
    {
        ekf_free(ekf);
        return false;
    }

    /* Initialize identity matrix */
    memset(ekf->I, 0, n * n * sizeof(float));
    for (size_t i = 0; i < n; i++)
    {
        ekf->I[i * n + i] = 1.0f;
    }

    /* Initialize angle flags to NULL (disabled by default) */
    ekf->state_is_angle = NULL;
    ekf->meas_is_angle = NULL;

    printf("[EKF] Initialized successfully (n=%zu, m=%zu, dim_u=%zu)\n", n, m, dim_u);
    return true;
}

void ekf_free(EKF_State *ekf)
{
    if (ekf == NULL)
        return;

    free(ekf->x);
    free(ekf->P);
    free(ekf->Q);
    free(ekf->R);
    free(ekf->F);
    free(ekf->H);
    free(ekf->x_pred);
    free(ekf->P_pred);
    free(ekf->z_pred);
    free(ekf->y);
    free(ekf->S);
    free(ekf->K);
    free(ekf->F_T);
    free(ekf->H_T);
    free(ekf->temp_nn);
    free(ekf->temp_nn2);
    free(ekf->temp_mm);
    free(ekf->temp_nm);
    free(ekf->temp_mn);
    free(ekf->temp_n);
    free(ekf->temp_m);
    free(ekf->I);
    free(ekf->state_is_angle);
    free(ekf->meas_is_angle);

    memset(ekf, 0, sizeof(EKF_State));
}

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

void ekf_set_state(EKF_State *ekf, const float *x, const float *P)
{
    if (ekf == NULL || x == NULL || P == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Invalid parameters in ekf_set_state\n");
        return;
    }

    memcpy(ekf->x, x, ekf->n * sizeof(float));
    memcpy(ekf->P, P, ekf->n * ekf->n * sizeof(float));
}

void ekf_set_noise(EKF_State *ekf, const float *Q, const float *R)
{
    if (ekf == NULL || Q == NULL || R == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Invalid parameters in ekf_set_noise\n");
        return;
    }

    memcpy(ekf->Q, Q, ekf->n * ekf->n * sizeof(float));
    memcpy(ekf->R, R, ekf->m * ekf->m * sizeof(float));
}

void ekf_get_state(const EKF_State *ekf, float *x, float *P)
{
    if (ekf == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Invalid EKF state in ekf_get_state\n");
        return;
    }

    if (x != NULL)
    {
        memcpy(x, ekf->x, ekf->n * sizeof(float));
    }

    if (P != NULL)
    {
        memcpy(P, ekf->P, ekf->n * ekf->n * sizeof(float));
    }
}

void ekf_set_angle_states(EKF_State *ekf, const bool *state_is_angle, const bool *meas_is_angle)
{
    if (ekf == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Invalid EKF state in ekf_set_angle_states\n");
        return;
    }

    /* Free existing angle flags if any */
    free(ekf->state_is_angle);
    free(ekf->meas_is_angle);
    ekf->state_is_angle = NULL;
    ekf->meas_is_angle = NULL;

    /* Allocate and copy state angle flags */
    if (state_is_angle != NULL)
    {
        ekf->state_is_angle = (bool *)malloc(ekf->n * sizeof(bool));
        if (ekf->state_is_angle != NULL)
        {
            memcpy(ekf->state_is_angle, state_is_angle, ekf->n * sizeof(bool));
        }
        else
        {
            fprintf(stderr, "[EKF WARNING] Failed to allocate state_is_angle array\n");
        }
    }

    /* Allocate and copy measurement angle flags */
    if (meas_is_angle != NULL)
    {
        ekf->meas_is_angle = (bool *)malloc(ekf->m * sizeof(bool));
        if (ekf->meas_is_angle != NULL)
        {
            memcpy(ekf->meas_is_angle, meas_is_angle, ekf->m * sizeof(bool));
        }
        else
        {
            fprintf(stderr, "[EKF WARNING] Failed to allocate meas_is_angle array\n");
        }
    }
}

/* ============================================================================
 * PREDICTION STEP
 * ============================================================================ */

void ekf_predict(EKF_State *ekf, const float *u)
{
    if (ekf == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Invalid EKF state in ekf_predict\n");
        return;
    }

    size_t n = ekf->n;

    /* Step 1: Propagate state through nonlinear function
     * x̄ = f(x, u)
     */
    ekf->f(ekf->x, u, ekf->x_pred);

    /* Step 2: Compute state Jacobian F = ∂f/∂x at current state */
    ekf->jac_f(ekf->x, u, ekf->F);

    /* Step 3: Propagate covariance P̄ = F·P·F^T + Q */

    /* Compute F^T (copy F, then transpose) */
    memcpy(ekf->F_T, ekf->F, n * n * sizeof(float));
    tran(ekf->F_T, n, n);

    /* Compute temp_nn = P·F^T */
    mul(ekf->P, ekf->F_T, ekf->temp_nn, n, n, n);

    /* Compute P_pred = F·temp_nn = F·P·F^T */
    mul(ekf->F, ekf->temp_nn, ekf->P_pred, n, n, n);

    /* Add process noise: P̄ = F·P·F^T + Q */
    for (size_t i = 0; i < n * n; i++)
    {
        ekf->P_pred[i] += ekf->Q[i];
    }

    /* Update state and covariance (prediction becomes current estimate) */
    memcpy(ekf->x, ekf->x_pred, n * sizeof(float));
    memcpy(ekf->P, ekf->P_pred, n * n * sizeof(float));
}

/* ============================================================================
 * UPDATE STEP
 * ============================================================================ */

void ekf_update(EKF_State *ekf, const float *z)
{
    if (ekf == NULL || z == NULL)
    {
        fprintf(stderr, "[EKF ERROR] Invalid parameters in ekf_update\n");
        return;
    }

    size_t n = ekf->n;
    size_t m = ekf->m;

    /* Step 1: Compute predicted measurement ẑ = h(x̄) */
    ekf->h(ekf->x, ekf->z_pred);

    /* Step 2: Compute measurement Jacobian H = ∂h/∂x at predicted state */
    ekf->jac_h(ekf->x, ekf->H);

    /* Step 3: Compute innovation y = z - ẑ */
    /* Use angle_diff() for angular measurements */
    for (size_t i = 0; i < m; i++)
    {
        if (ekf->meas_is_angle != NULL && ekf->meas_is_angle[i])
        {
            ekf->y[i] = angle_diff(z[i], ekf->z_pred[i]);
        }
        else
        {
            ekf->y[i] = z[i] - ekf->z_pred[i];
        }
    }

    /* Step 4: Compute innovation covariance S = H·P·H^T + R */

    /* Compute H^T (copy H, then transpose) */
    memcpy(ekf->H_T, ekf->H, m * n * sizeof(float));
    tran(ekf->H_T, m, n);

    /* Compute temp_nm = P·H^T */
    mul(ekf->P, ekf->H_T, ekf->temp_nm, n, n, m);

    /* Compute S = H·temp_nm = H·P·H^T */
    mul(ekf->H, ekf->temp_nm, ekf->S, m, n, m);

    /* Add measurement noise: S = H·P·H^T + R */
    for (size_t i = 0; i < m * m; i++)
    {
        ekf->S[i] += ekf->R[i];
    }

    /* Step 5: Compute Kalman gain K = P·H^T·S^-1 */

    /* Invert S (in-place) */
    memcpy(ekf->temp_mm, ekf->S, m * m * sizeof(float));
    inv(ekf->temp_mm, m);

    /* Compute K = temp_nm · S^-1 = P·H^T·S^-1 */
    mul(ekf->temp_nm, ekf->temp_mm, ekf->K, n, m, m);

    /* Step 6: Update state x = x̄ + K·y */
    mul(ekf->K, ekf->y, ekf->temp_n, n, m, 1);
    for (size_t i = 0; i < n; i++)
    {
        ekf->x[i] += ekf->temp_n[i];
    }

    /* Normalize angular states to [-π, π] */
    if (ekf->state_is_angle != NULL)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (ekf->state_is_angle[i])
            {
                ekf->x[i] = normalize_angle(ekf->x[i]);
            }
        }
    }

    /* Step 7: Update covariance using Joseph form for numerical stability
     * P = (I - K·H)·P·(I - K·H)^T + K·R·K^T
     * This form guarantees symmetry and positive semi-definiteness
     */

    /* Compute I_KH = I - K·H */
    mul(ekf->K, ekf->H, ekf->temp_nn, n, m, n);
    for (size_t i = 0; i < n * n; i++)
    {
        ekf->temp_nn[i] = ekf->I[i] - ekf->temp_nn[i];
    }

    /* Compute temp_nn2 = I_KH · P */
    memcpy(ekf->temp_nn2, ekf->P, n * n * sizeof(float));
    mul(ekf->temp_nn, ekf->temp_nn2, ekf->P_pred, n, n, n);

    /* Compute I_KH^T (transpose of I_KH) */
    memcpy(ekf->temp_nn2, ekf->temp_nn, n * n * sizeof(float));
    tran(ekf->temp_nn2, n, n);

    /* Compute P = (I_KH · P) · I_KH^T */
    mul(ekf->P_pred, ekf->temp_nn2, ekf->P, n, n, n);

    /* Add K·R·K^T term for Joseph form */
    /* Compute temp_nm = K·R */
    mul(ekf->K, ekf->R, ekf->temp_nm, n, m, m);

    /* Compute K^T (transpose of K) */
    memcpy(ekf->temp_mn, ekf->K, n * m * sizeof(float));
    tran(ekf->temp_mn, n, m);

    /* Compute temp_nn = (K·R)·K^T */
    mul(ekf->temp_nm, ekf->temp_mn, ekf->temp_nn, n, m, n);

    /* Add to P: P = (I-KH)·P·(I-KH)^T + K·R·K^T */
    for (size_t i = 0; i < n * n; i++)
    {
        ekf->P[i] += ekf->temp_nn[i];
    }
}

/* ============================================================================
 * END OF FILE
 * ============================================================================ */
