/*
 ============================================================================
 Name        : ukf.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Unscented Kalman Filter (UKF) Implementation in C
               Based on Van der Merwe's Scaled Sigma Point Algorithm
 ============================================================================
*/

#include "ukf.h"
#include "../LinearAlgebra/angle_utils.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * MEMORY ALLOCATION HELPERS
 * ============================================================================ */

static float *allocate_vector(size_t size)
{
    float *vec = (float *)calloc(size, sizeof(float));
    return vec;
}

static float *allocate_matrix(size_t rows, size_t cols)
{
    float *mat = (float *)calloc(rows * cols, sizeof(float));
    return mat;
}

static void free_if_not_null(void *ptr)
{
    if (ptr != NULL)
    {
        free(ptr);
    }
}

/* ============================================================================
 * INITIALIZATION AND CLEANUP
 * ============================================================================ */

bool ukf_init(UKF_State *ukf, size_t n, size_t m, size_t dim_u,
              float alpha, float beta, float kappa,
              ukf_state_transition_fn fx, ukf_measurement_fn hx)
{

    if (ukf == NULL || fx == NULL || hx == NULL)
    {
        return false;
    }

    /* Initialize parameters */
    ukf->params.n = n;
    ukf->params.m = m;
    ukf->params.dim_u = dim_u;
    ukf->params.alpha = alpha;
    ukf->params.beta = beta;
    ukf->params.kappa = kappa;
    ukf->params.lambda = alpha * alpha * (n + kappa) - n;
    ukf->params.num_sigmas = 2 * n + 1;

    ukf->fx = fx;
    ukf->hx = hx;

    size_t num_sigmas = ukf->params.num_sigmas;

    /* Allocate core state vectors and matrices */
    ukf->x = allocate_vector(n);
    ukf->P = allocate_matrix(n, n);
    ukf->Q = allocate_matrix(n, n);
    ukf->R = allocate_matrix(m, m);

    /* Allocate predicted state */
    ukf->x_pred = allocate_vector(n);
    ukf->P_pred = allocate_matrix(n, n);

    /* Allocate sigma points for state */
    ukf->sigmas_f.points = allocate_matrix(num_sigmas, n);
    ukf->sigmas_f.Wm = allocate_vector(num_sigmas);
    ukf->sigmas_f.Wc = allocate_vector(num_sigmas);

    /* Allocate sigma points for measurement */
    ukf->sigmas_h.points = allocate_matrix(num_sigmas, m);
    ukf->sigmas_h.Wm = ukf->sigmas_f.Wm; /* Share weights */
    ukf->sigmas_h.Wc = ukf->sigmas_f.Wc;

    /* Allocate measurement prediction */
    ukf->z_pred = allocate_vector(m);
    ukf->P_z = allocate_matrix(m, m);
    ukf->P_xz = allocate_matrix(n, m);

    /* Allocate Kalman gain */
    ukf->K = allocate_matrix(n, m);

    /* Allocate working memory */
    ukf->temp_n = allocate_vector(n);
    ukf->temp_m = allocate_vector(m);
    ukf->temp_nn = allocate_matrix(n, n);
    ukf->temp_mm = allocate_matrix(m, m);
    ukf->temp_nm = allocate_matrix(n, m);

    /* Check if any allocation failed */
    if (ukf->x == NULL || ukf->P == NULL || ukf->Q == NULL || ukf->R == NULL ||
        ukf->x_pred == NULL || ukf->P_pred == NULL ||
        ukf->sigmas_f.points == NULL || ukf->sigmas_f.Wm == NULL || ukf->sigmas_f.Wc == NULL ||
        ukf->sigmas_h.points == NULL ||
        ukf->z_pred == NULL || ukf->P_z == NULL || ukf->P_xz == NULL ||
        ukf->K == NULL ||
        ukf->temp_n == NULL || ukf->temp_m == NULL ||
        ukf->temp_nn == NULL || ukf->temp_mm == NULL || ukf->temp_nm == NULL)
    {
        ukf_free(ukf);
        return false;
    }

    /* Compute weights */
    ukf_compute_weights(ukf);

    /* Initialize angle arrays to NULL (disabled by default) */
    ukf->state_is_angle = NULL;
    ukf->meas_is_angle = NULL;

    return true;
}

void ukf_free(UKF_State *ukf)
{
    if (ukf == NULL)
    {
        return;
    }

    free_if_not_null(ukf->x);
    free_if_not_null(ukf->P);
    free_if_not_null(ukf->Q);
    free_if_not_null(ukf->R);
    free_if_not_null(ukf->x_pred);
    free_if_not_null(ukf->P_pred);
    free_if_not_null(ukf->sigmas_f.points);
    free_if_not_null(ukf->sigmas_f.Wm);
    free_if_not_null(ukf->sigmas_f.Wc);
    free_if_not_null(ukf->sigmas_h.points);
    free_if_not_null(ukf->z_pred);
    free_if_not_null(ukf->P_z);
    free_if_not_null(ukf->P_xz);
    free_if_not_null(ukf->K);
    free_if_not_null(ukf->temp_n);
    free_if_not_null(ukf->temp_m);
    free_if_not_null(ukf->temp_nn);
    free_if_not_null(ukf->temp_mm);
    free_if_not_null(ukf->temp_nm);
    free_if_not_null(ukf->state_is_angle);
    free_if_not_null(ukf->meas_is_angle);

    memset(ukf, 0, sizeof(UKF_State));
}

void ukf_set_state(UKF_State *ukf, const float *x, const float *P)
{
    if (ukf == NULL || x == NULL || P == NULL)
    {
        return;
    }

    size_t n = ukf->params.n;
    memcpy(ukf->x, x, n * sizeof(float));
    memcpy(ukf->P, P, n * n * sizeof(float));
}

void ukf_set_noise(UKF_State *ukf, const float *Q, const float *R)
{
    if (ukf == NULL || Q == NULL || R == NULL)
    {
        return;
    }

    size_t n = ukf->params.n;
    size_t m = ukf->params.m;
    memcpy(ukf->Q, Q, n * n * sizeof(float));
    memcpy(ukf->R, R, m * m * sizeof(float));
}

void ukf_get_state(const UKF_State *ukf, float *x_out, float *P_out)
{
    if (ukf == NULL || x_out == NULL)
    {
        return;
    }

    size_t n = ukf->params.n;
    memcpy(x_out, ukf->x, n * sizeof(float));

    if (P_out != NULL)
    {
        memcpy(P_out, ukf->P, n * n * sizeof(float));
    }
}

bool ukf_set_angle_states(UKF_State *ukf, const bool *state_is_angle, size_t n,
                          const bool *meas_is_angle, size_t m)
{
    if (ukf == NULL)
    {
        return false;
    }

    if (n != ukf->params.n || m != ukf->params.m)
    {
        fprintf(stderr, "[UKF ERROR] ukf_set_angle_states: dimension mismatch\n");
        return false;
    }

    /* Free existing arrays if they exist */
    free_if_not_null(ukf->state_is_angle);
    free_if_not_null(ukf->meas_is_angle);
    ukf->state_is_angle = NULL;
    ukf->meas_is_angle = NULL;

    /* Allocate and copy state_is_angle if provided */
    if (state_is_angle != NULL)
    {
        ukf->state_is_angle = (bool *)malloc(n * sizeof(bool));
        if (ukf->state_is_angle == NULL)
        {
            fprintf(stderr, "[UKF ERROR] Failed to allocate state_is_angle array\n");
            return false;
        }
        memcpy(ukf->state_is_angle, state_is_angle, n * sizeof(bool));
    }

    /* Allocate and copy meas_is_angle if provided */
    if (meas_is_angle != NULL)
    {
        ukf->meas_is_angle = (bool *)malloc(m * sizeof(bool));
        if (ukf->meas_is_angle == NULL)
        {
            fprintf(stderr, "[UKF ERROR] Failed to allocate meas_is_angle array\n");
            free_if_not_null(ukf->state_is_angle);
            ukf->state_is_angle = NULL;
            return false;
        }
        memcpy(ukf->meas_is_angle, meas_is_angle, m * sizeof(bool));
    }

    return true;
}

/* ============================================================================
 * SIGMA POINT FUNCTIONS (Van der Merwe's Algorithm)
 * ============================================================================ */

void ukf_compute_weights(UKF_State *ukf)
{
    if (ukf == NULL)
    {
        return;
    }

    size_t n = ukf->params.n;
    float lambda = ukf->params.lambda;
    float alpha = ukf->params.alpha;
    float beta = ukf->params.beta;
    size_t num_sigmas = ukf->params.num_sigmas;

    /* Weight for mean of center point (i=0) */
    ukf->sigmas_f.Wm[0] = lambda / (n + lambda);

    /* Weight for covariance of center point (i=0) */
    ukf->sigmas_f.Wc[0] = lambda / (n + lambda) + (1.0f - alpha * alpha + beta);

    /* Weights for remaining sigma points (i=1..2n) */
    float w = 1.0f / (2.0f * (n + lambda));
    for (size_t i = 1; i < num_sigmas; i++)
    {
        ukf->sigmas_f.Wm[i] = w;
        ukf->sigmas_f.Wc[i] = w;
    }
}

bool ukf_compute_sigma_points(const UKF_State *ukf, const float *x,
                              const float *P, float *sigmas)
{
    if (ukf == NULL || x == NULL || P == NULL || sigmas == NULL)
    {
        return false;
    }

    size_t n = ukf->params.n;
    float lambda = ukf->params.lambda;
    float scale = n + lambda;

    /* Allocate temporary matrix for scaled covariance */
    float *P_scaled = allocate_matrix(n, n);
    if (P_scaled == NULL)
    {
        return false;
    }

    /* Scale covariance: P_scaled = (n + λ) * P */
    for (size_t i = 0; i < n * n; i++)
    {
        P_scaled[i] = scale * P[i];
    }

    /* Compute Cholesky decomposition: L * L^T = P_scaled */
    float *L = allocate_matrix(n, n);
    if (L == NULL)
    {
        free(P_scaled);
        return false;
    }

    bool chol_success = chol(P_scaled, L, n);

    /* If Cholesky fails, try adding small regularization term */
    if (!chol_success)
    {
        fprintf(stderr, "[UKF WARNING] Cholesky decomposition failed, adding regularization\n");
        
        /* Add small diagonal regularization: P_scaled = P_scaled + ε*I */
        const float epsilon = 1e-9f;
        for (size_t i = 0; i < n; i++)
        {
            P_scaled[i * n + i] += epsilon;
        }
        
        /* Retry Cholesky decomposition */
        chol_success = chol(P_scaled, L, n);
        
        if (!chol_success)
        {
            fprintf(stderr, "[UKF ERROR] Cholesky failed even with regularization\n");
            fprintf(stderr, "[UKF ERROR] Covariance matrix may be ill-conditioned\n");
            free(P_scaled);
            free(L);
            return false;
        }
    }

    free(P_scaled);

    /* First sigma point is the mean: χ₀ = x */
    memcpy(sigmas, x, n * sizeof(float));

    /* Generate sigma points: χᵢ = x + L[i] for i=1..n */
    for (size_t i = 0; i < n; i++)
    {
        float *sigma = sigmas + (i + 1) * n; /* Sigma point i+1 */
        for (size_t j = 0; j < n; j++)
        {
            sigma[j] = x[j] + L[i * n + j]; /* x + row i of L */
        }
    }

    /* Generate sigma points: χᵢ = x - L[i-n] for i=n+1..2n */
    for (size_t i = 0; i < n; i++)
    {
        float *sigma = sigmas + (n + i + 1) * n; /* Sigma point n+i+1 */
        for (size_t j = 0; j < n; j++)
        {
            sigma[j] = x[j] - L[i * n + j]; /* x - row i of L */
        }
    }

    free(L);
    return true;
}

void ukf_unscented_transform(const float *sigmas, const float *Wm, const float *Wc,
                             size_t num_sigmas, size_t dim, const float *noise_cov,
                             const bool *is_angle, float *mean_out, float *cov_out)
{
    if (sigmas == NULL || Wm == NULL || Wc == NULL || mean_out == NULL || cov_out == NULL)
    {
        return;
    }

    /* Compute mean: mean = Σ Wm[i] * sigmas[i] */
    /* Use circular_mean() for angular components */
    memset(mean_out, 0, dim * sizeof(float));

    for (size_t j = 0; j < dim; j++)
    {
        if (is_angle != NULL && is_angle[j])
        {
            /* Angular component: extract column j from sigmas and use circular_mean */
            float *angles = allocate_vector(num_sigmas);
            if (angles == NULL)
            {
                return;
            }

            for (size_t i = 0; i < num_sigmas; i++)
            {
                angles[i] = sigmas[i * dim + j];
            }

            mean_out[j] = circular_mean(angles, Wm, num_sigmas);
            free(angles);
        }
        else
        {
            /* Linear component: arithmetic weighted mean */
            for (size_t i = 0; i < num_sigmas; i++)
            {
                const float *sigma = sigmas + i * dim;
                mean_out[j] += Wm[i] * sigma[j];
            }
        }
    }

    /* Compute covariance: cov = Σ Wc[i] * (sigmas[i] - mean)(sigmas[i] - mean)^T */
    memset(cov_out, 0, dim * dim * sizeof(float));

    float *diff = allocate_vector(dim);
    if (diff == NULL)
    {
        return;
    }

    for (size_t i = 0; i < num_sigmas; i++)
    {
        const float *sigma = sigmas + i * dim;
        float weight = Wc[i];

        /* Compute difference: diff = sigma - mean */
        /* Use angle_diff() for angular components */
        for (size_t j = 0; j < dim; j++)
        {
            if (is_angle != NULL && is_angle[j])
            {
                diff[j] = angle_diff(sigma[j], mean_out[j]);
            }
            else
            {
                diff[j] = sigma[j] - mean_out[j];
            }
        }

        /* Add weighted outer product: cov += Wc[i] * diff * diff^T */
        for (size_t row = 0; row < dim; row++)
        {
            for (size_t col = 0; col < dim; col++)
            {
                cov_out[row * dim + col] += weight * diff[row] * diff[col];
            }
        }
    }

    free(diff);

    /* Add noise covariance if provided */
    if (noise_cov != NULL)
    {
        for (size_t i = 0; i < dim * dim; i++)
        {
            cov_out[i] += noise_cov[i];
        }
    }
}

/* ============================================================================
 * PREDICT STEP
 * ============================================================================ */

bool ukf_predict(UKF_State *ukf, const float *u)
{
    if (ukf == NULL)
    {
        return false;
    }

    size_t n = ukf->params.n;
    size_t num_sigmas = ukf->params.num_sigmas;

    /* Step 1: Generate sigma points from current state (x, P) */
    bool sigma_success = ukf_compute_sigma_points(ukf, ukf->x, ukf->P, ukf->sigmas_f.points);
    if (!sigma_success)
    {
        return false;
    }

    /* Step 2: Propagate each sigma point through state transition function f(x, u) */
    float *temp_sigma = allocate_vector(n);
    if (temp_sigma == NULL)
    {
        return false;
    }

    for (size_t i = 0; i < num_sigmas; i++)
    {
        const float *sigma_in = ukf->sigmas_f.points + i * n;
        float *sigma_out = ukf->sigmas_f.points + i * n;

        /* Apply state transition: Y[i] = f(χ[i], u) */
        ukf->fx(sigma_in, u, temp_sigma);
        memcpy(sigma_out, temp_sigma, n * sizeof(float));
    }

    free(temp_sigma);

    /* Step 3: Apply unscented transform to get predicted mean and covariance */
    ukf_unscented_transform(ukf->sigmas_f.points, ukf->sigmas_f.Wm, ukf->sigmas_f.Wc,
                            num_sigmas, n, ukf->Q,
                            ukf->state_is_angle, ukf->x_pred, ukf->P_pred);

    return true;
}

/* ============================================================================
 * UPDATE STEP
 * ============================================================================ */

bool ukf_update(UKF_State *ukf, const float *z)
{
    if (ukf == NULL || z == NULL)
    {
        return false;
    }

    size_t n = ukf->params.n;
    size_t m = ukf->params.m;
    size_t num_sigmas = ukf->params.num_sigmas;

    /* Step 1: Transform predicted sigma points through measurement function h(x) */
    float *temp_meas = allocate_vector(m);
    if (temp_meas == NULL)
    {
        return false;
    }

    for (size_t i = 0; i < num_sigmas; i++)
    {
        const float *sigma_f = ukf->sigmas_f.points + i * n;
        float *sigma_h = ukf->sigmas_h.points + i * m;

        /* Apply measurement function: Z[i] = h(Y[i]) */
        ukf->hx(sigma_f, temp_meas);
        memcpy(sigma_h, temp_meas, m * sizeof(float));
    }

    free(temp_meas);

    /* Step 2: Compute predicted measurement mean and covariance using UT */
    ukf_unscented_transform(ukf->sigmas_h.points, ukf->sigmas_h.Wm, ukf->sigmas_h.Wc,
                            num_sigmas, m, ukf->R,
                            ukf->meas_is_angle, ukf->z_pred, ukf->P_z);

    /* Step 3: Compute cross-covariance P_xz */
    /* P_xz = Σ Wc[i] * (Y[i] - x_pred)(Z[i] - z_pred)^T */
    memset(ukf->P_xz, 0, n * m * sizeof(float));

    float *diff_x = allocate_vector(n);
    float *diff_z = allocate_vector(m);
    if (diff_x == NULL || diff_z == NULL)
    {
        free_if_not_null(diff_x);
        free_if_not_null(diff_z);
        return false;
    }

    for (size_t i = 0; i < num_sigmas; i++)
    {
        const float *sigma_f = ukf->sigmas_f.points + i * n;
        const float *sigma_h = ukf->sigmas_h.points + i * m;
        float weight = ukf->sigmas_f.Wc[i];

        /* Compute differences */
        /* Use angle_diff() for angular components */
        for (size_t j = 0; j < n; j++)
        {
            if (ukf->state_is_angle != NULL && ukf->state_is_angle[j])
            {
                diff_x[j] = angle_diff(sigma_f[j], ukf->x_pred[j]);
            }
            else
            {
                diff_x[j] = sigma_f[j] - ukf->x_pred[j];
            }
        }
        for (size_t j = 0; j < m; j++)
        {
            if (ukf->meas_is_angle != NULL && ukf->meas_is_angle[j])
            {
                diff_z[j] = angle_diff(sigma_h[j], ukf->z_pred[j]);
            }
            else
            {
                diff_z[j] = sigma_h[j] - ukf->z_pred[j];
            }
        }

        /* Add weighted outer product: P_xz += Wc[i] * diff_x * diff_z^T */
        for (size_t row = 0; row < n; row++)
        {
            for (size_t col = 0; col < m; col++)
            {
                ukf->P_xz[row * m + col] += weight * diff_x[row] * diff_z[col];
            }
        }
    }

    free(diff_x);
    free(diff_z);

    /* Step 4: Compute Kalman gain K = P_xz * P_z^(-1) */
    /* We solve: P_z^T * K^T = P_xz^T for K^T, then transpose */

    /* Copy P_z to temp_mm for inversion */
    memcpy(ukf->temp_mm, ukf->P_z, m * m * sizeof(float));

    /* Invert P_z */
    bool inv_success = inv(ukf->temp_mm, m);
    if (!inv_success)
    {
        return false;
    }

    /* Compute K = P_xz * inv(P_z) */
    mul(ukf->P_xz, ukf->temp_mm, ukf->K, n, m, m);

    /* Step 5: Compute innovation (residual): y = z - z_pred */
    /* Use angle_diff() for angular measurements */
    for (size_t i = 0; i < m; i++)
    {
        if (ukf->meas_is_angle != NULL && ukf->meas_is_angle[i])
        {
            ukf->temp_m[i] = angle_diff(z[i], ukf->z_pred[i]);
        }
        else
        {
            ukf->temp_m[i] = z[i] - ukf->z_pred[i];
        }
    }

    /* Step 6: Update state: x = x_pred + K * y */
    memcpy(ukf->x, ukf->x_pred, n * sizeof(float));
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = 0; j < m; j++)
        {
            ukf->x[i] += ukf->K[i * m + j] * ukf->temp_m[j];
        }
    }

    /* Normalize angular states to [-π, π] */
    if (ukf->state_is_angle != NULL)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (ukf->state_is_angle[i])
            {
                ukf->x[i] = normalize_angle(ukf->x[i]);
            }
        }
    }

    /* Step 7: Update covariance: P = P_pred - K * P_z * K^T */

    /* Compute temp_nm = K * P_z */
    mul(ukf->K, ukf->P_z, ukf->temp_nm, n, m, m);

    /* Compute temp_nn = temp_nm * K^T = K * P_z * K^T */
    float *K_T = allocate_matrix(m, n);
    if (K_T == NULL)
    {
        return false;
    }

    /* Transpose K */
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = 0; j < m; j++)
        {
            K_T[j * n + i] = ukf->K[i * m + j];
        }
    }

    mul(ukf->temp_nm, K_T, ukf->temp_nn, n, m, n);
    free(K_T);

    /* P = P_pred - temp_nn */
    memcpy(ukf->P, ukf->P_pred, n * n * sizeof(float));
    for (size_t i = 0; i < n * n; i++)
    {
        ukf->P[i] -= ukf->temp_nn[i];
    }

    return true;
}

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

void ukf_print_state(const UKF_State *ukf)
{
    if (ukf == NULL)
    {
        return;
    }

    size_t n = ukf->params.n;

    printf("\n=== UKF State ===\n");
    printf("State vector x [%zu]:\n", n);
    for (size_t i = 0; i < n; i++)
    {
        printf("  x[%zu] = %.6f\n", i, ukf->x[i]);
    }

    printf("\nCovariance diagonal P:\n");
    for (size_t i = 0; i < n; i++)
    {
        printf("  P[%zu,%zu] = %.6f\n", i, i, ukf->P[i * n + i]);
    }
    printf("================\n\n");
}
