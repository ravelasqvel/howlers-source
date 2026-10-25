/*
 ============================================================================
 Name        : pf.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Particle Filter (PF) Implementation in C
               Based on Sequential Importance Resampling (SIR) algorithm
 ============================================================================
*/

#include "pf.h"
#include "../LinearAlgebra/angle_utils.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

/* ============================================================================
 * RANDOM NUMBER GENERATION UTILITIES
 * ============================================================================ */

/**
 * @brief Generate random float in [0, 1]
 */
static float randf(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/**
 * @brief Generate random float in [min, max]
 */
static float uniform_rand(float min, float max)
{
    return min + (max - min) * randf();
}

/**
 * @brief Generate Gaussian random number (Box-Muller transform)
 */
static float randn(void)
{
    static int has_spare = 0;
    static float spare;

    if (has_spare)
    {
        has_spare = 0;
        return spare;
    }

    has_spare = 1;
    float u, v, s;
    do
    {
        u = randf() * 2.0f - 1.0f;
        v = randf() * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return u * s;
}

/* ============================================================================
 * MEMORY ALLOCATION HELPERS
 * ============================================================================ */

static float *allocate_vector(size_t size)
{
    float *vec = (float *)calloc(size, sizeof(float));
    return vec;
}

static size_t *allocate_size_array(size_t size)
{
    size_t *arr = (size_t *)calloc(size, sizeof(size_t));
    return arr;
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

bool pf_init(PF_State *pf, size_t N, size_t state_dim, size_t dim_u,
             size_t measurement_dim,
             pf_state_transition_fn fx,
             pf_measurement_likelihood_fn likelihood,
             PF_ResampleMethod resample_method)
{
    if (pf == NULL || fx == NULL || likelihood == NULL || N == 0)
    {
        return false;
    }

    /* Initialize random seed */
    srand((unsigned int)time(NULL));

    /* Initialize parameters */
    pf->params.N = N;
    pf->params.state_dim = state_dim;
    pf->params.dim_u = dim_u;
    pf->params.measurement_dim = measurement_dim;
    pf->params.resample_threshold = 0.5f; /* Resample when N_eff < N/2 */
    pf->params.resample_method = resample_method;

    pf->fx = fx;
    pf->likelihood = likelihood;

    /* Allocate particles and weights */
    pf->particles = allocate_vector(N * state_dim);
    pf->weights = allocate_vector(N);

    /* Allocate noise arrays */
    pf->process_std = allocate_vector(state_dim);

    /* Allocate state estimate */
    pf->state_mean = allocate_vector(state_dim);
    pf->state_variance = allocate_vector(state_dim);

    /* Allocate working memory */
    pf->temp_particle = allocate_vector(state_dim);
    pf->temp_noise = allocate_vector(state_dim);
    pf->cumsum = allocate_vector(N);
    pf->indexes = allocate_size_array(N);
    pf->particles_copy = allocate_vector(N * state_dim);

    /* Check if any allocation failed */
    if (pf->particles == NULL || pf->weights == NULL ||
        pf->process_std == NULL ||
        pf->state_mean == NULL || pf->state_variance == NULL ||
        pf->temp_particle == NULL || pf->temp_noise == NULL ||
        pf->cumsum == NULL || pf->indexes == NULL ||
        pf->particles_copy == NULL)
    {
        pf_free(pf);
        return false;
    }

    /* Initialize weights uniformly */
    for (size_t i = 0; i < N; i++)
    {
        pf->weights[i] = 1.0f / (float)N;
    }

    /* Initialize angle array to NULL (disabled by default) */
    pf->state_is_angle = NULL;

    return true;
}

void pf_free(PF_State *pf)
{
    if (pf == NULL)
    {
        return;
    }

    free_if_not_null(pf->particles);
    free_if_not_null(pf->weights);
    free_if_not_null(pf->process_std);
    free_if_not_null(pf->state_mean);
    free_if_not_null(pf->state_variance);
    free_if_not_null(pf->temp_particle);
    free_if_not_null(pf->temp_noise);
    free_if_not_null(pf->cumsum);
    free_if_not_null(pf->indexes);
    free_if_not_null(pf->particles_copy);
    free_if_not_null(pf->state_is_angle);

    memset(pf, 0, sizeof(PF_State));
}

bool pf_set_angle_states(PF_State *pf, const bool *state_is_angle, size_t state_dim)
{
    if (pf == NULL)
    {
        return false;
    }

    if (state_dim != pf->params.state_dim)
    {
        fprintf(stderr, "[PF ERROR] pf_set_angle_states: dimension mismatch\n");
        return false;
    }

    /* Free existing array if it exists */
    free_if_not_null(pf->state_is_angle);
    pf->state_is_angle = NULL;

    /* Allocate and copy state_is_angle if provided */
    if (state_is_angle != NULL)
    {
        pf->state_is_angle = (bool *)malloc(state_dim * sizeof(bool));
        if (pf->state_is_angle == NULL)
        {
            fprintf(stderr, "[PF ERROR] Failed to allocate state_is_angle array\n");
            return false;
        }
        memcpy(pf->state_is_angle, state_is_angle, state_dim * sizeof(bool));
    }

    return true;
}

/* ============================================================================
 * PARTICLE CREATION
 * ============================================================================ */

void pf_create_uniform_particles(PF_State *pf, const float *ranges)
{
    if (pf == NULL || ranges == NULL)
    {
        return;
    }

    size_t N = pf->params.N;
    size_t state_dim = pf->params.state_dim;

    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j < state_dim; j++)
        {
            float min = ranges[j * 2];
            float max = ranges[j * 2 + 1];
            pf->particles[i * state_dim + j] = uniform_rand(min, max);
        }

        /* Wrap heading to [0, 2*pi] if state includes heading (last dimension) */
        if (state_dim >= 3)
        {
            float heading = pf->particles[i * state_dim + (state_dim - 1)];
            pf->particles[i * state_dim + (state_dim - 1)] = fmodf(heading, 2.0f * M_PI);
            if (pf->particles[i * state_dim + (state_dim - 1)] < 0)
            {
                pf->particles[i * state_dim + (state_dim - 1)] += 2.0f * M_PI;
            }
        }
    }

    /* Initialize weights uniformly */
    for (size_t i = 0; i < N; i++)
    {
        pf->weights[i] = 1.0f / (float)N;
    }
}

void pf_create_gaussian_particles(PF_State *pf, const float *mean, const float *std)
{
    if (pf == NULL || mean == NULL || std == NULL)
    {
        return;
    }

    size_t N = pf->params.N;
    size_t state_dim = pf->params.state_dim;

    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j < state_dim; j++)
        {
            pf->particles[i * state_dim + j] = mean[j] + randn() * std[j];
        }

        /* Wrap heading to [0, 2*pi] if state includes heading (last dimension) */
        if (state_dim >= 3)
        {
            float heading = pf->particles[i * state_dim + (state_dim - 1)];
            pf->particles[i * state_dim + (state_dim - 1)] = fmodf(heading, 2.0f * M_PI);
            if (pf->particles[i * state_dim + (state_dim - 1)] < 0)
            {
                pf->particles[i * state_dim + (state_dim - 1)] += 2.0f * M_PI;
            }
        }
    }

    /* Initialize weights uniformly */
    for (size_t i = 0; i < N; i++)
    {
        pf->weights[i] = 1.0f / (float)N;
    }
}

/* ============================================================================
 * CORE FILTER OPERATIONS
 * ============================================================================ */

void pf_predict(PF_State *pf, const float *u)
{
    if (pf == NULL)
    {
        return;
    }

    size_t N = pf->params.N;
    size_t state_dim = pf->params.state_dim;

    /* Move each particle according to process model with noise */
    for (size_t i = 0; i < N; i++)
    {
        float *particle = &pf->particles[i * state_dim];

        /* Generate process noise for this particle */
        for (size_t j = 0; j < state_dim; j++)
        {
            pf->temp_noise[j] = randn() * pf->process_std[j];
        }

        /* Apply state transition function */
        pf->fx(particle, u, pf->temp_noise, pf->temp_particle);

        /* Copy result back to particle */
        memcpy(particle, pf->temp_particle, state_dim * sizeof(float));

        /* Normalize angular states using state_is_angle flags */
        if (pf->state_is_angle != NULL)
        {
            for (size_t j = 0; j < state_dim; j++)
            {
                if (pf->state_is_angle[j])
                {
                    /* Wrap angle to [0, 2π] */
                    particle[j] = fmodf(particle[j], 2.0f * M_PI);
                    if (particle[j] < 0.0f)
                    {
                        particle[j] += 2.0f * M_PI;
                    }
                }
            }
        }
    }
}

void pf_update(PF_State *pf, const float *measurement,
               const float *R, const void *user_data)
{
    if (pf == NULL || measurement == NULL)
    {
        return;
    }

    size_t N = pf->params.N;
    size_t state_dim = pf->params.state_dim;

    /* Update weights based on measurement likelihood */
    for (size_t i = 0; i < N; i++)
    {
        float *particle = &pf->particles[i * state_dim];

        /* Compute likelihood for this particle */
        float likelihood = pf->likelihood(particle, measurement, R, user_data);

        /* Update weight (multiply by likelihood) */
        pf->weights[i] *= likelihood;
    }

    /* Add small epsilon to avoid round-off to zero */
    for (size_t i = 0; i < N; i++)
    {
        pf->weights[i] += 1.0e-30f;
    }

    /* Normalize weights */
    pf_normalize_weights(pf->weights, N);
}

void pf_estimate(PF_State *pf)
{
    if (pf == NULL)
    {
        return;
    }

    size_t N = pf->params.N;
    size_t state_dim = pf->params.state_dim;

    /* Initialize mean and variance to zero */
    memset(pf->state_mean, 0, state_dim * sizeof(float));
    memset(pf->state_variance, 0, state_dim * sizeof(float));

    /* Compute weighted mean for each state component */
    for (size_t j = 0; j < state_dim; j++)
    {
        if (pf->state_is_angle != NULL && pf->state_is_angle[j])
        {
            /* Angular component: use circular_mean */
            float *angles = allocate_vector(N);
            if (angles == NULL)
            {
                continue;
            }

            for (size_t i = 0; i < N; i++)
            {
                angles[i] = pf->particles[i * state_dim + j];
            }

            pf->state_mean[j] = circular_mean(angles, pf->weights, N);
            free(angles);
        }
        else
        {
            /* Linear component: arithmetic weighted mean */
            for (size_t i = 0; i < N; i++)
            {
                pf->state_mean[j] += pf->weights[i] * pf->particles[i * state_dim + j];
            }
        }
    }

    /* Compute weighted variance */
    /* Use angle_diff() for angular components */
    for (size_t i = 0; i < N; i++)
    {
        float weight = pf->weights[i];
        for (size_t j = 0; j < state_dim; j++)
        {
            float diff;
            if (pf->state_is_angle != NULL && pf->state_is_angle[j])
            {
                diff = angle_diff(pf->particles[i * state_dim + j], pf->state_mean[j]);
            }
            else
            {
                diff = pf->particles[i * state_dim + j] - pf->state_mean[j];
            }
            pf->state_variance[j] += weight * diff * diff;
        }
    }
}

bool pf_resample_if_needed(PF_State *pf)
{
    if (pf == NULL)
    {
        return false;
    }

    size_t N = pf->params.N;
    float threshold = pf->params.resample_threshold;

    /* Compute effective number of particles */
    float n_eff = pf_neff(pf->weights, N);

    /* Resample if N_eff is below threshold */
    if (n_eff < N * threshold)
    {
        /* Generate resampling indexes using specified method */
        switch (pf->params.resample_method)
        {
        case PF_RESAMPLE_MULTINOMIAL:
            pf_resample_multinomial(pf->weights, N, pf->indexes);
            break;
        case PF_RESAMPLE_RESIDUAL:
            pf_resample_residual(pf->weights, N, pf->indexes);
            break;
        case PF_RESAMPLE_STRATIFIED:
            pf_resample_stratified(pf->weights, N, pf->indexes);
            break;
        case PF_RESAMPLE_SYSTEMATIC:
            pf_resample_systematic(pf->weights, N, pf->indexes);
            break;
        default:
            pf_resample_systematic(pf->weights, N, pf->indexes);
            break;
        }

        /* Perform resampling */
        pf_resample_from_index(pf, pf->indexes);

        return true;
    }

    return false;
}

/* ============================================================================
 * RESAMPLING ALGORITHMS
 * ============================================================================ */

float pf_neff(const float *weights, size_t N)
{
    float sum_sq = 0.0f;
    for (size_t i = 0; i < N; i++)
    {
        sum_sq += weights[i] * weights[i];
    }
    return 1.0f / sum_sq;
}

/**
 * @brief Binary search in cumulative sum array
 */
static size_t binary_search(const float *cumsum, size_t N, float value)
{
    size_t left = 0;
    size_t right = N - 1;

    while (left < right)
    {
        size_t mid = left + (right - left) / 2;
        if (value > cumsum[mid])
        {
            left = mid + 1;
        }
        else
        {
            right = mid;
        }
    }
    return left;
}

void pf_resample_multinomial(const float *weights, size_t N, size_t *indexes)
{
    if (weights == NULL || indexes == NULL || N == 0)
    {
        return;
    }

    /* Compute cumulative sum */
    float *cumsum = (float *)malloc(N * sizeof(float));
    if (cumsum == NULL)
    {
        return;
    }

    cumsum[0] = weights[0];
    for (size_t i = 1; i < N; i++)
    {
        cumsum[i] = cumsum[i - 1] + weights[i];
    }
    cumsum[N - 1] = 1.0f; /* Avoid round-off error */

    /* Sample N random values and find their positions */
    for (size_t i = 0; i < N; i++)
    {
        float r = randf();
        indexes[i] = binary_search(cumsum, N, r);
    }

    free(cumsum);
}

void pf_resample_residual(const float *weights, size_t N, size_t *indexes)
{
    if (weights == NULL || indexes == NULL || N == 0)
    {
        return;
    }

    /* Allocate temporary arrays */
    size_t *num_copies = (size_t *)calloc(N, sizeof(size_t));
    float *residual = (float *)malloc(N * sizeof(float));
    if (num_copies == NULL || residual == NULL)
    {
        free(num_copies);
        free(residual);
        return;
    }

    /* Take int(N * w) copies of each particle */
    size_t k = 0;
    for (size_t i = 0; i < N; i++)
    {
        num_copies[i] = (size_t)(N * weights[i]);
        for (size_t j = 0; j < num_copies[i]; j++)
        {
            if (k < N)
            {
                indexes[k++] = i;
            }
        }
    }

    /* Compute residuals */
    float residual_sum = 0.0f;
    for (size_t i = 0; i < N; i++)
    {
        residual[i] = N * weights[i] - (float)num_copies[i];
        residual_sum += residual[i];
    }

    /* Normalize residuals */
    if (residual_sum > 0.0f)
    {
        for (size_t i = 0; i < N; i++)
        {
            residual[i] /= residual_sum;
        }

        /* Use multinomial resampling on residuals for remaining particles */
        float *cumsum = (float *)malloc(N * sizeof(float));
        if (cumsum != NULL)
        {
            cumsum[0] = residual[0];
            for (size_t i = 1; i < N; i++)
            {
                cumsum[i] = cumsum[i - 1] + residual[i];
            }
            cumsum[N - 1] = 1.0f;

            while (k < N)
            {
                float r = randf();
                indexes[k++] = binary_search(cumsum, N, r);
            }

            free(cumsum);
        }
    }

    free(num_copies);
    free(residual);
}

void pf_resample_stratified(const float *weights, size_t N, size_t *indexes)
{
    if (weights == NULL || indexes == NULL || N == 0)
    {
        return;
    }

    /* Compute cumulative sum */
    float *cumsum = (float *)malloc(N * sizeof(float));
    if (cumsum == NULL)
    {
        return;
    }

    cumsum[0] = weights[0];
    for (size_t i = 1; i < N; i++)
    {
        cumsum[i] = cumsum[i - 1] + weights[i];
    }
    cumsum[N - 1] = 1.0f;

    /* Generate stratified samples */
    size_t i = 0, j = 0;
    while (i < N && j < N)
    {
        /* Random position within stratum i */
        float position = ((float)i + randf()) / (float)N;

        if (position < cumsum[j])
        {
            indexes[i] = j;
            i++;
        }
        else
        {
            j++;
        }
    }

    free(cumsum);
}

void pf_resample_systematic(const float *weights, size_t N, size_t *indexes)
{
    if (weights == NULL || indexes == NULL || N == 0)
    {
        return;
    }

    /* Compute cumulative sum */
    float *cumsum = (float *)malloc(N * sizeof(float));
    if (cumsum == NULL)
    {
        return;
    }

    cumsum[0] = weights[0];
    for (size_t i = 1; i < N; i++)
    {
        cumsum[i] = cumsum[i - 1] + weights[i];
    }
    cumsum[N - 1] = 1.0f;

    /* Single random offset for all samples */
    float offset = randf() / (float)N;

    /* Generate systematic samples */
    size_t i = 0, j = 0;
    while (i < N && j < N)
    {
        float position = ((float)i + offset) / (1.0f / (float)N) / (float)N;

        if (position < cumsum[j])
        {
            indexes[i] = j;
            i++;
        }
        else
        {
            j++;
        }
    }

    free(cumsum);
}

void pf_resample_from_index(PF_State *pf, const size_t *indexes)
{
    if (pf == NULL || indexes == NULL)
    {
        return;
    }

    size_t N = pf->params.N;
    size_t state_dim = pf->params.state_dim;

    /* Copy current particles to temporary storage */
    memcpy(pf->particles_copy, pf->particles, N * state_dim * sizeof(float));

    /* Resample particles according to indexes */
    for (size_t i = 0; i < N; i++)
    {
        size_t idx = indexes[i];
        memcpy(&pf->particles[i * state_dim],
               &pf->particles_copy[idx * state_dim],
               state_dim * sizeof(float));
    }

    /* Reset all weights to uniform */
    for (size_t i = 0; i < N; i++)
    {
        pf->weights[i] = 1.0f / (float)N;
    }
}

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

void pf_set_process_noise(PF_State *pf, const float *std)
{
    if (pf == NULL || std == NULL)
    {
        return;
    }

    size_t state_dim = pf->params.state_dim;
    memcpy(pf->process_std, std, state_dim * sizeof(float));
}

void pf_normalize_weights(float *weights, size_t N)
{
    if (weights == NULL || N == 0)
    {
        return;
    }

    float sum = 0.0f;
    for (size_t i = 0; i < N; i++)
    {
        sum += weights[i];
    }

    if (sum > 0.0f)
    {
        for (size_t i = 0; i < N; i++)
        {
            weights[i] /= sum;
        }
    }
    else
    {
        /* If all weights are zero, reset to uniform */
        for (size_t i = 0; i < N; i++)
        {
            weights[i] = 1.0f / (float)N;
        }
    }
}

void pf_print_state(const PF_State *pf)
{
    if (pf == NULL)
    {
        return;
    }

    printf("\n========================================\n");
    printf("Particle Filter State\n");
    printf("========================================\n");
    printf("Number of particles: %zu\n", pf->params.N);
    printf("State dimension: %zu\n", pf->params.state_dim);
    // printf("Measurement std: %.4f\n", pf->measurement_std);

    printf("\nEstimated State (mean):\n");
    for (size_t i = 0; i < pf->params.state_dim; i++)
    {
        printf("  [%zu]: %.4f\n", i, pf->state_mean[i]);
    }

    printf("\nEstimated Variance:\n");
    for (size_t i = 0; i < pf->params.state_dim; i++)
    {
        printf("  [%zu]: %.4f\n", i, pf->state_variance[i]);
    }

    float n_eff = pf_neff(pf->weights, pf->params.N);
    printf("\nEffective N: %.2f (%.1f%%)\n",
           n_eff, 100.0f * n_eff / (float)pf->params.N);

    printf("========================================\n\n");
}
