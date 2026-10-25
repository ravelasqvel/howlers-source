/*
 ============================================================================
 Name        : pf.h
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Particle Filter (PF) Implementation in C
               Based on Sequential Importance Resampling (SIR) algorithm
 ============================================================================

 REFERENCES:
 [1] Arulampalam, M. S., et al. (2002). "A tutorial on particle filters for
     online nonlinear/non-Gaussian Bayesian tracking"
 [2] Ristic, B., et al. (2004). "Beyond the Kalman filter: Particle filters
     for tracking applications"

 ALGORITHM OVERVIEW:

 The Particle Filter is a Monte Carlo-based filtering technique that:
 1. Represents the posterior distribution with a set of weighted particles
 2. Each particle represents a possible state of the system
 3. Weights are proportional to the likelihood of measurements
 4. Resampling prevents particle degeneracy

 Key Steps:
 - Initialize: Generate N particles uniformly or from Gaussian
 - Predict: Move particles according to process model with noise
 - Update: Weight particles based on measurement likelihood
 - Resample: Discard low-weight particles, duplicate high-weight ones
 - Estimate: Compute weighted mean and variance

 ============================================================================
*/

#ifndef PF_H_
#define PF_H_

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
     * @brief State transition function pointer type (predict step)
     *
     * This function propagates each particle forward in time according to
     * the control input and process model: x_k = f(x_k-1, u_k)
     *
     * @param particle     Input particle state [state_dim]
     * @param u            Control input [control_dim] (can be NULL if no control)
     * @param noise        Process noise to add [state_dim]
     * @param particle_out Output particle state [state_dim]
     *
     * Example for robot motion (u = [angular_vel, linear_vel]):
     *   heading = particle[2] + u[0] + noise[0];
     *   distance = u[1] + noise[1];
     *   particle_out[0] = particle[0] + cos(heading) * distance;  // x
     *   particle_out[1] = particle[1] + sin(heading) * distance;  // y
     *   particle_out[2] = heading;                                 // heading
     */
    typedef void (*pf_state_transition_fn)(const float *particle, const float *u,
                                           const float *noise, float *particle_out);

    /**
     * @brief Measurement likelihood function pointer type
     *
     * This function computes the likelihood that a particle matches
     * the measurement: p(z | particle)
     *
     * @param particle      Particle state [state_dim]
     * @param measurement   Actual measurement vector [measurement_dim]
     * @param R             Measurement noise covariance matrix [measurement_dim x measurement_dim]
     * @param user_data     Optional user data (e.g., landmarks, maps, etc.) - can be NULL
     *
     * @return Likelihood value (probability, should be > 0)
     *
     * Example 1: Direct state measurement (no user_data needed)
     *   // Measurement is position [mx, my], particle state is [x, y, heading]
     *   float dx = particle[0] - measurement[0];
     *   float dy = particle[1] - measurement[1];
     *   float dist_sq = dx*dx + dy*dy;
     *   return exp(-dist_sq / (2*R[0]));
     *
     * Example 2: Landmark-based measurement (user_data = landmarks array)
     *   // user_data points to landmark array [num_landmarks x 2]
     *   const float* landmarks = (const float*)user_data;
     *   float likelihood = 1.0;
     *   for each landmark:
     *     compute distance from particle to landmark
     *     compare with measurement
     *     multiply likelihood
     *   return likelihood;
     */
    typedef float (*pf_measurement_likelihood_fn)(const float *particle,
                                                  const float *measurement,
                                                  const float *R,
                                                  const void *user_data);

    /* ============================================================================
     * ENUMERATIONS
     * ============================================================================ */

    /**
     * @brief Resampling algorithm options
     */
    typedef enum
    {
        PF_RESAMPLE_MULTINOMIAL, /**< Simple random resampling O(N log N) */
        PF_RESAMPLE_RESIDUAL,    /**< Residual resampling O(N) */
        PF_RESAMPLE_STRATIFIED,  /**< Stratified resampling O(N) */
        PF_RESAMPLE_SYSTEMATIC   /**< Systematic resampling O(N) - Recommended */
    } PF_ResampleMethod;

    /**
     * @brief Particle initialization method
     */
    typedef enum
    {
        PF_INIT_UNIFORM, /**< Uniform distribution over range */
        PF_INIT_GAUSSIAN /**< Gaussian distribution around mean */
    } PF_InitMethod;

    /* ============================================================================
     * DATA STRUCTURES
     * ============================================================================ */

    /**
     * @brief Particle Filter Parameters
     */
    typedef struct
    {
        size_t N;                          /**< Number of particles */
        size_t state_dim;                  /**< State dimension (e.g., x, y, heading = 3) */
        size_t dim_u;                      /**< Control input dimension */
        size_t measurement_dim;            /**< Measurement dimension */
        float resample_threshold;          /**< Resample when N_eff < N * threshold (e.g., 0.5) */
        PF_ResampleMethod resample_method; /**< Resampling algorithm to use */
    } PF_Params;

    /**
     * @brief Particle Filter State
     *
     * Contains all particles, weights, and working memory for the filter.
     */
    typedef struct
    {
        PF_Params params; /**< Filter parameters */

        /* Particles and weights */
        float *particles; /**< Particle array [N x state_dim] */
        float *weights;   /**< Weight array [N] - normalized probabilities */

        /* Process noise */
        float *process_std; /**< Process noise std dev [state_dim] */

        /* State estimate */
        float *state_mean;     /**< Estimated state (weighted mean) [state_dim] */
        float *state_variance; /**< Estimated variance [state_dim] */

        /* User-provided functions */
        pf_state_transition_fn fx;               /**< State transition function */
        pf_measurement_likelihood_fn likelihood; /**< Measurement likelihood function */

        /* Working memory */
        float *temp_particle;  /**< Temporary particle [state_dim] */
        float *temp_noise;     /**< Temporary noise [state_dim] */
        float *cumsum;         /**< Cumulative sum for resampling [N] */
        size_t *indexes;       /**< Resampling indexes [N] */
        float *particles_copy; /**< Copy of particles for resampling [N x state_dim] */

        /* Angle handling (optional) */
        bool *state_is_angle; /**< Array [state_dim] indicating if state component is an angle */

    } PF_State;

    /* ============================================================================
     * FUNCTION PROTOTYPES - INITIALIZATION AND CLEANUP
     * ============================================================================ */

    /**
     * @brief Initialize Particle Filter
     *
     * @param pf        Pointer to PF_State structure
     * @param N         Number of particles
     * @param state_dim State dimension
     * @param dim_u     Control input dimension (use 0 if no control)
     * @param measurement_dim Measurement dimension
     * @param fx        State transition function
     * @param likelihood Measurement likelihood function
     * @param resample_method Resampling algorithm
     *
     * @return true if successful, false otherwise
     */
    bool pf_init(PF_State *pf, size_t N, size_t state_dim, size_t dim_u,
                 size_t measurement_dim,
                 pf_state_transition_fn fx,
                 pf_measurement_likelihood_fn likelihood,
                 PF_ResampleMethod resample_method);

    /**
     * @brief Free all memory allocated for the Particle Filter
     *
     * @param pf Pointer to PF_State structure
     */
    void pf_free(PF_State *pf);

    /* ============================================================================
     * FUNCTION PROTOTYPES - PARTICLE CREATION
     * ============================================================================ */

    /**
     * @brief Create uniformly distributed particles
     *
     * @param pf        Pointer to PF_State
     * @param ranges    Array of [min, max] for each state dimension [state_dim x 2]
     */
    void pf_create_uniform_particles(PF_State *pf, const float *ranges);

    /**
     * @brief Create Gaussian distributed particles
     *
     * @param pf    Pointer to PF_State
     * @param mean  Mean for each state dimension [state_dim]
     * @param std   Standard deviation for each state dimension [state_dim]
     */
    void pf_create_gaussian_particles(PF_State *pf, const float *mean, const float *std);

    /* ============================================================================
     * FUNCTION PROTOTYPES - CORE FILTER OPERATIONS
     * ============================================================================ */

    /**
     * @brief Predict step - move particles according to process model
     *
     * Applies the state transition function to each particle with
     * added process noise.
     *
     * @param pf Pointer to PF_State
     * @param u  Control input vector [dim_u] (can be NULL if no control)
     */
    void pf_predict(PF_State *pf, const float *u);

    /**
     * @brief Update step - weight particles based on measurements
     *
     * Uses the measurement likelihood function to update particle weights
     * according to how well they match the measurements.
     *
     * @param pf            Pointer to PF_State
     * @param measurement   Measurement vector [measurement_dim]
     * @param R             Measurement noise covariance matrix [measurement_dim x measurement_dim]
     * @param user_data     Optional user data passed to likelihood function (e.g., landmarks, map, etc.) - can be NULL
     */
    void pf_update(PF_State *pf, const float *measurement,
                   const float *R, const void *user_data);

    /**
     * @brief Compute state estimate from particles
     *
     * Computes the weighted mean and variance of the particles.
     * Results are stored in pf->state_mean and pf->state_variance.
     *
     * @param pf Pointer to PF_State
     */
    void pf_estimate(PF_State *pf);

    /**
     * @brief Resample particles if effective N is too low
     *
     * Checks N_eff and performs resampling if it falls below threshold.
     * Uses the resampling method specified in pf->params.resample_method.
     *
     * @param pf Pointer to PF_State
     * @return true if resampling was performed, false otherwise
     */
    bool pf_resample_if_needed(PF_State *pf);

    /**
     * @brief Configure which state components are angles
     *
     * Sets boolean array indicating which state components are angular (periodic).
     * This enables circular statistics when computing weighted mean of particles.
     *
     * Angular states use circular_mean() which handles wraparound correctly
     * (e.g., average of 179° and -179° gives ±180° instead of 0°).
     *
     * Example for robot state [x, y, θ]:
     * @code
     *   bool state_is_angle[] = {false, false, true};  // θ is angle
     *   pf_set_angle_states(&pf, state_is_angle, 3);
     * @endcode
     *
     * @param pf             Pointer to PF_State structure
     * @param state_is_angle Array [state_dim] indicating if state component is angle (NULL disables)
     * @param state_dim      Number of state components
     * @return               true if successful, false if memory allocation failed
     */
    bool pf_set_angle_states(PF_State *pf, const bool *state_is_angle, size_t state_dim);

    /* ============================================================================
     * FUNCTION PROTOTYPES - RESAMPLING ALGORITHMS
     * ============================================================================ */

    /**
     * @brief Compute effective number of particles (N_eff)
     *
     * N_eff = 1 / sum(weights^2)
     * Indicates how many particles meaningfully contribute to the estimate.
     *
     * @param weights Array of normalized weights [N]
     * @param N       Number of particles
     *
     * @return Effective number of particles
     */
    float pf_neff(const float *weights, size_t N);

    /**
     * @brief Multinomial resampling (simple random resampling)
     *
     * O(N log N) algorithm. Simple but not optimal.
     * Samples from cumulative sum using binary search.
     *
     * @param weights Normalized weights [N]
     * @param N       Number of particles
     * @param indexes Output resampling indexes [N]
     */
    void pf_resample_multinomial(const float *weights, size_t N, size_t *indexes);

    /**
     * @brief Residual resampling
     *
     * O(N) algorithm. Takes int(N*w) copies of each particle, then
     * uses multinomial resampling on the residuals.
     *
     * @param weights Normalized weights [N]
     * @param N       Number of particles
     * @param indexes Output resampling indexes [N]
     */
    void pf_resample_residual(const float *weights, size_t N, size_t *indexes);

    /**
     * @brief Stratified resampling
     *
     * O(N) algorithm. Divides [0,1] into N equal sections and randomly
     * samples one particle from each section.
     *
     * @param weights Normalized weights [N]
     * @param N       Number of particles
     * @param indexes Output resampling indexes [N]
     */
    void pf_resample_stratified(const float *weights, size_t N, size_t *indexes);

    /**
     * @brief Systematic resampling (recommended)
     *
     * O(N) algorithm. Similar to stratified but uses a single random offset
     * for all divisions, ensuring samples are exactly 1/N apart.
     *
     * @param weights Normalized weights [N]
     * @param N       Number of particles
     * @param indexes Output resampling indexes [N]
     */
    void pf_resample_systematic(const float *weights, size_t N, size_t *indexes);

    /**
     * @brief Perform resampling using computed indexes
     *
     * Replaces particles with copies according to the index array.
     * Resets all weights to 1/N.
     *
     * @param pf      Pointer to PF_State
     * @param indexes Resampling indexes [N]
     */
    void pf_resample_from_index(PF_State *pf, const size_t *indexes);

    /* ============================================================================
     * UTILITY FUNCTIONS
     * ============================================================================ */

    /**
     * @brief Set process noise standard deviations
     *
     * @param pf  Pointer to PF_State
     * @param std Standard deviations [state_dim]
     */
    void pf_set_process_noise(PF_State *pf, const float *std);

    /**
     * @brief Normalize weights so they sum to 1.0
     *
     * @param weights Weight array [N]
     * @param N       Number of weights
     */
    void pf_normalize_weights(float *weights, size_t N);

    /**
     * @brief Print particle filter state (for debugging)
     *
     * @param pf Pointer to PF_State
     */
    void pf_print_state(const PF_State *pf);

#ifdef __cplusplus
}
#endif

#endif /* !PF_H_ */
