/*
 ============================================================================
 Name        : angle_utils.h
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Angle handling utilities for state estimation

 IMPORTANT: Angles in state estimation require special treatment because
 they are periodic quantities that wrap around at ±π. Standard arithmetic
 operations (addition, subtraction, averaging) do not work correctly for
 angles and can cause filter divergence or incorrect estimates.

 This module provides three critical functions:

 1. normalize_angle(): Wraps angles to [-π, π]
    - Use after state updates to keep angles in valid range
    - Prevents angle drift over time

 2. angle_diff(): Computes the shortest angular difference
    - Use for innovation (measurement residual) calculations
    - Ensures differences are in [-π, π] (e.g., 170° - (-170°) = -20°, not 340°)

 3. circular_mean(): Computes the circular mean of angles
    - CRITICAL for UKF sigma point averaging
    - CRITICAL for PF particle averaging
    - Prevents incorrect averaging (e.g., avg(179°, -179°) = ±180°, not 0°)

 WHERE TO USE:

 EKF (Extended Kalman Filter):
   - angle_diff() in innovation calculation (y = z - h(x))
   - normalize_angle() after state update

 UKF (Unscented Kalman Filter):
   - circular_mean() when computing mean of sigma points ⭐ CRITICAL
   - angle_diff() in covariance calculation
   - normalize_angle() after state update

 PF (Particle Filter):
   - circular_mean() when computing weighted mean of particles
   - normalize_angle() after particle updates

 ============================================================================
*/

#ifndef ANGLE_UTILS_H_
#define ANGLE_UTILS_H_

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Normalize an angle to the range [-π, π]
     *
     * This function wraps an angle to the principal value in [-π, π].
     * Essential for preventing angle drift and maintaining consistency.
     *
     * @param angle Input angle in radians (can be any value)
     * @return Normalized angle in [-π, π]
     *
     * Example:
     *   normalize_angle(3.5π) → -0.5π
     *   normalize_angle(-3π) → π
     */
    float normalize_angle(float angle);

    /**
     * @brief Compute the shortest angular difference between two angles
     *
     * Computes angle1 - angle2 and normalizes to [-π, π]. This ensures
     * that the difference takes the shortest path around the circle.
     *
     * @param angle1 First angle in radians
     * @param angle2 Second angle in radians
     * @return Angular difference in [-π, π]
     *
     * Example:
     *   angle_diff(170°, -170°) → -20° (not 340°)
     *   angle_diff(10°, 350°) → 20° (not -340°)
     *
     * USE IN FILTERS:
     *   EKF/UKF: innovation[i] = angle_diff(measurement[i], prediction[i])
     *   Covariance: diff[i] = angle_diff(sigma[i], mean[i])
     */
    float angle_diff(float angle1, float angle2);

    /**
     * @brief Compute the circular (directional) mean of a set of angles
     *
     * Computes the weighted circular mean using the unit vector method.
     * This is the CORRECT way to average angles and is essential for
     * UKF sigma point averaging and PF particle averaging.
     *
     * Mathematical approach:
     *   1. Convert each angle to unit vector: (cos θ, sin θ)
     *   2. Compute weighted sum of vectors
     *   3. Extract angle from resulting vector: atan2(Σw·sin θ, Σw·cos θ)
     *
     * @param angles Array of angles in radians [size n]
     * @param weights Array of weights [size n] (must sum to 1.0)
     * @param n Number of angles
     * @return Circular mean angle in [-π, π]
     *
     * Example:
     *   angles = [179°, -179°, 178°], weights = [0.33, 0.33, 0.34]
     *   Arithmetic mean: 59° ❌ WRONG
     *   Circular mean: ~179° ✅ CORRECT
     *
     * CRITICAL FOR:
     *   UKF: Averaging sigma points after propagation
     *   PF: Computing weighted mean of particles
     */
    float circular_mean(const float *angles, const float *weights, size_t n);

    /**
     * @brief Compute circular mean for unweighted angles
     *
     * Convenience function for equal weights (1/n for each angle).
     *
     * @param angles Array of angles in radians [size n]
     * @param n Number of angles
     * @return Circular mean angle in [-π, π]
     */
    float circular_mean_unweighted(const float *angles, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* ANGLE_UTILS_H_ */
