/*
 ============================================================================
 Name        : angle_utils.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Angle handling utilities implementation
 ============================================================================
*/

#include "angle_utils.h"

/* ============================================================================
 * ANGLE NORMALIZATION
 * ============================================================================ */

float normalize_angle(float angle)
{
    /* Normalize to [-π, π] using fmod for efficiency */
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f)
    {
        angle += 2.0f * M_PI;
    }
    return angle - M_PI;
}

/* ============================================================================
 * ANGLE DIFFERENCE
 * ============================================================================ */

float angle_diff(float angle1, float angle2)
{
    /* Compute shortest angular difference */
    return normalize_angle(angle1 - angle2);
}

/* ============================================================================
 * CIRCULAR MEAN (WEIGHTED)
 * ============================================================================ */

float circular_mean(const float *angles, const float *weights, size_t n)
{
    if (angles == NULL || weights == NULL || n == 0)
    {
        return 0.0f;
    }

    /* Convert angles to unit vectors and compute weighted sum */
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    for (size_t i = 0; i < n; i++)
    {
        float w = weights[i];
        float angle = angles[i];
        sin_sum += w * sinf(angle);
        cos_sum += w * cosf(angle);
    }

    /* Compute angle of resulting vector */
    return atan2f(sin_sum, cos_sum);
}

/* ============================================================================
 * CIRCULAR MEAN (UNWEIGHTED)
 * ============================================================================ */

float circular_mean_unweighted(const float *angles, size_t n)
{
    if (angles == NULL || n == 0)
    {
        return 0.0f;
    }

    /* Convert angles to unit vectors and sum */
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    for (size_t i = 0; i < n; i++)
    {
        float angle = angles[i];
        sin_sum += sinf(angle);
        cos_sum += cosf(angle);
    }

    /* Compute angle of resulting vector */
    return atan2f(sin_sum, cos_sum);
}

/* ============================================================================
 * END OF FILE
 * ============================================================================ */
