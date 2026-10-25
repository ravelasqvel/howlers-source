/*
 ============================================================================
 Name        : headers.h
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Common headers and definitions for the project
 ============================================================================
*/

#ifndef HEADERS_H_
#define HEADERS_H_

/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <float.h>

/* Enumerations for LinearAlgebra library */

/**
 * @brief Convolution shape options
 */
typedef enum
{
    CONV_SHAPE_FULL, /* Full convolution */
    CONV_SHAPE_SAME, /* Same size as input */
    CONV_SHAPE_VALID /* Valid (no padding) */
} CONV_SHAPE;

/**
 * @brief Matrix norm methods
 */
typedef enum
{
    NORM_METHOD_1,   /* 1-norm (max column sum) */
    NORM_METHOD_2,   /* 2-norm (spectral norm) */
    NORM_METHOD_INF, /* Infinity norm (max row sum) */
    NORM_METHOD_FROB /* Frobenius norm */
} NORM_METHOD;

/* Constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_E
#define M_E 2.71828182845904523536
#endif

#ifndef MIN_VALUE
#define MIN_VALUE 1e-9f
#endif

/* Utility macros */
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define SIGN(x) ((x) < 0 ? -1 : 1)

#endif /* !HEADERS_H_ */
