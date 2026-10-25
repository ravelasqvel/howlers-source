/**
 * \file        control_senfusion.h
 * \brief       Control and Sensor Fusion library
 * \details
 * 
 * 
 * \author      MaverickST
 * \version     0.0.4
 * \date        16/05/2025
 * \copyright   Unlicensed
 */

#ifndef __CONTROL_SENFUSION_H__
#define __CONTROL_SENFUSION_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MIN(a,b) ((a)<(b)?(a):(b)) ///< Macro to get the minimum value
#define MAX(a,b) ((a)>(b)?(a):(b)) ///< Macro to get the maximum value

/**
 * @brief Sensor Fusion structure
 * 
 */
typedef struct
{
    float x_pred[2][1]; ///< Predicted state vector
    float P_pred[2][2]; ///< Predicted error covariance matrix
    float x[2][1]; ///< State vector

    float a_imu; ///< IMU acceleration
    float v_imu_prev; ///< IMU velocity
    float p_enc; ///< Encoder position
    float p_lidar; ///< Lidar position

    float p_ref[2]; ///< Reference position
    uint32_t k; ///< Sample time
    float dt; ///< Time step

    ///< Kalman filter Variables
    float P[2][2]; ///< Error covariance matrix
    float Q[2][2]; ///< Process noise covariance matrix
    float R[3][3]; ///< Measurement noise covariance matrix

    ///< Process model
    /**
     * @brief Process model: x(k+1) = A*x(k) + B*u(k)
     * State vector: x = [x, y]
    */
    float u; ///< Control input
    float A[2][2]; ///< Process model matrix
    float B[2][1]; ///< Control input matrix
    float C[2][2]; ///< Process model matrix

    ///< Measurement model
    /**
     * @brief Measurement model: z(k) = H*x(k) + v(k)
     * Measurement vector: z = [p_enc, p_lidar, v_imu]
    */
    float z[3][1]; ///< Measurement vector
    float H[3][2]; ///< Measurement matrix
    float H_T[2][3]; ///< Transpose of the measurement matrix

    float pos;
    float vel; ///< Velocity

} senfusion_t;

/**
 * @brief Initialize the control and sensor fusion structure
 * 
 * @param senfusion Pointer to the control and sensor fusion structure
 */
void senfusion_init(senfusion_t *senfusion, float dt);

/**
 * @brief Predict the next state of the system
 * 
 * @param senfusion Pointer to the control and sensor fusion structure
 * @param u Control input
 */
void senfusion_predict(senfusion_t *senfusion, float u);

/**
 * @brief Update the control and sensor fusion structure
 * 
 * @param senfusion Pointer to the control and sensor fusion structure
 */
void senfusion_update(senfusion_t *senfusion, float p_enc, float p_lidar, float a_imu, uint32_t k);

/**
 * @brief Get the position from the sensor fusion
 * 
 * @param senfusion 
 * @return float 
 */
static inline float senfusion_get_pos(senfusion_t *senfusion)
{
    return senfusion->x[0][0]; ///< Get the position from the sensor fusion
}

/**
 * @brief Get the velocity from the sensor fusion
 * 
 * @param senfusion 
 * @return float 
 */
static inline float senfusion_get_vel(senfusion_t *senfusion)
{
    return senfusion->x[1][0]; ///< Get the velocity from the sensor fusion
}

///<-------------------------------------------------------------
///<--------------- LINEAR ALGEBRA FUNCTIONS --------------------
///<-------------------------------------------------------------


/**
 * @brief Matrix multiplication
 * 
 * A[rowsA][colsA] * B[colsA][colsB] = C[rowsA][colsB]
 * 
 */
void matrixMultiplication(int rowsA, int colsA, int colsB, float A[rowsA][colsA], float B[colsA][colsB], float C[rowsA][colsB]);

/**
 * @brief Matrix addition and subtraction
 * 
 * A[rows][cols] + B[rows][cols] = C[rows][cols]
 * 
 */
void matrixAddSub(int rows, int cols, float A[rows][cols], float B[rows][cols], float C[rows][cols], char operation);

/**
 * @brief Matrix transpose
 * 
 * A[rows][cols] = B[cols][rows]
 * 
 */
void matrixTranspose(int rows, int cols, float A[rows][cols], float B[cols][rows]);

/// <-------------------------------------------------------------
/// <------------------ LU DECOMPOSITION -------------------------
/// <-------------------------------------------------------------

#define N 3       // Size of matrix A (NxN)
#define M 2       // Number of columns in B (right-hand sides)

void LU_decomposition(float A[N][N], float L[N][N], float U[N][N]);

void forward_substitution(float L[N][N], float b[N], float y[N]);

void backward_substitution(float U[N][N], float y[N], float x[N]);


/**
 * @brief This function solves the linear system Ax = B using LU decomposition.
 * It will be used to computes the Kalman Gain using LU decomposition.
 * 
 * This function avoids directly inverting the innovation covariance matrix S.
 * Instead, it uses LU decomposition to solve a linear system.
 * 
 * Solve X = B * inv(A) using LU decomposition without explicit inversion.
 * 
 * Given a square matrix A (N×N) and a matrix B (M×N), this routine computes
 * X = B * inv(A) by solving each row of B as a right‐hand‐side system:
 *      X[i,:] * A = B[i,:]
 * which is reformulated as
 *      (A^T) * z = b
 * for z = X[i,:]^T, b = B[i,:]^T.
 *
 * @param  A    Input matrix A of size N×N.
 * @param  B    Input matrix B of size M×N.
 * @param  X    Output matrix X of size M×N, containing B * inv(A).
 * 
 * This approach is numerically stable and efficient compared to computing inv(S).
 */
void solve_LU_system(float A[N][N], float B[M][N], float K[M][N]);

// -------------------------------------------------------------
// ---------------------- KALMAN FILTER 1D ---------------------
// -------------------------------------------------------------

/**
 * @brief Kalman filter 1D structure
 * 
 */
typedef struct {
    float x;  // estimated value
    float P;  // estimation error covariance
    float Q;  // process noise
    float R;  // measurement noise
} kalman1D_t;

/**
 * @brief Initialize the Kalman filter 1D structure
 * 
 * @param kf Pointer to the Kalman filter 1D structure
 * @param Q Process noise covariance
 * @param R Measurement noise covariance
 */
void kalman1D_init(kalman1D_t *kf, float Q, float R);

/**
 * @brief Update the Kalman filter 1D structure with a new measurement
 * 
 * @param kf Pointer to the Kalman filter 1D structure
 * @param meas Measurement value
 * @return float Estimated value
 */
float kalman1D_update(kalman1D_t *kf, float meas);

///< -------------------------------------------------------------
///< ------- ROBOT MODEL TRANSFORMATIONS AND FUNTIONS ------------
///< -------------------------------------------------------------

#define WHEEL_RADIUS 0.03f ///< Wheel radius in meters
#define ROBOT_RADIUS 0.16f ///< Robot diameter in meters
#define DELTA 0.523598  ///< Robot delta angle in radians (1.047198 for 60 degrees) (0.523598 radians for 30 degrees)

/**
 * @brief Given the velocity body velocity (vbx, vby) and the wheel base (wb), 
 * this function calculates the inverse kinematics to update the control and sensor fusion structure.
 * 
 * [ w1 ]       [ -sin(delta)  -cos(delta)   d ]     [ v_bx     ]
 * [ w2 ] = n/r [     1            0         d ]  *  [ v_by     ]
 * [ w3 ]       [ -sin(delta)   cos(delta)   d ]     [ omega_b  ]
 * 
 * @param vbx 
 * @param vby 
 * @param wb 
 * @param w1 Pointer to the first wheel speed
 * @param w2 Pointer to the second wheel speed
 * @param w3 Pointer to the third wheel speed
 */
void calc_invkinematics(float vbx, float vby, float wb, float *w1, float *w2, float *w3);

#endif // __CONTROL_SENFUSION_H__
