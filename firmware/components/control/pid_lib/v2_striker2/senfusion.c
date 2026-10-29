#include "senfusion.h"

void senfusion_init(senfusion_t *senfusion, float dt)
{
    ///< ----------------- SENSOR FUSION -----------------
    senfusion->dt = dt;
    senfusion->k = 0;
    senfusion->x[0][0] = 0;
    senfusion->x[1][0] = 0;
    senfusion->z[0][0] = 0;
    senfusion->z[1][0] = 0;

    ///< Initialize the process model: x(k+1) = A*x(k) + B*u
    senfusion->A[0][0] = 0.9955;
    senfusion->A[0][1] = 0.00931;
    senfusion->A[1][0] = -0.8805;
    senfusion->A[1][1] = 0.8638;
    senfusion->B[0][0] = 8.81e-05;
    senfusion->B[1][0] = 0.0172;
    senfusion->C[0][0] = 1;
    senfusion->C[0][1] = 0;
    senfusion->C[1][0] = 0;
    senfusion->C[1][1] = 1;

    ///< Initialize the measurement model: z(k) = H*x(k) + v(k)
    senfusion->H[0][0] = 1; 
    senfusion->H[0][1] = 0;
    senfusion->H[1][0] = 1;
    senfusion->H[1][1] = 0;
    senfusion->H[2][0] = 0;
    senfusion->H[2][1] = 1;
    senfusion->H_T[0][0] = 1;
    senfusion->H_T[0][1] = 1;
    senfusion->H_T[0][2] = 0;
    senfusion->H_T[1][0] = 0;
    senfusion->H_T[1][1] = 0;
    senfusion->H_T[1][2] = 1;

    ///< Initialize the error covariance matrix: diagonal matrix
    senfusion->P[0][0] = 1;
    senfusion->P[0][1] = 0;
    senfusion->P[1][0] = 0;
    senfusion->P[1][1] = 1;

    ///< Initialize the process noise covariance matrix: diagonal matrix
    float sigma_a = 1.0;
    senfusion->Q[0][0] = 0.25 * dt * dt * dt * dt * sigma_a;
    senfusion->Q[0][1] = 0.5 * dt * dt * dt * sigma_a;
    senfusion->Q[1][0] = 0.5 * dt * dt * dt * sigma_a;
    senfusion->Q[1][1] = dt * dt * sigma_a;

    ///< Initialize the measurement noise covariance matrix: diagonal matrix
    ///< The covariance for Encoder and Lidar are constant, but the covariance for IMU is time varying
    //< for encoder: sigma_p_enc = 7.84 um
    //< for lidar: sigma_p_lidar = 0.02 m
    //< for IMU: sigma_v_imu = sqrt(k) * sigma_a * dt, sigma_a = 0.0148 m/s^2
    float sigma_p_enc = 0.000784; ///< 7.84 um
    float sigma_p_lidar = 0.02; ///< 2 cm
    float sigma_v_imu = sqrt(senfusion->k) * 0.0148 * senfusion->dt; ///< sqrt(k) * sigma_a * dt
    senfusion->R[0][0] = sigma_p_enc * sigma_p_enc; ///< Encoder
    senfusion->R[0][1] = 0;
    senfusion->R[1][0] = 0;
    senfusion->R[1][1] = sigma_p_lidar * sigma_p_lidar; ///< Lidar
    senfusion->R[2][0] = 0;
    senfusion->R[2][1] = 0;
    senfusion->R[2][2] = sigma_v_imu * sigma_v_imu; ///< IMU
}

void senfusion_predict(senfusion_t *senfusion, float u)
{
    float x_pred[2][1] = {0};
    float P_pred[2][2] = {0};

    ///< Predict the next state: x_pred = A * x + B * u
    matrixMultiplication(2, 2, 1, senfusion->A, senfusion->x, x_pred); // A * x
    senfusion->x_pred[0][0] = x_pred[0][0] + senfusion->B[0][0] * u; 
    senfusion->x_pred[1][0] = x_pred[1][0] + senfusion->B[1][0] * u;

    ///< Predict the next error covariance matrix: P_pred = A * P * Aᵀ + Q
    matrixMultiplication(2, 2, 2, senfusion->A, senfusion->P, P_pred); // A * P
    float A_transpose[2][2] = {0};
    matrixTranspose(2, 2, senfusion->A, A_transpose);
    matrixMultiplication(2, 2, 2, P_pred, A_transpose, senfusion->P_pred); // A * P * Aᵀ
    matrixAddSub(2, 2, senfusion->P_pred, senfusion->Q, senfusion->P_pred, '+'); // A * P * Aᵀ + Q

}

void senfusion_update(senfusion_t *senfusion, float p_enc, float p_lidar, float a_imu, uint32_t k)
{
    
    senfusion->R[2][2] = sqrt(k) * 0.0148 * senfusion->dt; ///< Update the IMU covariance
    senfusion->k = k; ///< Update the sample time

    ///< Covariance innovation: S = H * Pₖ|ₖ₋₁ * Hᵀ + R
    float S[3][3] = {0};
    float H_P_pred[3][2] = {0};
    matrixMultiplication(3, 2, 2, senfusion->H, senfusion->P_pred, H_P_pred); // H * Pₖ|ₖ₋₁
    matrixMultiplication(3, 2, 3, H_P_pred, senfusion->H_T, S); // H * Pₖ|ₖ₋₁ * Hᵀ
    matrixAddSub(3, 3, S, senfusion->R, S, '+'); // H * Pₖ|ₖ₋₁ * Hᵀ + R
    // print S
    // printf("S: %.4f, %.4f, %.4f\n %.4f, %.4f, %.4f\n %.4f, %.4f, %.4f\n", S[0][0], S[0][1], S[0][2], S[1][0], S[1][1], S[1][2], S[2][0], S[2][1], S[2][2]);

    ///< Kalman gain: K = Pₖ|ₖ₋₁ * Hᵀ * inv(S)  , 2x2 * 2x3 * 3x3 = 2x3
    float K[2][3] = {0};
    float P_pred_H_T[2][3] = {0};
    matrixMultiplication(2, 2, 3, senfusion->P_pred, senfusion->H_T, P_pred_H_T); // Pₖ|ₖ₋₁ * Hᵀ
    solve_LU_system(S, P_pred_H_T, K); // solve the linear system S * K = Pₖ|ₖ₋₁ * Hᵀ
    // printf("K: %.4f, %.4f, %.4f\n %.4f, %.4f, %.4f\n", K[0][0], K[0][1], K[0][2], K[1][0], K[1][1], K[1][2]);

    ///< Innovation (residue): y = z - H * xₖ|ₖ₋₁
    float v_imu = senfusion->v_imu_prev + a_imu * senfusion->dt; ///< IMU velocity: v_imu = v_imu_prev + a_imu * dt
    float z[3][1] = {{p_enc}, {p_lidar}, {v_imu}}; ///< Measurement vector
    float y[3][1] = {0};
    matrixMultiplication(3, 2, 2, senfusion->H, senfusion->x_pred, y); // H * xₖ|ₖ₋₁
    matrixAddSub(3, 1, z, y, y, '-'); // z - H * xₖ|ₖ₋₁
    senfusion->v_imu_prev = v_imu; ///< Update the IMU velocity for the next iteration
    // printf("y: %.4f, %.4f, %.4f\n", y[0][0], y[1][0], y[2][0]);

    ///< Update the state: xₖ = xₖ|ₖ₋₁ + K * y
    float x[2][1] = {0};
    matrixMultiplication(2, 3, 1, K, y, x); // K * y
    senfusion->x[0][0] = senfusion->x_pred[0][0] + x[0][0]; // xₖ = xₖ|ₖ₋₁ + K * y
    senfusion->x[1][0] = senfusion->x_pred[1][0] + x[1][0]; // xₖ = xₖ|ₖ₋₁ + K * y
    // printf("x: %.4f, %.4f\n", x[0][0], x[1][0]);

    ///< Update the error covariance matrix: Pₖ = (I - K * H) * Pₖ|ₖ₋₁
    float I_K_H[2][2] = {0};
    float KH[2][2] = {0};
    matrixMultiplication(2, 3, 2, K, senfusion->H, KH); // K * H
    I_K_H[0][0] = 1 - KH[0][0]; // I - K * H
    I_K_H[0][1] = -KH[0][1];
    I_K_H[1][0] = -KH[1][0];
    I_K_H[1][1] = 1 - KH[1][1];
    matrixMultiplication(2, 2, 2, I_K_H, senfusion->P_pred, senfusion->P); // (I - K * H) * Pₖ|ₖ₋₁
    // printf("KH: %.4f, %.4f\n %.4f, %.4f\n", KH[0][0], KH[0][1], KH[1][0], KH[1][1]);
    // printf("P: %.4f, %.4f\n %.4f, %.4f\n", senfusion->P[0][0], senfusion->P[0][1], senfusion->P[1][0], senfusion->P[1][1]);
}

///<-------------------------------------------------------------
///<--------------- LINEAR ALGEBRA FUNCTIONS --------------------
///<-------------------------------------------------------------

void matrixMultiplication(int rowsA, int colsA, int colsB, float A[rowsA][colsA], float B[colsA][colsB], float C[rowsA][colsB]) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            C[i][j] = 0;  // Initialize each element of C
            for (int k = 0; k < colsA; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrixAddSub(int rows, int cols, float A[rows][cols], float B[rows][cols], float C[rows][cols], char operation) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (operation == '+') {
                C[i][j] = A[i][j] + B[i][j];  // Add of matrices
            } else if (operation == '-') {
                C[i][j] = A[i][j] - B[i][j];  // Sub of matrices
            } else {
                printf("Invalid operation. Use '+' to add or '-' to sub.\n");
                return;
            }
        }
    }
}

void matrixTranspose(int rows, int cols, float A[rows][cols], float B[cols][rows]) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            B[j][i] = A[i][j];  // Transpose of matrix
        }
    }
}

/// <-------------------------------------------------------------
/// <------------------ LU DECOMPOSITION -------------------------
/// <-------------------------------------------------------------

#define N 3       // Size of matrix A (NxN)
#define M 2       // Number of columns in B (right-hand sides)

void LU_decomposition(float A[N][N], float L[N][N], float U[N][N]) {
    for (int i = 0; i < N; i++) {
        // Compute U matrix
        for (int k = i; k < N; k++) {
            float sum = 0.0;
            for (int j = 0; j < i; j++)
                sum += L[i][j] * U[j][k];
            U[i][k] = A[i][k] - sum;
        }

        // Compute L matrix
        for (int k = i; k < N; k++) {
            if (i == k)
                L[i][i] = 1.0;
            else {
                float sum = 0.0;
                for (int j = 0; j < i; j++)
                    sum += L[k][j] * U[j][i];
                L[k][i] = (A[k][i] - sum) / U[i][i];
            }
        }
    }
}

void forward_substitution(float L[N][N], float b[N], float y[N]) { // L·y = b
    for (int i = 0; i < N; i++) {
        float s = b[i];
        for (int j = 0; j < i; j++) s -= L[i][j] * y[j];
        y[i] = s;  // L[i][i] == 1
    }
}

void backward_substitution(float U[N][N], float y[N], float x[N]) { // U·x = y
    for (int i = N - 1; i >= 0; i--) {
        float s = y[i];
        for (int j = i + 1; j < N; j++) s -= U[i][j] * x[j];
        x[i] = s / U[i][i];
    }
}

void solve_LU_system(float A[N][N], float B[M][N], float X[M][N]) {
    float A_T[N][N];
    float L[N][N] = {0}, U[N][N] = {0};

    // 1) Transpose A to get A_T = A^T
    matrixTranspose(N, N, A, A_T);

    // 2) Factorize A_T into L and U: A_T = L * U
    LU_decomposition(A_T, L, U);

    // 3) For each row i of B, solve (A^T) * z = b, then set X[i,:] = z^T
    for (int i = 0; i < M; i++) {
        float b[N], y[N], z[N];

        // 3a) Build the right-hand side vector b = B[i,:]^T
        for (int k = 0; k < N; k++) {
            b[k] = B[i][k];
        }

        // 3b) Forward substitution: L * y = b
        forward_substitution(L, b, y);

        // 3c) Backward substitution: U * z = y
        backward_substitution(U, y, z);

        // 3d) Store the solution z^T into the i-th row of X
        for (int j = 0; j < N; j++) {
            X[i][j] = z[j];
        }
    }
}

void kalman1D_init(kalman1D_t *kf, float Q, float R)
{
    kf->x = 0.0f;  // Initial estimated value
    kf->P = 1.0f;  // Initial estimation error covariance
    kf->Q = Q;     // Process noise covariance
    kf->R = R;     // Measurement noise covariance
}

float kalman1D_update(kalman1D_t *kf, float meas)
{
    // Prediction step
    kf->P += kf->Q;  // Update estimation error covariance

    // Measurement update step
    float K = kf->P / (kf->P + kf->R);  // Kalman gain
    kf->x += K * (meas - kf->x);  // Update estimated value
    kf->P *= (1 - K);  // Update estimation error covariance

    return kf->x;  // Return the estimated value
}

void calc_invkinematics(float vbx, float vby, float wb, float *w1, float *w2, float *w3)
{
    *w1 = (-vbx*sin(DELTA) - vby*cos(DELTA) + ROBOT_RADIUS*wb) / WHEEL_RADIUS;
    *w2 = (vbx + ROBOT_RADIUS * wb) / WHEEL_RADIUS;
    *w3 = (-vbx*sin(DELTA) + vby*cos(DELTA) + ROBOT_RADIUS*wb) / WHEEL_RADIUS;
}
