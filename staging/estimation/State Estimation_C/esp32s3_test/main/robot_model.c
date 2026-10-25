/*
 ============================================================================
 Name        : robot_model.c
 Author      : MaverickST
 Version     : 1.0
 Copyright   : MIT
 Description : Omnidirectional Robot Model Implementation
 ============================================================================
*/

#include "robot_model.h"
#include <string.h>

/* ============================================================================
 * STATE TRANSITION FUNCTION
 * ============================================================================ */

void robot_dynamics(const float *x, const float *u, float *x_out)
{
    /* Extract state - WORLD FRAME VELOCITIES (matching Python) */
    float x_pos = x[0];
    float y_pos = x[1];
    float phi = x[2];
    float vx_w = x[3];
    float vy_w = x[4];
    float omega = x[5];
    
    /* Extract control (accelerations in body frame) */
    float ax_b = u[0];
    float ay_b = u[1];
    
    /* Precompute trigonometric functions */
    float c = cosf(phi);
    float s = sinf(phi);
    
    /* Transform acceleration from body to world frame */
    float ax_w = c * ax_b - s * ay_b;
    float ay_w = s * ax_b + c * ay_b;
    
    /* Velocity update (world frame) */
    float vx_w_new = vx_w + ax_w * ROBOT_DT;
    float vy_w_new = vy_w + ay_w * ROBOT_DT;
    
    /* Position update (using world velocities) */
    float x_new = x_pos + vx_w * ROBOT_DT;
    float y_new = y_pos + vy_w * ROBOT_DT;
    
    /* Orientation update */
    float phi_new = phi + omega * ROBOT_DT;
    
    /* Angular velocity (random walk model) */
    float omega_new = omega;
    
    /* Output state */
    x_out[0] = x_new;
    x_out[1] = y_new;
    x_out[2] = normalize_angle(phi_new);
    x_out[3] = vx_w_new;
    x_out[4] = vy_w_new;
    x_out[5] = omega_new;
}

/* ============================================================================
 * MEASUREMENT FUNCTION
 * ============================================================================ */

void robot_measurement(const float *x, float *z_out)
{
    /* Extract state - WORLD FRAME VELOCITIES */
    float phi = x[2];
    float vx_w = x[3];
    float vy_w = x[4];
    float omega = x[5];
    
    /* Transform world velocities to body frame */
    float c = cosf(phi);
    float s = sinf(phi);
    
    float vx_b =  c * vx_w + s * vy_w;
    float vy_b = -s * vx_w + c * vy_w;
    
    /* Measurement: [vx_b, vy_b, omega, psi] */
    z_out[0] = vx_b;
    z_out[1] = vy_b;
    z_out[2] = omega;
    z_out[3] = phi;  /* psi = phi */
}

/* ============================================================================
 * STATE JACOBIAN (for EKF)
 * ============================================================================ */

void robot_jacobian_F(const float *x, const float *u, float *F)
{
    /* Initialize to zero */
    memset(F, 0, STATE_DIM * STATE_DIM * sizeof(float));
    
    /* Extract state and input */
    float psi = x[2];
    float ax_b = u[0];
    float ay_b = u[1];
    
    /* Precompute trigonometric functions */
    float cos_psi = cosf(psi);
    float sin_psi = sinf(psi);
    
    /* F is 6x6 matrix (row-major order): F[i][j] = F[i * 6 + j]
     * State: x = [x, y, psi, vx_w, vy_w, omega]
     * 
     * From Python:
     * F[0, 3] = dt
     * F[1, 4] = dt
     * F[2, 5] = dt
     * F[3, 2] = (-s * ax_b - c * ay_b) * dt
     * F[4, 2] = ( c * ax_b - s * ay_b) * dt
     */
    
    /* Position derivatives */
    F[0 * STATE_DIM + 0] = 1.0f;  /* dx/dx */
    F[0 * STATE_DIM + 3] = ROBOT_DT;  /* dx/dvx_w */
    
    F[1 * STATE_DIM + 1] = 1.0f;  /* dy/dy */
    F[1 * STATE_DIM + 4] = ROBOT_DT;  /* dy/dvy_w */
    
    /* Orientation */
    F[2 * STATE_DIM + 2] = 1.0f;  /* dpsi/dpsi */
    F[2 * STATE_DIM + 5] = ROBOT_DT;  /* dpsi/domega */
    
    /* Velocity derivatives w.r.t psi (rotation effect) */
    F[3 * STATE_DIM + 2] = ROBOT_DT * (-sin_psi * ax_b - cos_psi * ay_b);  /* dvx_w/dpsi */
    F[3 * STATE_DIM + 3] = 1.0f;  /* dvx_w/dvx_w */
    
    F[4 * STATE_DIM + 2] = ROBOT_DT * (cos_psi * ax_b - sin_psi * ay_b);  /* dvy_w/dpsi */
    F[4 * STATE_DIM + 4] = 1.0f;  /* dvy_w/dvy_w */
    
    /* Angular velocity (random walk) */
    F[5 * STATE_DIM + 5] = 1.0f;  /* domega/domega */
}

/* ============================================================================
 * MEASUREMENT JACOBIAN (for EKF)
 * ============================================================================ */

void robot_jacobian_H(const float *x, float *H)
{
    /* Initialize to zero */
    memset(H, 0, MEAS_DIM * STATE_DIM * sizeof(float));
    
    /* Extract state */
    float psi = x[2];
    float vx_w = x[3];
    float vy_w = x[4];
    
    /* Precompute trigonometric functions */
    float cos_psi = cosf(psi);
    float sin_psi = sinf(psi);
    
    /* H is 4x6 matrix (row-major order): H[i][j] = H[i * 6 + j]
     * Measurement: z = [vx_b, vy_b, omega, psi]
     * State: x = [x, y, psi, vx_w, vy_w, omega]
     * 
     * From Python:
     * vx_b =  cos(psi) * vx_w + sin(psi) * vy_w
     * vy_b = -sin(psi) * vx_w + cos(psi) * vy_w
     * 
     * H[0, 2] = -sin(psi) * vx_w + cos(psi) * vy_w  (dvx_b/dpsi)
     * H[0, 3] =  cos(psi)                            (dvx_b/dvx_w)
     * H[0, 4] =  sin(psi)                            (dvx_b/dvy_w)
     * 
     * H[1, 2] = -cos(psi) * vx_w - sin(psi) * vy_w  (dvy_b/dpsi)
     * H[1, 3] = -sin(psi)                            (dvy_b/dvx_w)
     * H[1, 4] =  cos(psi)                            (dvy_b/dvy_w)
     * 
     * H[2, 5] = 1.0  (domega/domega)
     * H[3, 2] = 1.0  (dpsi/dpsi)
     */
    
    /* vx_b measurement */
    H[0 * STATE_DIM + 2] = -sin_psi * vx_w + cos_psi * vy_w;  /* dvx_b/dpsi */
    H[0 * STATE_DIM + 3] =  cos_psi;                           /* dvx_b/dvx_w */
    H[0 * STATE_DIM + 4] =  sin_psi;                           /* dvx_b/dvy_w */
    
    /* vy_b measurement */
    H[1 * STATE_DIM + 2] = -cos_psi * vx_w - sin_psi * vy_w;  /* dvy_b/dpsi */
    H[1 * STATE_DIM + 3] = -sin_psi;                           /* dvy_b/dvx_w */
    H[1 * STATE_DIM + 4] =  cos_psi;                           /* dvy_b/dvy_w */
    
    /* omega measurement */
    H[2 * STATE_DIM + 5] = 1.0f;  /* domega/domega */
    
    /* psi measurement */
    H[3 * STATE_DIM + 2] = 1.0f;  /* dpsi/dpsi */
}
