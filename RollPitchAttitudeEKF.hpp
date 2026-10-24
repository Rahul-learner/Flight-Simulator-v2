#pragma once

#include <math.h>
#include <string.h>
#include <stdint.h>

/**
 * @file RollPitchAttitudeEKF.hpp
 * @brief High-Precision 2-Axis (Roll & Pitch) Error-State Extended Kalman Filter (MEKF)
 *        specifically designed for 6-DOF IMUs (Accelerometer + Gyroscope) without Magnetometer.
 * 
 * Why Remove Yaw?
 * In a 6-DOF IMU without an external magnetic compass, the Yaw (heading) angle is mathematically
 * unobservable from the gravity vector. Attempting to estimate Yaw results in open-loop gyro
 * integration drift. By removing Yaw and formulating the filter directly around the gravity
 * reference vector, Roll and Pitch remain 100% observable, zero-drift, and bounded forever!
 * 
 * Features:
 * - 4-State Error Formulation: [delta_theta_x, delta_theta_y, delta_bx, delta_by]
 * - Closed-Form Rodrigues Kinematics for gravity vector propagation
 * - Joseph-Form Stabilized Covariance Update (guarantees positive semi-definiteness)
 * - Dynamic Acceleration Gating (prevents artificial horizon tilting under centrifugal turns)
 * - Online Gyroscope Bias Estimation on X and Y axes
 * - Zero Yaw Drift forever!
 */

class RollPitchAttitudeEKF {
public:
    // Estimated unit gravity vector in body frame: gb = [gx, gy, gz]^T
    // In level orientation: gb = [0, 0, 1]^T
    float gb[3];

    // Gyroscope bias estimates in rad/s [bx, by, bz]
    float bg[3];

    // Covariance matrix P (4x4): [delta_theta(2), delta_bias(2)]
    float P[4][4];

    // Process & Measurement Noise Tuning
    float q_gyro_noise;   // Process noise for angular rates (rad/s)^2
    float q_bias_noise;   // Process noise for gyro bias drift (rad/s^2)^2
    float r_accel_noise;  // Accelerometer measurement variance (g^2)

    // Dynamic acceleration rejection settings
    bool dynamic_rejection_enabled;
    float dynamic_accel_threshold; // Deviation from 1.0g (default 0.35g)

    // Diagnostics
    float last_dt;
    float last_accel_norm;
    bool last_accel_gated;

    RollPitchAttitudeEKF() {
        reset();
    }

    void reset(float initial_roll_rad = 0.0f, float initial_pitch_rad = 0.0f) {
        // Compute initial gravity vector from roll and pitch
        float sp = sinf(initial_pitch_rad);
        float cp = cosf(initial_pitch_rad);
        float sr = sinf(initial_roll_rad);
        float cr = cosf(initial_roll_rad);

        gb[0] = -sp;
        gb[1] = cp * sr;
        gb[2] = cp * cr;
        normalizeGb();

        bg[0] = 0.0f;
        bg[1] = 0.0f;
        bg[2] = 0.0f;

        q_gyro_noise  = 1.0e-5f;  // Gyro angular random walk
        q_bias_noise  = 1.0e-8f;  // Bias drift rate
        r_accel_noise = 0.030f;   // Accelerometer noise covariance

        dynamic_rejection_enabled = true;
        dynamic_accel_threshold = 0.35f;

        last_dt = 0.0f;
        last_accel_norm = 1.0f;
        last_accel_gated = false;

        memset(P, 0, sizeof(P));
        P[0][0] = 0.05f;    // Initial roll uncertainty
        P[1][1] = 0.05f;    // Initial pitch uncertainty
        P[2][2] = 1.0e-4f;  // Initial gyro X bias uncertainty
        P[3][3] = 1.0e-4f;  // Initial gyro Y bias uncertainty
    }

    void initFromSensors(float ax_init, float ay_init, float az_init,
                        float bx_init = 0.0f, float by_init = 0.0f, float bz_init = 0.0f) {
        float norm = sqrtf(ax_init*ax_init + ay_init*ay_init + az_init*az_init);
        if (norm > 0.01f) {
            gb[0] = ax_init / norm;
            gb[1] = ay_init / norm;
            gb[2] = az_init / norm;
        } else {
            gb[0] = 0.0f; gb[1] = 0.0f; gb[2] = 1.0f;
        }

        bg[0] = bx_init;
        bg[1] = by_init;
        bg[2] = bz_init;

        memset(P, 0, sizeof(P));
        P[0][0] = 0.01f;
        P[1][1] = 0.01f;
        P[2][2] = 1.0e-4f;
        P[3][3] = 1.0e-4f;
    }

    /**
     * @brief EKF Prediction Step: High-Rate Rodrigues Kinematic Propagation
     * @param dt Sampling period in seconds
     * @param gx Gyroscope X rate in rad/s
     * @param gy Gyroscope Y rate in rad/s
     * @param gz Gyroscope Z rate in rad/s
     */
    void predict(float dt, float gx, float gy, float gz) {
        last_dt = dt;
        if (dt <= 0.0f || dt > 0.1f) return;

        // Unbias angular velocity
        float wx = gx - bg[0];
        float wy = gy - bg[1];
        float wz = gz - bg[2];

        // 1. Exact Rodrigues Rotation Propagation for the Gravity Vector:
        // d(gb)/dt = -omega x gb = gb x omega
        // Rotates gb by angle theta = ||omega|| * dt around axis u = omega / ||omega||
        float rx = -wx * dt;
        float ry = -wy * dt;
        float rz = -wz * dt;
        float theta = sqrtf(rx*rx + ry*ry + rz*rz);

        if (theta > 1.0e-7f) {
            float inv_theta = 1.0f / theta;
            float ux = rx * inv_theta;
            float uy = ry * inv_theta;
            float uz = rz * inv_theta;

            float s = sinf(theta);
            float c = cosf(theta);
            float c1 = 1.0f - c;

            // R(u, theta) * gb
            float u_dot_gb = ux * gb[0] + uy * gb[1] + uz * gb[2];
            float u_cross_gb[3] = {
                uy * gb[2] - uz * gb[1],
                uz * gb[0] - ux * gb[2],
                ux * gb[1] - uy * gb[0]
            };

            float new_gb[3];
            new_gb[0] = c * gb[0] + s * u_cross_gb[0] + c1 * u_dot_gb * ux;
            new_gb[1] = c * gb[1] + s * u_cross_gb[1] + c1 * u_dot_gb * uy;
            new_gb[2] = c * gb[2] + s * u_cross_gb[2] + c1 * u_dot_gb * uz;

            gb[0] = new_gb[0];
            gb[1] = new_gb[1];
            gb[2] = new_gb[2];
        }

        normalizeGb();

        // 2. Propagate Covariance P: P = F * P * F^T + Q
        // State transition Jacobian F (4x4):
        // F = [ [ 1,  wz*dt, -dt,   0 ],
        //       [-wz*dt,  1,   0, -dt ],
        //       [ 0,      0,   1,   0 ],
        //       [ 0,      0,   0,   1 ] ]
        float wz_dt = wz * dt;
        float F[4][4] = {
            { 1.0f,   wz_dt, -dt,   0.0f },
            {-wz_dt,  1.0f,   0.0f, -dt  },
            { 0.0f,   0.0f,   1.0f, 0.0f },
            { 0.0f,   0.0f,   0.0f, 1.0f }
        };

        // FP = F * P (4x4)
        float FP[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += F[i][k] * P[k][j];
                }
                FP[i][j] = sum;
            }
        }

        // P = FP * F^T + Q
        float dt2 = dt * dt;
        float q_gyro = q_gyro_noise * dt2;
        float q_bias = q_bias_noise * dt;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += FP[i][k] * F[j][k];
                }
                if (i == j) {
                    sum += (i < 2) ? q_gyro : q_bias;
                }
                P[i][j] = sum;
            }
        }

        symmetrizeP();
    }

    /**
     * @brief EKF Measurement Update Step: Accelerometer Tilt Correction
     * @param ax Raw accelerometer X in g
     * @param ay Raw accelerometer Y in g
     * @param az Raw accelerometer Z in g
     */
    void update(float ax, float ay, float az) {
        float a_norm = sqrtf(ax*ax + ay*ay + az*az);
        last_accel_norm = a_norm;

        if (a_norm < 0.1f) return;

        // Dynamic Acceleration Gating & Adaptive Noise
        float delta_a = fabsf(a_norm - 1.0f);
        if (dynamic_rejection_enabled && delta_a > dynamic_accel_threshold) {
            last_accel_gated = true;
            return; // Inhibit accel update during violent dynamic G-forces
        }
        last_accel_gated = false;

        float R_adaptive = r_accel_noise;
        if (dynamic_rejection_enabled && delta_a > 0.05f) {
            R_adaptive *= (1.0f + 25.0f * delta_a * delta_a);
        }

        // Normalized accelerometer measurement
        float inv_norm = 1.0f / a_norm;
        float ax_n = ax * inv_norm;
        float ay_n = ay * inv_norm;
        float az_n = az * inv_norm;

        // Innovation y = a_meas - gb (3x1)
        float y[3] = {
            ax_n - gb[0],
            ay_n - gb[1],
            az_n - gb[2]
        };

        // Measurement Jacobian H (3x4):
        // H = d(gb)/d(delta_x) = [ [   0, -gb[2], 0, 0 ],
        //                          [ gb[2],    0, 0, 0 ],
        //                          [-gb[1], gb[0], 0, 0 ] ]
        float H[3][4] = {
            {   0.0f, -gb[2], 0.0f, 0.0f },
            {  gb[2],   0.0f, 0.0f, 0.0f },
            { -gb[1],  gb[0], 0.0f, 0.0f }
        };

        // HP = H * P (3x4)
        float HP[3][4];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                HP[i][j] = H[i][0] * P[0][j] + H[i][1] * P[1][j] +
                           H[i][2] * P[2][j] + H[i][3] * P[3][j];
            }
        }

        // S = HP * H^T + R * I (3x3)
        float S[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = HP[i][0] * H[j][0] + HP[i][1] * H[j][1] +
                            HP[i][2] * H[j][2] + HP[i][3] * H[j][3];
                if (i == j) sum += R_adaptive;
                S[i][j] = sum;
            }
        }

        // Invert 3x3 matrix S
        float S_inv[3][3];
        if (!invert3x3(S, S_inv)) {
            return;
        }

        // PHT = P * H^T (4x3)
        float PHT[4][3];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                PHT[i][j] = P[i][0] * H[j][0] + P[i][1] * H[j][1] +
                            P[i][2] * H[j][2] + P[i][3] * H[j][3];
            }
        }

        // Kalman Gain K = PHT * S_inv (4x3)
        float K[4][3];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                K[i][j] = PHT[i][0] * S_inv[0][j] +
                          PHT[i][1] * S_inv[1][j] +
                          PHT[i][2] * S_inv[2][j];
            }
        }

        // Error State Correction: dx = K * y (4x1)
        float dx[4];
        for (int i = 0; i < 4; i++) {
            dx[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
        }

        // Stabilized Joseph-Form Covariance Update:
        // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        float I_KH[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float kh = K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j];
                I_KH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
            }
        }

        float temp[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += I_KH[i][k] * P[k][j];
                }
                temp[i][j] = sum;
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += temp[i][k] * I_KH[j][k];
                }
                // K * R * K^T
                float krk = (K[i][0] * K[j][0] + K[i][1] * K[j][1] + K[i][2] * K[j][2]) * R_adaptive;
                P[i][j] = sum + krk;
            }
        }

        symmetrizeP();

        // Inject error state into gravity vector:
        // delta_gb = - [delta_theta]_x * gb
        float dtheta_x = dx[0];
        float dtheta_y = dx[1];

        gb[0] += -dtheta_y * gb[2];
        gb[1] +=  dtheta_x * gb[2];
        gb[2] += (dtheta_y * gb[0] - dtheta_x * gb[1]);
        normalizeGb();

        // Inject gyro biases with soft clamp
        const float MAX_BIAS_STEP = 0.002f;
        float dbx = dx[2];
        float dby = dx[3];
        if (dbx >  MAX_BIAS_STEP) dbx =  MAX_BIAS_STEP;
        if (dbx < -MAX_BIAS_STEP) dbx = -MAX_BIAS_STEP;
        if (dby >  MAX_BIAS_STEP) dby =  MAX_BIAS_STEP;
        if (dby < -MAX_BIAS_STEP) dby = -MAX_BIAS_STEP;

        bg[0] += dbx;
        bg[1] += dby;
    }

    /**
     * @brief Extract Euler Angles in degrees (Pure Roll & Pitch, Zero Yaw)
     * @param roll_deg Output roll angle (-180 to +180 deg)
     * @param pitch_deg Output pitch angle (-90 to +90 deg)
     */
    void getEuler(float& roll_deg, float& pitch_deg) const {
        // Pitch: rotation about Y-axis
        // gb[0] = -sin(pitch)
        float sp = -gb[0];
        if (sp > 1.0f) sp = 1.0f;
        if (sp < -1.0f) sp = -1.0f;
        float pitch_rad = asinf(sp);

        // Roll: rotation about X-axis
        // gb[1] = cos(pitch) * sin(roll)
        // gb[2] = cos(pitch) * cos(roll)
        float roll_rad = atan2f(gb[1], gb[2]);

        constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
        roll_deg  = roll_rad * RAD_TO_DEG;
        pitch_deg = pitch_rad * RAD_TO_DEG;
    }

    /**
     * @brief Get Pure 2-Axis Unit Quaternion [w, x, y, z] with Yaw = 0.0
     */
    void getQuaternion(float q[4]) const {
        float r_deg, p_deg;
        getEuler(r_deg, p_deg);

        constexpr float DEG_TO_RAD_2 = (3.14159265358979323846f / 180.0f) * 0.5f;
        float cr = cosf(r_deg * DEG_TO_RAD_2);
        float sr = sinf(r_deg * DEG_TO_RAD_2);
        float cp = cosf(p_deg * DEG_TO_RAD_2);
        float sp = sinf(p_deg * DEG_TO_RAD_2);

        // With yaw = 0: cy = 1.0, sy = 0.0
        q[0] = cr * cp;        // w
        q[1] = sr * cp;        // x
        q[2] = cr * sp;        // y
        q[3] = -sr * sp;       // z
    }

private:
    void normalizeGb() {
        float norm = sqrtf(gb[0]*gb[0] + gb[1]*gb[1] + gb[2]*gb[2]);
        if (norm > 1.0e-7f) {
            float inv = 1.0f / norm;
            gb[0] *= inv;
            gb[1] *= inv;
            gb[2] *= inv;
        } else {
            gb[0] = 0.0f; gb[1] = 0.0f; gb[2] = 1.0f;
        }
    }

    void symmetrizeP() {
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                float avg = 0.5f * (P[i][j] + P[j][i]);
                P[i][j] = avg;
                P[j][i] = avg;
            }
            if (P[i][i] < 1.0e-8f) P[i][i] = 1.0e-8f;
        }
    }

    bool invert3x3(const float A[3][3], float inv[3][3]) {
        float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

        if (fabsf(det) < 1.0e-9f) return false;

        float inv_det = 1.0f / det;
        inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
        inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
        inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;

        inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
        inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
        inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;

        inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
        inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
        inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

        return true;
    }
};
