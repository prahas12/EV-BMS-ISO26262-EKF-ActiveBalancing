/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_ekf.c
 * Description: Production Discrete Extended Kalman Filter implementation.
 * ============================================================================
 */

#include "bms_ekf.h"
#include <math.h>

/* Cell Model Reference Constants (LG / Samsung 21700 NMC) */
static const float R0_REF        = 0.0150f;     /* 15 mOhm */
static const float R1_REF        = 0.0085f;     /* 8.5 mOhm */
static const float C1_REF        = 1500.0f;     /* Farads */
static const float R2_REF        = 0.0120f;     /* 12.0 mOhm */
static const float C2_REF        = 6000.0f;     /* Farads */
static const float Q_NOMINAL_AS  = 18000.0f;    /* 5.0 Ah * 3600 */
static const float COULOMB_EFF   = 0.998f;      /* Charge efficiency */
static const float EA_R0         = 22000.0f;    /* J/mol */
static const float R_GAS         = 8.314f;      /* J/(mol K) */
static const float T_REF_K       = 298.15f;     /* 25 deg C in K */

/* Process Noise Q Covariance Diagonals */
static const float Q_SOC = 2.0e-6f;
static const float Q_V1  = 1.0e-5f;
static const float Q_V2  = 1.0e-5f;

/* Measurement Noise R Variance */
static const float R_MEAS = 1.0e-3f;

/* 6th-Order Calibrated NMC OCV Polynomial Coefficients */
static const float P_OCV[7] = {
    -1.94295f,   /* s^6 */
     8.52070f,   /* s^5 */
   -18.71091f,   /* s^4 */
    21.62724f,   /* s^3 */
   -12.16665f,   /* s^2 */
     3.71743f,   /* s^1 */
     3.15347f    /* s^0 */
};

/* Analytic Derivative dVoc/dSoC Coefficients */
static const float P_DVOC[6] = {
    -11.65770f,  /* 6 * p1 */
     42.60350f,  /* 5 * p2 */
    -74.84364f,  /* 4 * p3 */
     64.88172f,  /* 3 * p4 */
    -24.33330f,  /* 2 * p5 */
      3.71743f   /* 1 * p6 */
};

float bms_ekf_get_ocv(float soc)
{
    float s = (soc < 0.0f) ? 0.0f : ((soc > 1.0f) ? 1.0f : soc);
    /* Horner's method for efficient, numerically stable polynomial evaluation */
    float val = P_OCV[0];
    for (int i = 1; i < 7; ++i) {
        val = val * s + P_OCV[i];
    }
    return val;
}

float bms_ekf_get_dvoc_dsoc(float soc)
{
    float s = (soc < 0.0f) ? 0.0f : ((soc > 1.0f) ? 1.0f : soc);
    float val = P_DVOC[0];
    for (int i = 1; i < 6; ++i) {
        val = val * s + P_DVOC[i];
    }
    return (val < 0.05f) ? 0.05f : val; /* Protect against negative gradient */
}

void bms_ekf_init(bms_ekf_t *filter, float initial_soc)
{
    if (filter == NULL) return;

    filter->x[0] = (initial_soc < 0.0f) ? 0.0f : ((initial_soc > 1.0f) ? 1.0f : initial_soc);
    filter->x[1] = 0.0f;
    filter->x[2] = 0.0f;

    /* Initialize P Covariance */
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            filter->P[r][c] = 0.0f;
        }
    }
    filter->P[0][0] = 0.20f;
    filter->P[1][1] = 0.01f;
    filter->P[2][2] = 0.01f;

    filter->soc_estimate = filter->x[0];
    filter->vt_estimate  = bms_ekf_get_ocv(filter->x[0]);
    filter->sigma_bound  = 3.0f * sqrtf(filter->P[0][0]);
}

void bms_ekf_step(bms_ekf_t *filter, float I_L, float Vt_meas, float T_degC, float dt)
{
    if (filter == NULL) return;

    /* 1. Arrhenius Temperature Scaling */
    float T_K = T_degC + 273.15f;
    float arrhenius = expf((EA_R0 / R_GAS) * (1.0f / T_K - 1.0f / T_REF_K));
    float R0 = R0_REF * arrhenius;

    /* RC Time Constants */
    float alpha1 = expf(-dt / (R1_REF * C1_REF));
    float alpha2 = expf(-dt / (R2_REF * C2_REF));

    float eta = (I_L < 0.0f) ? COULOMB_EFF : 1.0f;

    /* 2. State Prediction: x_pred = Ad * x + Bd * I_L */
    float x_pred[3];
    x_pred[0] = filter->x[0] - (eta * dt / Q_NOMINAL_AS) * I_L;
    x_pred[1] = alpha1 * filter->x[1] + R1_REF * (1.0f - alpha1) * I_L;
    x_pred[2] = alpha2 * filter->x[2] + R2_REF * (1.0f - alpha2) * I_L;

    /* Clamping predicted SoC */
    if (x_pred[0] < 0.0f) x_pred[0] = 0.0f;
    if (x_pred[0] > 1.0f) x_pred[0] = 1.0f;

    /* 3. Covariance Prediction: P_pred = Ad * P * Ad^T + Q */
    float P_pred[3][3];
    float a[3] = {1.0f, alpha1, alpha2};

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            P_pred[r][c] = a[r] * filter->P[r][c] * a[c];
        }
    }
    P_pred[0][0] += Q_SOC;
    P_pred[1][1] += Q_V1;
    P_pred[2][2] += Q_V2;

    /* 4. Measurement Prediction */
    float Voc_pred = bms_ekf_get_ocv(x_pred[0]);
    float Vt_est   = Voc_pred - x_pred[1] - x_pred[2] - I_L * R0;

    /* 5. Jacobian C = [dVoc/dSoC, -1, -1] */
    float C[3];
    C[0] = bms_ekf_get_dvoc_dsoc(x_pred[0]);
    C[1] = -1.0f;
    C[2] = -1.0f;

    /* 6. Innovation Covariance S = C * P_pred * C^T + R */
    /* C * P_pred (1x3 vector) */
    float CP[3];
    for (int j = 0; j < 3; ++j) {
        CP[j] = C[0] * P_pred[0][j] + C[1] * P_pred[1][j] + C[2] * P_pred[2][j];
    }
    float S = CP[0] * C[0] + CP[1] * C[1] + CP[2] * C[2] + R_MEAS;

    /* 7. Kalman Gain: K = P_pred * C^T / S (3x1 vector) */
    float K[3];
    /* P_pred * C^T equals CP^T because P_pred is symmetric */
    K[0] = CP[0] / S;
    K[1] = CP[1] / S;
    K[2] = CP[2] / S;

    /* 8. State Update: x = x_pred + K * (Vt_meas - Vt_est) */
    float y_innov = Vt_meas - Vt_est;
    filter->x[0] = x_pred[0] + K[0] * y_innov;
    filter->x[1] = x_pred[1] + K[1] * y_innov;
    filter->x[2] = x_pred[2] + K[2] * y_innov;

    /* Clamping updated SoC */
    if (filter->x[0] < 0.0f) filter->x[0] = 0.0f;
    if (filter->x[0] > 1.0f) filter->x[0] = 1.0f;

    /* 9. Joseph Form Covariance Update: P = (I - KC) * P_pred * (I - KC)^T + K * R * K^T */
    float IKC[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float delta = (r == c) ? 1.0f : 0.0f;
            IKC[r][c] = delta - K[r] * C[c];
        }
    }

    /* Temp = IKC * P_pred */
    float temp[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            temp[r][c] = IKC[r][0] * P_pred[0][c] +
                         IKC[r][1] * P_pred[1][c] +
                         IKC[r][2] * P_pred[2][c];
        }
    }

    /* P = temp * IKC^T + K * R * K^T */
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = temp[r][0] * IKC[c][0] +
                        temp[r][1] * IKC[c][1] +
                        temp[r][2] * IKC[c][2];
            filter->P[r][c] = sum + K[r] * R_MEAS * K[c];
        }
    }

    /* Symmetrize */
    for (int r = 0; r < 3; ++r) {
        for (int c = r + 1; c < 3; ++c) {
            float avg = 0.5f * (filter->P[r][c] + filter->P[c][r]);
            filter->P[r][c] = avg;
            filter->P[c][r] = avg;
        }
    }

    /* Store filtered outputs */
    filter->soc_estimate = filter->x[0];
    filter->vt_estimate  = Vt_est;
    filter->sigma_bound  = 3.0f * sqrtf((filter->P[0][0] > 1e-9f) ? filter->P[0][0] : 1e-9f);
}
