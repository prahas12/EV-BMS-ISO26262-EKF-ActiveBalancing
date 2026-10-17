/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_ekf.h
 * Description: Production Discrete Extended Kalman Filter (EKF) interface.
 * ============================================================================
 */

#ifndef BMS_EKF_H
#define BMS_EKF_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes an EKF filter instance for a battery cell.
 * @param filter Pointer to bms_ekf_t instance
 * @param initial_soc Initial SoC estimate [0.0 to 1.0]
 */
void bms_ekf_init(bms_ekf_t *filter, float initial_soc);

/**
 * @brief Executes one discrete EKF prediction & correction cycle.
 * @param filter Pointer to bms_ekf_t instance
 * @param I_L Measured cell load current [A] (pos = discharge, neg = charge)
 * @param Vt_meas Measured terminal voltage [V]
 * @param T_degC Measured cell temperature [deg C]
 * @param dt Sample interval [s] (e.g. 0.1s for 10 Hz)
 */
void bms_ekf_step(bms_ekf_t *filter, float I_L, float Vt_meas, float T_degC, float dt);

/**
 * @brief Computes Open-Circuit Voltage Voc from SoC.
 */
float bms_ekf_get_ocv(float soc);

/**
 * @brief Computes analytic derivative dVoc/dSoC.
 */
float bms_ekf_get_dvoc_dsoc(float soc);

#ifdef __cplusplus
}
#endif

#endif /* BMS_EKF_H */
