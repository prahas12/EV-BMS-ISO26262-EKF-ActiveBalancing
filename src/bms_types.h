/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_types.h
 * Description: Data types, enumerations, and safety constants for BMS firmware.
 * Compliance: MISRA-C:2012, ISO 26262 ASIL-D
 * ============================================================================
 */

#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pack Configuration */
#define BMS_NUM_CELLS_SERIES        (14U)
#define BMS_NOMINAL_CELL_CAP_AH     (5.0f)
#define BMS_NOMINAL_CELL_CAP_AS     (18000.0f)  /* 5.0 Ah * 3600 */
#define BMS_NOMINAL_CELL_VOLT       (3.70f)
#define BMS_NOMINAL_PACK_VOLT       (51.80f)

/* ISO 26262 ASIL-D Supervisory States */
typedef enum {
    BMS_STATE_INIT          = 1U,
    BMS_STATE_STANDBY       = 2U,
    BMS_STATE_PRECHARGE     = 3U,
    BMS_STATE_NORMAL_RUN    = 4U,
    BMS_STATE_DERATING      = 5U,
    BMS_STATE_SAFE_FAULT    = 6U
} bms_state_t;

/* ISO 26262 Fault Identifiers */
typedef enum {
    BMS_FAULT_NONE              = 0U,
    BMS_FAULT_OVER_VOLTAGE      = 1U,   /* SG01: Cell V > 4.25 V */
    BMS_FAULT_UNDER_VOLTAGE     = 2U,   /* SG02: Cell V < 2.75 V */
    BMS_FAULT_OVER_CURRENT_DIS  = 3U,   /* SG03: Discharge I > 120 A */
    BMS_FAULT_OVER_CURRENT_CHG  = 4U,   /* SG03: Charge I < -25 A */
    BMS_FAULT_OVER_TEMPERATURE  = 5U,   /* SG04: Cell T > 60 deg C */
    BMS_FAULT_UNDER_TEMPERATURE = 6U,   /* SG04: Cell T < -20 deg C */
    BMS_FAULT_PRECHARGE_TIMEOUT = 7U,   /* SG05: Precharge > 500 ms without Vbus reach */
    BMS_FAULT_CONTACTOR_WELD    = 8U    /* SG05: Bus V > 10 V when commanded open */
} bms_fault_t;

/* High-Voltage Contactor Commanded States */
typedef struct {
    bool main_positive;     /* 1 = CLOSED, 0 = OPEN */
    bool main_negative;     /* 1 = CLOSED, 0 = OPEN */
    bool precharge_relay;   /* 1 = CLOSED, 0 = OPEN */
} bms_contactors_t;

/* Dynamic Current Limits [A] */
typedef struct {
    float max_discharge_current;
    float max_charge_current;
} bms_current_limits_t;

/* EKF Internal State & Covariance (Per Cell) */
typedef struct {
    float x[3];             /* State: [SoC, V_RC1, V_RC2] */
    float P[3][3];          /* Error Covariance Matrix */
    float soc_estimate;     /* Filtered SoC [0.0 to 1.0] */
    float vt_estimate;      /* Predicted terminal voltage [V] */
    float sigma_bound;      /* 3-sigma confidence interval */
} bms_ekf_t;

/* Cell Analog Measurements */
typedef struct {
    float cell_voltages[BMS_NUM_CELLS_SERIES];      /* [V] */
    float cell_temperatures[BMS_NUM_CELLS_SERIES];  /* [deg C] */
    float pack_current;                             /* [A] (+ = discharge, - = charge) */
    float bus_voltage;                              /* [V] Inverter DC link */
} bms_measurements_t;

#ifdef __cplusplus
}
#endif

#endif /* BMS_TYPES_H */
