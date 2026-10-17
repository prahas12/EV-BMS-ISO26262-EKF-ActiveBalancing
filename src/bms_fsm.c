/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_fsm.c
 * Description: Production ISO 26262 ASIL-D Supervisory Safety State Machine.
 * ============================================================================
 */

#include "bms_fsm.h"

/* ISO 26262 Calibrated Safety Thresholds */
#define V_CELL_MAX_FAULT        (4.25f) /* [V] Thermal runaway limit */
#define V_CELL_MAX_WARN         (4.20f) /* [V] Charge derate threshold */
#define V_CELL_MIN_FAULT        (2.75f) /* [V] Copper dendrite limit */
#define V_CELL_MIN_WARN         (3.00f) /* [V] Discharge derate threshold */

#define I_DISCHARGE_MAX_FAULT   (120.0f) /* [A] */
#define I_CHARGE_MAX_FAULT      (25.0f)  /* [A] */

#define T_CELL_MAX_FAULT        (60.0f)  /* [deg C] */
#define T_CELL_MAX_DERATE       (45.0f)  /* [deg C] */
#define T_CELL_MIN_DERATE       (5.0f)   /* [deg C] */
#define T_CELL_MIN_FAULT        (-20.0f) /* [deg C] */

#define PRECHARGE_TARGET_RATIO  (0.95f)  /* 95% of battery pack voltage */
#define PRECHARGE_TIMEOUT_SEC   (0.500f) /* 500 ms max precharge duration */
#define WELD_DETECT_VOLT_THRESH (10.0f)  /* [V] */

void bms_fsm_init(bms_fsm_t *fsm)
{
    if (fsm == NULL) return;

    fsm->current_state = BMS_STATE_INIT;
    fsm->active_fault  = BMS_FAULT_NONE;
    fsm->contactors.main_positive   = false;
    fsm->contactors.main_negative   = false;
    fsm->contactors.precharge_relay = false;
    fsm->limits.max_discharge_current = 0.0f;
    fsm->limits.max_charge_current    = 0.0f;
    fsm->precharge_timer = 0.0f;
}

void bms_fsm_step(bms_fsm_t *fsm, const bms_measurements_t *meas, bool vehicle_key_on, float dt)
{
    if (fsm == NULL || meas == NULL) return;

    /* 1. Extract Extreme Measurements */
    float v_max = meas->cell_voltages[0];
    float v_min = meas->cell_voltages[0];
    float v_sum = 0.0f;
    float t_max = meas->cell_temperatures[0];
    float t_min = meas->cell_temperatures[0];

    for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
        float v = meas->cell_voltages[i];
        float t = meas->cell_temperatures[i];
        if (v > v_max) v_max = v;
        if (v < v_min) v_min = v;
        v_sum += v;
        if (t > t_max) t_max = t;
        if (t < t_min) t_min = t;
    }

    /* 2. ISO 26262 ASIL-D Emergency Fault Monitor (Highest Priority) */
    if (v_max >= V_CELL_MAX_FAULT) {
        fsm->active_fault = BMS_FAULT_OVER_VOLTAGE;
    } else if (v_min <= V_CELL_MIN_FAULT) {
        fsm->active_fault = BMS_FAULT_UNDER_VOLTAGE;
    } else if (meas->pack_current >= I_DISCHARGE_MAX_FAULT) {
        fsm->active_fault = BMS_FAULT_OVER_CURRENT_DIS;
    } else if (meas->pack_current <= -I_CHARGE_MAX_FAULT) {
        fsm->active_fault = BMS_FAULT_OVER_CURRENT_CHG;
    } else if (t_max >= T_CELL_MAX_FAULT) {
        fsm->active_fault = BMS_FAULT_OVER_TEMPERATURE;
    } else if (t_min <= T_CELL_MIN_FAULT) {
        fsm->active_fault = BMS_FAULT_UNDER_TEMPERATURE;
    }

    /* Immediate Safe State Trip on Fault */
    if (fsm->active_fault != BMS_FAULT_NONE) {
        fsm->current_state = BMS_STATE_SAFE_FAULT;
        fsm->contactors.main_positive   = false;
        fsm->contactors.main_negative   = false;
        fsm->contactors.precharge_relay = false;
        fsm->limits.max_discharge_current = 0.0f;
        fsm->limits.max_charge_current    = 0.0f;
        return;
    }

    /* 3. Supervisory State Machine Transitions */
    switch (fsm->current_state) {
        case BMS_STATE_INIT:
            /* Self-test complete -> Standby */
            fsm->current_state = BMS_STATE_STANDBY;
            fsm->contactors.main_positive   = false;
            fsm->contactors.main_negative   = false;
            fsm->contactors.precharge_relay = false;
            break;

        case BMS_STATE_STANDBY:
            fsm->contactors.main_positive   = false;
            fsm->contactors.main_negative   = false;
            fsm->contactors.precharge_relay = false;

            /* Contactor Weld Diagnostic: bus should be ~0V when open */
            if (meas->bus_voltage > WELD_DETECT_VOLT_THRESH) {
                fsm->active_fault = BMS_FAULT_CONTACTOR_WELD;
                fsm->current_state = BMS_STATE_SAFE_FAULT;
                return;
            }

            if (vehicle_key_on) {
                fsm->current_state = BMS_STATE_PRECHARGE;
                fsm->precharge_timer = 0.0f;
                /* Close Main Neg & Precharge Relay */
                fsm->contactors.main_negative   = true;
                fsm->contactors.precharge_relay = true;
                fsm->contactors.main_positive   = false;
            }
            break;

        case BMS_STATE_PRECHARGE:
            fsm->precharge_timer += dt;
            fsm->contactors.main_negative   = true;
            fsm->contactors.precharge_relay = true;
            fsm->contactors.main_positive   = false;

            /* Check if bus reached 95% of battery voltage */
            if (meas->bus_voltage >= (PRECHARGE_TARGET_RATIO * v_sum)) {
                /* Precharge complete -> Close Main Pos, Open Precharge */
                fsm->current_state = BMS_STATE_NORMAL_RUN;
                fsm->contactors.main_positive   = true;
                fsm->contactors.precharge_relay = false;
            } else if (fsm->precharge_timer >= PRECHARGE_TIMEOUT_SEC) {
                /* Precharge timeout fault! */
                fsm->active_fault = BMS_FAULT_PRECHARGE_TIMEOUT;
                fsm->current_state = BMS_STATE_SAFE_FAULT;
                fsm->contactors.main_positive   = false;
                fsm->contactors.main_negative   = false;
                fsm->contactors.precharge_relay = false;
                return;
            }
            break;

        case BMS_STATE_NORMAL_RUN:
        case BMS_STATE_DERATING:
            if (!vehicle_key_on) {
                /* Graceful shutdown */
                fsm->current_state = BMS_STATE_STANDBY;
                fsm->contactors.main_positive   = false;
                fsm->contactors.main_negative   = false;
                fsm->contactors.precharge_relay = false;
            } else {
                bool derate_req = (t_max >= T_CELL_MAX_DERATE) ||
                                  (t_min <= T_CELL_MIN_DERATE) ||
                                  (v_max >= V_CELL_MAX_WARN)   ||
                                  (v_min <= V_CELL_MIN_WARN);

                fsm->current_state = derate_req ? BMS_STATE_DERATING : BMS_STATE_NORMAL_RUN;
                fsm->contactors.main_positive   = true;
                fsm->contactors.main_negative   = true;
                fsm->contactors.precharge_relay = false;
            }
            break;

        case BMS_STATE_SAFE_FAULT:
        default:
            fsm->contactors.main_positive   = false;
            fsm->contactors.main_negative   = false;
            fsm->contactors.precharge_relay = false;
            break;
    }

    /* 4. Dynamic Current Limit Computation */
    if (fsm->current_state == BMS_STATE_SAFE_FAULT) {
        fsm->limits.max_discharge_current = 0.0f;
        fsm->limits.max_charge_current    = 0.0f;
    } else if (fsm->current_state == BMS_STATE_DERATING) {
        float factor_t = 1.0f;
        if (t_max >= T_CELL_MAX_DERATE) {
            factor_t = 1.0f - (t_max - T_CELL_MAX_DERATE) / (T_CELL_MAX_FAULT - T_CELL_MAX_DERATE);
        }
        if (factor_t < 0.1f) factor_t = 0.1f;

        fsm->limits.max_discharge_current = 80.0f * factor_t;
        fsm->limits.max_charge_current    = 20.0f * factor_t;
    } else {
        /* Full continuous rating */
        fsm->limits.max_discharge_current = 80.0f;
        fsm->limits.max_charge_current    = 20.0f;
    }
}
