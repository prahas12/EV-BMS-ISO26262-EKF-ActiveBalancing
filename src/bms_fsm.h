/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_fsm.h
 * Description: ISO 26262 ASIL-D Supervisory Safety State Machine interface.
 * ============================================================================
 */

#ifndef BMS_FSM_H
#define BMS_FSM_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bms_state_t          current_state;
    bms_fault_t          active_fault;
    bms_contactors_t     contactors;
    bms_current_limits_t limits;
    float                precharge_timer;
} bms_fsm_t;

/**
 * @brief Initializes the supervisory state machine.
 */
void bms_fsm_init(bms_fsm_t *fsm);

/**
 * @brief Executes one cyclic supervisory safety check and state update.
 * @param fsm Pointer to state machine context
 * @param meas Sensor telemetry inputs
 * @param vehicle_key_on Vehicle ignition / power request (1=ON, 0=OFF)
 * @param dt Task cycle interval [s] (e.g. 0.01 for 10 ms fast safety task)
 */
void bms_fsm_step(bms_fsm_t *fsm, const bms_measurements_t *meas, bool vehicle_key_on, float dt);

#ifdef __cplusplus
}
#endif

#endif /* BMS_FSM_H */
