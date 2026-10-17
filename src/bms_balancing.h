/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_balancing.h
 * Description: Active / Passive Cell Balancing Controller interface in C.
 * ============================================================================
 */

#ifndef BMS_BALANCING_H
#define BMS_BALANCING_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    balancing_currents[BMS_NUM_CELLS_SERIES]; /* [A] (+ = discharge/donate, - = receive) */
    bool     is_balancing_active;
    uint32_t donor_cell_idx;     /* Cell with highest SoC */
    uint32_t receiver_cell_idx;  /* Cell with lowest SoC */
    float    delta_soc_spread;   /* Peak-to-peak spread [0 to 1] */
} bms_balancing_controller_t;

/**
 * @brief Initializes balancing controller.
 */
void bms_balancing_init(bms_balancing_controller_t *ctrl);

/**
 * @brief Computes active/passive balancing commands.
 * @param ctrl Pointer to controller context
 * @param soc_cells Array of estimated SoCs for all 14 cells
 * @param v_cells Array of measured cell voltages
 * @param current_bms_state Operating mode of supervisory state machine
 */
void bms_balancing_step(bms_balancing_controller_t *ctrl,
                        const float soc_cells[BMS_NUM_CELLS_SERIES],
                        const float v_cells[BMS_NUM_CELLS_SERIES],
                        bms_state_t current_bms_state);

#ifdef __cplusplus
}
#endif

#endif /* BMS_BALANCING_H */
