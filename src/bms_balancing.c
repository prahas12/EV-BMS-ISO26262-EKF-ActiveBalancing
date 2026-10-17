/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_balancing.c
 * Description: Production C implementation of active charge shuttling balancing.
 * ============================================================================
 */

#include "bms_balancing.h"

#define BAL_SOC_DELTA_THRESH   (0.015f) /* Activate if delta SoC > 1.5% */
#define BAL_MIN_SOC_ENABLE     (0.20f)  /* Enable only above 20% average SoC */
#define BAL_MAX_SOC_ENABLE     (0.98f)  /* Inhibit above 98% */
#define I_ACTIVE_MAX_A         (1.50f)  /* 1.5 A active charge shuttling */
#define ACTIVE_DC_EFFICIENCY   (0.88f)  /* 88% bidirectional converter efficiency */

void bms_balancing_init(bms_balancing_controller_t *ctrl)
{
    if (ctrl == NULL) return;

    for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
        ctrl->balancing_currents[i] = 0.0f;
    }
    ctrl->is_balancing_active = false;
    ctrl->donor_cell_idx     = 0;
    ctrl->receiver_cell_idx  = 0;
    ctrl->delta_soc_spread   = 0.0f;
}

void bms_balancing_step(bms_balancing_controller_t *ctrl,
                        const float soc_cells[BMS_NUM_CELLS_SERIES],
                        const float v_cells[BMS_NUM_CELLS_SERIES],
                        bms_state_t current_bms_state)
{
    (void)v_cells; /* Reserved for voltage-based hysteresis */
    if (ctrl == NULL || soc_cells == NULL) return;

    /* Reset currents */
    for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
        ctrl->balancing_currents[i] = 0.0f;
    }
    ctrl->is_balancing_active = false;

    /* Safety inhibitors: do not balance in Init, Precharge, or Fault */
    if (current_bms_state == BMS_STATE_SAFE_FAULT ||
        current_bms_state == BMS_STATE_INIT       ||
        current_bms_state == BMS_STATE_PRECHARGE) {
        return;
    }

    /* Find extremes */
    float soc_max = soc_cells[0];
    float soc_min = soc_cells[0];
    uint32_t idx_max = 0;
    uint32_t idx_min = 0;
    float sum_soc = 0.0f;

    for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
        float s = soc_cells[i];
        sum_soc += s;
        if (s > soc_max) {
            soc_max = s;
            idx_max = i;
        }
        if (s < soc_min) {
            soc_min = s;
            idx_min = i;
        }
    }

    float mean_soc = sum_soc / (float)BMS_NUM_CELLS_SERIES;
    ctrl->delta_soc_spread  = soc_max - soc_min;
    ctrl->donor_cell_idx    = idx_max;
    ctrl->receiver_cell_idx = idx_min;

    if (mean_soc < BAL_MIN_SOC_ENABLE || mean_soc > BAL_MAX_SOC_ENABLE) {
        return; /* Outside allowable operating window */
    }

    if (ctrl->delta_soc_spread > BAL_SOC_DELTA_THRESH) {
        ctrl->is_balancing_active = true;

        /* Active Bidirectional DC-DC Balancing */
        /* Donor cell discharges with I_cmd */
        ctrl->balancing_currents[idx_max] = I_ACTIVE_MAX_A;

        /* Receiver cell receives I_cmd * efficiency */
        ctrl->balancing_currents[idx_min] = - (I_ACTIVE_MAX_A * ACTIVE_DC_EFFICIENCY);
    }
}
