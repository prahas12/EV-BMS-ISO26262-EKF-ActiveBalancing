/**
 * ============================================================================
 * EV-BMS-ISO26262-EKF-ActiveBalancing
 * File: bms_main.c
 * Description: Standalone Embedded Application & Cyclic Executive Simulation.
 * ============================================================================
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "bms_types.h"
#include "bms_ekf.h"
#include "bms_fsm.h"
#include "bms_balancing.h"

int main(void)
{
    printf("BMS Embedded Controller Simulation\n\n");

    bms_fsm_t                  fsm;
    bms_balancing_controller_t balancer;
    bms_ekf_t                  cell_filters[BMS_NUM_CELLS_SERIES];
    bms_measurements_t         meas;

    bms_fsm_init(&fsm);
    bms_balancing_init(&balancer);

    float ground_truth_soc[BMS_NUM_CELLS_SERIES] = {
        0.82f, 0.85f, 0.88f, 0.84f, 0.86f, 0.81f, 0.89f,
        0.83f, 0.87f, 0.85f, 0.80f, 0.88f, 0.84f, 0.86f
    };

    for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
        bms_ekf_init(&cell_filters[i], 0.60f);
        meas.cell_temperatures[i] = 25.0f;
    }
    meas.pack_current = 0.0f;
    meas.bus_voltage  = 0.0f;

    const float dt_slow = 0.100f; /* 100 ms EKF estimation & balancing task rate */
    float sim_time = 0.0f;
    bool vehicle_key = true;      /* Driver turns ignition key ON */

    printf("\n[Execution Trace] Simulating Precharge & Normal Drive Loop:\n");
    printf(" Time [s] | State       | Pos | Neg | Pre | V_pack [V] | V_bus [V] | Cell 1 SoC | Spread [%%]\n");
    printf("----------------------------------------------------------------------------------------\n");

    /* Run 200 cycles of 100 ms (20 seconds of simulation) */
    for (int step = 0; step < 200; ++step) {
        sim_time += dt_slow;

        /* Simulated Load Current (WLTP-style dynamic pulse) */
        if (fsm.current_state == BMS_STATE_NORMAL_RUN || fsm.current_state == BMS_STATE_DERATING) {
            meas.pack_current = 10.0f * sinf(0.3f * sim_time) + 4.0f;
        } else {
            meas.pack_current = 0.0f;
        }

        /* Update plant ground truth & simulated sensor measurements */
        float v_pack_sum = 0.0f;
        for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
            float i_cell = meas.pack_current + balancer.balancing_currents[i];
            /* dSoC = - I / Q_n * dt */
            ground_truth_soc[i] -= (dt_slow / BMS_NOMINAL_CELL_CAP_AS) * i_cell;
            if (ground_truth_soc[i] < 0.0f) ground_truth_soc[i] = 0.0f;
            if (ground_truth_soc[i] > 1.0f) ground_truth_soc[i] = 1.0f;

            /* Cell terminal voltage */
            float ocv = bms_ekf_get_ocv(ground_truth_soc[i]);
            meas.cell_voltages[i] = ocv - i_cell * 0.015f;
            v_pack_sum += meas.cell_voltages[i];
        }

        /* Inverter DC Link Capacitor Precharge Dynamics */
        if (fsm.contactors.main_positive && fsm.contactors.main_negative) {
            meas.bus_voltage = v_pack_sum; /* Direct low-impedance connection */
        } else if (fsm.contactors.precharge_relay && fsm.contactors.main_negative) {
            /* Exponential precharge rise through 50 Ohm resistor into 1500 uF */
            float rc = 50.0f * 0.0015f; /* 75 ms time constant */
            meas.bus_voltage += (v_pack_sum - meas.bus_voltage) * (1.0f - expf(-dt_slow / rc));
        } else {
            meas.bus_voltage *= 0.95f; /* Bleeding down */
        }

        /* Fast Safety Task (FSM step) */
        bms_fsm_step(&fsm, &meas, vehicle_key, dt_slow);

        /* Slow Task: EKF Update for all cells */
        float est_socs[BMS_NUM_CELLS_SERIES];
        for (uint32_t i = 0; i < BMS_NUM_CELLS_SERIES; ++i) {
            float i_cell = meas.pack_current + balancer.balancing_currents[i];
            bms_ekf_step(&cell_filters[i], i_cell, meas.cell_voltages[i], meas.cell_temperatures[i], dt_slow);
            est_socs[i] = cell_filters[i].soc_estimate;
        }

        /* Slow Task: Active Balancing Controller */
        bms_balancing_step(&balancer, est_socs, meas.cell_voltages, fsm.current_state);

        /* Print telemetry at key intervals */
        if (step == 0 || step == 3 || step == 8 || (step % 20 == 0)) {
            const char *state_str = "UNKNOWN";
            switch (fsm.current_state) {
                case BMS_STATE_INIT:       state_str = "INIT      "; break;
                case BMS_STATE_STANDBY:    state_str = "STANDBY   "; break;
                case BMS_STATE_PRECHARGE:  state_str = "PRECHARGE "; break;
                case BMS_STATE_NORMAL_RUN: state_str = "NORMAL_RUN"; break;
                case BMS_STATE_DERATING:   state_str = "DERATING  "; break;
                case BMS_STATE_SAFE_FAULT: state_str = "SAFE_FAULT"; break;
            }

            printf("  %5.2f   | %s |  %d  |  %d  |  %d  |   %6.2f   |   %6.2f  |   %5.2f%%   |   %4.2f%%\n",
                   sim_time,
                   state_str,
                   fsm.contactors.main_positive ? 1 : 0,
                   fsm.contactors.main_negative ? 1 : 0,
                   fsm.contactors.precharge_relay ? 1 : 0,
                   v_pack_sum,
                   meas.bus_voltage,
                   cell_filters[0].soc_estimate * 100.0f,
                   balancer.delta_soc_spread * 100.0f);
        }
    }

    printf("----------------------------------------------------------------------------------------\n\n");
    printf("Simulation completed successfully.\n");
    printf("Cell 1 estimated SoC: %.2f%%\n", cell_filters[0].soc_estimate * 100.0f);
    printf("Balancing status: %s (Cell %u -> Cell %u)\n\n",
           balancer.is_balancing_active ? "ACTIVE" : "IDLE",
           balancer.donor_cell_idx + 1,
           balancer.receiver_cell_idx + 1);

    return 0;
}
