function sim_results = run_drive_cycle_simulation(cycle_name, duration_sec, initial_soc_guess)
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Closed-Loop Drive Cycle Simulation & Telemetry Logger
% =========================================================================
% Runs an end-to-end simulation of the 14S Li-ion battery pack with:
%   - Plant Model (2nd-Order ECM, thermal dynamics, sensor noise)
%   - BMS Supervisory State Machine (ISO 26262 precharge & safety)
%   - Discrete Extended Kalman Filter (EKF) SoC estimation
%   - Recursive Least Squares (RLS) SoH estimation
%   - Active / Passive Cell Balancing
% =========================================================================

if nargin < 1, cycle_name = 'WLTP'; end
if nargin < 2, duration_sec = 1200; end % 20 minutes default
if nargin < 3, initial_soc_guess = 0.50; end

fprintf('[BMS Simulation] Running cycle: %s (%d s, initial SoC guess: %.1f%%)\n', ...
    cycle_name, duration_sec, initial_soc_guess * 100);

% 1. Load Parameters & Drive Cycle
bms_params = init_bms_parameters();
dt = bms_params.ekf.dt;
cycle = generate_drive_cycles(cycle_name, duration_sec, dt);
time = cycle.time;
N_steps = length(time);
N_cells = bms_params.pack.N_series;

% 2. Initialize Plant State
plant_state.soc_true = bms_params.dispersion.SoC_spread(:);
plant_state.v1_true  = zeros(N_cells, 1);
plant_state.v2_true  = zeros(N_cells, 1);
plant_state.T_true   = bms_params.thermal.T_ambient * ones(N_cells, 1);
plant_state.V_bus    = 0.0; % Start discharged

% 3. Initialize Controller & Estimator States
% EKF states for all 14 cells: [SoC; V1; V2]
x_ekf = repmat([initial_soc_guess; 0.0; 0.0], 1, N_cells);
P_ekf = repmat(bms_params.ekf.P0, [1, 1, N_cells]);

% RLS states for pack-level SoH
theta_rls = bms_params.rls.theta0;
P_rls     = bms_params.rls.P0;

% State machine initial state
bms_state       = 1; % STATE_INIT
fault_code      = 0; % FAULT_NONE
contactors      = [0, 0, 0];
precharge_timer = 0.0;
vehicle_req     = 1; % Vehicle requested ON

% Balancing states
I_bal = zeros(N_cells, 1);

% 4. Preallocate Telemetry Logs
log_time           = time;
log_soc_true       = zeros(N_steps, N_cells);
log_soc_est        = zeros(N_steps, N_cells);
log_soc_err        = zeros(N_steps, N_cells);
log_sigma_bounds   = zeros(N_steps, N_cells);
log_vt_true        = zeros(N_steps, N_cells);
log_vt_meas        = zeros(N_steps, N_cells);
log_T_cells        = zeros(N_steps, N_cells);
log_I_bal          = zeros(N_steps, N_cells);
log_I_pack_actual  = zeros(N_steps, 1);
log_I_pack_meas    = zeros(N_steps, 1);
log_V_pack         = zeros(N_steps, 1);
log_V_bus          = zeros(N_steps, 1);
log_bms_state      = zeros(N_steps, 1);
log_fault_code     = zeros(N_steps, 1);
log_contactors     = zeros(N_steps, 3);
log_I_dis_limit    = zeros(N_steps, 1);
log_R0_est         = zeros(N_steps, 1);
log_SoH_pct        = zeros(N_steps, 1);

fprintf(' Progress: ');
print_interval = floor(N_steps / 10);

%% 5. Master Closed-Loop Execution
for k = 1:N_steps
    t = time(k);
    if mod(k, print_interval) == 0
        fprintf('%d%%.. ', round(k / N_steps * 100));
    end

    % A. Desired Load Current from Drive Cycle
    I_cmd = cycle.current(k);

    % Apply dynamic derating limits from supervisory state machine
    if k > 1
        I_cmd = min(I_dis_limit, max(-I_chg_limit, I_cmd));
    end

    % B. Plant Simulator Step
    [plant_state, telemetry] = battery_pack_plant_simulator( ...
        I_cmd, I_bal, contactors, plant_state, dt, bms_params);

    % Extract measurements available to BMS Analog Front-End (AFE)
    vt_meas = telemetry.vt_meas;
    I_meas  = telemetry.I_pack_meas;
    T_meas  = telemetry.T_meas;
    V_bus   = telemetry.V_bus;

    % C. Supervisory State Machine Step (ISO 26262 ASIL-D)
    if bms_state == 3 % STATE_PRECHARGE
        precharge_timer = precharge_timer + dt;
    else
        precharge_timer = 0.0;
    end

    [bms_state, fault_code, contactors, I_dis_limit, I_chg_limit] = ...
        bms_supervisory_state_machine( ...
            vt_meas, I_meas, T_meas, V_bus, vehicle_req, ...
            bms_state, fault_code, precharge_timer, dt, bms_params);

    % D. EKF SoC Estimation Step (for all 14 cells)
    soc_estimates = zeros(N_cells, 1);
    sigma_bounds  = zeros(N_cells, 1);

    for c = 1:N_cells
        % Account for balancing current in individual cell current input
        I_cell_meas = I_meas + I_bal(c);
        [soc_c, ~, P_diag_c, x_next_c, P_next_c] = ekf_soc_estimator( ...
            I_cell_meas, vt_meas(c), T_meas(c), dt, ...
            x_ekf(:, c), P_ekf(:, :, c), bms_params);

        x_ekf(:, c)      = x_next_c;
        P_ekf(:, :, c)   = P_next_c;
        soc_estimates(c) = soc_c;
        sigma_bounds(c)  = 3.0 * sqrt(max(1e-9, P_diag_c(1))); % 3-sigma bound
    end

    % E. RLS SoH Estimation Step (Representative Cell 1)
    [R0_est, SoH_pct, theta_rls, P_rls] = rls_soh_estimator( ...
        I_meas, vt_meas(1), soc_estimates(1), x_ekf(2, 1), x_ekf(3, 1), ...
        theta_rls, P_rls, bms_params);

    % F. Active / Passive Cell Balancing Controller Step
    [I_bal, ~, ~, ~, ~] = bms_active_balancing( ...
        soc_estimates, vt_meas, bms_state, bms_params);

    % G. Telemetry Logging
    log_soc_true(k, :)      = plant_state.soc_true';
    log_soc_est(k, :)       = soc_estimates';
    log_soc_err(k, :)       = soc_estimates' - plant_state.soc_true';
    log_sigma_bounds(k, :)  = sigma_bounds';
    log_vt_true(k, :)       = telemetry.vt_true';
    log_vt_meas(k, :)       = telemetry.vt_meas';
    log_T_cells(k, :)       = plant_state.T_true';
    log_I_bal(k, :)         = I_bal';
    log_I_pack_actual(k)    = telemetry.I_pack_actual;
    log_I_pack_meas(k)      = telemetry.I_pack_meas;
    log_V_pack(k)           = telemetry.V_pack_true;
    log_V_bus(k)            = telemetry.V_bus;
    log_bms_state(k)        = bms_state;
    log_fault_code(k)       = fault_code;
    log_contactors(k, :)    = contactors;
    log_I_dis_limit(k)      = I_dis_limit;
    log_R0_est(k)           = R0_est;
    log_SoH_pct(k)          = SoH_pct;
end
fprintf('Done!\n');

%% 6. Compute Validation Metrics
% Ignore initial transient settling period (first 100 seconds) for steady-state RMSE
idx_ss = time >= 100.0;
rmse_per_cell = sqrt(mean(log_soc_err(idx_ss, :).^2, 1)) * 100.0; % [%]
max_err_ss    = max(abs(log_soc_err(idx_ss, :)), [], 'all') * 100.0; % [%]
overall_rmse  = mean(rmse_per_cell);

% Settling time: time for estimation error to fall below 2%
err_cell1 = abs(log_soc_err(:, 1)) * 100.0;
idx_settled = find(err_cell1 <= 2.0, 1, 'first');
if ~isempty(idx_settled)
    settling_time_sec = time(idx_settled);
else
    settling_time_sec = NaN;
end

% Initial vs Final Cell SoC spread (Balancing efficacy)
initial_spread_pct = (max(log_soc_true(1, :)) - min(log_soc_true(1, :))) * 100.0;
final_spread_pct   = (max(log_soc_true(end, :)) - min(log_soc_true(end, :))) * 100.0;

fprintf('\n[BMS Metrics - %s]\n', cycle_name);
fprintf('  Initial SoC True Avg : %.2f%% | Initial Guess: %.2f%%\n', mean(log_soc_true(1, :)) * 100, initial_soc_guess * 100);
fprintf('  EKF Settling Time    : %.1f s (<2%% error)\n', settling_time_sec);
fprintf('  Steady-State RMSE    : %.3f%% (Max error: %.3f%%)\n', overall_rmse, max_err_ss);
fprintf('  Cell SoC Spread      : %.2f%% -> %.2f%% (Delta: %.1f%%)\n', ...
    initial_spread_pct, final_spread_pct, (initial_spread_pct - final_spread_pct) / initial_spread_pct * 100);
fprintf('  Estimated R0 / SoH   : %.4f Ohm | %.1f%%\n', log_R0_est(end), log_SoH_pct(end));

% Package Results
sim_results.time            = log_time;
sim_results.soc_true        = log_soc_true;
sim_results.soc_est         = log_soc_est;
sim_results.soc_err         = log_soc_err;
sim_results.sigma_bounds    = log_sigma_bounds;
sim_results.vt_true         = log_vt_true;
sim_results.vt_meas         = log_vt_meas;
sim_results.T_cells         = log_T_cells;
sim_results.I_bal           = log_I_bal;
sim_results.I_pack_actual   = log_I_pack_actual;
sim_results.I_pack_meas     = log_I_pack_meas;
sim_results.V_pack          = log_V_pack;
sim_results.V_bus           = log_V_bus;
sim_results.bms_state       = log_bms_state;
sim_results.fault_code      = log_fault_code;
sim_results.contactors      = log_contactors;
sim_results.I_dis_limit     = log_I_dis_limit;
sim_results.R0_est          = log_R0_est;
sim_results.SoH_pct         = log_SoH_pct;
sim_results.metrics.overall_rmse      = overall_rmse;
sim_results.metrics.max_err_ss        = max_err_ss;
sim_results.metrics.settling_time_sec = settling_time_sec;
sim_results.metrics.initial_spread    = initial_spread_pct;
sim_results.metrics.final_spread      = final_spread_pct;
sim_results.cycle_name                = cycle_name;

% Save to results folder
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~isfolder(results_dir), mkdir(results_dir); end
save(fullfile(results_dir, sprintf('bms_%s_results.mat', lower(cycle_name))), 'sim_results');

end
