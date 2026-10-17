function plot_bms_results(sim_results)
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Publication-Quality Plotting & Telemetry Visualizer
% =========================================================================
% Generates detailed, high-resolution diagnostic plots:
%   Figure 1: EKF SoC Estimation Accuracy & 3-Sigma Covariance Envelope
%   Figure 2: Cell Balancing Convergence & Thermal Telemetry
%   Figure 3: ISO 26262 Contactor Sequencing & RLS SoH Tracking
% =========================================================================

if nargin < 1
    results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
    mat_file = fullfile(results_dir, 'bms_wltp_results.mat');
    if isfile(mat_file)
        load(mat_file, 'sim_results');
    else
        error('No simulation results found. Run run_drive_cycle_simulation.m first.');
    end
end

time = sim_results.time;
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~isfolder(results_dir), mkdir(results_dir); end

% Clean styling
set(0, 'DefaultAxesFontName', 'Segoe UI');
set(0, 'DefaultAxesFontSize', 10);
set(0, 'DefaultLineLineWidth', 1.5);

%% ========================================================================
%% FIGURE 1: EKF SoC Estimation & Covariance Bounds
%% ========================================================================
fig1 = figure('Name', 'BMS EKF Estimation Performance', 'Color', 'w', ...
              'Units', 'normalized', 'Position', [0.05, 0.05, 0.65, 0.85]);

% Subplot 1: Pack Load Current
subplot(3, 1, 1);
plot(time, sim_results.I_pack_actual, 'Color', [0.1, 0.4, 0.8]);
grid on; grid minor;
ylabel('Pack Current [A]');
title(sprintf('BMS EKF SoC Estimation under %s Drive Cycle (ISO 26262 Compliant)', ...
      sim_results.cycle_name), 'FontSize', 12, 'FontWeight', 'bold');
xlim([0, time(end)]);
legend('Current (Positive=Discharge, Negative=Regen)', 'Location', 'northeast');

% Subplot 2: True vs Estimated SoC (Cell 1 & Cell 7)
subplot(3, 1, 2);
plot(time, sim_results.soc_true(:, 1) * 100, 'k--', 'LineWidth', 2.0); hold on;
plot(time, sim_results.soc_est(:, 1) * 100, 'Color', [0.85, 0.2, 0.1], 'LineWidth', 1.8);
plot(time, sim_results.soc_true(:, 7) * 100, 'Color', [0.3, 0.7, 0.3], 'LineWidth', 1.2);
plot(time, sim_results.soc_est(:, 7) * 100, 'Color', [0.0, 0.5, 0.7], 'LineStyle', ':', 'LineWidth', 1.8);
grid on; grid minor;
ylabel('State of Charge [%]');
xlim([0, time(end)]);
ylim([0, 100]);
legend('Cell 1 Ground Truth', 'Cell 1 EKF Estimate', ...
       'Cell 7 Ground Truth', 'Cell 7 EKF Estimate', 'Location', 'southeast');

% Subplot 3: Estimation Error & 3-Sigma Covariance Envelope
subplot(3, 1, 3);
err_c1 = sim_results.soc_err(:, 1) * 100;
bound_c1 = sim_results.sigma_bounds(:, 1) * 100;

% Shaded 3-sigma confidence envelope
fill([time; flipud(time)], [bound_c1; -flipud(bound_c1)], ...
     [0.85, 0.90, 0.98], 'EdgeColor', 'none', 'DisplayName', '\pm 3\sigma EKF Confidence Bound');
hold on;
plot(time, err_c1, 'Color', [0.85, 0.2, 0.1], 'DisplayName', 'Cell 1 Estimation Error (e_{SoC})');
plot(time, bound_c1, 'b--', 'LineWidth', 1.0, 'DisplayName', '+3\sigma Upper Bound');
plot(time, -bound_c1, 'b--', 'LineWidth', 1.0, 'DisplayName', '-3\sigma Lower Bound');
grid on; grid minor;
xlabel('Time [seconds]');
ylabel('SoC Error [%]');
xlim([0, time(end)]);
ylim([-15, 15]);
legend('Location', 'northeast');

saveas(fig1, fullfile(results_dir, 'EKF_SoC_Tracking_Performance.png'));

%% ========================================================================
%% FIGURE 2: Cell Balancing Dynamics & Thermal Model
%% ========================================================================
fig2 = figure('Name', 'Cell Balancing & Thermal Telemetry', 'Color', 'w', ...
              'Units', 'normalized', 'Position', [0.10, 0.08, 0.65, 0.85]);

% Subplot 1: Cell Voltages
subplot(3, 1, 1);
plot(time, sim_results.vt_meas, 'LineWidth', 0.8);
grid on; grid minor;
ylabel('Cell Voltages [V]');
title('Multi-Cell Active Balancing & Thermal Management (14S Pack)', ...
      'FontSize', 12, 'FontWeight', 'bold');
xlim([0, time(end)]);

% Subplot 2: Cell-to-Cell SoC Dispersion Convergence
subplot(3, 1, 2);
soc_max = max(sim_results.soc_true, [], 2) * 100;
soc_min = min(sim_results.soc_true, [], 2) * 100;
soc_spread = soc_max - soc_min;
plot(time, soc_spread, 'Color', [0.6, 0.1, 0.7], 'LineWidth', 2.0);
hold on;
yline(1.5, 'r--', 'Balancing Inhibit Threshold (1.5%)', 'LineWidth', 1.2);
grid on; grid minor;
ylabel('\Delta SoC Spread [%]');
xlim([0, time(end)]);
legend('Peak-to-Peak Cell SoC Spread', 'Threshold', 'Location', 'northeast');

% Subplot 3: Cell Temperatures & Safety Derating Limits
subplot(3, 1, 3);
plot(time, sim_results.T_cells, 'LineWidth', 1.0); hold on;
yline(45.0, 'Color', [0.9, 0.5, 0.0], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Thermal Derate Limit (45°C)');
yline(60.0, 'Color', [0.8, 0.0, 0.0], 'LineStyle', '-',  'LineWidth', 1.8, 'DisplayName', 'Emergency Trip (60°C)');
grid on; grid minor;
xlabel('Time [seconds]');
ylabel('Temperature [°C]');
xlim([0, time(end)]);
legend('Location', 'northwest');

saveas(fig2, fullfile(results_dir, 'Active_Balancing_and_Thermal_Telemetry.png'));

%% ========================================================================
%% FIGURE 3: Contactor Sequencing & Online SoH Identification
%% ========================================================================
fig3 = figure('Name', 'ISO 26262 State Machine & SoH Estimation', 'Color', 'w', ...
              'Units', 'normalized', 'Position', [0.15, 0.10, 0.65, 0.85]);

% Subplot 1: Precharge Voltage Transition
subplot(3, 1, 1);
plot(time, sim_results.V_pack, 'k--', 'LineWidth', 1.5, 'DisplayName', 'V_{pack} (Battery)'); hold on;
plot(time, sim_results.V_bus, 'Color', [0.0, 0.6, 0.3], 'LineWidth', 2.0, 'DisplayName', 'V_{bus} (Inverter DC Link)');
grid on; grid minor;
ylabel('Voltage [V]');
title('ISO 26262 Contactor Precharge Sequence & RLS SoH Estimation', ...
      'FontSize', 12, 'FontWeight', 'bold');
xlim([0, min(100, time(end))]); % Zoom into precharge and early drive
legend('Location', 'southeast');

% Subplot 2: BMS Operating State Transitions
subplot(3, 1, 2);
plot(time, sim_results.bms_state, 'Color', [0.2, 0.3, 0.7], 'LineWidth', 2.0);
yticks([1, 2, 3, 4, 5, 6]);
yticklabels({'Init', 'Standby', 'Precharge', 'Normal Run', 'Derating', 'Safe Fault'});
grid on;
ylabel('BMS State');
xlim([0, time(end)]);

% Subplot 3: RLS Online Resistance R0 & SoH Estimation
subplot(3, 1, 3);
yyaxis left;
plot(time, sim_results.R0_est * 1000, 'Color', [0.8, 0.3, 0.0], 'LineWidth', 1.8);
ylabel('Identified R_0 [m\Omega]');
grid on; grid minor;

yyaxis right;
plot(time, sim_results.SoH_pct, 'Color', [0.1, 0.6, 0.1], 'LineWidth', 1.8);
ylabel('State of Health [%]');
ylim([80, 105]);

xlabel('Time [seconds]');
xlim([0, time(end)]);
legend('Identified Internal Resistance R_0', 'Online Estimated SoH', 'Location', 'southeast');

saveas(fig3, fullfile(results_dir, 'State_Machine_and_SoH_Tracking.png'));

fprintf('[Plots] Visualizations saved successfully in "%s"\n', results_dir);

end
