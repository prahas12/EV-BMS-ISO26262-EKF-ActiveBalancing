function pass = test_ekf_convergence()
% Validates EKF convergence from an initial SoC estimation error (55% estimate vs 85% true).
% Verifies settling within 2% error band.

fprintf('Running [test_ekf_convergence]... ');

bms_params = init_bms_parameters();
dt = bms_params.ekf.dt;
duration = 240; % 4 minutes
time = 0:dt:duration;
N = length(time);

% Plant ground truth
true_soc = 0.85;
true_v1  = 0.0;
true_v2  = 0.0;
T_cell   = 25.0;

% Initial estimator state (30% offset)
x_ekf = [0.55; 0.0; 0.0];
P_ekf = bms_params.ekf.P0;

soc_err_log = zeros(N, 1);
settled_idx = [];

for k = 1:N
    % Dynamic current excitation
    I_L = 8.0 * sin(0.1 * time(k)) + 3.0;

    % Plant step
    Q_n = bms_params.pack.Q_nominal_As;
    R0  = bms_params.cell.R0_ref;
    R1  = bms_params.cell.R1_ref; C1 = bms_params.cell.C1_ref;
    R2  = bms_params.cell.R2_ref; C2 = bms_params.cell.C2_ref;

    true_soc = true_soc - (dt / Q_n) * I_L;
    true_v1  = exp(-dt/(R1*C1))*true_v1 + R1*(1 - exp(-dt/(R1*C1)))*I_L;
    true_v2  = exp(-dt/(R2*C2))*true_v2 + R2*(1 - exp(-dt/(R2*C2)))*I_L;

    poly_c = bms_params.ocv.poly_coeff;
    Voc_true = polyval(poly_c, true_soc);
    Vt_meas  = Voc_true - true_v1 - true_v2 - I_L*R0 + 0.003*randn();

    % EKF step
    [soc_est, ~, ~, x_ekf, P_ekf] = ekf_soc_estimator( ...
        I_L, Vt_meas, T_cell, dt, x_ekf, P_ekf, bms_params);

    err_pct = abs(soc_est - true_soc) * 100.0;
    soc_err_log(k) = err_pct;

    if isempty(settled_idx) && (err_pct < 2.0)
        settled_idx = k;
    end
end

settling_time = time(settled_idx);
final_err     = soc_err_log(end);

% Assertions
assert(~isempty(settled_idx), 'EKF failed to converge within test duration!');
assert(settling_time < 180.0, sprintf('Settling time (%.1fs) exceeded limit (180s)!', settling_time));
assert(final_err < 1.5, sprintf('Final error (%.2f%%) exceeded 1.5%% limit!', final_err));

fprintf('PASS (Settled in %.1f s, Final Error: %.3f%%)\n', settling_time, final_err);
pass = true;
end
