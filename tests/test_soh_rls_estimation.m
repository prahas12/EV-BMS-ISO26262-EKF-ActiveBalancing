function pass = test_soh_rls_estimation()
% Verifies online parameter estimation of degraded internal resistance (R0) via RLS.

fprintf('Running [test_soh_rls_estimation]... ');

bms_params = init_bms_parameters();
dt = bms_params.ekf.dt;
duration = 300;
time = 0:dt:duration;
N = length(time);

% Degraded cell parameters
R0_aged = 1.60 * bms_params.cell.R0_ref; % 24 mOhm vs 15 mOhm fresh
true_soc = 0.75;
true_v1  = 0.0;
true_v2  = 0.0;

% Initial RLS states (fresh guess)
theta_rls = bms_params.cell.R0_ref;
P_rls     = bms_params.rls.P0;

Q_n = bms_params.pack.Q_nominal_As;
R1  = bms_params.cell.R1_ref; C1 = bms_params.cell.C1_ref;
R2  = bms_params.cell.R2_ref; C2 = bms_params.cell.C2_ref;
poly_c = bms_params.ocv.poly_coeff;

for k = 1:N
    I_L = 10.0 * sin(0.2 * time(k)) + 4.0 * cos(0.05 * time(k));

    % Plant step
    true_soc = true_soc - (dt / Q_n) * I_L;
    true_v1  = exp(-dt/(R1*C1))*true_v1 + R1*(1 - exp(-dt/(R1*C1)))*I_L;
    true_v2  = exp(-dt/(R2*C2))*true_v2 + R2*(1 - exp(-dt/(R2*C2)))*I_L;

    Voc_true = polyval(poly_c, true_soc);
    Vt_meas  = Voc_true - true_v1 - true_v2 - I_L*R0_aged + 0.002*randn();

    % RLS Step
    [R0_est, SoH_pct, theta_rls, P_rls] = rls_soh_estimator( ...
        I_L, Vt_meas, true_soc, true_v1, true_v2, theta_rls, P_rls, bms_params);
end

rel_err_pct = abs(R0_est - R0_aged) / R0_aged * 100.0;

% Assertions
assert(rel_err_pct < 6.0, sprintf('RLS failed to identify R0 accurately! Rel Err: %.2f%%', rel_err_pct));
assert(SoH_pct < 55.0 && SoH_pct > 30.0, sprintf('SoH estimate (%.1f%%) out of expected range!', SoH_pct));

fprintf('PASS (True R0: %.4f Ohm, Est R0: %.4f Ohm, Rel Err: %.2f%%, SoH: %.1f%%)\n', ...
    R0_aged, R0_est, rel_err_pct, SoH_pct);
pass = true;
end
