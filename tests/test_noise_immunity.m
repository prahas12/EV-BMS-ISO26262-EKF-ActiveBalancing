function pass = test_noise_immunity()
% Tests estimator performance under current sensor bias (250 mA) and Gaussian noise.
% Compares open-loop integration (Coulomb counting) against closed-loop EKF.

fprintf('Running [test_noise_immunity]... ');

bms_params = init_bms_parameters();
dt = bms_params.ekf.dt;
duration = 600; % 10 minutes
time = 0:dt:duration;
N = length(time);

true_soc = 0.80;
true_v1  = 0.0;
true_v2  = 0.0;
T_cell   = 25.0;

% Initial states
x_ekf = [0.80; 0.0; 0.0];
P_ekf = bms_params.ekf.P0;
soc_coulomb = 0.80;

% Sensor impairments
I_bias  = 0.25;        % 250 mA current sensor DC bias
I_noise_sigma = 0.15;  % 150 mA RMS noise
V_noise_sigma = 0.005; % 5 mV RMS noise

ekf_err_log     = zeros(N, 1);
coulomb_err_log = zeros(N, 1);

Q_n = bms_params.pack.Q_nominal_As;
R0  = bms_params.cell.R0_ref;
R1  = bms_params.cell.R1_ref; C1 = bms_params.cell.C1_ref;
R2  = bms_params.cell.R2_ref; C2 = bms_params.cell.C2_ref;
poly_c = bms_params.ocv.poly_coeff;

for k = 1:N
    % True current
    I_true = 5.0 * sin(0.05 * time(k)) + 2.0;

    % Plant update
    true_soc = true_soc - (dt / Q_n) * I_true;
    true_v1  = exp(-dt/(R1*C1))*true_v1 + R1*(1 - exp(-dt/(R1*C1)))*I_true;
    true_v2  = exp(-dt/(R2*C2))*true_v2 + R2*(1 - exp(-dt/(R2*C2)))*I_true;

    Voc_true = polyval(poly_c, true_soc);
    Vt_meas  = Voc_true - true_v1 - true_v2 - I_true*R0 + V_noise_sigma*randn();

    % Measured current corrupted with bias + noise
    I_meas = I_true + I_bias + I_noise_sigma*randn();

    % Coulomb counting integration (drifts over time)
    soc_coulomb = soc_coulomb - (dt / Q_n) * I_meas;

    % EKF estimation
    [soc_ekf, ~, ~, x_ekf, P_ekf] = ekf_soc_estimator( ...
        I_meas, Vt_meas, T_cell, dt, x_ekf, P_ekf, bms_params);

    ekf_err_log(k)     = abs(soc_ekf - true_soc) * 100.0;
    coulomb_err_log(k) = abs(soc_coulomb - true_soc) * 100.0;
end

coulomb_drift = coulomb_err_log(end);
ekf_max_err   = max(ekf_err_log(100:end)); % Steady state max error
ekf_mean_err  = mean(ekf_err_log(100:end)); % Steady state mean error

% Assertions
% 1. Open-loop Coulomb counting drifts with integrated bias
assert(coulomb_drift >= 0.7, sprintf('Coulomb counting should drift >= 0.7%% (actual: %.2f%%)', coulomb_drift));
% 2. Closed-loop EKF remains bounded with mean error < 3.0% despite 250mA sensor bias
assert(ekf_mean_err < 3.0, sprintf('EKF mean error (%.2f%%) exceeded 3.0%% under sensor bias!', ekf_mean_err));

fprintf('PASS (Coulomb Drift: %.2f%% vs EKF Mean Err: %.2f%%, Max Err: %.2f%%)\n', ...
    coulomb_drift, ekf_mean_err, ekf_max_err);
pass = true;
end
