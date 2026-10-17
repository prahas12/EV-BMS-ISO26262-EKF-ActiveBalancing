function [R0_est, SoH_R0_pct, theta_next, P_next] = rls_soh_estimator( ...
    I_L, Vt_meas, soc_est, v1_est, v2_est, theta_prev, P_prev, bms_params)
%#codegen
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Recursive Least Squares (RLS) Online Parameter Identification for SoH
% =========================================================================
% Tracks cell internal resistance R0 degradation online.
% End of Life (EOL) is reached when R0 increases by 100% (2.0x BOL value).
%
% Inputs:
%   I_L        : Load current [A]
%   Vt_meas    : Measured terminal voltage [V]
%   soc_est    : EKF-filtered SoC estimate [0 to 1]
%   v1_est     : RC1 polarization voltage [V]
%   v2_est     : RC2 polarization voltage [V]
%   theta_prev : Previous estimate of parameter [R0 in Ohms]
%   P_prev     : Previous covariance scalar
%   bms_params : BMS parameter structure
%
% Outputs:
%   R0_est      : Filtered online internal resistance [Ohms]
%   SoH_R0_pct  : State of Health percentage based on resistance [0 to 100%]
%   theta_next  : Updated parameter estimate
%   P_next      : Updated covariance scalar
% =========================================================================

lambda     = bms_params.rls.lambda;
R0_ref_bol = bms_params.cell.R0_ref;

% Only update RLS when sufficient current is flowing (avoid division by near-zero)
if abs(I_L) > 1.0
    % Compute estimated OCV at current SoC
    Voc_est = polyval(bms_params.ocv.poly_coeff, soc_est);

    % Observation: y = Voc - V1 - V2 - Vt = I_L * R0
    y_meas = Voc_est - v1_est - v2_est - Vt_meas;
    phi    = I_L; % Regressor

    % Prediction error
    error_innov = y_meas - phi * theta_prev;

    % Kalman / RLS gain
    K_rls = (P_prev * phi) / (lambda + phi * P_prev * phi);

    % Parameter update
    theta_raw = theta_prev + K_rls * error_innov;

    % Clamp parameter to physical bounds [0.5x R0_ref to 4.0x R0_ref]
    theta_next = max(0.5 * R0_ref_bol, min(4.0 * R0_ref_bol, theta_raw));

    % Covariance update
    P_next = (1.0 / lambda) * (P_prev - K_rls * phi * P_prev);

    % Upper bound P to prevent runaway during long low-excitation periods
    P_next = min(1e5, max(1e-4, P_next));
else
    % Zero/low current: retain previous estimates
    theta_next = theta_prev;
    P_next     = P_prev;
end

R0_est = theta_next;

% State of Health percentage calculation:
% SoH = 100% when R0 = R0_ref_bol
% SoH = 0%   when R0 = 2.0 * R0_ref_bol (EOL threshold)
eol_ratio = bms_params.rls.R0_eol_ratio;
delta_ratio = (R0_est - R0_ref_bol) / ((eol_ratio - 1.0) * R0_ref_bol);
SoH_R0_pct  = max(0.0, min(100.0, (1.0 - delta_ratio) * 100.0));

end
