function [soc_est, Vt_est, P_diag, x_next, P_next] = ekf_soc_estimator( ...
    I_L, Vt_meas, T_degC, dt, x_prev, P_prev, bms_params)
%#codegen
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Discrete Extended Kalman Filter (EKF) for 2nd-Order Thevenin ECM
% =========================================================================
% Inputs:
%   I_L        : Measured cell current [A] (positive = discharge, negative = charge)
%   Vt_meas    : Measured terminal voltage [V]
%   T_degC     : Measured cell temperature [deg C]
%   dt         : Sample step duration [s]
%   x_prev     : Previous state vector [3x1] = [SoC; V_RC1; V_RC2]
%   P_prev     : Previous error covariance matrix [3x3]
%   bms_params : Struct containing physical & covariance parameters
%
% Outputs:
%   soc_est    : Estimated State of Charge [0.0 to 1.0]
%   Vt_est     : Estimated terminal voltage [V]
%   P_diag     : Diagonal elements of updated covariance [3x1] (for 3-sigma bounds)
%   x_next     : Updated state vector [3x1]
%   P_next     : Updated covariance matrix [3x3] (Joseph stabilized form)
% =========================================================================

%% 1. Extract Cell Parameters & Temperature Derating
Q_n = bms_params.pack.Q_nominal_As;
eta = 1.0;
if I_L < 0.0
    eta = bms_params.cell.coulomb_eff; % Charging efficiency
end

% Arrhenius temperature scaling for internal ohmic resistance
T_K = T_degC + 273.15;
arrhenius_factor = exp((bms_params.cell.Ea_R0 / bms_params.cell.R_gas) * ...
                       (1.0 / T_K - 1.0 / bms_params.cell.T_ref));
R0 = bms_params.cell.R0_ref * arrhenius_factor;
R1 = bms_params.cell.R1_ref;
C1 = bms_params.cell.C1_ref;
R2 = bms_params.cell.R2_ref;
C2 = bms_params.cell.C2_ref;

% RC Time constants
tau1 = R1 * C1;
tau2 = R2 * C2;

alpha1 = exp(-dt / tau1);
alpha2 = exp(-dt / tau2);

%% 2. Discrete State Transition Matrices (Ad, Bd)
% x = [SoC; V1; V2]
Ad = [1.0,    0.0,    0.0;
      0.0, alpha1,    0.0;
      0.0,    0.0, alpha2];

Bd = [- (eta * dt) / Q_n;
      R1 * (1.0 - alpha1);
      R2 * (1.0 - alpha2)];

%% 3. Time Update (Prediction Step)
% State prediction
x_pred = Ad * x_prev + Bd * I_L;

% Bound predicted SoC within physical limits [0, 1]
x_pred(1) = max(0.0, min(1.0, x_pred(1)));

% Error covariance prediction
Q_cov  = bms_params.ekf.Q;
P_pred = Ad * P_prev * Ad' + Q_cov;

%% 4. Non-Linear Measurement Prediction
soc_p = x_pred(1);
v1_p  = x_pred(2);
v2_p  = x_pred(3);

% Evaluate Open Circuit Voltage OCV(SoC) using polyval
Voc_pred = polyval(bms_params.ocv.poly_coeff, soc_p);

% Entropic temperature voltage adjustment: dVoc/dT * (T - 25)
Voc_pred = Voc_pred + bms_params.thermal.dVoc_dT * (T_degC - 25.0);

% Terminal voltage estimate: Vt = Voc - V1 - V2 - I_L*R0
Vt_est = Voc_pred - v1_p - v2_p - I_L * R0;

%% 5. Measurement Jacobian (Linearization at x_pred)
% C = d(Vt)/dx = [dVoc/dSoC, -1, -1]
dVoc_dSoC = polyval(bms_params.ocv.dpoly_coeff, soc_p);

% Protect against gradient inversion near extremes
dVoc_dSoC = max(0.05, dVoc_dSoC);

C = [dVoc_dSoC, -1.0, -1.0];

%% 6. Measurement Update (Correction Step)
R_meas = bms_params.ekf.R;

% Innovation (Residual)
y_innov = Vt_meas - Vt_est;

% Innovation Covariance
S = C * P_pred * C' + R_meas;

% Kalman Gain: K = P_pred * C' / S
K = (P_pred * C') / S;

% Updated State Vector
x_next = x_pred + K * y_innov;

% Enforce physical bounds on updated SoC
x_next(1) = max(0.0, min(1.0, x_next(1)));

% Joseph Form Covariance Update: guarantees positive semi-definiteness
% P_next = (I - K*C) * P_pred * (I - K*C)' + K * R_meas * K'
I3 = eye(3);
IKC = I3 - K * C;
P_next = IKC * P_pred * IKC' + (K * R_meas) * K';

% Symmetrize covariance to prevent numerical asymmetry
P_next = 0.5 * (P_next + P_next');

%% 7. Outputs
soc_est = x_next(1);
P_diag  = [P_next(1,1); P_next(2,2); P_next(3,3)];

end
