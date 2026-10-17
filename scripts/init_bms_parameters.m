function bms = init_bms_parameters()
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Parameter Initialization Script
% =========================================================================
% Defines all physical, electrical, electrochemical, and algorithmic
% parameters for a 14S Li-ion (NMC 21700) battery pack and its 
% ISO 26262-compliant Battery Management System.
% =========================================================================

%% 1. Pack Configuration
bms.pack.N_series      = 14;          % 14 cells in series (14S)
bms.pack.N_parallel    = 1;           % 1 cell in parallel (expandable)
bms.pack.V_nominal_cell = 3.70;       % Nominal cell voltage [V]
bms.pack.V_nominal     = bms.pack.N_series * bms.pack.V_nominal_cell; % 51.8 V
bms.pack.C_nominal_Ah  = 5.0;         % Nominal cell capacity [Ah] (LG M50T / Samsung 50E)
bms.pack.Q_nominal_As  = bms.pack.C_nominal_Ah * 3600; % [Ampere-seconds / Coulombs]
bms.pack.E_nominal_Wh  = bms.pack.V_nominal * bms.pack.C_nominal_Ah; % ~259 Wh

%% 2. 2nd-Order Thevenin Equivalent Circuit Model (ECM)
% Continuous model: 
%   dSoC/dt = - I_L / Q_n
%   dV1/dt  = - V1/(R1*C1) + I_L/C1
%   dV2/dt  = - V2/(R2*C2) + I_L/C2
%   V_t     = OCV(SoC, T) - V1 - V2 - I_L*R0

% Nominal values at 25 deg C:
bms.cell.R0_ref = 0.0150;     % Ohmic internal resistance [Ohms] (15 mOhm)
bms.cell.R1_ref = 0.0085;     % Activation polarization resistance [Ohms] (8.5 mOhm)
bms.cell.C1_ref = 1500.0;     % Activation polarization capacitance [Farads] (tau1 = 12.75s)
bms.cell.R2_ref = 0.0120;     % Concentration polarization resistance [Ohms] (12.0 mOhm)
bms.cell.C2_ref = 6000.0;     % Concentration polarization capacitance [Farads] (tau2 = 72s)
bms.cell.coulomb_eff = 0.998; % Coulombic efficiency during charging (eta)

% Temperature dependence (Arrhenius scaling)
bms.cell.Ea_R0 = 22000;       % Activation energy for R0 [J/mol]
bms.cell.R_gas = 8.314;       % Universal gas constant [J/(mol K)]
bms.cell.T_ref = 298.15;      % Reference temperature (25 deg C in Kelvin)

% Open Circuit Voltage (OCV) vs SoC polynomial coefficients (NMC 811 chemistry)
% OCV(SoC) = p1*s^6 + p2*s^5 + p3*s^4 + p4*s^3 + p5*s^2 + p6*s + p7
% Strictly bounds OCV between 3.15V (0% SoC) and 4.20V (100% SoC)
bms.ocv.poly_coeff = [ ...
    -1.94295, ...   % s^6
     8.52070, ...   % s^5
   -18.71091, ...   % s^4
    21.62724, ...   % s^3
   -12.16665, ...   % s^2
     3.71743, ...   % s^1
     3.15347  ...   % s^0 (OCV at 0% SoC ~ 3.15V)
];

% Precomputed OCV lookup table for fast interpolation (SoC = 0.0 : 0.01 : 1.0)
bms.ocv.soc_bp = linspace(0.0, 1.0, 101);
bms.ocv.ocv_table = polyval(bms.ocv.poly_coeff, bms.ocv.soc_bp);

% Analytic derivative coefficients for Jacobian dVoc/dSoC
% dVoc/ds = 6*p1*s^5 + 5*p2*s^4 + 4*p3*s^3 + 3*p4*s^2 + 2*p5*s + p6
bms.ocv.dpoly_coeff = [ ...
    6 * bms.ocv.poly_coeff(1), ...
    5 * bms.ocv.poly_coeff(2), ...
    4 * bms.ocv.poly_coeff(3), ...
    3 * bms.ocv.poly_coeff(4), ...
    2 * bms.ocv.poly_coeff(5), ...
    bms.ocv.poly_coeff(6)      ...
];

%% 3. Thermal Dynamics Model (Lumped Parameter)
% m * Cp * dT/dt = Q_gen - Q_conv - Q_cooling
bms.thermal.m_cell      = 0.070;   % Mass per cell [kg] (70 grams)
bms.thermal.Cp_cell     = 1050;    % Specific heat capacity [J/(kg*K)]
bms.thermal.h_conv_A    = 0.18;    % Convective dissipation coefficient [W/K]
bms.thermal.T_ambient   = 25.0;    % Ambient temperature [deg C]
bms.thermal.T_cool_max  = 5.0;     % Active cooling max power [W per cell]
bms.thermal.dVoc_dT     = 0.00035; % Entropic temperature coefficient [V/K]

%% 4. Discrete Extended Kalman Filter (EKF) Tuning
% Sample time for discrete EKF updates:
bms.ekf.dt = 0.1; % [seconds] (10 Hz update rate)

% Process noise covariance Q:
% States: [SoC; V1; V2]
% Balanced to allow rapid initial convergence while providing strong
% steady-state immunity against sensor bias and measurement noise
bms.ekf.Q = diag([2.0e-6, 1.0e-5, 1.0e-5]);

% Measurement noise covariance R:
% Voltage sensor measurement error variance (sigma_v ~ 31 mV equivalent)
bms.ekf.R = 1.0e-3; 

% Initial estimation error covariance P0:
% High initial uncertainty enables immediate convergence from unknown initial SoC
bms.ekf.P0 = diag([0.20, 0.01, 0.01]);

% Initial SoC state estimate
bms.ekf.initial_soc_guess = 0.50; % Nominal initial filter state

%% 5. Recursive Least Squares (RLS) SoH Estimator Tuning
bms.rls.lambda     = 0.9995;  % Forgetting factor (close to 1 for slow aging dynamics)
bms.rls.P0         = 1000.0;  % Initial parameter covariance
bms.rls.theta0     = bms.cell.R0_ref; % Initial guess for R0 [Ohms]
bms.rls.R0_eol_ratio = 2.0;   % R0 at End of Life (EOL) is 200% of nominal

%% 6. Active & Passive Balancing Parameters
bms.balancing.mode              = 'active'; % 'active' or 'passive'
bms.balancing.soc_delta_thresh  = 0.015;    % Activate if (SoC_max - SoC_min) > 1.5%
bms.balancing.v_delta_thresh    = 0.015;    % Activate if (V_max - V_min) > 15 mV
bms.balancing.min_soc_enable    = 0.20;     % Only balance when pack SoC > 20%
bms.balancing.max_soc_enable    = 0.98;     % Balance up to 98%
bms.balancing.I_active_max      = 1.50;     % Max active balancing charge shuttle [A]
bms.balancing.active_efficiency = 0.88;     % Bidirectional DC-DC efficiency (88%)
bms.balancing.R_passive_bleed   = 33.0;     % Passive shunt bleed resistor [Ohms] (~110mA)

%% 7. ISO 26262 ASIL-D Functional Safety Thresholds
% Safety Goal 1 (SG01): Prevent Cell Over-Voltage (Thermal runaway prevention)
bms.safety.V_cell_max_fault   = 4.25;  % Immediate contactor open [V]
bms.safety.V_cell_max_warn    = 4.20;  % Charge current derating [V]

% Safety Goal 2 (SG02): Prevent Cell Under-Voltage (Copper dendrite prevention)
bms.safety.V_cell_min_fault   = 2.75;  % Immediate contactor open [V]
bms.safety.V_cell_min_warn    = 3.00;  % Discharge current derating [V]

% Safety Goal 3 (SG03): Prevent Pack Over-Current
bms.safety.I_discharge_max_fault = 120.0; % Max discharge current [A] (24C pulse)
bms.safety.I_charge_max_fault    = 25.0;  % Max regenerative/charge current [A] (5C)

% Safety Goal 4 (SG04): Prevent Over & Under-Temperature
bms.safety.T_cell_max_fault   = 60.0;  % Max cell temperature [deg C]
bms.safety.T_cell_max_derate  = 45.0;  % Begin linear thermal derating [deg C]
bms.safety.T_cell_min_derate  = 5.0;   % Begin cold temperature derating [deg C]
bms.safety.T_cell_min_fault   = -20.0; % Min safe temperature [deg C]

% Safety Goal 5 (SG05): High-Voltage Contactor Sequencing & Weld Detection
bms.safety.R_precharge        = 50.0;  % Precharge resistor [Ohms]
bms.safety.precharge_timeout  = 0.500; % Max time allowed to reach 95% bus voltage [s]
bms.safety.precharge_target   = 0.95;  % 95% of battery voltage required before closing main
bms.safety.weld_detect_thresh = 10.0;  % If V_bus > 10V when commanded OPEN -> Weld Fault
bms.safety.FTTI_seconds       = 0.200; % Fault Tolerant Time Interval: 200 ms max response

%% 8. Cell-to-Cell Manufacturing Dispersion (Plant Simulation)
% Realistic spread among the 14 cells in the pack
rng(42); % Deterministic seed for reproducible testing
bms.dispersion.R0_spread  = bms.cell.R0_ref * (1.0 + 0.08 * (rand(1, bms.pack.N_series) - 0.5));
bms.dispersion.Q_spread   = bms.pack.Q_nominal_As * (1.0 + 0.04 * (rand(1, bms.pack.N_series) - 0.5));
bms.dispersion.SoC_spread = [0.82, 0.85, 0.88, 0.84, 0.86, 0.81, 0.89, ...
                             0.83, 0.87, 0.85, 0.80, 0.88, 0.84, 0.86]; % Initial unbalanced SoC

fprintf('[BMS Init] Pack: %dS%dP (%.1fV, %.1fAh, %.1fWh) initialized successfully.\n', ...
    bms.pack.N_series, bms.pack.N_parallel, bms.pack.V_nominal, bms.pack.C_nominal_Ah, bms.pack.E_nominal_Wh);
end
