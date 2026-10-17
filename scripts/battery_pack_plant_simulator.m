function [plant_state_next, telemetry] = battery_pack_plant_simulator( ...
    I_pack_cmd, I_bal_vector, contactors, plant_state_prev, dt, bms_params)
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% High-Fidelity 14-Cell 2nd-Order Thevenin Battery Plant & Thermal Model
% =========================================================================
% Models the physical ground truth of the 14-cell battery pack:
%   - 2nd-order ECM dynamics per cell
%   - Coupled thermal Joule heating & convection
%   - Cell-to-cell capacity, resistance, and initial SoC dispersion
%   - Precharge circuit & Inverter DC-link bus dynamics
%   - Realistic sensor noise (voltage, current, temperature) and sensor bias
% =========================================================================

N = bms_params.pack.N_series;

% Unpack previous plant states:
% plant_state_prev.soc_true  : [Nx1]
% plant_state_prev.v1_true   : [Nx1]
% plant_state_prev.v2_true   : [Nx1]
% plant_state_prev.T_true    : [Nx1] [deg C]
% plant_state_prev.V_bus     : scalar [V]

soc_true = plant_state_prev.soc_true;
v1_true  = plant_state_prev.v1_true;
v2_true  = plant_state_prev.v2_true;
T_true   = plant_state_prev.T_true;
V_bus    = plant_state_prev.V_bus;

% Check contactor state: contactors = [MainPos, MainNeg, Precharge]
main_closed      = (contactors(1) == 1) && (contactors(2) == 1);
precharge_closed = (contactors(3) == 1) && (contactors(2) == 1);

% Effective pack current flowing through cells
if main_closed
    I_pack_actual = I_pack_cmd;
elseif precharge_closed
    % Current limited by precharge resistor
    V_pack_approx = sum(polyval(bms_params.ocv.poly_coeff, soc_true));
    I_pack_actual = max(0.0, (V_pack_approx - V_bus) / bms_params.safety.R_precharge);
else
    I_pack_actual = 0.0; % Open circuit
end

% Individual cell currents (Pack current + active balancing current)
I_cells = I_pack_actual + I_bal_vector;

%% 1. Update Cell ECM States (Ground Truth)
poly_c = bms_params.ocv.poly_coeff;
vt_true = zeros(N, 1);
voc_true = zeros(N, 1);

for k = 1:N
    Q_k = bms_params.dispersion.Q_spread(k);
    R0_k = bms_params.dispersion.R0_spread(k);
    R1_k = bms_params.cell.R1_ref;
    C1_k = bms_params.cell.C1_ref;
    R2_k = bms_params.cell.R2_ref;
    C2_k = bms_params.cell.C2_ref;

    % Coulombic efficiency
    eta_k = 1.0;
    if I_cells(k) < 0
        eta_k = bms_params.cell.coulomb_eff;
    end

    % Arrhenius resistance scaling with temperature
    T_K = T_true(k) + 273.15;
    arrhenius = exp((bms_params.cell.Ea_R0 / bms_params.cell.R_gas) * ...
                    (1.0 / T_K - 1.0 / bms_params.cell.T_ref));
    R0_eff = R0_k * arrhenius;

    % State integration
    tau1 = R1_k * C1_k;
    tau2 = R2_k * C2_k;
    alpha1 = exp(-dt / tau1);
    alpha2 = exp(-dt / tau2);

    % Update states
    soc_true(k) = soc_true(k) - (eta_k * dt / Q_k) * I_cells(k);
    soc_true(k) = max(0.0, min(1.0, soc_true(k)));

    v1_true(k)  = alpha1 * v1_true(k) + R1_k * (1.0 - alpha1) * I_cells(k);
    v2_true(k)  = alpha2 * v2_true(k) + R2_k * (1.0 - alpha2) * I_cells(k);

    % Ground truth OCV
    s = soc_true(k);
    voc_true(k) = polyval(poly_c, s);

    % Terminal voltage
    vt_true(k) = voc_true(k) - v1_true(k) - v2_true(k) - I_cells(k) * R0_eff;

    %% 2. Coupled Thermal Dynamics
    % Joule heating + polarization loss
    Q_joule = (I_cells(k)^2) * R0_eff;
    Q_polar = (v1_true(k)^2 / R1_k) + (v2_true(k)^2 / R2_k);
    Q_gen   = Q_joule + Q_polar;

    % Convective cooling to ambient
    Q_loss = bms_params.thermal.h_conv_A * (T_true(k) - bms_params.thermal.T_ambient);

    % Active chiller cooling kicks in if T > 35 C
    if T_true(k) > 35.0
        Q_chiller = min(bms_params.thermal.T_cool_max, 0.4 * (T_true(k) - 35.0));
    else
        Q_chiller = 0.0;
    end

    dT_dt = (Q_gen - Q_loss - Q_chiller) / (bms_params.thermal.m_cell * bms_params.thermal.Cp_cell);
    T_true(k) = T_true(k) + dT_dt * dt;
end

V_pack_true = sum(vt_true);

%% 3. Inverter DC Link Bus Dynamics
C_link = 0.0015; % 1500 uF inverter DC link capacitor
if main_closed
    V_bus = V_pack_true;
elseif precharge_closed
    % Charging through precharge resistor
    dV_bus = (V_pack_true - V_bus) / (bms_params.safety.R_precharge * C_link) * dt;
    V_bus = min(V_pack_true, V_bus + dV_bus);
else
    % Bleeding through inverter bleeder resistor (50 kOhm)
    R_bleed_bus = 50000.0;
    dV_bus = - V_bus / (R_bleed_bus * C_link) * dt;
    V_bus = max(0.0, V_bus + dV_bus);
end

%% 4. Sensor Imperfections (Noise & Sensor Bias)
% Typical automotive BMS AFE (Analog Front-End like TI BQ79616 or ADI ADBMS6815):
% Voltage noise: sigma = 4.5 mV
% Current Hall-effect sensor: sigma = 0.15 A, bias = +0.08 A
% NTC Thermistor: sigma = 0.25 deg C
noise_v = 0.0045 * randn(N, 1);
noise_i = 0.15 * randn() + 0.08; % Includes 80 mA sensor bias
noise_T = 0.25 * randn(N, 1);

vt_meas = vt_true + noise_v;
I_meas  = I_pack_actual + noise_i;
T_meas  = T_true + noise_T;

%% 5. Pack Outputs
plant_state_next.soc_true = soc_true;
plant_state_next.v1_true  = v1_true;
plant_state_next.v2_true  = v2_true;
plant_state_next.T_true   = T_true;
plant_state_next.V_bus    = V_bus;

telemetry.vt_true       = vt_true;
telemetry.vt_meas       = vt_meas;
telemetry.voc_true      = voc_true;
telemetry.soc_true      = soc_true;
telemetry.T_true        = T_true;
telemetry.T_meas        = T_meas;
telemetry.I_pack_actual = I_pack_actual;
telemetry.I_pack_meas   = I_meas;
telemetry.V_pack_true   = V_pack_true;
telemetry.V_bus         = V_bus;

end
