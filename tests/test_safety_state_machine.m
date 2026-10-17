function pass = test_safety_state_machine()
% Verifies contactor precharge sequencing, weld detection, and fault trip logic.

fprintf('Running [test_safety_state_machine]... ');

bms_params = init_bms_parameters();
dt = 0.01; % 10 ms fast supervisory task rate
N = bms_params.pack.N_series;

% Helper variables
v_nominal = 3.70 * ones(N, 1);
T_nominal = 25.0 * ones(N, 1);
I_zero    = 0.0;

%% Test 1: Normal Precharge Sequence
state = 2; % STANDBY
fault = 0;
V_bus = 0.0;
vehicle_req = 1; % Key ON

% Step 1: Request precharge
[state, fault, contactors, ~, ~] = bms_supervisory_state_machine( ...
    v_nominal, I_zero, T_nominal, V_bus, vehicle_req, state, fault, 0.0, dt, bms_params);
assert(state == 3, 'Failed to transition to PRECHARGE');
assert(isequal(contactors, [0, 1, 1]), 'Incorrect precharge contactors');

% Step 2: Bus charges to 96%
V_bus = 0.96 * sum(v_nominal);
[state, fault, contactors, ~, ~] = bms_supervisory_state_machine( ...
    v_nominal, I_zero, T_nominal, V_bus, vehicle_req, state, fault, 0.2, dt, bms_params);
assert(state == 4, 'Failed to transition to NORMAL_RUN after precharge');
assert(isequal(contactors, [1, 1, 0]), 'Main contactors not closed');

%% Test 2: Over-Voltage Fault Injection (SG01)
v_ov = v_nominal;
v_ov(3) = 4.30; % 4.30V > 4.25V threshold
[state, fault, contactors, I_dis_lim, I_chg_lim] = bms_supervisory_state_machine( ...
    v_ov, I_zero, T_nominal, V_bus, vehicle_req, state, fault, 0.0, dt, bms_params);
assert(state == 6, 'Failed to transition to SAFE STATE on Overvoltage');
assert(fault == 1, 'Wrong fault code for Overvoltage');
assert(isequal(contactors, [0, 0, 0]), 'Contactors not opened on Overvoltage');
assert(I_dis_lim == 0 && I_chg_lim == 0, 'Current limits not zeroed in fault');

%% Test 3: Under-Voltage Fault Injection (SG02)
v_uv = v_nominal;
v_uv(5) = 2.60; % 2.60V < 2.75V threshold
[state, fault, contactors, ~, ~] = bms_supervisory_state_machine( ...
    v_uv, I_zero, T_nominal, V_bus, vehicle_req, 4, 0, 0.0, dt, bms_params);
assert(state == 6 && fault == 2, 'Failed Under-Voltage trip');

%% Test 4: Over-Temperature Fault Injection (SG04)
T_ot = T_nominal;
T_ot(2) = 65.0; % 65 C > 60 C threshold
[state, fault, contactors, ~, ~] = bms_supervisory_state_machine( ...
    v_nominal, I_zero, T_ot, V_bus, vehicle_req, 4, 0, 0.0, dt, bms_params);
assert(state == 6 && fault == 5, 'Failed Over-Temperature trip');

%% Test 5: Precharge Timeout Fault Injection
V_bus_stuck = 10.0; % Stalled bus
precharge_timer = 0.55; % > 500 ms timeout
[state, fault, contactors, ~, ~] = bms_supervisory_state_machine( ...
    v_nominal, I_zero, T_nominal, V_bus_stuck, vehicle_req, 3, 0, precharge_timer, dt, bms_params);
assert(state == 6 && fault == 7, 'Failed Precharge Timeout trip');

%% Test 6: Contactor Weld Detection (SG05)
V_bus_welded = 48.0; % Bus has voltage while in Standby
[state, fault, contactors, ~, ~] = bms_supervisory_state_machine( ...
    v_nominal, I_zero, T_nominal, V_bus_welded, 0, 2, 0, 0.0, dt, bms_params);
assert(state == 6 && fault == 8, 'Failed Contactor Weld detection');

fprintf('PASS (All 6 ISO 26262 Safety Scenarios Verified)\n');
pass = true;
end
