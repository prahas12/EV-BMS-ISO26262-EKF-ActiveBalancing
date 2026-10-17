function [bms_state, fault_code, contactors, I_dis_limit, I_chg_limit] = ...
    bms_supervisory_state_machine( ...
        v_cells, I_pack, T_cells, V_bus, vehicle_req, ...
        state_prev, fault_prev, precharge_timer, dt, bms_params)
%#codegen
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% ISO 26262 ASIL-D Supervisory Safety State Machine & Contactor Manager
% =========================================================================
% Implements deterministic functional safety control, contactor sequencing,
% precharge timing, contactor weld diagnostics, and thermal/voltage derating.
%
% States:
%   1 = BMS_STATE_INIT
%   2 = BMS_STATE_STANDBY
%   3 = BMS_STATE_PRECHARGE
%   4 = BMS_STATE_NORMAL_RUN
%   5 = BMS_STATE_DERATING
%   6 = BMS_STATE_SAFE_STATE_FAULT
%
% Contactors struct/vector: [MainPos, MainNeg, PrechargeRelay] (1=CLOSED, 0=OPEN)
% =========================================================================

% Constant Enumerations
STATE_INIT         = 1;
STATE_STANDBY      = 2;
STATE_PRECHARGE    = 3;
STATE_NORMAL_RUN   = 4;
STATE_DERATING     = 5;
STATE_SAFE_FAULT   = 6;

FAULT_NONE         = 0;
FAULT_OVER_VOLT    = 1;
FAULT_UNDER_VOLT   = 2;
FAULT_OVER_CURR_D  = 3;
FAULT_OVER_CURR_C  = 4;
FAULT_OVER_TEMP    = 5;
FAULT_UNDER_TEMP   = 6;
FAULT_PRECHG_TO    = 7;
FAULT_CONT_WELD    = 8;

% Default safe assignments for all output arguments
bms_state   = state_prev;
fault_code  = fault_prev;
contactors  = [0, 0, 0];
I_dis_limit = 0.0;
I_chg_limit = 0.0;

% 1. Evaluate Pack Extremes
v_max = max(v_cells);
v_min = min(v_cells);
t_max = max(T_cells);
t_min = min(T_cells);
V_pack = sum(v_cells);

%% 2. ISO 26262 ASIL-D Emergency Fault Diagnostics (Highest Priority)
active_fault = fault_prev;

if v_max >= bms_params.safety.V_cell_max_fault
    active_fault = FAULT_OVER_VOLT;
elseif v_min <= bms_params.safety.V_cell_min_fault
    active_fault = FAULT_UNDER_VOLT;
elseif I_pack >= bms_params.safety.I_discharge_max_fault
    active_fault = FAULT_OVER_CURR_D;
elseif I_pack <= -bms_params.safety.I_charge_max_fault
    active_fault = FAULT_OVER_CURR_C;
elseif t_max >= bms_params.safety.T_cell_max_fault
    active_fault = FAULT_OVER_TEMP;
elseif t_min <= bms_params.safety.T_cell_min_fault
    active_fault = FAULT_UNDER_TEMP;
end

%% 3. State Machine Transition Logic
if active_fault ~= FAULT_NONE
    % Immediate trip to emergency safe state within FTTI (Fault Tolerant Time Interval)
    bms_state = STATE_SAFE_FAULT;
    fault_code = active_fault;
    contactors = [0, 0, 0]; % Open all contactors immediately
    I_dis_limit = 0.0;
    I_chg_limit = 0.0;
    return;
end

% Normal State Transitions
switch state_prev
    case STATE_INIT
        % Power-on self test successful
        bms_state = STATE_STANDBY;
        contactors = [0, 0, 0];

    case STATE_STANDBY
        contactors = [0, 0, 0];
        % Check for contactor weld fault (bus voltage should be ~0V when open)
        if V_bus > bms_params.safety.weld_detect_thresh
            bms_state = STATE_SAFE_FAULT;
            fault_code = FAULT_CONT_WELD;
            return;
        end

        % Vehicle drive/charge requested
        if vehicle_req >= 1
            bms_state = STATE_PRECHARGE;
            % Close Main Negative & Precharge Relay
            contactors = [0, 1, 1];
        else
            bms_state = STATE_STANDBY;
        end

    case STATE_PRECHARGE
        % Keep Main Negative & Precharge closed
        contactors = [0, 1, 1];

        % Check precharge completion
        if V_bus >= (bms_params.safety.precharge_target * V_pack)
            % Successful precharge -> Transition to normal run
            bms_state = STATE_NORMAL_RUN;
            contactors = [1, 1, 0]; % Close Main Positive, Open Precharge
        elseif precharge_timer >= bms_params.safety.precharge_timeout
            % Precharge timed out! Resistor would overheat or short exists
            bms_state = STATE_SAFE_FAULT;
            fault_code = FAULT_PRECHG_TO;
            contactors = [0, 0, 0];
            return;
        else
            bms_state = STATE_PRECHARGE;
        end

    case {STATE_NORMAL_RUN, STATE_DERATING}
        if vehicle_req == 0
            % Vehicle switched off -> graceful shutdown to standby
            bms_state = STATE_STANDBY;
            contactors = [0, 0, 0];
        else
            % Check if operating conditions require derating
            is_derating = (t_max >= bms_params.safety.T_cell_max_derate) || ...
                          (t_min <= bms_params.safety.T_cell_min_derate) || ...
                          (v_max >= bms_params.safety.V_cell_max_warn)   || ...
                          (v_min <= bms_params.safety.V_cell_min_warn);

            if is_derating
                bms_state = STATE_DERATING;
            else
                bms_state = STATE_NORMAL_RUN;
            end
            contactors = [1, 1, 0]; % Main Pos & Main Neg CLOSED
        end

    otherwise
        bms_state = STATE_SAFE_FAULT;
        contactors = [0, 0, 0];
end

fault_code = active_fault;

%% 4. Dynamic Current Limit Derating Calculation
if bms_state == STATE_SAFE_FAULT
    I_dis_limit = 0.0;
    I_chg_limit = 0.0;
elseif bms_state == STATE_DERATING
    % Thermal derating factor
    if t_max >= bms_params.safety.T_cell_max_derate
        thermal_factor = max(0.1, 1.0 - (t_max - bms_params.safety.T_cell_max_derate) / ...
                         (bms_params.safety.T_cell_max_fault - bms_params.safety.T_cell_max_derate));
    elseif t_min <= bms_params.safety.T_cell_min_derate
        thermal_factor = max(0.1, (t_min - bms_params.safety.T_cell_min_fault) / ...
                         (bms_params.safety.T_cell_min_derate - bms_params.safety.T_cell_min_fault));
    else
        thermal_factor = 1.0;
    end

    % Voltage derating factor
    if v_min <= bms_params.safety.V_cell_min_warn
        v_dis_factor = max(0.1, (v_min - bms_params.safety.V_cell_min_fault) / ...
                       (bms_params.safety.V_cell_min_warn - bms_params.safety.V_cell_min_fault));
    else
        v_dis_factor = 1.0;
    end

    if v_max >= bms_params.safety.V_cell_max_warn
        v_chg_factor = max(0.1, (bms_params.safety.V_cell_max_fault - v_max) / ...
                       (bms_params.safety.V_cell_max_fault - bms_params.safety.V_cell_max_warn));
    else
        v_chg_factor = 1.0;
    end

    I_dis_limit = bms_params.safety.I_discharge_max_fault * min(thermal_factor, v_dis_factor);
    I_chg_limit = bms_params.safety.I_charge_max_fault * min(thermal_factor, v_chg_factor);
else
    % Normal operating limits
    I_dis_limit = 80.0;  % Continuous discharge limit [A]
    I_chg_limit = 20.0;  % Continuous charge limit [A]
end

end
