function [I_bal_vector, bal_active, donor_idx, receiver_idx, delta_soc] = ...
    bms_active_balancing(soc_cells, v_cells, bms_state, bms_params)
%#codegen
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Active / Passive Cell Balancing Controller
% =========================================================================
% Balances a 14-cell Li-ion string to prevent premature pack cutoff
% and maximize usable energy capacity.
%
% Inputs:
%   soc_cells  : Estimated SoC for each cell [14x1 vector]
%   v_cells    : Measured terminal voltage for each cell [14x1 vector]
%   bms_state  : Current operating mode of the BMS (integer code)
%   bms_params : BMS parameters structure
%
% Outputs:
%   I_bal_vector : Net balancing current for each cell [14x1 vector] [A]
%                  (positive = discharging/donating, negative = receiving)
%   bal_active   : Boolean flag indicating if balancing is active
%   donor_idx    : Index of cell with highest SoC (1-based)
%   receiver_idx : Index of cell with lowest SoC (1-based)
%   delta_soc    : Current peak-to-peak SoC spread [0 to 1]
% =========================================================================

N = bms_params.pack.N_series;
I_bal_vector = zeros(N, 1);
bal_active   = false;
donor_idx    = 1;
receiver_idx = 1;

% Compute spreads
[max_soc, donor_idx]    = max(soc_cells);
[min_soc, receiver_idx] = min(soc_cells);
delta_soc = max_soc - min_soc;

% Balancing inhibitor checks:
% Do not balance if in FAULT state (state == 6), INIT (state == 1), or PRECHARGE (state == 3)
% Or if average SoC is below minimum threshold (e.g. 20%)
mean_soc = mean(soc_cells);
if (bms_state == 6) || (bms_state == 1) || (bms_state == 3) || ...
   (mean_soc < bms_params.balancing.min_soc_enable) || ...
   (mean_soc > bms_params.balancing.max_soc_enable)
    return; % Balancing disabled
end

% Check activation threshold
if delta_soc > bms_params.balancing.soc_delta_thresh
    bal_active = true;

    if strcmpi(bms_params.balancing.mode, 'active')
        % Active Balancing (Bidirectional Inductive/Capacitive DC-DC)
        % Shuttles charge from highest cell to lowest cell
        I_cmd = bms_params.balancing.I_active_max;
        eta   = bms_params.balancing.active_efficiency;

        % Donor cell discharges
        I_bal_vector(donor_idx) = I_cmd;

        % Receiver cell charges (accounting for DC-DC converter efficiency)
        I_bal_vector(receiver_idx) = - (I_cmd * eta);

    else
        % Passive Shunt Resistor Balancing
        % Bleeds charge from any cell above the mean + half threshold
        target_soc = mean_soc + 0.5 * bms_params.balancing.soc_delta_thresh;
        R_bleed    = bms_params.balancing.R_passive_bleed;

        for k = 1:N
            if soc_cells(k) > target_soc
                % Passive bleed current I = V / R_bleed (positive = discharging)
                I_bal_vector(k) = v_cells(k) / R_bleed;
            end
        end
    end
end

end
