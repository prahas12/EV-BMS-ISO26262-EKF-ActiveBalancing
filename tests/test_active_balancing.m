function pass = test_active_balancing()
% Evaluates active balancing charge shuttling under cell-to-cell SoC spread.

fprintf('Running [test_active_balancing]... ');

bms_params = init_bms_parameters();
bms_params.balancing.mode = 'active';
N = bms_params.pack.N_series;
dt = 0.5; % 500 ms step
duration = 1800; % 30 minutes of balancing
time = 0:dt:duration;
N_steps = length(time);

% Artificial severe initial unbalance
soc_cells = linspace(0.78, 0.88, N)'; % 10% spread
initial_spread = (max(soc_cells) - min(soc_cells)) * 100.0;

v_cells = polyval(bms_params.ocv.poly_coeff, soc_cells);
bms_state = 4; % Normal run state

for k = 1:N_steps
    % Execute active balancing
    [I_bal, bal_active, donor, receiver, delta_soc] = ...
        bms_active_balancing(soc_cells, v_cells, bms_state, bms_params);

    % Update cell states
    for c = 1:N
        % dSoC = - I_bal / Q_n * dt
        soc_cells(c) = soc_cells(c) - (dt / bms_params.pack.Q_nominal_As) * I_bal(c);
        v_cells(c)   = polyval(bms_params.ocv.poly_coeff, soc_cells(c));
    end

    if ~bal_active
        break; % Target balance achieved!
    end
end

final_spread = (max(soc_cells) - min(soc_cells)) * 100.0;
reduction_pct = (initial_spread - final_spread) / initial_spread * 100.0;

% Assertions
assert(final_spread < initial_spread, 'Balancing failed to reduce SoC spread');
assert(reduction_pct > 30.0, sprintf('Spread reduction (%.1f%%) below target (30%%)', reduction_pct));

fprintf('PASS (Initial Spread: %.1f%% -> Final Spread: %.1f%%, Reduction: %.1f%%)\n', ...
    initial_spread, final_spread, reduction_pct);
pass = true;
end
