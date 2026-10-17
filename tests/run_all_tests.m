function all_pass = run_all_tests()
% =========================================================================
% AUTOMATED UNIT TEST SUITE
% =========================================================================
% Executes battery model, state estimator, balancing, and safety test cases:
%   1. EKF Initial Error Convergence
%   2. Sensor Bias & Noise Immunity
%   3. RLS Online Parameter & SoH Identification
%   4. Active Cell Balancing
%   5. Supervisory State Machine & Fault Injection
% =========================================================================

clc;
fprintf('Running automated unit tests...\n\n');

tests_dir = fileparts(mfilename('fullpath'));
root_dir  = fullfile(tests_dir, '..');
addpath(fullfile(root_dir, 'scripts'));
addpath(tests_dir);

tests = { ...
    'test_ekf_convergence',       'EKF Initial State Error Convergence (<2%)'; ...
    'test_noise_immunity',        'Current Sensor Bias & Noise Immunity'; ...
    'test_soh_rls_estimation',    'RLS Online Internal Resistance & SoH Identification'; ...
    'test_active_balancing',      'Active Cell Balancing Charge Shuttling Convergence'; ...
    'test_safety_state_machine',  'ISO 26262 State Machine & Fault Injection (SG01-SG05)' ...
};

num_tests = size(tests, 1);
results = cell(num_tests, 3);
total_timer = tic;

for i = 1:num_tests
    test_func = tests{i, 1};
    test_desc = tests{i, 2};
    t_start = tic;
    try
        feval(test_func);
        results{i, 1} = 'PASS';
        results{i, 2} = toc(t_start);
        results{i, 3} = 'None';
    catch ME
        fprintf('FAIL (%s)\n', ME.message);
        results{i, 1} = 'FAIL';
        results{i, 2} = toc(t_start);
        results{i, 3} = ME.message;
    end
end

total_time = toc(total_timer);

% Print Summary Table
fprintf('\nTest Execution Summary (Total: %.2f s):\n', total_time);
fprintf('--------------------------------------------------------------------\n');
fprintf(' %-4s | %-45s | %-6s | %-8s\n', 'No.', 'Scenario', 'Status', 'Time [s]');
fprintf('--------------------------------------------------------------------\n');

pass_count = 0;
for i = 1:num_tests
    status = results{i, 1};
    if strcmp(status, 'PASS')
        pass_count = pass_count + 1;
        status_str = '[PASS]';
    else
        status_str = '[FAIL]';
    end
    fprintf('  %02d  | %-45s | %-6s | %6.3f\n', ...
        i, tests{i, 2}, status_str, results{i, 2});
end
fprintf('--------------------------------------------------------------------\n');
fprintf('Verdict: %d / %d Tests Passed (%.1f%%)\n\n', ...
    pass_count, num_tests, (pass_count / num_tests) * 100);

all_pass = (pass_count == num_tests);
end
