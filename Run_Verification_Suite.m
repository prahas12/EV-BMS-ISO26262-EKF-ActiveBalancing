% =========================================================================
% AUTOMATED VERIFICATION & TEST SUITE
% =========================================================================
% Headless test runner:
%   1. Runs automated unit tests for estimators, balancing, and state machine
%   2. Simulates closed-loop WLTP drive cycle and exports performance plots
%   3. Compiles and executes native C firmware target
% =========================================================================

clc; clear; close all;

fprintf('Running automated verification suite...\n\n');

projRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projRoot, 'models'));
addpath(fullfile(projRoot, 'scripts'));
addpath(fullfile(projRoot, 'tests'));
addpath(fullfile(projRoot, 'results'));
addpath(fullfile(projRoot, 'src'));

suite_timer = tic;

% 1. Unit Tests
fprintf('[1/3] Executing unit tests...\n');
test_verdict = run_all_tests();

if ~test_verdict
    error('Unit tests failed.');
end

% 2. Closed-Loop Drive Cycle Simulation
fprintf('\n[2/3] Executing closed-loop WLTP simulation...\n');
wltp_res = run_drive_cycle_simulation('WLTP', 600, 0.65);
plot_bms_results(wltp_res);

% 3. C Firmware Build and Execution
fprintf('\n[3/3] Compiling and running C firmware target...\n');
src_dir = fullfile(projRoot, 'src');
old_pwd = pwd;
cd(src_dir);

c_build_cmd = 'gcc -std=c99 -Wall -Wextra -O2 -I. bms_main.c bms_ekf.c bms_fsm.c bms_balancing.c -o bms_firmware_sim.exe -lm';
[status, out] = system(c_build_cmd);

if status == 0
    [run_status, run_out] = system('.\bms_firmware_sim.exe');
    disp(run_out);
else
    fprintf('C compilation error: %s\n', out);
end

cd(old_pwd);
total_elapsed = toc(suite_timer);

fprintf('Verification finished in %.2f seconds.\n', total_elapsed);
fprintf('Plots saved to: %s\n\n', fullfile(projRoot, 'results'));
