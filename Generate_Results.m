% =========================================================================
% AUTOMATED RESULTS GENERATION
% =========================================================================
% Headless execution of drive cycle simulation and plot generation.
% =========================================================================

clc; clear; close all;

currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir, 'models'));
addpath(fullfile(currentDir, 'scripts'));
addpath(fullfile(currentDir, 'results'));

fprintf('Loading parameters...\n');
bms_params = init_bms_parameters();

fprintf('Running closed-loop WLTP simulation...\n');
sim_results = run_drive_cycle_simulation('WLTP', 600, 0.65);

fprintf('Exporting telemetry plots...\n');
plot_bms_results(sim_results);

fprintf('Results saved to /results directory.\n');
