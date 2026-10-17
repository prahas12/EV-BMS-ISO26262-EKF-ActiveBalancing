% =========================================================================
% EV BATTERY MANAGEMENT SYSTEM (BMS) - EKF & ACTIVE BALANCING
% =========================================================================
% Main launcher script for the BMS simulation environment.
% Initializes workspace parameters, configures compiler paths, and opens
% the core Simulink architecture.
% =========================================================================

clc; clear; close all;

fprintf('Initializing BMS Simulation Environment...\n');

% 1. Add dependencies to MATLAB path
projRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projRoot, 'models'));
addpath(fullfile(projRoot, 'scripts'));
addpath(fullfile(projRoot, 'tests'));
addpath(fullfile(projRoot, 'results'));
addpath(fullfile(projRoot, 'src'));

% 2. Configure C Compiler Path
mingw_path = 'C:\MinGW';
if isfolder(mingw_path)
    setenv('MW_MINGW64_LOC', mingw_path);
    try
        evalc('mex -setup C');
    catch
    end
end

% 3. Initialize Battery Pack and Controller Parameters
fprintf('Loading battery pack and estimator parameters...\n');
bms_params = init_bms_parameters();
assignin('base', 'bms_params', bms_params);

% 4. Generate default drive cycle profile
wltp_cycle = generate_drive_cycles('WLTP', 600, bms_params.ekf.dt);
I_drive_series = timeseries(wltp_cycle.current, wltp_cycle.time);
assignin('base', 'wltp_cycle', wltp_cycle);
assignin('base', 'I_drive_series', I_drive_series);

fprintf('Environment Setup Complete.\n');

% 5. Open the Simulink Architecture
modelName = 'EV_BMS_ISO26262_Model';
fprintf('Opening Simulink Model: %s...\n', modelName);
if isfile(fullfile(projRoot, 'models', [modelName, '.slx']))
    open_system(modelName);
else
    build_bms_simulink_model();
    open_system(modelName);
end

fprintf('\nSimulation Ready.\n');
fprintf('--> Run "Run_Verification_Suite.m" to execute automated unit tests and drive cycle benchmarks.\n\n');
