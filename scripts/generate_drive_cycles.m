function cycle = generate_drive_cycles(cycle_name, duration_sec, dt)
% =========================================================================
% EV-BMS-ISO26262-EKF-ActiveBalancing
% Standard Automotive Drive Cycle Generator
% =========================================================================
% Generates time-stamped load current profiles [A] for battery pack testing.
% Positive current = Discharge (accelerating / driving)
% Negative current = Charge (regenerative braking / grid charging)
%
% Supported profiles:
%   - 'WLTP'  : Scaled WLTP Class 3 driving cycle with dynamic regen
%   - 'US06'  : US06 Aggressive High-Acceleration Highway cycle
%   - 'DST'   : Dynamic Stress Test (USABC battery evaluation protocol)
%   - 'HPPC'  : Hybrid Pulse Power Characterization (pulse-rest sequence)
%   - 'PULSE' : High initial error convergence test profile
% =========================================================================

if nargin < 1, cycle_name = 'WLTP'; end
if nargin < 2, duration_sec = 1800; end % Default 30 minutes
if nargin < 3, dt = 0.1; end           % Default 100 ms sample time

time = (0:dt:duration_sec)';
N = length(time);
current = zeros(N, 1);

switch upper(cycle_name)
    case 'WLTP'
        % Scaled WLTP Class 3 velocity-derived current profile
        % Comprises Low, Medium, High, and Extra-High speed phases
        omega_low  = 2*pi / 200;
        omega_med  = 2*pi / 120;
        omega_high = 2*pi / 60;
        
        for k = 1:N
            t = time(k);
            phase = mod(t, 1800);
            if phase < 589
                % Low & Medium Speed (urban stop-and-go)
                base = 3.5 + 4.0*sin(omega_low*t) + 2.5*sin(3*omega_low*t + 0.5);
                regen = -2.5 * max(0, sin(omega_med*t + 1.2));
            elseif phase < 1022
                % High Speed (suburban cruising + passing)
                base = 6.0 + 5.5*sin(omega_med*t) + 3.0*cos(omega_high*t);
                regen = -4.0 * max(0, sin(omega_high*t + 0.8));
            else
                % Extra High Speed (highway cruising with high-power overtakes)
                base = 9.0 + 7.0*sin(omega_high*t) + 4.0*sin(2.5*omega_high*t);
                regen = -5.5 * max(0, sin(omega_high*t + 1.5));
            end
            % Zero current during idle vehicle stops (red lights)
            if mod(t, 150) < 18
                current(k) = 0.0;
            else
                current(k) = max(-8.0, min(25.0, base + regen));
            end
        end

    case 'US06'
        % US06: Highly aggressive, rapid accelerations and deep regenerative pulses
        for k = 1:N
            t = time(k);
            harmonic = sin(0.12*t) + 1.2*cos(0.25*t) - 0.8*sin(0.05*t);
            if harmonic > 0.8
                % Hard acceleration burst
                current(k) = min(35.0, 15.0 + 15.0*harmonic);
            elseif harmonic < -0.8
                % Hard regenerative braking
                current(k) = max(-15.0, -10.0 + 6.0*harmonic);
            else
                current(k) = 4.0 * harmonic + 2.0;
            end
        end

    case 'DST'
        % Dynamic Stress Test: 360-second repetitive power steps
        step_pattern = [ ...
            0,  16, 0.0;   % Rest
            16, 44, 4.0;   % Low discharge
            44, 56, 8.0;   % Medium discharge
            56, 68, 16.0;  % High discharge
            68, 80, -7.0;  % Regen charge
            80, 160, 2.5;  % Cruising
            160, 180, 12.0;% Acceleration
            180, 200, -8.0;% Regen
            200, 360, 3.5  % Sustained drive
        ];
        for k = 1:N
            t_mod = mod(time(k), 360);
            for s = 1:size(step_pattern, 1)
                if t_mod >= step_pattern(s, 1) && t_mod < step_pattern(s, 2)
                    current(k) = step_pattern(s, 3);
                    break;
                end
            end
        end

    case 'HPPC'
        % Hybrid Pulse Power Characterization:
        % 10s Discharge Pulse -> 40s Rest -> 10s Charge Pulse -> Rest
        for k = 1:N
            t_mod = mod(time(k), 120);
            if t_mod >= 10 && t_mod < 20
                current(k) = 15.0;  % 3C Discharge pulse
            elseif t_mod >= 60 && t_mod < 70
                current(k) = -10.0; % 2C Regen pulse
            else
                current(k) = 0.0;   % Relaxation/Rest
            end
        end

    case 'PULSE'
        % Simple pulsed profile for fast EKF initial error convergence validation
        for k = 1:N
            t = time(k);
            if mod(t, 60) < 30
                current(k) = 5.0;
            else
                current(k) = -2.0;
            end
        end

    otherwise
        error('Unknown drive cycle: %s. Supported: WLTP, US06, DST, HPPC, PULSE.', cycle_name);
end

% Build output structure
cycle.name         = cycle_name;
cycle.time         = time;
cycle.current      = current;
cycle.dt           = dt;
cycle.duration_sec = duration_sec;
cycle.sample_count = N;

end
