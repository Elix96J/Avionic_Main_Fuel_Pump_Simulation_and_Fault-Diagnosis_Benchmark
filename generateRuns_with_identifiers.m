clearvars; 
close all; 
clc;
Simulink.sdi.clear;
Simulink.sdi.close;
Simulink.sdi.setAutoArchiveMode(false);
% Falls der alte MATLAB-Connector noch existiert: ausschalten

if exist('connector','file')
    try
        connector off;
    catch
    end
end

% generateFaultyRuns.m
% Creates sequences from the Simscape model mainFuelPumpV4

mdl        = 'mainFuelPumpV4';
load_system(mdl)

nsim       = 1000;                           % 1000 runs (1e3)
tStop      = 30;                          % Simulation time [s]
tSampleLog = 0:0.005:tStop;               % Sample rate (default 200 Hz)
nStep      = numel(tSampleLog);

% signal names
vars = ["p_Pump", "p_FMU", "p_Shut", "p_Combustion", ...
        "Flow_PRV", "Flow_Bypass", "Flow_Pump", ...
        "Flow_Engine1", "Flow_Engine2", "Flow_Engine3", "Flow_Engine4", ...
        "RPM_Motor", "Signal_Throttle"];

% Initialize matrix
data = NaN(nStep * nsim, numel(vars) + 1);  % +1 für Zeit
idx = 1;
% -------------------------------------------------------------------------
% CHOOSE FAULT DATASET:
% 0 = all healthy runs
% 1..9 = fault datasets; runs 1..50 are healthy, 51..1000 are faulty
% Select specific fault case by number
% 0 = no fault (healthy run)
% 1 = white noise amplitude fault
% 2 = sensor dropout (p_FMU)
% 3 = sensor dropout (Flow_Engine1)
% 4 = pump leakage
% 5 = pump displacement variation
% 6 = PRV leakage
% 7 = FMU orifice area fault
% 8 = Bypass valve area fault
% 9 = boost (pre-)pressure too high
% -------------------------------------------------------------------------
chosenFault = 0;   % <--- SET THIS MANUALLY (0..9)

for k = 1:nsim 
    if chosenFault == 0
        fault = 0;           % Purely healthy dataset (all 1000 runs healthy)
    else
        if k <= 50
            fault = 0;       % runs 1..50: healthy behaviour
        else
            fault = chosenFault;    % runs 51..1000: selected fault mode
        end
    end

    % parameter variations for each run 
    etaVol   = round(0.90 + 0.02*randn, 4);                % volumetric efficiency [-]
    Ainj     = round(25 * (1 + 0.03*randn), 4);            % Injector nozzle area [mm²]
    pChamber = round(12  * (1 + 0.1*randn), 4);            % Combustion engine pressure [bar]
    % ---Fault injection---
    % Defaults (healthy)
    wnAmp = 0.05;                                           % [-]
    %sensorDrop1 = false;                                   % boolean
    %sensorDrop2 = false;                                   % boolean
    pumpDisp = 40.7;                                        % [cm^3/rev]
    PRVLeakageArea = 1e-8;                                  % [m^2]
    A_FMU = 100;                                            % [mm^2]
    A_bypass = 157;                                         % [mm^2]
    pressure_boost = 2;                                     % [bar]

    switch fault
        case 1 % white noise amplitude fault
            wnAmp = 0.25;

        % case 2 and 3 are handled at signal level below (sensor dropouts)

        case 4 % pump leakage
            etaVol = round(0.80 + 0.02*randn, 4);

        case 5 % displacement fault (reduction)
            pumpDisp = 35;  % [cm^3/rev] 
        
        case 6 % PRV leakage
            PRVLeakageArea = 1e-6; % [m^2]

        case 7 % FMU orifice area fault
            A_FMU = 50;  % [mm^2]

        case 8 % Bypass valve area reduced
            A_bypass = 60; % [mm^2]

        case 9 % Boost pressure too high
            pressure_boost = 5; % [bar]
    end

    % Apply variations in model parameters
    % for randomness for all runs regardless if healthy or faulty
    set_param([mdl '/Main Gear Pump'], 'vol_eff_nominal', num2str(etaVol));
    set_param([mdl '/Injector nozzle'], 'orifice_area_constant', num2str(Ainj));
    set_param([mdl '/Combustion engine'], 'tank_pressure', num2str(pChamber));
    
    % for faults
    set_param([mdl '/Main Gear Pump'],'displacement', num2str(pumpDisp));

    set_param([mdl '/White Noise_100Hz'],'Cov', num2str(wnAmp));
    set_param([mdl '/White Noise_10Hz'],'Cov', num2str(wnAmp));

    set_param([mdl '/Pressure Relief Valve'],'area_leak', num2str(PRVLeakageArea));

    set_param([mdl '/FMU'],'area_max', num2str(A_FMU));

    set_param([mdl '/Bypass'],'area_max', num2str(A_bypass));

    set_param([mdl '/Tank_Reservoir1'],'reservoir_pressure', num2str(pressure_boost));

    % start simulation
    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter(...
        'StopTime', num2str(tStop), ...
        'Solver', 'daessc', ...
        'SimulationMode', 'normal', ...
        'SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout',...
        'RelTol', '1e-4', ...     
        'AbsTol', '1e-6');

    simout = sim(in);

    % scope data
    signals = struct();
    signalNames = vars;
    for v = 1:length(signalNames)
        sigName  = char(signalNames(v));  
        scopeVar = ['ScopeData_' sigName];
        try
            sigStruct = simout.get(scopeVar);
            ts = timeseries(sigStruct.signals.values, sigStruct.time);
            ts_resampled = resample(ts, tSampleLog);
            dataVec = ts_resampled.Data;
             %---fault injections at signal level
            if fault == 2 && strcmp(sigName, 'p_FMU')
                dataVec = zeros(size(dataVec));  % SensorDrop1 → p_FMU = 0
            elseif fault == 3 && strcmp(sigName, 'Flow_Engine1')
                dataVec = zeros(size(dataVec));  % SensorDrop2 → Flow_Engine1 = 0
            end

            signals.(signalNames(v)) = dataVec;

        catch
            warning('Signal "%s" not found', signalNames(v));
            signals.(signalNames(v)) = NaN(nStep,1); % fill column with NaN if signal name not found
        end
    end

    % write the results
    rowBlock = tSampleLog.';
    for v = 1:length(signalNames)
        sigName = char(signalNames(v));
        rowBlock = [rowBlock signals.(sigName)];
    end
    data(idx:idx+nStep-1,:) = rowBlock;
    idx = idx + nStep;
end

UID1_run = strings(nStep,1);   % Unique Identifier 1 (phase)
UID1_run(1:1001)      = "idle";
UID1_run(1002:1239)   = "ramp up";
UID1_run(1240:nStep)  = "running";

UID2_run = strings(nStep,1);   % Unique Identifier 2 (speed profile)
UID2_run(1:2402)      = "slow1";
UID2_run(2403:3001)   = "accelerating";
UID2_run(3002:3401)   = "fast";
UID2_run(3402:4001)   = "decelerating";
UID2_run(4002:nStep)  = "slow2";

UID1_all = repmat(UID1_run, nsim, 1);
UID2_all = repmat(UID2_run, nsim, 1);


% write the table
T = array2table(data, 'VariableNames', ['t', vars]);
%add identifiers
T.UID1 = UID1_all;
T.UID2 = UID2_all;

fmt = '%.5e';
numericNames = ["t", vars];  % these are numeric

% formatting  of single entries/cells
for v = numericNames
    vn  = v;                         % string name
    col = T.(vn);                    % numeric vector
    T.(vn) = cellfun(@(x) sprintf(fmt,x), num2cell(col), 'UniformOutput', false);
end

% ---------- Controlled fault-based filename & message ----------
switch fault
    case 0
        filename = 'healthyFuelPumpRuns.csv';
        msg = '1000 healthy fuel pump runs generated successfully and saved!';
    case 1
        filename = 'fault01_whiteNoise.csv';
        msg = 'Runs with Fault 01 (White Noise Amplitude) saved.';
    case 2
        filename = 'fault02_sensorDrop_pFMU.csv';
        msg = 'Runs with Fault 02 (Sensor Dropout p_FMU) saved.';
    case 3
        filename = 'fault03_sensorDrop_Engine1.csv';
        msg = 'Runs with Fault 03 (Sensor Dropout Flow_Engine1) saved.';
    case 4
        filename = 'fault04_pumpLeakage.csv';
        msg = 'Runs with Fault 04 (Pump Leakage) saved.';
    case 5
        filename = 'fault05_pumpDisplacement.csv';
        msg = 'Runs with Fault 05 (Displacement Fault) saved.';
    case 6
        filename = 'fault06_PRVLeakage.csv';
        msg = 'Runs with Fault 06 (PRV Leakage) saved.';
    case 7
        filename = 'fault07_orifice_FMU.csv';
        msg = 'Runs with Fault 07 (FMU Orifice Fault) saved.';
    case 8
        filename = 'fault08_orifice_Bypass.csv';
        msg = 'Runs with Fault 08 (Bypass Valve Orifice Fault) saved.';
    case 9
        filename = 'fault09_highBoostPressure.csv';
        msg = 'Runs with Fault 09 (Boost Pressure Too High) saved.';
 
    otherwise
        filename = 'unknownFault.csv';
        msg = 'Fault type not recognized. Data saved as "unknownFault.csv".';
end

% Save
writetable(T, filename, 'WriteVariableNames', true);
disp(msg);