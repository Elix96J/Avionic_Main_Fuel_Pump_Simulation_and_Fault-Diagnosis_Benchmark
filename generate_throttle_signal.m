% generate_throttle_signal.m
% Erstellt Throttle-Profil als Simulink-kompatibles Signal für Signal Editor

% Zeit und Werte (z. B. throttle positions)
time = [10    12    15    17    20    30]';
value = [0.2 0.2 0.9 0.9 0.3 0.3]';

% Erstelle timeseries-Objekt
ts = timeseries(value, time);
ts.Name = 'Throttle';

% Erstelle Dataset-Objekt (für Signal Editor erforderlich)
scenario = Simulink.SimulationData.Dataset;
scenario = addElement(scenario, ts);

% Speichere als .mat-Datei
save('throttle_signal.mat', 'scenario');