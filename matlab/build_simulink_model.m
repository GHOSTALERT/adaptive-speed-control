%% Build the Simulink model adaptive_speed_control.slx
% Three parallel closed loops (Smooth, Normal, Aggressive PID) around the
% plant 1/(1200 s + 50), all fed by a 30 m/s step and shown on one scope,
% plus a fourth loop whose PID gains are chosen by a Multiport Switch from
% a "profile" constant (1 = Smooth, 2 = Normal, 3 = Aggressive).
%
% Run from the matlab/ folder:   build_simulink_model
% Then:                          sim('adaptive_speed_control')

mdl = 'adaptive_speed_control';
if bdIsLoaded(mdl); close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'StopTime', '150', 'Solver', 'ode45', 'MaxStep', '0.05');

profiles = {'Smooth', 80, 5, 2; 'Normal', 150, 20, 5; 'Aggressive', 200, 30, 10};

add_block('simulink/Sources/Step', [mdl '/Reference'], ...
          'Time', '0', 'Before', '0', 'After', '30', 'Position', [30 230 60 260]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], ...
          'Inputs', '3', 'Position', [720 120 725 300]);
add_block('simulink/Sinks/Scope', [mdl '/Speed Scope'], 'Position', [780 195 810 225]);
add_line(mdl, 'Mux/1', 'Speed Scope/1');
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace'], ...
          'VariableName', 'v_profiles', 'SaveFormat', 'Timeseries', 'Position', [780 260 850 290]);
add_line(mdl, 'Mux/1', 'To Workspace/1');

% Three fixed-gain loops
for k = 1:3
    [name, kp, ki, kd] = profiles{k, :};
    y0 = 60 + (k - 1) * 120;
    sum_blk = sprintf('Error %s', name);
    pid_blk = sprintf('PID %s', name);
    plant_blk = sprintf('Plant %s', name);

    add_block('simulink/Math Operations/Sum', [mdl '/' sum_blk], ...
              'Inputs', '|+-', 'IconShape', 'round', 'Position', [150 y0 170 y0+20]);
    add_block('simulink/Continuous/PID Controller', [mdl '/' pid_blk], ...
              'P', num2str(kp), 'I', num2str(ki), 'D', num2str(kd), 'N', '1000', ...
              'Position', [230 y0-10 300 y0+30]);
    add_block('simulink/Continuous/Transfer Fcn', [mdl '/' plant_blk], ...
              'Numerator', '[1]', 'Denominator', '[1200 50]', 'Position', [370 y0-10 470 y0+30]);

    add_line(mdl, 'Reference/1', [sum_blk '/1'], 'autorouting', 'on');
    add_line(mdl, [sum_blk '/1'], [pid_blk '/1']);
    add_line(mdl, [pid_blk '/1'], [plant_blk '/1']);
    add_line(mdl, [plant_blk '/1'], sprintf('Mux/%d', k), 'autorouting', 'on');
    add_line(mdl, [plant_blk '/1'], [sum_blk '/2'], 'autorouting', 'on');
end

% Selectable loop: gains picked by a Multiport Switch
y0 = 450;
add_block('simulink/Sources/Constant', [mdl '/Profile'], ...
          'Value', '2', 'Position', [150 y0+120 180 y0+140]);
add_block('simulink/Math Operations/Sum', [mdl '/Error Selected'], ...
          'Inputs', '|+-', 'IconShape', 'round', 'Position', [150 y0 170 y0+20]);
add_line(mdl, 'Reference/1', 'Error Selected/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Multiport Switch', [mdl '/Profile Switch'], ...
          'Inputs', '3', 'Position', [470 y0-20 500 y0+140]);
add_line(mdl, 'Profile/1', 'Profile Switch/1', 'autorouting', 'on');
for k = 1:3
    [name, kp, ki, kd] = profiles{k, :};
    blk = sprintf('PID Sel %s', name);
    add_block('simulink/Continuous/PID Controller', [mdl '/' blk], ...
              'P', num2str(kp), 'I', num2str(ki), 'D', num2str(kd), 'N', '1000', ...
              'Position', [260 y0-30+(k-1)*55 330 y0+5+(k-1)*55]);
    add_line(mdl, 'Error Selected/1', [blk '/1'], 'autorouting', 'on');
    add_line(mdl, [blk '/1'], sprintf('Profile Switch/%d', k + 1), 'autorouting', 'on');
end
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant Selected'], ...
          'Numerator', '[1]', 'Denominator', '[1200 50]', 'Position', [560 y0+40 660 y0+80]);
add_line(mdl, 'Profile Switch/1', 'Plant Selected/1', 'autorouting', 'on');
add_line(mdl, 'Plant Selected/1', 'Error Selected/2', 'autorouting', 'on');
add_block('simulink/Sinks/Scope', [mdl '/Selected Scope'], 'Position', [720 y0+45 750 y0+75]);
add_line(mdl, 'Plant Selected/1', 'Selected Scope/1', 'autorouting', 'on');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), [mdl '.slx']));
fprintf('Saved %s.slx. Change the Profile constant (1/2/3) to switch gains.\n', mdl);
