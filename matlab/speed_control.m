%% Adaptive Driver-Behaviour-Aware Speed Control System
% BECE302L Control Systems, Winter 2025-26
%
% Vehicle plant, fixed P / PI / PID, gain-scheduled (adaptive) PID for three
% driver profiles, state-space pole placement, Bode / Nyquist / root locus,
% disturbance rejection and stepinfo metrics. Produces Figures 1-10 of the
% report and saves them as PNG files in ../results/matlab/.
%
% Requires: Control System Toolbox.

clear; clc; close all;

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'matlab');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

%% System modelling: m dv/dt = u - b v
m = 1200;           % vehicle mass [kg]
b = 50;             % damping (drag + friction) [N s/m]
v_ref = 30;         % reference speed [m/s] (108 km/h)

G = tf([1], [m, b]);                       % G(s) = 1 / (1200 s + 50)
fprintf('Plant: DC gain %.3f, time constant %.1f s, pole %.4f rad/s\n', ...
        dcgain(G), m / b, -b / m);

%% Fixed controllers
Kp_fixed = 100;                            % P
Kp_pi = 120;  Ki_pi = 10;                  % PI
Kp_pid = 150; Ki_pid = 20; Kd_pid = 5;     % PID

C_P   = tf([Kp_fixed], [1]);
C_PI  = tf([Kp_pi, Ki_pi], [1, 0]);
C_PID = tf([Kd_pid, Kp_pid, Ki_pid], [1, 0]);

CL_P   = feedback(C_P * G, 1);
CL_PI  = feedback(C_PI * G, 1);
CL_PID = feedback(C_PID * G, 1);

fprintf('P controller steady-state error: %.3f (%.0f %%)\n', ...
        b / (Kp_fixed + b), 100 * b / (Kp_fixed + b));

%% Adaptive gain scheduling
gains.Smooth     = struct('Kp',  80, 'Ki',  5, 'Kd',  2);
gains.Normal     = struct('Kp', 150, 'Ki', 20, 'Kd',  5);
gains.Aggressive = struct('Kp', 200, 'Ki', 30, 'Kd', 10);

make_CL = @(g) feedback(tf([g.Kd, g.Kp, g.Ki], [1 0]) * G, 1);
CL_smooth = make_CL(gains.Smooth);
CL_normal = make_CL(gains.Normal);
CL_aggr   = make_CL(gains.Aggressive);

%% State space and pole placement
A_ss = -b/m; B_ss = 1/m; C_ss = 1; D_ss = 0;
sys_ss = ss(A_ss, B_ss, C_ss, D_ss);

Co = ctrb(A_ss, B_ss); Ob = obsv(A_ss, C_ss);
fprintf('Controllability rank %d, observability rank %d\n', rank(Co), rank(Ob));

desired_pole = -5;
K_sf = place(A_ss, B_ss, desired_pole);
A_cl = A_ss - B_ss * K_sf;
Nbar = -1 / (C_ss * inv(A_cl) * B_ss);
CL_sf = ss(A_cl, B_ss * Nbar, C_ss, D_ss);
fprintf('State feedback: K = %.0f, A_cl = %.2f, Nbar = %.0f\n', K_sf, A_cl, Nbar);

%% Time-domain metrics
t = 0:0.01:150;
names = {'Fixed P', 'Fixed PI', 'Fixed PID', 'Adapt-Smooth', 'Adapt-Normal', 'Adapt-Aggressive'};
systems = {CL_P, CL_PI, CL_PID, CL_smooth, CL_normal, CL_aggr};

fprintf('\n%-18s %8s %10s %8s %8s\n', 'Controller', 'Rise', 'Settling', 'OS %', 'Peak');
metrics = zeros(numel(systems), 4);
for k = 1:numel(systems)
    S = stepinfo(v_ref * systems{k});
    metrics(k, :) = [S.RiseTime, S.SettlingTime, S.Overshoot, S.PeakTime];
    fprintf('%-18s %8.2f %10.2f %8.2f %8.2f\n', names{k}, metrics(k, :));
end

%% Gain and phase margins
fprintf('\n%-6s %8s %10s %12s\n', 'Ctrl', 'GM (dB)', 'PM (deg)', 'Wgc (rad/s)');
ol = {C_P * G, C_PI * G, C_PID * G};
olnames = {'P', 'PI', 'PID'};
for k = 1:3
    [Gm, Pm, ~, Wpm] = margin(ol{k});
    fprintf('%-6s %8.2f %10.2f %12.4f\n', olnames{k}, 20 * log10(Gm), Pm, Wpm);
end

%% Driver-profile reference signals for Figure 2
r_smooth = min(t / 20, 1) * v_ref;                     % gentle ramp
r_normal = v_ref * ones(size(t));                      % step
r_aggr   = v_ref * ones(size(t));
r_aggr(t >= 20 & t < 22) = 1.25 * v_ref;               % +25 % burst

%% Figure 1: P vs PI vs PID
f1 = figure('Name', 'Fig 1');
[y1, t1] = step(v_ref * CL_P, t);   plot(t1, y1, 'r--', 'LineWidth', 2); hold on;
[y2, t2] = step(v_ref * CL_PI, t);  plot(t2, y2, 'b-.', 'LineWidth', 2);
[y3, t3] = step(v_ref * CL_PID, t); plot(t3, y3, 'g-',  'LineWidth', 2);
yline(v_ref, 'k:'); grid on;
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Step response: P vs PI vs PID (fixed gains)'); legend('P', 'PI', 'PID');

%% Figure 2: adaptive profiles
f2 = figure('Name', 'Fig 2');
plot(t, lsim(CL_smooth, r_smooth, t), 'b', 'LineWidth', 2); hold on;
plot(t, lsim(CL_normal, r_normal, t), 'g', 'LineWidth', 2);
plot(t, lsim(CL_aggr,   r_aggr,   t), 'r', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Adaptive gain-scheduled PID: driver profiles');
legend('Smooth', 'Normal', 'Aggressive');

%% Figure 3: fixed PID vs adaptive (normal)
f3 = figure('Name', 'Fig 3');
plot(t3, y3, 'r--', 'LineWidth', 2.5); hold on;
[yn, tn] = step(v_ref * CL_normal, t); plot(tn, yn, 'b', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Fixed PID vs adaptive PID (normal driver)'); legend('Fixed PID', 'Adaptive PID');

%% Figures 4 and 5: root locus
f4 = figure('Name', 'Fig 4'); rlocus(G);         title('Root locus: plant G(s)');
f5 = figure('Name', 'Fig 5'); rlocus(C_PID * G); title('Root locus: C_{PID}(s) G(s)');

%% Figure 6: Bode
f6 = figure('Name', 'Fig 6');
bode(C_P * G, C_PI * G, C_PID * G); grid on;
legend('C_P G', 'C_{PI} G', 'C_{PID} G'); title('Bode: open-loop transfer functions');

%% Figure 7: Nyquist
f7 = figure('Name', 'Fig 7'); nyquist(C_PID * G); title('Nyquist: C_{PID}(s) G(s)');

%% Figure 8: state feedback vs PID
f8 = figure('Name', 'Fig 8');
plot(t3, y3, 'r--', 'LineWidth', 2); hold on;
[ys, ts] = step(v_ref * CL_sf, t); plot(ts, ys, 'm', 'LineWidth', 2);
grid on; xlim([0 80]); xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Fixed PID vs state feedback (pole at -5)'); legend('Fixed PID', 'State feedback');

%% Figure 9: disturbance rejection
% d enters at the plant input; S(s) = G / (1 + C G).
d_amp = 5;
t_d = 0:0.01:120;
S_P   = feedback(G, C_P);
S_PI  = feedback(G, C_PI);
S_PID = feedback(G, C_PID);

f9 = figure('Name', 'Fig 9');
subplot(1, 2, 1);
[y_d_P, ~]   = impulse(d_amp * S_P,   t_d); plot(t_d, -y_d_P,   'r', 'LineWidth', 2); hold on;
[y_d_PI, ~]  = impulse(d_amp * S_PI,  t_d); plot(t_d, -y_d_PI,  'b', 'LineWidth', 2);
[y_d_PID, ~] = impulse(d_amp * S_PID, t_d); plot(t_d, -y_d_PID, 'g', 'LineWidth', 2);
grid on; xlim([0 80]); xlabel('Time (s)'); ylabel('Speed deviation (m/s)');
title('Impulse disturbance'); legend('P', 'PI', 'PID');
subplot(1, 2, 2);
plot(t_d, -step(d_amp * S_P,   t_d), 'r', 'LineWidth', 2); hold on;
plot(t_d, -step(d_amp * S_PI,  t_d), 'b', 'LineWidth', 2);
plot(t_d, -step(d_amp * S_PID, t_d), 'g', 'LineWidth', 2);
grid on; xlim([0 80]); xlabel('Time (s)'); ylabel('Speed deviation (m/s)');
title('Step (sustained) disturbance'); legend('P', 'PI', 'PID');

%% Figure 10: dashboard
f10 = figure('Name', 'Fig 10', 'Position', [50 50 1500 1000]);
subplot(3,3,1); plot(t1, y1, 'r', t2, y2, 'b', t3, y3, 'g'); grid on; title('P vs PI vs PID');
subplot(3,3,2); plot(t, step(v_ref*CL_smooth, t), 'b', t, yn, 'g', t, step(v_ref*CL_aggr, t), 'r');
                grid on; title('Adaptive profiles');
subplot(3,3,3); plot(t3, y3, 'r--', tn, yn, 'b'); grid on; title('Fixed vs adaptive');
subplot(3,3,4); rlocus(G); title('Root locus G(s)');
[mag, ph, w] = bode(C_PID * G, logspace(-4, 2, 500));
subplot(3,3,5); semilogx(w, 20*log10(squeeze(mag))); grid on; title('Bode magnitude (PID OL)');
subplot(3,3,6); semilogx(w, squeeze(ph)); grid on; title('Bode phase (PID OL)');
subplot(3,3,7); plot(t_d, -step(d_amp*S_P, t_d), 'r', t_d, -step(d_amp*S_PI, t_d), 'b', ...
                     t_d, -step(d_amp*S_PID, t_d), 'g'); grid on; title('Disturbance rejection');
subplot(3,3,8); plot(t3, y3, 'r--', ts, ys, 'm'); grid on; title('PID vs state feedback');
subplot(3,3,9); bar(metrics([1 2 3 4 6], 1:3)); grid on; title('Rise / Settling / OS%');
set(gca, 'XTickLabel', {'P', 'PI', 'PID', 'Smooth', 'Aggr'});
legend('Rise (s)', 'Settling (s)', 'OS (%)');

%% Save figures
figs = {f1, f2, f3, f4, f5, f6, f7, f8, f9, f10};
for k = 1:numel(figs)
    saveas(figs{k}, fullfile(out_dir, sprintf('fig%02d.png', k)));
end
fprintf('\nFigures saved to %s\n', out_dir);
