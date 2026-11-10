%% Myoelectric gripper - system identification and PI design
% Identifies the servo command response to a sustained muscle contraction
% (treated as a step input) and designs a PI controller with the modulus
% optimum criterion.
clear; clc; close all;

%% Load recorded data (LabVIEW export, ~100 Hz)
data = readmatrix('../data/servo_step.txt');

t_full   = data(:,1);   % time [s]
dir_full = data(:,2);   % servo direction (+1 / -1)
cmd_full = data(:,4);   % servo command [V]

%% Extract the transient: from contraction onset until the command saturates at 5 V
idx_start = find(t_full >= 31.5 & dir_full == 1 & cmd_full < 0.1, 1, 'first');
idx_end   = find(t_full > t_full(idx_start) & cmd_full >= 4.99, 1, 'first');

y_step = cmd_full(idx_start:idx_end);
N      = length(y_step);
Ts     = 0.01;
t_step = (0:N-1)' * Ts;
u_step = ones(N, 1);

%% Candidate 1: pure integrator (slope of a linear fit)
p = polyfit(t_step, y_step, 1);
fprintf('Integrator slope K = %.4f V/s\n', p(1));

%% Candidate 2: first-order model with tfest
data_id = iddata(y_step, u_step, Ts);
H_proces = tfest(data_id, 1, 0, 'Ts', 0)

y_model = lsim(H_proces, u_step, t_step);
figure('Name', 'Identification');
plot(t_step, y_step, 'b-', 'LineWidth', 1.8); hold on;
plot(t_step, y_model, 'r-', 'LineWidth', 1.8); hold off;
grid on; box on;
xlabel('Time [s]'); ylabel('Servo command [V]');
title('Step response: measured vs. first-order model');
legend('Measured', 'tfest model', 'Location', 'southeast');

%% Open-loop process gain and time constant
[num, den] = tfdata(H_proces, 'v');
Kp_proc = num(end) / den(end);   % static gain
T_proc  = 1 / den(end);          % time constant [s]
fprintf('Process: K = %.3f, T = %.3f s\n', Kp_proc, T_proc);

%% Unity feedback without controller (shows steady-state error)
T_BD = feedback(H_proces, 1);
stepinfo(T_BD)

%% PI controller - modulus optimum
Tsigma = 0.05;                        % sum of neglected small time constants [s]
Ti = T_proc;                          % controller zero cancels the process pole
Kp = 1 / (2 * Kp_proc * Tsigma);
Ki = Kp / Ti;
fprintf('PI: Kp = %.3f, Ti = %.3f s, Ki = %.3f\n', Kp, Ti, Ki);

H_reg = tf([Kp Ki], [1 0]);
T_BI  = feedback(H_reg * H_proces, 1);   % closed loop
T_u   = feedback(H_reg, H_proces);       % reference -> controller output
stepinfo(T_BI)

figure('Name', 'Closed loop with PI'); step(T_BI); grid on;
figure('Name', 'Controller output, r = 4.5 V'); step(4.5 * T_u); grid on;
