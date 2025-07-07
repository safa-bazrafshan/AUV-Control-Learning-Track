% DOBC with LQR - Final Version
clc; clear; close all;

%% System Dynamics (2nd-order AUV depth model)
A = [0 1; 0 0];
B = [0; 1];
E = [0; 1]; % disturbance enters same as control
C = [1 0];

%% LQR Controller
Q = diag([10, 2]);
R = 1;
K = lqr(A, B, Q, R);

%% Simulation Settings
T = 20;
dt = 0.01;
t = 0:dt:T;
N = length(t);

%% Disturbance: sinusoidal current
d = 0.5 * sin(1.2 * t);

%% Initialization
x = zeros(2, N); % AUV states
x_nom = zeros(2, N); % Nominal model (no disturbance)
y = zeros(1, N); y_nom = zeros(1, N);
u_nom = zeros(1, N); u_dobc = zeros(1, N);
d_hat = zeros(1, N);

%% DOBC settings
tau = 0.5; % time constant of LPF

%% Simulation Loop
for k = 1:N-1
    % Nominal LQR Control
    u_nom(k) = -K * x(:,k);

    % Nominal model
    dx_nom = A * x_nom(:,k) + B * u_nom(k);
    x_nom(:,k+1) = x_nom(:,k) + dx_nom * dt;
    y_nom(k) = C * x_nom(:,k);

    % Real plant with true disturbance
    dx = A * x(:,k) + B * u_nom(k) + E * d(k);
    x(:,k+1) = x(:,k) + dx * dt;
    y(k) = C * x(:,k);

    % Disturbance Estimation (DO)
    e_obs = y(k) - y_nom(k);
    d_hat(k+1) = d_hat(k) + (1/tau)*(e_obs - d_hat(k)) * dt;

    % Apply DOBC
    u_dobc(k) = u_nom(k) - d_hat(k);

    % Apply updated input to actual plant
    dx = A * x(:,k) + B * u_dobc(k) + E * d(k);
    x(:,k+1) = x(:,k) + dx * dt;
end

%% Final update for last point
y(N) = C * x(:,N);
y_nom(N) = C * x_nom(:,N);

%% Plotting
figure;
subplot(3,1,1)
plot(t, y, 'b', 'LineWidth', 2); hold on;
plot(t, y_nom, 'r--', 'LineWidth', 1.5);
legend('AUV (DOBC)', 'Nominal Model');
xlabel('Time (s)'); ylabel('Depth (m)');
title('AUV Depth Tracking with LQR + DOBC (Final)');
grid on;

subplot(3,1,2)
plot(t, d, 'k', 'LineWidth', 1.5); hold on;
plot(t, d_hat, 'g--', 'LineWidth', 1.5);
legend('True Disturbance', 'Estimated Disturbance');
xlabel('Time (s)'); ylabel('d(t)');
title('Disturbance Estimation via Observer');
grid on;

subplot(3,1,3)
plot(t, u_nom, 'r', 'LineWidth', 1.2); hold on;
plot(t, u_dobc, 'b', 'LineWidth', 1.2);
legend('Nominal Control', 'DOBC Control');
xlabel('Time (s)'); ylabel('u(t)');
title('Control Effort Comparison');
grid on;