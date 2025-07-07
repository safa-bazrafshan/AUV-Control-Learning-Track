% DOBC with LQR (Corrected) for AUV Depth Control
clc; clear; close all;

%% System Parameters
A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
E = [0; 1]; % Disturbance enters like control input
x0 = [0; 0];

%% LQR Controller
Q = diag([10 1]);
R = 1;
K = lqr(A, B, Q, R);

%% Simulation settings
dt = 0.01;
T = 20;
t = 0:dt:T;
N = length(t);

%% Disturbance signal
d = 0.5 * sin(0.5 * t); % True disturbance

%% Initialization
x = zeros(2, N); % Plant with DOBC
x_nom = zeros(2, N); % Nominal plant (for observer)
y = zeros(1, N);
y_nom = zeros(1, N);
u_nom = zeros(1, N);
u_dobc = zeros(1, N);
d_hat = zeros(1, N);

tau = 0.5; % Observer time constant (LPF)

%% Simulation loop
for k = 1:N-1
    % Nominal LQR control
    u_nom(k) = -K * x(:,k);

    % Nominal model output (no disturbance)
    dx_nom = A * x_nom(:,k) + B * u_nom(k);
    x_nom(:,k+1) = x_nom(:,k) + dx_nom * dt;
    y_nom(k) = C * x_nom(:,k);

    % Real plant output with true disturbance
    dx = A * x(:,k) + B * u_nom(k) + E * d(k);
    x(:,k+1) = x(:,k) + dx * dt;
    y(k) = C * x(:,k);

    % Disturbance observer (LPF)
    error = y(k) - y_nom(k);
    d_hat(k+1) = d_hat(k) + (1/tau)*(error - d_hat(k))*dt;

    % DOBC input (new)
    u_dobc(k) = u_nom(k) - d_hat(k);

    % Apply DOBC input again to plant (overwrite)
    dx = A * x(:,k) + B * u_dobc(k) + E * d(k);
    x(:,k+1) = x(:,k) + dx * dt;
end

%% Plot results
figure;

subplot(3,1,1)
plot(t, y, 'b', 'LineWidth', 2); hold on;
plot(t, y_nom, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Depth (m)');
legend('AUV with DOBC', 'Nominal Model');
title('AUV Depth Tracking with LQR + DOBC (Corrected)');
grid on;

subplot(3,1,2)
plot(t, d, 'k', 'LineWidth', 1.5); hold on;
plot(t, d_hat, 'g--', 'LineWidth', 1.5);
legend('True d(t)', 'Estimated \^d(t)');
xlabel('Time (s)'); ylabel('Disturbance');
title('Disturbance Estimation');
grid on;

subplot(3,1,3)
plot(t, u_nom, 'r', 'LineWidth', 1.2); hold on;
plot(t, u_dobc, 'b', 'LineWidth', 1.2);
legend('Nominal u', 'DOBC u');
xlabel('Time (s)'); ylabel('Control Input');
title('Control Effort with and without DOBC');
grid on;