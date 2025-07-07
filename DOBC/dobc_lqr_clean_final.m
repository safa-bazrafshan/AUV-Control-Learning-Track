% Final version: Clean DOBC with LQR control for AUV depth
clc; clear; close all;

%% System parameters
A = [0 1; 0 0];
B = [0; 1];
E = [0; 1]; % disturbance enters similarly to control input
C = [1 0];

%% LQR design
Q = diag([15 2]);
R = 1;
K = lqr(A, B, Q, R);

%% Simulation settings
T = 20;
dt = 0.01;
t = 0:dt:T;
N = length(t);

%% Disturbance definition
d = 0.5 * sin(1.2 * t); % true disturbance

%% Initialization
x = zeros(2, N); % AUV states
x_nom = zeros(2, N); % Nominal model states
y = zeros(1, N);
y_nom = zeros(1, N);
u_nom = zeros(1, N);
d_hat = zeros(1, N);
u_dobc = zeros(1, N);

tau = 1; % observer time constant (adjustable)

%% Simulation loop
for k = 1:N-1
    % Nominal control
    u_nom(k) = -K * x(:,k);

    % Nominal model output (without disturbance)
    dx_nom = A * x_nom(:,k) + B * u_nom(k);
    x_nom(:,k+1) = x_nom(:,k) + dx_nom * dt;
    y_nom(k) = C * x_nom(:,k);

    % Disturbance estimation
    e_obs = C * x(:,k) - y_nom(k);
    d_hat(k+1) = d_hat(k) + (1/tau)*(e_obs - d_hat(k)) * dt;

    % DOBC: apply corrected control
    u_dobc(k) = u_nom(k) - d_hat(k);

    % Real plant with disturbance
    dx = A * x(:,k) + B * u_dobc(k) + E * d(k);
    x(:,k+1) = x(:,k) + dx * dt;

    % Actual output
    y(k) = C * x(:,k);
end

% Final step
y(N) = C * x(:,N);
y_nom(N) = C * x_nom(:,N);

%% Plotting
figure;

subplot(3,1,1)
plot(t, y, 'b', 'LineWidth', 2); hold on;
plot(t, y_nom, 'r--', 'LineWidth', 1.5);
legend('AUV with DOBC', 'Nominal model');
xlabel('Time (s)'); ylabel('Depth (m)');
title('AUV Depth Tracking with LQR + DOBC');
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
title('Control Effort (Nominal vs DOBC)');
grid on;