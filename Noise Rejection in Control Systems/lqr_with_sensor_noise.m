% LQR Control of AUV with Sensor Noise
clc; clear; close all;

%% System parameters
A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
x0 = [0; 0];

%% LQR design
Q = diag([10 2]);
R = 1;
K = lqr(A, B, Q, R);

%% Simulation settings
T = 20;
dt = 0.01;
t = 0:dt:T;
N = length(t);

%% Noise settings
noise_std = 0.3; % standard deviation of sensor noise

%% Initialization
x = zeros(2, N);
y = zeros(1, N);
y_noisy = zeros(1, N);
u = zeros(1, N);

%% Simulation loop
for k = 1:N-1
    % Output with noise
    y(k) = C * x(:,k);
    noise = noise_std * randn(); % Gaussian noise
    y_noisy(k) = y(k) + noise;

    % LQR control using noisy output
    x_hat = [y_noisy(k); x(2,k)]; % we assume we can measure velocity accurately
    u(k) = -K * x_hat;

    % State update (no disturbance)
    dx = A * x(:,k) + B * u(k);
    x(:,k+1) = x(:,k) + dx * dt;
end
y(N) = C * x(:,N);
y_noisy(N) = y(N) + noise_std * randn();

%% Plot results
figure;
subplot(2,1,1)
plot(t, y, 'b', 'LineWidth', 2); hold on;
plot(t, y_noisy, 'r--', 'LineWidth', 1);
legend('True Depth', 'Noisy Depth');
xlabel('Time (s)'); ylabel('Depth (m)');
title('AUV Depth Tracking with Sensor Noise');
grid on;

subplot(2,1,2)
plot(t, u, 'k', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Control Input');
title('Control Input Affected by Sensor Noise');
grid on;