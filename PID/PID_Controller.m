% PID Controller for AUV Depth Control
clc; clear; close all;

% System parameters
m = 5;
b = 4;
k = 3;

% PID gains
Kp = 50;
Ki = 5;
Kd = 15;

% Simulation settings
dt = 0.01;
T = 20;
t = 0:dt:T;
N = length(t);

% Reference depth
z_ref = 5 * ones(1, N);

% Initialization
z = zeros(1, N);
v = zeros(1, N);
u = zeros(1, N);
e_int = 0;

for i = 1:N-1
    e = z_ref(i) - z(i); % Error
    e_int = e_int + e * dt; % Integral of error
    de = -v(i); % Derivative of error

    % PID control
    u(i) = Kp * e + Ki * e_int + Kd * de;

    % System dynamics
    a = (1/m) * (u(i) - b * v(i) - k * z(i));
    v(i+1) = v(i) + a * dt;
    z(i+1) = z(i) + v(i+1) * dt;
end

% Plotting
figure;
plot(t, z_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, z, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Depth (m)');
legend('Reference Depth', 'AUV Depth');
title('AUV Depth Control using PID Controller');
grid on;