% Sliding Mode Control with Disturbance
clc; clear; close all;

% System parameters
m = 5;
b = 4;
k = 3;

% Simulation settings
dt = 0.01;
T = 10;
t = 0:dt:T;
N = length(t);

% Reference signal
z_ref = 5 * ones(1, N);

% Initialization
z = zeros(1, N);
dz = zeros(1, N);
u = zeros(1, N);
d = 0.5 * sin(2 * pi * 0.5 * t);  % Disturbance (water current)

% SMC parameters
lambda = 2;
eta = 10;

for i = 1:N-1
    % Error and sliding surface
    e  = z_ref(i) - z(i);
    de = -dz(i);
    s = de + lambda * e;

    % Equivalent control
    u_eq = m * (lambda * de + lambda^2 * e + (b/m)*dz(i) + (k/m)*z(i));

    % Final control law
    u(i) = u_eq - eta * tanh(5 * s);

    % System dynamics with disturbance
    ddz = (1/m) * (u(i) + d(i) - b*dz(i) - k*z(i));
    dz(i+1) = dz(i) + ddz * dt;
    z(i+1)  = z(i)  + dz(i) * dt;
end

% Plotting
figure;
plot(t, z_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, z, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Depth (m)');
legend('Reference', 'AUV Depth');
title('Sliding Mode Control with Disturbance');
grid on;
figure;
plot(t, u, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control input u(t)');
title('Control Signal in SMC with Disturbance');
grid on;