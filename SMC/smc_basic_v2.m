% Sliding Mode Control - Basic Implementation (Clean Version)
clc; clear; close all;

% System parameters
m = 5;      % Mass
b = 4;      % Damping coefficient
k = 3;      % Stiffness

% Simulation parameters
dt = 0.01;
T = 10;
t = 0:dt:T;
N = length(t);

% Reference signal
z_ref = 5 * ones(1, N);

% Initialization
z = zeros(1, N);        % Depth
dz = zeros(1, N);       % Velocity
u = zeros(1, N);        % Control input

% SMC parameters
lambda = 2;
eta = 30;

for i = 1:N-1
    % Error and sliding surface
    e  = z_ref(i) - z(i);
    de = -dz(i);
    s = de + lambda * e;

    % Control law (simplified without u_eq)
    u(i) = - eta * tanh(10 * s);

    % System dynamics
    ddz = (1/m) * (u(i) - b*dz(i) - k*z(i));
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
title('AUV Depth Control using Sliding Mode Control');
grid on;