% Sliding Mode Control - Basic Implementation
clc; clear; close all;

% System parameters
m = 5;     % Mass
b = 4;     % Damping
k = 3;     % Stiffness

% Simulation settings
dt = 0.01;                   % Time step
T = 10;                      % Total simulation time
t = 0:dt:T;
N = length(t);

% Desired depth
z_ref = 5 * ones(1, N);

% Initialization
z = zeros(1, N);     % Position
dz = zeros(1, N);    % Velocity
u = zeros(1, N);     % Control input

% SMC parameters
lambda = 2;       % Sliding surface gain
eta = 10;         % Reaching gain

for i = 1:N-1
    % Error and sliding surface
    e = z_ref(i) - z(i);
    de = -dz(i);
    s = de + lambda * e;

    % Equivalent control (estimated)
    u_eq = m * (lambda * de - k*z(i)/m - b*dz(i)/m);

    % Final control law
    u(i) = u_eq - eta * tanh(10 * s);  % smoothed sign function

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