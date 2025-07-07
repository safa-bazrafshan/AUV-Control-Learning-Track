% MRAC for AUV Depth Control - Improved Version
clc; clear; close all;

%% Parameters
% Reference model: z'' + 2*z' + z = r
a_m1 = 2;  % damping
a_m0 = 1;  % stiffness

% Adaptive gain (smaller for smoother adaptation)
gamma = 3;

% Damping for plant (simulate friction in water)
damping = 2;

% Time settings
dt = 0.01;
T = 20;
t = 0:dt:T;
N = length(t);

% Reference input
r = 5 * ones(1, N);

% Initialize states
z = zeros(1, N);      % plant position
v = zeros(1, N);      % plant velocity
z_m = zeros(1, N);    % model position
v_m = zeros(1, N);    % model velocity
theta = [0; 0];       % adaptive parameters

% Data storage
u = zeros(1, N);
theta_hist = zeros(2, N);

%% Simulation loop
for i = 1:N-1
    % Error
    e = z(i) - z_m(i);

    % Regressor
    phi = [r(i); -v(i)];

    % Control law
    u(i) = theta' * phi;

    % Plant dynamics with damping
    a = -damping * v(i) + u(i);
    v(i+1) = v(i) + a * dt;
    z(i+1) = z(i) + v(i+1) * dt;

    % Reference model
    a_m = -a_m1 * v_m(i) - a_m0 * z_m(i) + a_m0 * r(i);
    v_m(i+1) = v_m(i) + a_m * dt;
    z_m(i+1) = z_m(i) + v_m(i+1) * dt;

    % Adaptive law
    theta_dot = -gamma * phi * e;
    theta = theta + theta_dot * dt;
    theta_hist(:, i+1) = theta;
end

%% Plotting
figure;
subplot(2,1,1)
plot(t, z, 'b', 'LineWidth', 2); hold on;
plot(t, z_m, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Depth (m)');
legend('AUV', 'Reference Model');
title('AUV Depth Tracking using MRAC (Improved)');
grid on;

subplot(2,1,2)
plot(t, theta_hist(1,:), 'g', 'LineWidth', 1.5); hold on;
plot(t, theta_hist(2,:), 'm', 'LineWidth', 1.5);
legend('\theta_1','\theta_2');
xlabel('Time (s)'); ylabel('Adaptive Parameters');
title('MRAC Parameter Convergence');
grid on;