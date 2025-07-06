% Simulation of AUV depth control using trained fuzzy controller
clc; clear; close all;

% System parameters
m = 5;
b = 4;
k = 3;

% Simulation settings
dt = 0.01;
T = 15;
t = 0:dt:T;
N = length(t);

% Reference depth
z_ref = 5 * ones(1, N);

% Initialization
z = zeros(1, N);
dz = zeros(1, N);
u = zeros(1, N);

% Load trained fuzzy controller
fis = readfis('fuzzy_controller');

for i = 1:N-1
    % Error and delta error
    e = z_ref(i) - z(i);
    de = -dz(i);  % since ref is constant

    % Evaluate control using fuzzy system
    u(i) = evalfis([e de], fis);

    % AUV dynamics
    ddz = (1/m) * (u(i) - b*dz(i) - k*z(i));
    dz(i+1) = dz(i) + ddz * dt;
    z(i+1)  = z(i)  + dz(i) * dt;
end

% Final control value
u(end) = u(end-1);

% Plotting
figure;
plot(t, z_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, z, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Depth (m)');
legend('Reference', 'AUV Depth');
title('AUV Depth Response with Fuzzy Controller');
grid on;