% Final Tuned Version - Fuzzy Rule-Based Controller with Output Gain
clc; clear; close all;

% --- AUV System Parameters ---
m = 5;  % mass
b = 4;  % damping
k = 3;  % stiffness

% --- Simulation Settings ---
dt = 0.01;
T = 20;
t = 0:dt:T;
N = length(t);
z_ref = 5 * ones(1, N);  % Desired depth

% Initial states
z = zeros(1, N);     % depth
dz = zeros(1, N);    % velocity
u = zeros(1, N);     % control input

% --- Membership Function (triangular) ---
mf = @(x, a, b, c) max(min((x - a)/(b - a), (c - x)/(c - b)), 0);
centers = linspace(-5, 5, 5);  % 5 MFs: NL, NS, ZE, PS, PL

% --- Tuned Rule Table (stronger control values) ---
rule_table = [ ...
    -20  -16  -12   -8   -4;
    -16  -12   -8   -4    0;
    -12   -8    0    8   12;
     -4    0    8   12   16;
      4    8   12   16   20];

% --- Simulation Loop ---
for i = 1:N-1
    e = z_ref(i) - z(i);   % error
    de = -dz(i);           % delta error

    % Fuzzification
    mu_e = arrayfun(@(c) mf(e, c-2, c, c+2), centers);
    mu_de = arrayfun(@(c) mf(de, c-2, c, c+2), centers);

    % Fuzzy inference (weighted rule output)
    u_sum = 0;
    w_sum = 0;
    for m = 1:5
        for n = 1:5
            w = mu_e(m) * mu_de(n);
            u_rule = rule_table(m, n);
            u_sum = u_sum + w * u_rule;
            w_sum = w_sum + w;
        end
    end

    % Defuzzification with output gain
    gain = 2.5;  % ← Adjustable gain factor
    u(i) = gain * (u_sum / (w_sum + 1e-6));  % Apply gain

    % AUV dynamics update
    ddz = (1/m) * (u(i) - b * dz(i) - k * z(i));
    dz(i+1) = dz(i) + ddz * dt;
    z(i+1)  = z(i) + dz(i+1) * dt;
end

% --- Plotting ---
figure;
plot(t, z_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, z, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Depth (m)');
legend('Reference Depth', 'AUV Depth');
title('AUV Depth Control using Tuned Fuzzy Controller (with Gain)');
grid on;