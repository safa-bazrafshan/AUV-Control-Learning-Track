% Train ANFIS-based fuzzy controller for AUV depth control
clc; clear; close all;

% Generate training data
e = linspace(-5, 5, 301);       % Error
de = linspace(-5, 5, 301);      % Delta error
[X1, X2] = meshgrid(e, de);     % 2D grid for input space
E = X1(:);                      % Flattened error
DE = X2(:);                     % Flattened delta error

% Define target output (control signal)
Kp = 3;
Kd = 2;
U = Kp * E + Kd * DE;

% Combine inputs and output
training_data = [E DE U];

% Generate initial FIS with 5 MFs per input
fis = genfis1(training_data, 5);  % Number of MFs = 5

% Train ANFIS
[trained_fis, training_error] = anfis(training_data, fis, 50);

% Save the trained FIS
writefis(trained_fis, 'fuzzy_controller');

% Plot training error
figure;
plot(training_error, 'LineWidth', 1.8);
xlabel('Epoch');
ylabel('RMSE');
title('ANFIS Training Error');
grid on;