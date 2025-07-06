% Final version: ANFIS training with genfis3 and basic options
clc; clear; close all;

% Generate training data
e = linspace(-5, 5, 41);
de = linspace(-5, 5, 41);
[X1, X2] = meshgrid(e, de);
E = X1(:);
DE = X2(:);

% Output control rule
Kp = 3;
Kd = 2;
U = Kp * E + Kd * DE;

training_input = [E, DE];
training_output = U;

% Generate FIS using genfis3 (Sugeno-type)
fis = genfis3(training_input, training_output, 'sugeno');

% Train ANFIS (simplified call)
[trained_fis, trainError] = anfis([training_input training_output], fis, 50);

% Save FIS
writefis(trained_fis, 'fuzzy_controller');

% Plot error
figure;
plot(trainError, 'LineWidth', 1.5);
xlabel('Epoch');
ylabel('Training Error (RMSE)');
title('ANFIS Training Error');
grid on;