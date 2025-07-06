% ANFIS training with genfis3 (better performance than genfis1)
clc; clear; close all;

% Generate training data
e = linspace(-5, 5, 41);       % Coarser for fuzzy rules
de = linspace(-5, 5, 41);
[X1, X2] = meshgrid(e, de);
E = X1(:);
DE = X2(:);

% Output control rule (positive to drive toward reference)
Kp = 3;
Kd = 2;
U = Kp * E + Kd * DE;

training_input = [E, DE];
training_output = U;

% Generate FIS with genfis3 (grid partitioning)
fis = genfis3(training_input, training_output, 'sugeno');

% Train ANFIS
[trained_fis, error] = anfis([training_input training_output], fis, ...
                             anfisOptions('EpochNumber', 50));

% Save trained controller
writefis(trained_fis, 'fuzzy_controller');

% Plot training error
figure;
plot(error, 'LineWidth', 1.5);
xlabel('Epoch');
ylabel('RMSE');
title('ANFIS Training Error (genfis3)');
grid on;