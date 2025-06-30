% AUV State-Space Model
% Author: Safa Bazrafshan
% Description: State-space representation and step response of AUV depth dynamics

clc; clear; close all;

% System parameters
m = 5;    % mass (kg)
b = 4;    % damping (N·s/m)
k = 3;    % stiffness (N/m)

% State-space matrices
A = [0      1;
    -k/m  -b/m];

B = [0;
     1/m];

C = [1  0];  % Output is depth (z)
D = 0;

% Build state-space system
sys_ss = ss(A, B, C, D);

eig_A = eig(A);
disp('Eigenvalues of A:');
disp(eig_A);

% Step response
figure;
step(sys_ss);
title('AUV Depth Response (State-Space Model)');
xlabel('Time (s)');
ylabel('Depth (m)');
grid on;

