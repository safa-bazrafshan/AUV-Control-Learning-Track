% LQR Tuning Experiment
clc; clear; close all;

m = 5; b = 4; k = 3;
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

Q_cases = {
    [1 0; 0 1], ...
    [100 0; 0 1], ...
    [1 0; 0 100], ...
    [100 0; 0 100]
};
R_cases = [1, 10, 0.1, 0.01];

for i = 1:length(Q_cases)
    Q = Q_cases{i};
    R = R_cases(i);
    K = lqr(A, B, Q, R);
    A_cl = A - B*K;
    sys_cl = ss(A_cl, B, C, D);
    
    fprintf('--- Case %d ---\n', i);
    disp(Q); disp(R); disp(K);

    figure(i);
    step(sys_cl, 5);
    title(sprintf('Step Response - Q=%.0f,%.0f R=%.2f', Q(1), Q(4), R));
    xlabel('Time (s)'); ylabel('Depth (m)'); grid on;
end