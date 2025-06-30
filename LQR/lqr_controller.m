% LQR Controller Basic Design
clc; clear; close all;

m = 5; b = 4; k = 3;
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

Q = C'*C;  % Same as [1 0; 0 0]
R = 1;

K = lqr(A, B, Q, R);
disp('LQR gain matrix K:');
disp(K);

Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);
step(sys_cl);
title('AUV Depth Response with LQR Controller');
xlabel('Time (s)');
ylabel('Depth (m)');
grid on;