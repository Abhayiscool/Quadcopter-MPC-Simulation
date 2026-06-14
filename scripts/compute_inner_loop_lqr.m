% --- DJI F450 Attitude Subsystem Control Design ---
Ixx = 0.0094;
Iyy = 0.0100;
Izz = 0.0187;

% Extract A_att (6x6) and B_att (6x3)
A_att = [zeros(3,3), eye(3);
         zeros(3,3), zeros(3,3)];
     
B_att = [zeros(3,3);
         diag([1/Ixx, 1/Iyy, 1/Izz])];

% --- Define LQR Tuning Matrices ---
% Q penalizes errors in: [phi, theta, psi, p, q, r]
Q_diag = [500, 500, 300, 10, 10, 5]; 
Q = diag(Q_diag);

% R penalizes control effort on torques: [U2, U3, U4]
R_diag = [0.1, 0.1, 0.1]; 
R = diag(R_diag);

% --- Compute Optimal Gain Matrix K ---
% Solves the Continuous-time Algebraic Riccati Equation (CARE)
[K_att, S, E] = lqr(A_att, B_att, Q, R);

disp('Optimal Inner Loop LQR Gain Matrix K_att calculated:');
disp(K_att);