% --- DJI F450 Parameters ---
m = 1.250;      
g = 9.81;       
Ixx = 0.0094;
Iyy = 0.0100;
Izz = 0.0187;

% --- Construct the A Matrix (12x12) ---
A = zeros(12, 12);
A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;     % Velocity to Position
A(4,8) = g;                             % Pitch to X-acceleration
A(5,7) = -g;                            % Roll to Y-acceleration
A(7,10) = 1; A(8,11) = 1; A(9,12) = 1;  % Angular rates to Euler angles

% --- Construct the B Matrix (12x4) ---
B = zeros(12, 4);
B(6,1) = 1/m;                           % Thrust to Z-acceleration
B(10,2) = 1/Ixx;                        % Roll torque to Roll accel
B(11,3) = 1/Iyy;                        % Pitch torque to Pitch accel
B(12,4) = 1/Izz;                        % Yaw torque to Yaw accel

% --- Construct C and D Matrices ---
% We assume we have sensors to measure all 12 states (e.g., via a Kalman Filter)
C = eye(12);    
D = zeros(12, 4);

% --- Create State-Space Object ---
sys_quad = ss(A, B, C, D);
sys_quad.StateName = {'x','y','z','x_dot','y_dot','z_dot','phi','theta','psi','p','q','r'};
sys_quad.InputName = {'delta_U1','U2','U3','U4'};

disp('Linear state-space model created successfully!');