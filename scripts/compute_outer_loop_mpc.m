% --- DJI F450 Position Subsystem ---
m = 1.250;      
g = 9.81;

% State-space matrices for Position (X, Y, Z, Vx, Vy, Vz)
A_pos = [zeros(3,3), eye(3); 
         zeros(3,3), zeros(3,3)];
     
% Inputs: [Pitch (theta), Roll (phi), delta_Thrust (U1)]
B_pos = [zeros(3,3); 
         g,  0,  0; 
         0, -g,  0; 
         0,  0, 1/m];
     
C_pos = eye(6); % We assume we can measure all positions and velocities
D_pos = zeros(6,3);

sys_pos = ss(A_pos, B_pos, C_pos, D_pos);

% --- MPC Controller Setup ---
Ts = 0.05; % Sample time for outer loop (20 Hz)
sys_discrete = c2d(sys_pos, Ts); % MPC requires a discrete-time model

% Define Horizons
PredictionHorizon = 20; 
ControlHorizon = 5;

% Create MPC Object
mpcobj = mpc(sys_discrete, Ts, PredictionHorizon, ControlHorizon);

% --- Define Physical Constraints (The power of MPC) ---
max_tilt = deg2rad(30); % Do not allow the drone to tilt more than 30 degrees

% Manipulated Variables (MV) Constraints: [Theta, Phi, delta_U1]
mpcobj.MV(1).Min = -max_tilt; 
mpcobj.MV(1).Max = max_tilt;  

mpcobj.MV(2).Min = -max_tilt; 
mpcobj.MV(2).Max = max_tilt;  

% Thrust constraints (Limit to max motor capacity)
max_thrust = 20; % Newtons total
mpcobj.MV(3).Min = -m*g;          % Lower bound (free fall)
mpcobj.MV(3).Max = max_thrust - m*g; % Upper bound (max thrust above hover)

% --- Tuning Weights ---
% Penalize position tracking errors (X, Y, Z heavily weighted)
mpcobj.Weights.OutputVariables = [100, 100, 100, 10, 10, 10]; 

% Penalize aggressive control changes
mpcobj.Weights.ManipulatedVariablesRate = [0.1, 0.1, 0.1];

disp('MPC Object created successfully! Ready for Simulink integration.');