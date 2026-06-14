% main_setup.m
disp('Initializing Quadcopter Simulation Environment...');

% Add subfolders to MATLAB path so it can find the scripts and models
addpath(genpath(pwd));

% 1. Load Parameters and Linearize
disp('Running Step 1: System Linearization...');
run('scripts/setup_linear_model.m');

% 2. Compute Inner-Loop LQR
disp('Running Step 2: LQR Inner-Loop Design...');
run('scripts/compute_inner_loop_lqr.m');

% 3. Compute Outer-Loop MPC
disp('Running Step 3: MPC Outer-Loop Design...');
run('scripts/compute_outer_loop_mpc.m');

% 4. Generate 3D Helix Trajectory
disp('Running Step 4: Trajectory Generation...');
run('scripts/generate_trajectory.m');

disp('Setup Complete! You can now open models/quadr_function.slx and click Run.');