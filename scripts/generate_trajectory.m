% --- Trajectory Parameters ---
T_final = 20;       % Flight time (seconds)
Ts = 0.05;          % Sample time matching the MPC (20 Hz)
t = 0:Ts:T_final;   % Time vector

R = 2.0;            % Radius of the helix (meters)
omega = 0.5;        % Angular velocity (rad/s)
vz = 0.5;           % Vertical climb rate (m/s)
z0 = 10.0;          % Starting altitude (meters - matching our hover)

% --- Generate Position ---
x_ref = R * cos(omega * t);
y_ref = R * sin(omega * t);
z_ref = z0 + vz * t;

% --- Generate Velocity ---
vx_ref = -R * omega * sin(omega * t);
vy_ref = R * omega * cos(omega * t);
vz_ref = vz * ones(1, length(t));

% --- Combine into a Reference Matrix ---
% The MPC expects a 6-column matrix: [x, y, z, vx, vy, vz]
ref_data = [x_ref', y_ref', z_ref', vx_ref', vy_ref', vz_ref'];

% --- Create a Simulink Timeseries Object ---
% This object will be read by a "From Workspace" block in Simulink
trajectory_ts = timeseries(ref_data, t);

% --- Plot the Trajectory to Verify ---
figure;
plot3(x_ref, y_ref, z_ref, 'b-', 'LineWidth', 2);
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Altitude Z (m)');
title('Target 3D Helix Trajectory');
disp('Trajectory generated and saved as "trajectory_ts"');