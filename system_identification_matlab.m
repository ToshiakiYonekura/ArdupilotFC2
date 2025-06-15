%% Hexacopter System Identification and Control Design
% MATLAB/Simulink implementation for 6-motor ArduCopter
% Based on physical parameters and control theory

clear all; close all; clc;

%% Physical Parameters (System Identification Results)
% Vehicle specifications
m = 2.5;            % Mass (kg)
L = 0.25;           % Arm length (m)
g = 9.81;           % Gravity (m/s²)

% Moments of inertia (kg⋅m²) - estimated from geometry
Ixx = 0.0347;       % Roll inertia
Iyy = 0.0347;       % Pitch inertia  
Izz = 0.0617;       % Yaw inertia

% Motor parameters
kT = 3.16e-6;       % Thrust coefficient (N⋅s²)
kM = 7.94e-8;       % Torque coefficient (N⋅m⋅s²)
tau_motor = 0.15;   % Motor time constant (s)
T_max = 15;         % Max thrust per motor (N)

% Drag coefficients
Cd_x = 0.25;        % X-axis drag coefficient (N⋅s²/m²)
Cd_y = 0.25;        % Y-axis drag coefficient
Cd_z = 0.30;        % Z-axis drag coefficient

%% Motor Mixing Matrix for Hexacopter X-Configuration
% Motor positions (degrees from forward):
% M1: 30°(CCW), M2: 90°(CW), M3: 150°(CCW), M4: 210°(CW), M5: 270°(CCW), M6: 330°(CW)

motor_angles = [30, 90, 150, 210, 270, 330] * pi/180;  % Convert to radians
motor_directions = [1, -1, 1, -1, 1, -1];              % CCW=+1, CW=-1

% Mixing matrix: [Roll, Pitch, Yaw, Throttle] for each motor
M = zeros(6, 4);
for i = 1:6
    M(i, 1) = sin(motor_angles(i));                % Roll factor
    M(i, 2) = cos(motor_angles(i));                % Pitch factor  
    M(i, 3) = motor_directions(i);                 % Yaw factor
    M(i, 4) = 1;                                   % Throttle factor
end

fprintf('Motor Mixing Matrix:\n');
fprintf('     Roll   Pitch   Yaw   Throttle\n');
for i = 1:6
    fprintf('M%d: %6.3f %6.3f %6.3f %6.3f\n', i, M(i,1), M(i,2), M(i,3), M(i,4));
end

%% Linearized State-Space Model (Hover Condition)
% State vector: x = [p, q, r, φ, θ, ψ, u, v, w, xN, yE, zD]'
% Input vector: u = [τx, τy, τz, T]' (torques and total thrust)

% System matrices
A = zeros(12, 12);
B = zeros(12, 4);

% Angular dynamics: [p_dot; q_dot; r_dot] = inv(I) * [τx; τy; τz]
A(1:3, 1:3) = [-1/tau_motor, 0, 0;              % Motor dynamics
               0, -1/tau_motor, 0;
               0, 0, -1/tau_motor];

% Kinematic equations: [φ_dot; θ_dot; ψ_dot] = [p; q; r]
A(4:6, 1:3) = eye(3);

% Translational dynamics (linearized around hover)
A(7, 5) = -g;           % u_dot = -g*θ
A(8, 4) = g;            % v_dot = g*φ
% w_dot has no coupling in linearized hover model

% Position dynamics: [xN_dot; yE_dot; zD_dot] = [u; v; w]
A(10:12, 7:9) = eye(3);

% Input matrix
B(1, 1) = L/Ixx;        % Roll torque
B(2, 2) = L/Iyy;        % Pitch torque
B(3, 3) = 1/Izz;        % Yaw torque
B(9, 4) = -1/m;         % Thrust (NED frame, negative for upward)

% Output matrix (all states measurable)
C = eye(12);
D = zeros(12, 4);

% Create state-space system
sys_lin = ss(A, B, C, D);
sys_lin.StateName = {'p', 'q', 'r', 'phi', 'theta', 'psi', 'u', 'v', 'w', 'xN', 'yE', 'zD'};
sys_lin.InputName = {'tau_x', 'tau_y', 'tau_z', 'T'};

fprintf('\nLinearized System Properties:\n');
fprintf('System is stable: %s\n', isstable(sys_lin));
fprintf('System poles: ');
fprintf('%.3f ', real(pole(sys_lin)));
fprintf('\n');

%% Control Design - PID Attitude Controller

% Rate controller gains (inner loop) - based on system ID
Kp_rate = [0.135, 0.135, 0.18];     % P gains for [roll, pitch, yaw] rates
Ki_rate = [0.135, 0.135, 0.018];    % I gains
Kd_rate = [0.0036, 0.0036, 0.0];    % D gains

% Attitude controller gains (outer loop)
Kp_att = [4.5, 4.5, 4.5];           % P gains for [roll, pitch, yaw] angles

% Create PID controllers
pid_roll_rate = pid(Kp_rate(1), Ki_rate(1), Kd_rate(1));
pid_pitch_rate = pid(Kp_rate(2), Ki_rate(2), Kd_rate(2));
pid_yaw_rate = pid(Kp_rate(3), Ki_rate(3), Kd_rate(3));

fprintf('\nPID Controller Gains:\n');
fprintf('Roll Rate:  Kp=%.3f, Ki=%.3f, Kd=%.6f\n', Kp_rate(1), Ki_rate(1), Kd_rate(1));
fprintf('Pitch Rate: Kp=%.3f, Ki=%.3f, Kd=%.6f\n', Kp_rate(2), Ki_rate(2), Kd_rate(2));
fprintf('Yaw Rate:   Kp=%.3f, Ki=%.3f, Kd=%.6f\n', Kp_rate(3), Ki_rate(3), Kd_rate(3));

%% LQR Controller Design (Alternative Advanced Control)

% Define cost matrices for LQR
Q = diag([100, 100, 50, 10, 10, 10, 1, 1, 1, 1, 1, 1]);  % State weights
R = diag([10, 10, 10, 1]);                                % Control weights

% Calculate LQR gains
[K_lqr, S, P] = lqr(A, B, Q, R);

fprintf('\nLQR Controller Gains:\n');
fprintf('K_lqr = \n');
disp(K_lqr);

% Closed-loop system with LQR
sys_cl_lqr = ss(A - B*K_lqr, B, C, D);
fprintf('LQR closed-loop is stable: %s\n', isstable(sys_cl_lqr));

%% Velocity Control Design

% Position controller gains
Kp_pos_xy = 1.0;        % Horizontal position P gain
Kp_pos_z = 1.0;         % Vertical position P gain

% Velocity controller gains  
Kp_vel_xy = 2.0;        % Horizontal velocity P gain
Ki_vel_xy = 1.0;        % Horizontal velocity I gain
Kd_vel_xy = 0.5;        % Horizontal velocity D gain

Kp_vel_z = 5.0;         % Vertical velocity P gain
Ki_vel_z = 2.0;         % Vertical velocity I gain

fprintf('\nVelocity Controller Gains:\n');
fprintf('Position XY: Kp=%.1f\n', Kp_pos_xy);
fprintf('Position Z:  Kp=%.1f\n', Kp_pos_z);
fprintf('Velocity XY: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', Kp_vel_xy, Ki_vel_xy, Kd_vel_xy);
fprintf('Velocity Z:  Kp=%.1f, Ki=%.1f\n', Kp_vel_z, Ki_vel_z);

%% EKF Parameters

% Process noise parameters (based on system ID)
sigma_gyro = 0.015;     % Gyro process noise (rad/s)
sigma_accel = 0.25;     % Accel process noise (m/s²)
sigma_gyro_bias = 1e-5; % Gyro bias process noise
sigma_accel_bias = 0.002; % Accel bias process noise

% Measurement noise parameters
sigma_gps_pos = 0.3;    % GPS position noise (m)
sigma_gps_vel = 0.3;    % GPS velocity noise (m/s)
sigma_baro = 0.6;       % Barometer noise (m)
sigma_mag = 0.05;       % Magnetometer noise (gauss)

fprintf('\nEKF Noise Parameters:\n');
fprintf('Gyro process noise:    %.3f rad/s\n', sigma_gyro);
fprintf('Accel process noise:   %.2f m/s²\n', sigma_accel);
fprintf('GPS position noise:    %.1f m\n', sigma_gps_pos);
fprintf('GPS velocity noise:    %.1f m/s\n', sigma_gps_vel);

%% Simulation and Analysis

% Time vector for simulation
t = 0:0.01:10;          % 10 second simulation
dt = 0.01;

% Step response analysis
figure(1);
subplot(2,2,1);
step(sys_lin(4,1), t);  % Roll angle response to roll torque
title('Roll Angle Response to Roll Torque');
grid on;

subplot(2,2,2);
step(sys_lin(5,2), t);  % Pitch angle response to pitch torque
title('Pitch Angle Response to Pitch Torque');
grid on;

subplot(2,2,3);
step(sys_lin(6,3), t);  % Yaw angle response to yaw torque
title('Yaw Angle Response to Yaw Torque');
grid on;

subplot(2,2,4);
step(sys_lin(9,4), t);  % Vertical velocity response to thrust
title('Vertical Velocity Response to Thrust');
grid on;

%% Motor Saturation Analysis

% Calculate maximum available torques
T_hover = m * g / 6;    % Thrust per motor at hover
T_available = T_max - T_hover;  % Available thrust for control

% Maximum torques
tau_max_roll = T_available * L * 3;    % 3 motors contribute to roll
tau_max_pitch = tau_max_roll;          % Same for pitch
tau_max_yaw = T_available * kM/kT * 6;  % All 6 motors contribute to yaw

fprintf('\nMotor Saturation Analysis:\n');
fprintf('Hover thrust per motor: %.2f N\n', T_hover);
fprintf('Available thrust per motor: %.2f N\n', T_available);
fprintf('Maximum roll torque: %.3f N⋅m\n', tau_max_roll);
fprintf('Maximum pitch torque: %.3f N⋅m\n', tau_max_pitch);
fprintf('Maximum yaw torque: %.3f N⋅m\n', tau_max_yaw);

%% Frequency Response Analysis

% Bode plot of attitude dynamics
figure(2);
subplot(2,2,1);
bode(sys_lin(4,1), {0.1, 100});  % Roll transfer function
title('Roll Dynamics Bode Plot');
grid on;

subplot(2,2,2);
bode(sys_lin(5,2), {0.1, 100});  % Pitch transfer function
title('Pitch Dynamics Bode Plot');
grid on;

subplot(2,2,3);
bode(sys_lin(6,3), {0.1, 100});  % Yaw transfer function
title('Yaw Dynamics Bode Plot');
grid on;

% Closed-loop analysis with PID controller
s = tf('s');
G_roll = sys_lin(4,1);
C_roll_rate = pid2(Kp_rate(1), Ki_rate(1), Kd_rate(1));
C_roll_att = Kp_att(1);

% Simplified closed-loop (rate loop)
T_roll_cl = feedback(C_roll_rate * G_roll, 1);

subplot(2,2,4);
step(T_roll_cl, t);
title('Closed-Loop Roll Step Response');
grid on;

%% Trajectory Generation Example

% 5th order polynomial trajectory
t_traj = 10;            % Trajectory time (s)
pos_start = [0; 0; 0];  % Start position (m)
pos_end = [5; 3; -2];   % End position (m)

% Polynomial coefficients for minimum snap trajectory
% Boundary conditions: p(0)=p0, p'(0)=0, p''(0)=0, p(T)=pf, p'(T)=0, p''(T)=0

A_traj = [1   0   0   0   0   0;       % p(0) = p0
          0   1   0   0   0   0;       % p'(0) = 0
          0   0   2   0   0   0;       % p''(0) = 0
          1   t_traj   t_traj^2   t_traj^3   t_traj^4   t_traj^5;     % p(T) = pf
          0   1   2*t_traj   3*t_traj^2   4*t_traj^3   5*t_traj^4;   % p'(T) = 0
          0   0   2   6*t_traj   12*t_traj^2   20*t_traj^3];         % p''(T) = 0

% Solve for each axis
traj_coeffs = zeros(6, 3);
for i = 1:3
    b_traj = [pos_start(i); 0; 0; pos_end(i); 0; 0];
    traj_coeffs(:,i) = A_traj \ b_traj;
end

% Generate trajectory
t_sim = 0:dt:t_traj;
pos_traj = zeros(length(t_sim), 3);
vel_traj = zeros(length(t_sim), 3);
acc_traj = zeros(length(t_sim), 3);

for k = 1:length(t_sim)
    t_k = t_sim(k);
    t_powers = [1; t_k; t_k^2; t_k^3; t_k^4; t_k^5];
    t_powers_vel = [0; 1; 2*t_k; 3*t_k^2; 4*t_k^3; 5*t_k^4];
    t_powers_acc = [0; 0; 2; 6*t_k; 12*t_k^2; 20*t_k^3];
    
    pos_traj(k,:) = (traj_coeffs' * t_powers)';
    vel_traj(k,:) = (traj_coeffs' * t_powers_vel)';
    acc_traj(k,:) = (traj_coeffs' * t_powers_acc)';
end

% Plot trajectory
figure(3);
subplot(3,1,1);
plot(t_sim, pos_traj);
xlabel('Time (s)'); ylabel('Position (m)');
title('Trajectory - Position');
legend('X', 'Y', 'Z');
grid on;

subplot(3,1,2);
plot(t_sim, vel_traj);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Trajectory - Velocity');
legend('X', 'Y', 'Z');
grid on;

subplot(3,1,3);
plot(t_sim, acc_traj);
xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
title('Trajectory - Acceleration');
legend('X', 'Y', 'Z');
grid on;

%% Performance Metrics

% Calculate bandwidth and margins for rate loops
[Gm_roll, Pm_roll, Wcg_roll, Wcp_roll] = margin(C_roll_rate * sys_lin(1,1));
BW_roll = bandwidth(T_roll_cl);

fprintf('\nControl System Performance:\n');
fprintf('Roll Rate Loop:\n');
fprintf('  Gain Margin: %.1f dB\n', 20*log10(Gm_roll));
fprintf('  Phase Margin: %.1f deg\n', Pm_roll);
fprintf('  Bandwidth: %.1f rad/s\n', BW_roll);

% Settling time and overshoot
step_info_roll = stepinfo(T_roll_cl);
fprintf('  Settling Time: %.3f s\n', step_info_roll.SettlingTime);
fprintf('  Overshoot: %.1f%%\n', step_info_roll.Overshoot);

%% Save Results

% Save system parameters for implementation
system_params = struct();
system_params.mass = m;
system_params.inertia = [Ixx, Iyy, Izz];
system_params.arm_length = L;
system_params.thrust_coeff = kT;
system_params.torque_coeff = kM;
system_params.motor_time_constant = tau_motor;
system_params.mixing_matrix = M;

control_params = struct();
control_params.rate_gains_p = Kp_rate;
control_params.rate_gains_i = Ki_rate;
control_params.rate_gains_d = Kd_rate;
control_params.attitude_gains_p = Kp_att;
control_params.lqr_gains = K_lqr;

ekf_params = struct();
ekf_params.process_noise = [sigma_gyro, sigma_accel, sigma_gyro_bias, sigma_accel_bias];
ekf_params.measurement_noise = [sigma_gps_pos, sigma_gps_vel, sigma_baro, sigma_mag];

save('hexacopter_system_id.mat', 'system_params', 'control_params', 'ekf_params', ...
     'sys_lin', 'traj_coeffs', 't_sim', 'pos_traj', 'vel_traj', 'acc_traj');

fprintf('\nResults saved to hexacopter_system_id.mat\n');
fprintf('System identification and control design completed.\n');