clc; clear; close all;

%% Constants definitions
n = 0; % Last digit of student number
N = 6000;                   % Number of samples
delta_t = 1;                % Sampling time (seconds)
time = (0:N)*delta_t;       % Time vector from 0 to N seconds

% Conversion factor
deg2rad_val = pi/180;
rad2deg_val = 180/pi;

% -------------------------------------------------------------------------
% SECTION FROM HOMEWORK 1 (for n=0) - Calculating True Angular Velocity
% -------------------------------------------------------------------------

% Initial attitude angles (rad) for n=0
psi_0 = -0.005 - 0.002*n;   % Yaw
theta_0 = 0.01 - 0.002*n;   % Pitch
phi_0 = -0.01 - 0.002*n;    % Roll

% Initial angular velocities (rad/s) for n=0
w1_0 = -0.002 - 0.0001*n;
w2_0 = 0.003 - 0.0001*n;
w3_0 = -0.004 - 0.0001*n;

% Mass moments of inertia (kg.m^2)
I1 = 2.1e-3;
I2 = 2.0e-3;
I3 = 1.9e-3;

% Disturbance torques (N.m)
L1 = 3.6e-10;
L2 = 3.6e-10;
L3 = 3.6e-10;

% --- Propagate Angular Velocities (Euler Method) ---
w = [w1_0, w2_0, w3_0]'; % Use column vector
w_true_history = zeros(3, N+1); % Store true angular velocity (rad/s)
w_true_history(:, 1) = w;

for i = 1:N
    w1 = w(1); w2 = w(2); w3 = w(3);
    dw1 = (-(I3 - I2)*w2*w3 + L1) / I1;
    dw2 = (-(I1 - I3)*w3*w1 + L2) / I2;
    dw3 = (-(I2 - I1)*w1*w2 + L3) / I3;
    
    % Euler integration step for omega
    w = w + [dw1; dw2; dw3] * delta_t;
    w_true_history(:, i+1) = w;
end

% w_true_history now contains omega_BN_true in rad/s for N+1 time steps

% -------------------------------------------------------------------------
% HOMEWORK 4 CALCULATIONS (for n=0)
% -------------------------------------------------------------------------

% --- Define Constants for HW4 with Unit Conversion ---
% Initial bias in deg/s
b_rg_0_deg = [5; 1+n; 1-n] * 1e-3; % deg/s
% Convert initial bias to rad/s
b_rg_0_rad = b_rg_0_deg * deg2rad_val; % rad/s

% Measurement noise standard deviation in deg/s
sigma_rg_deg = 1e-2; % deg/s
% Convert measurement noise std dev to rad/s
sigma_rg_rad = sigma_rg_deg * deg2rad_val; % rad/s

% Bias random walk noise standard deviation in deg/s^2
sigma_rgb_deg = 1e-3; % deg/s^2
% Convert bias random walk noise std dev to rad/s^2
sigma_rgb_rad = sigma_rgb_deg * deg2rad_val; % rad/s^2

% --- Simulate Gyro Bias (Random Walk) using Euler Method (Task 3) ---
b_rg_history_rad = zeros(3, N+1); % History of bias vector in rad/s
b_rg_history_rad(:, 1) = b_rg_0_rad; % Set initial bias

for k = 1:N
    % Generate bias random walk noise for this step (rad/s^2)
    v_rgb_k = sigma_rgb_rad * randn(3,1);
    % Euler integration for bias
    b_rg_history_rad(:, k+1) = b_rg_history_rad(:, k) + v_rgb_k * delta_t;
end

% --- Plot Gyro Bias Components (Task 3) ---
figure('Name', 'Gyroscope Bias Components');
plot(time, b_rg_history_rad(1,:) * rad2deg_val, 'r', 'DisplayName', 'b_{rg,x}');
hold on;
plot(time, b_rg_history_rad(2,:) * rad2deg_val, 'g', 'DisplayName', 'b_{rg,y}');
plot(time, b_rg_history_rad(3,:) * rad2deg_val, 'b', 'DisplayName', 'b_{rg,z}');
hold off;
xlabel('Time (s)');
ylabel('Gyroscope Bias (deg/s)');
title('Components of the Gyroscope Bias Vector (b_{rg})'); % No special chars here
legend;
grid on;

% --- Simulate Gyro Measurements (Task 4) ---
w_meas_case1 = zeros(3, N+1); % Case 1: omega_true + b_rg + v_rg
w_meas_case2 = zeros(3, N+1); % Case 2: omega_true + v_rg
w_meas_case3 = zeros(3, N+1); % Case 3: omega_true

for k = 1:(N+1)
    % Generate measurement noise for this step (rad/s)
    v_rg_k = sigma_rg_rad * randn(3,1);
    
    % True angular velocity (rad/s)
    omega_true_k = w_true_history(:, k);
    
    % Bias at this step (rad/s)
    b_rg_k = b_rg_history_rad(:, k);
    
    % Calculate measurements for the three cases (all in rad/s)
    w_meas_case3(:, k) = omega_true_k;
    w_meas_case2(:, k) = omega_true_k + v_rg_k;
    w_meas_case1(:, k) = omega_true_k + b_rg_k + v_rg_k;
end

% --- Plot Gyro Measurements (Task 4) ---
% Plot units should be rad/s

figure('Name', 'Gyro Measurements - X Component');
plot(time, w_meas_case1(1,:), 'r', 'DisplayName', 'Case 1: $\omega_{true} + b_{rg} + v_{rg}$'); % Use $ for LaTeX math
hold on;
plot(time, w_meas_case2(1,:), 'g', 'DisplayName', 'Case 2: $\omega_{true} + v_{rg}$'); % Use $ for LaTeX math
plot(time, w_meas_case3(1,:), 'b', 'DisplayName', 'Case 3: $\omega_{true}$'); % Use $ for LaTeX math
hold off;
xlabel('Time (s)');
ylabel('Measured Angular Velocity (rad/s)');
% <<< CORRECTION: Added 'Interpreter', 'latex' and used $ for math mode >>>
title('Gyro Measurements ($\tilde{\omega}_{BN}$) - X Component', 'Interpreter', 'latex');
legend('Interpreter', 'latex'); % Also set interpreter for legend
grid on;

figure('Name', 'Gyro Measurements - Y Component');
plot(time, w_meas_case1(2,:), 'r', 'DisplayName', 'Case 1: $\omega_{true} + b_{rg} + v_{rg}$'); % Use $ for LaTeX math
hold on;
plot(time, w_meas_case2(2,:), 'g', 'DisplayName', 'Case 2: $\omega_{true} + v_{rg}$'); % Use $ for LaTeX math
plot(time, w_meas_case3(2,:), 'b', 'DisplayName', 'Case 3: $\omega_{true}$'); % Use $ for LaTeX math
hold off;
xlabel('Time (s)');
ylabel('Measured Angular Velocity (rad/s)');
% <<< CORRECTION: Added 'Interpreter', 'latex' and used $ for math mode >>>
title('Gyro Measurements ($\tilde{\omega}_{BN}$) - Y Component', 'Interpreter', 'latex');
legend('Interpreter', 'latex'); % Also set interpreter for legend
grid on;

figure('Name', 'Gyro Measurements - Z Component');
plot(time, w_meas_case1(3,:), 'r', 'DisplayName', 'Case 1: $\omega_{true} + b_{rg} + v_{rg}$'); % Use $ for LaTeX math
hold on;
plot(time, w_meas_case2(3,:), 'g', 'DisplayName', 'Case 2: $\omega_{true} + v_{rg}$'); % Use $ for LaTeX math
plot(time, w_meas_case3(3,:), 'b', 'DisplayName', 'Case 3: $\omega_{true}$'); % Use $ for LaTeX math
hold off;
xlabel('Time (s)');
ylabel('Measured Angular Velocity (rad/s)');
% <<< CORRECTION: Added 'Interpreter', 'latex' and used $ for math mode >>>
title('Gyro Measurements ($\tilde{\omega}_{BN}$) - Z Component', 'Interpreter', 'latex');
legend('Interpreter', 'latex'); % Also set interpreter for legend
grid on;

% -------------------------------------------------------------------------
% END OF HOMEWORK 4 SIMULATION
% -------------------------------------------------------------------------