clc; clear; close all;

%% Constants definitions
n = 0; % Last digit of student number
N = 6000;                   % Number of samples
delta_t = 1;                % Sampling time (seconds)
time = (0:N)*delta_t;       % Time vector from 0 to N seconds

% -------------------------------------------------------------------------
% SECTION FROM HOMEWORK 1 (for n=0) - Calculating DCM (C Matrix)
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
w = [w1_0, w2_0, w3_0];
w_history = zeros(N+1, 3);
w_history(1, :) = w;

for i = 1:N
    dw1 = (-(I3 - I2)*w(2)*w(3) + L1) / I1;
    dw2 = (-(I1 - I3)*w(3)*w(1) + L2) / I2;
    dw3 = (-(I2 - I1)*w(1)*w(2) + L3) / I3;
    
    % Euler integration step for omega
    w = w + [dw1, dw2, dw3] * delta_t;
    w_history(i+1, :) = w;
end

% --- Propagate Euler Angles (Euler Method - Based on HW1 Eq. 2) ---
% Using the 3-2-1 sequence
euler_history = zeros(N+1, 3); % Stores [psi, theta, phi] in columns
euler_history(1, :) = [psi_0, theta_0, phi_0];

for i = 1:N
    psi_k   = euler_history(i, 1);
    theta_k = euler_history(i, 2);
    phi_k   = euler_history(i, 3);
    w_k     = w_history(i, :)'; % Use omega at the beginning of the interval
    
    % Kinematic Matrix (3-2-1 sequence)
    % Check for singularity near cos(theta) = 0
    if abs(cos(theta_k)) < 1e-8
        warning('Near singularity in Euler angles at step %d', i);
        dEuler_dt = zeros(3,1); % Assign zero derivatives
    else
        Kinematic_Matrix = (1/cos(theta_k)) * [ 0, sin(phi_k),             cos(phi_k);
                                                0, cos(phi_k)*cos(theta_k), -sin(phi_k)*cos(theta_k);
                                                cos(theta_k), sin(phi_k)*sin(theta_k), cos(phi_k)*sin(theta_k) ];
        dEuler_dt = Kinematic_Matrix * w_k;
    end
    
    % Euler integration step for angles [psi, theta, phi]
    euler_history(i+1, :) = euler_history(i, :) + dEuler_dt' * delta_t;
end

% --- Calculate DCM (C Matrix - Based on HW1 Section C / B.a results) ---
% Compute C based on the 3-2-1 Euler angles [psi, theta, phi] calculated above.
% DCM for 3-2-1 (Z-Y-X) rotation sequence is C = R_x(phi) * R_y(theta) * R_z(psi)
C_dcm = zeros(3, 3, N + 1);
for i = 1:(N + 1)
    psi   = euler_history(i, 1);
    theta = euler_history(i, 2);
    phi   = euler_history(i, 3);
    
    cos_psi = cos(psi); sin_psi = sin(psi);
    cos_theta = cos(theta); sin_theta = sin(theta);
    cos_phi = cos(phi); sin_phi = sin(phi);
    
    % Rotation matrices
    Rz = [ cos_psi, sin_psi, 0;
          -sin_psi, cos_psi, 0;
                 0,       0, 1 ];
           
    Ry = [ cos_theta, 0, -sin_theta;
                 0, 1,          0;
           sin_theta, 0,  cos_theta ];
           
    Rx = [ 1,        0,         0;
           0,  cos_phi,   sin_phi;
           0, -sin_phi,   cos_phi ];
           
    % DCM from Inertial (N) to Body (B) is C_BN = Rx * Ry * Rz
    C_dcm(:,:,i) = Rx * Ry * Rz;
end

% -------------------------------------------------------------------------
% HOMEWORK 3 CALCULATIONS
% -------------------------------------------------------------------------

% --- Define Constants for HW3 ---
b = [500; 900; 1200] + 100*n; % Bias vector for n=0 (nT)
sigma = 100;                   % Noise standard deviation (nT)

% --- Assume Bn (Magnetic Field in Inertial Frame) ---
% NOTE: This is an assumed Bn vector as HW2 results were not available.
Bn = zeros(3, N+1);
Orbit_Period_Assumption = 6000; % Assumed orbital period in seconds for variation
for j = 1:(N+1)
    t_current = time(j);
    Bn(1, j) = 20000 * cos(2 * pi * t_current / Orbit_Period_Assumption);
    Bn(2, j) = 20000 * sin(2 * pi * t_current / Orbit_Period_Assumption);
    Bn(3, j) = 30000; % Constant Z component assumption
end
% Bn is assumed to be in nT units.

% --- Generate Noise Vector (v) ---
v_noise = sigma * randn(3, N+1); % Zero-mean Gaussian white noise (nT)

% --- Calculate Magnetometer Measurements (Bb) for 3 Cases ---
Bb_case1 = zeros(3, N+1); % Bb = C*Bn + b + v
Bb_case2 = zeros(3, N+1); % Bb = C*Bn + v
Bb_case3 = zeros(3, N+1); % Bb = C*Bn

for j = 1:(N+1)
    % Rotation matrix from N to B
    C_current = C_dcm(:,:,j);
    Bn_current = Bn(:,j);
    
    Bb_case3(:,j) = C_current * Bn_current;
    Bb_case2(:,j) = Bb_case3(:,j) + v_noise(:,j);
    Bb_case1(:,j) = Bb_case3(:,j) + b + v_noise(:,j); % Bias 'b' added here
end

% --- Plot Raw Magnetic Field Measurements (Task 3) ---
figure('Name', 'Raw Magnetometer Measurements - X Component');
plot(time, Bb_case1(1,:), 'r', 'DisplayName', 'Case 1: C*Bn + b + v');
hold on;
plot(time, Bb_case2(1,:), 'g', 'DisplayName', 'Case 2: C*Bn + v');
plot(time, Bb_case3(1,:), 'b', 'DisplayName', 'Case 3: C*Bn');
hold off;
xlabel('Time (s)');
ylabel('Magnetic Field Strength (nT)');
title('Raw B_b - X Component');
legend;
grid on;

figure('Name', 'Raw Magnetometer Measurements - Y Component');
plot(time, Bb_case1(2,:), 'r', 'DisplayName', 'Case 1: C*Bn + b + v');
hold on;
plot(time, Bb_case2(2,:), 'g', 'DisplayName', 'Case 2: C*Bn + v');
plot(time, Bb_case3(2,:), 'b', 'DisplayName', 'Case 3: C*Bn');
hold off;
xlabel('Time (s)');
ylabel('Magnetic Field Strength (nT)');
title('Raw B_b - Y Component');
legend;
grid on;

figure('Name', 'Raw Magnetometer Measurements - Z Component');
plot(time, Bb_case1(3,:), 'r', 'DisplayName', 'Case 1: C*Bn + b + v');
hold on;
plot(time, Bb_case2(3,:), 'g', 'DisplayName', 'Case 2: C*Bn + v');
plot(time, Bb_case3(3,:), 'b', 'DisplayName', 'Case 3: C*Bn');
hold off;
xlabel('Time (s)');
ylabel('Magnetic Field Strength (nT)');
title('Raw B_b - Z Component');
legend;
grid on;

% --- Normalize Bb Vectors (Task 4) ---
Bb_norm_case1 = zeros(3, N+1);
Bb_norm_case2 = zeros(3, N+1);
Bb_norm_case3 = zeros(3, N+1);

for j = 1:(N+1)
    norm1 = norm(Bb_case1(:,j));
    norm2 = norm(Bb_case2(:,j));
    norm3 = norm(Bb_case3(:,j));
    
    if norm1 > 1e-10 % Avoid division by zero
        Bb_norm_case1(:,j) = Bb_case1(:,j) / norm1;
    end
    if norm2 > 1e-10
        Bb_norm_case2(:,j) = Bb_case2(:,j) / norm2;
    end
    if norm3 > 1e-10
        Bb_norm_case3(:,j) = Bb_case3(:,j) / norm3;
    end
end

% --- Plot Normalized Magnetic Field Measurements (Task 4) ---
figure('Name', 'Normalized Magnetometer Measurements - X Component');
plot(time, Bb_norm_case1(1,:), 'r', 'DisplayName', 'Case 1: Normalized');
hold on;
plot(time, Bb_norm_case2(1,:), 'g', 'DisplayName', 'Case 2: Normalized');
plot(time, Bb_norm_case3(1,:), 'b', 'DisplayName', 'Case 3: Normalized');
hold off;
xlabel('Time (s)');
ylabel('Normalized Magnetic Field Strength [-]');
title('Normalized B_b - X Component');
legend;
grid on;

figure('Name', 'Normalized Magnetometer Measurements - Y Component');
plot(time, Bb_norm_case1(2,:), 'r', 'DisplayName', 'Case 1: Normalized');
hold on;
plot(time, Bb_norm_case2(2,:), 'g', 'DisplayName', 'Case 2: Normalized');
plot(time, Bb_norm_case3(2,:), 'b', 'DisplayName', 'Case 3: Normalized');
hold off;
xlabel('Time (s)');
ylabel('Normalized Magnetic Field Strength [-]');
title('Normalized B_b - Y Component');
legend;
grid on;

figure('Name', 'Normalized Magnetometer Measurements - Z Component');
plot(time, Bb_norm_case1(3,:), 'r', 'DisplayName', 'Case 1: Normalized');
hold on;
plot(time, Bb_norm_case2(3,:), 'g', 'DisplayName', 'Case 2: Normalized');
plot(time, Bb_norm_case3(3,:), 'b', 'DisplayName', 'Case 3: Normalized');
hold off;
xlabel('Time (s)');
ylabel('Normalized Magnetic Field Strength [-]');
title('Normalized B_b - Z Component');
legend;
grid on;

% -------------------------------------------------------------------------
% END OF HOMEWORK 3 SIMULATION
% -------------------------------------------------------------------------