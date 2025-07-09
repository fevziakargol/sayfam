%% UZB 421E - Attitude Determination and Control - Homework 5
% Ahmet Fevzi Akargöl
% Student ID: 110200170
% B-Dot Control with Magnetorquers Simulation

clc; clear; close all;

%% Student Number and Simulation Parameters
n_student = 0; % Öğrenci numarasının son hanesi (Sizin için 0)
N_samples = 6000;    % Örnek sayısı
delta_t = 1;         % Örnekleme zamanı (s)
time_vec = (0:N_samples)*delta_t; % Zaman vektörü

rad2deg_val = 180/pi;
deg2rad_val = pi/180;

%% Initial Conditions (from HW1, n=0)
% Initial attitude angles (rad)
psi_0   = -0.005 - 0.002*n_student; % Yaw
theta_0 =  0.01  - 0.002*n_student; % Pitch
phi_0   = -0.01  - 0.002*n_student; % Roll
euler_initial = [psi_0; theta_0; phi_0]; % Sütun vektörü olarak

% Initial angular velocities (rad/s)
w1_0 = -0.002 - 0.0001*n_student;
w2_0 =  0.003 - 0.0001*n_student;
w3_0 = -0.004 - 0.0001*n_student;
omega_initial = [w1_0; w2_0; w3_0]; % Sütun vektörü olarak

%% Satellite Parameters (from HW1)
% Mass moments of inertia (kg.m^2)
I1 = 2.1e-3;
I2 = 2.0e-3;
I3 = 1.9e-3;
I_satellite = diag([I1, I2, I3]); % Eylemsizlik matrisi
I_min = I3; % Minimum eylemsizlik momenti (kg.m^2)

% External disturbance torques (N.m) - Assuming constant
L_D = [3.6e-10; 3.6e-10; 3.6e-10]; % Sütun vektörü olarak

%% Orbit Parameters (from HW2 & HW3)
T_orbit = 6000; % Yörünge periyodu (s) (HW3'teki varsayım)
xi_m = 80 * deg2rad_val; % Yörünge eğikliği (rad) (HW2'den, n=0 için 80 deg)

%% B-Dot Controller Gain (k_B)
k_B = (4*pi/T_orbit) * (1 + sin(xi_m)) * I_min;
fprintf('Calculated B-Dot Gain (k_B): %e\n', k_B);

%% Sensor Error Models (from HW3 & HW4, n=0)

% --- Magnetometer Errors (from HW3) ---
% Bias vector (nT)
b_mag_nT = [500; 900; 1200] + 100*n_student; % nT
% Noise standard deviation (nT)
sigma_mag_nT = 100; % nT

% --- Gyroscope Errors (from HW4) ---
% Initial bias (deg/s)
b_rg_0_deg_s = [5; 1+n_student; 1-n_student] * 1e-3; % deg/s
b_rg_0_rad_s = b_rg_0_deg_s * deg2rad_val; % rad/s

% Measurement noise standard deviation (deg/s)
sigma_rg_deg_s = 1e-2; % deg/s
sigma_rg_rad_s = sigma_rg_deg_s * deg2rad_val; % rad/s

sigma_rgb_deg_per_sqrt_Hz = 1e-3; % deg/s/sqrt(Hz)
sigma_rgb_rad_per_sqrt_Hz = sigma_rgb_deg_per_sqrt_Hz * deg2rad_val; % rad/s/sqrt(Hz)


%% Main Simulation Loop for Each Case (k=1 to 4)

results_L_applied = cell(1,4);
results_L_commanded = cell(1,4);
results_torque_diff_norm = cell(1,4);
results_euler_angles = cell(1,4);
results_angular_velocities = cell(1,4);
results_m_generated = cell(1,4); 

for k_case = 1:4
    fprintf('Running Simulation for Case k = %d\n', k_case);

    omega_history = zeros(3, N_samples + 1);
    euler_history = zeros(3, N_samples + 1);
    dcm_history   = zeros(3, 3, N_samples + 1); 

    L_applied_history   = zeros(3, N_samples); 
    L_commanded_history = zeros(3, N_samples);
    m_generated_history = zeros(3, N_samples);

    omega_history(:, 1) = omega_initial;
    euler_history(:, 1) = euler_initial;
    dcm_history(:,:,1)  = euler_to_dcm(euler_initial(1), euler_initial(2), euler_initial(3));

    b_rg_rad_s_current_sim = zeros(3, N_samples + 1);
    b_rg_rad_s_current_sim(:,1) = b_rg_0_rad_s;
    for t_idx_bias = 1:N_samples
        noise_rgb_step = sigma_rgb_rad_per_sqrt_Hz * randn(3,1) * sqrt(delta_t);
        b_rg_rad_s_current_sim(:, t_idx_bias+1) = b_rg_rad_s_current_sim(:, t_idx_bias) + noise_rgb_step;
    end

    for i = 1:N_samples
        omega_current = omega_history(:, i); 
        euler_current = euler_history(:, i); 
        C_BN_current  = dcm_history(:,:,i);  

        t_sim = time_vec(i); 
        Bn_inertial_nT = [20000 * cos(2 * pi * t_sim / T_orbit);
                          20000 * sin(2 * pi * t_sim / T_orbit);
                          30000]; 

        Bb_true_nT = C_BN_current * Bn_inertial_nT; 
        v_mag_nT = sigma_mag_nT * randn(3,1); 
        Bb_measured_case1_nT = Bb_true_nT + b_mag_nT + v_mag_nT; 
        Bb_measured_case2_nT = Bb_true_nT + v_mag_nT;             
        Bb_measured_case3_nT = Bb_true_nT;                       

        omega_BN_true_current = omega_current; 
        v_rg_rad_s = sigma_rg_rad_s * randn(3,1); 
        b_rg_current_step_rad_s = b_rg_rad_s_current_sim(:,i); 
        omega_tilde_BN_case1_rad_s = omega_BN_true_current + b_rg_current_step_rad_s + v_rg_rad_s; 
        omega_tilde_BN_case2_rad_s = omega_BN_true_current + v_rg_rad_s;                         
      
        B_b_for_control_nT = []; 
        omega_for_control_rad_s = []; 

        if k_case == 1 
            B_b_for_control_nT = Bb_measured_case1_nT;
            omega_for_control_rad_s = omega_tilde_BN_case1_rad_s;
        elseif k_case == 2 
            B_b_for_control_nT = Bb_measured_case1_nT;
            omega_for_control_rad_s = omega_tilde_BN_case2_rad_s;
        elseif k_case == 3 
            B_b_for_control_nT = Bb_measured_case2_nT;
            omega_for_control_rad_s = omega_tilde_BN_case1_rad_s;
        elseif k_case == 4 
            B_b_for_control_nT = Bb_measured_case2_nT;
            omega_for_control_rad_s = omega_tilde_BN_case2_rad_s;
        end

        B_b_for_control_T = B_b_for_control_nT * 1e-9; 
        norm_B_b_for_control = norm(B_b_for_control_T);
        if norm_B_b_for_control < 1e-9 
            b_hat_measured = [0;0;0];
        else
            b_hat_measured = B_b_for_control_T / norm_B_b_for_control;
        end

        if norm_B_b_for_control < 1e-9 
            m_gen_Am2 = [0;0;0];
        else
            m_gen_Am2 = -k_B / norm_B_b_for_control * cross(b_hat_measured, omega_for_control_rad_s);
        end
        m_generated_history(:,i) = m_gen_Am2;

        Bb_true_T = Bb_measured_case3_nT * 1e-9; 
        L_C_applied_Nm = cross(m_gen_Am2, Bb_true_T);
        L_applied_history(:,i) = L_C_applied_Nm;

        b_hat_outer_b_hat = b_hat_measured * b_hat_measured';
        L_C_commanded_Nm = -k_B * (eye(3) - b_hat_outer_b_hat) * omega_for_control_rad_s;
        L_commanded_history(:,i) = L_C_commanded_Nm;

        L_total_Nm = L_D + L_C_applied_Nm;
        omega_dot = I_satellite \ (L_total_Nm - cross(omega_current, I_satellite * omega_current));
        omega_new = omega_current + omega_dot * delta_t;
        omega_history(:, i+1) = omega_new;

        psi_curr   = euler_current(1);
        theta_curr = euler_current(2);
        phi_curr   = euler_current(3);
        
        if abs(cos(theta_curr)) < 1e-6 
            euler_dot = [0; omega_current(2); 0]; 
            if mod(i,100)==0 
                warning('Singularity reached in Euler angles at t=%f for case k=%d. Theta = %f deg.', t_sim, k_case, theta_curr*rad2deg_val);
            end
        else
            Kinematic_Matrix = (1/cos(theta_curr)) * [ 0, sin(phi_curr),             cos(phi_curr);
                                                       0, cos(phi_curr)*cos(theta_curr), -sin(phi_curr)*cos(theta_curr);
                                                       cos(theta_curr), sin(phi_curr)*sin(theta_curr), cos(phi_curr)*sin(theta_curr) ];
            euler_dot = Kinematic_Matrix * omega_current; 
        end

        euler_new = euler_current + euler_dot * delta_t;
        euler_history(:, i+1) = euler_new;
        dcm_history(:,:,i+1) = euler_to_dcm(euler_new(1), euler_new(2), euler_new(3));
    end 

    results_L_applied{k_case} = L_applied_history;
    results_L_commanded{k_case} = L_commanded_history;
    results_torque_diff_norm{k_case} = vecnorm(L_commanded_history - L_applied_history); 
    results_euler_angles{k_case} = euler_history;
    results_angular_velocities{k_case} = omega_history;
    results_m_generated{k_case} = m_generated_history;

    fprintf('Finished Simulation for Case k = %d\n\n', k_case);
end 

%% Text Output of Key Results
fprintf('\n\n--- NUMERICAL RESULTS SUMMARY ---\n');
output_step_interval = 500; % Kaç örnekte bir veri yazdırılacağını belirler (örneğin 100, 200, 500). delta_t=1s olduğu için bu aynı zamanda saniye cinsinden aralıktır.
if output_step_interval <= 0; output_step_interval = 1; end % Hatalı girişi engelle
time_points_for_output = unique([1, output_step_interval:output_step_interval:(N_samples+1), (N_samples+1)]);
for k_case_out = 1:4
    fprintf('\n--- Results for Case k = %d ---\n', k_case_out);
    
    current_L_applied = results_L_applied{k_case_out};
    current_torque_diff_norm = results_torque_diff_norm{k_case_out};
    current_euler_angles_rad = results_euler_angles{k_case_out};
    current_angular_velocities_rad_s = results_angular_velocities{k_case_out};

    fprintf('Initial Euler Angles (deg): [%.4f, %.4f, %.4f]\n', rad2deg_val*euler_initial(1), rad2deg_val*euler_initial(2), rad2deg_val*euler_initial(3));
    fprintf('Initial Angular Velocities (deg/s): [%.4f, %.4f, %.4f]\n', rad2deg_val*omega_initial(1), rad2deg_val*omega_initial(2), rad2deg_val*omega_initial(3));
    
    fprintf('\nTime-Stepped Data:\n');
    fprintf('%-10s | %-25s | %-30s | %-30s | %-25s\n', 'Time(s)', 'Euler Angles (deg)', 'Angular Velocities (deg/s)', 'Applied Control Torque (Nm)', 'Norm Torque Diff (Nm)');
    fprintf('%s\n', repmat('-',1,130));

    for tp_idx = 1:length(time_points_for_output)
        t_step = time_points_for_output(tp_idx); 
        actual_time_s = (t_step-1)*delta_t; 
        if t_step == 1 
             actual_time_s = 0;
        end
        
        euler_rad_tp = current_euler_angles_rad(:, t_step); 
        omega_rad_s_tp = current_angular_velocities_rad_s(:, t_step); 
        
        torque_idx = min(t_step, N_samples); 
        if t_step == 1 
            L_app_tp = current_L_applied(:, torque_idx);
            norm_diff_tp = current_torque_diff_norm(torque_idx);
        else 
             torque_idx_eff = min(t_step-1, N_samples); 
             if torque_idx_eff == 0; torque_idx_eff =1; end; 
             L_app_tp = current_L_applied(:, torque_idx_eff);
             norm_diff_tp = current_torque_diff_norm(torque_idx_eff);
        end
        
        if actual_time_s == 0
             fprintf('%-10.0f | [%.3f, %.3f, %.3f]      | [%.4f, %.4f, %.4f]         | (L_C at t=0 not stored)      | (Diff at t=0 not stored)\n', ...
                actual_time_s, ...
                rad2deg_val*euler_rad_tp(1), rad2deg_val*euler_rad_tp(2), rad2deg_val*euler_rad_tp(3), ...
                rad2deg_val*omega_rad_s_tp(1), rad2deg_val*omega_rad_s_tp(2), rad2deg_val*omega_rad_s_tp(3));
        else
            fprintf('%-10.0f | [%.3f, %.3f, %.3f]      | [%.4f, %.4f, %.4f]         | [%.2e, %.2e, %.2e]      | %.2e\n', ...
                actual_time_s, ...
                rad2deg_val*euler_rad_tp(1), rad2deg_val*euler_rad_tp(2), rad2deg_val*euler_rad_tp(3), ...
                rad2deg_val*omega_rad_s_tp(1), rad2deg_val*omega_rad_s_tp(2), rad2deg_val*omega_rad_s_tp(3), ...
                L_app_tp(1), L_app_tp(2), L_app_tp(3), ...
                norm_diff_tp);
        end
    end
    
    fprintf('\nSummary Statistics (for t=1s to t=6000s for torques):\n');
    mean_L_applied = mean(current_L_applied, 2); 
    std_L_applied = std(current_L_applied, 0, 2);   
    mean_torque_diff = mean(current_torque_diff_norm);
    std_torque_diff = std(current_torque_diff_norm);
    
    fprintf('  Mean L_applied (Nm): [%.2e, %.2e, %.2e]\n', mean_L_applied(1), mean_L_applied(2), mean_L_applied(3));
    fprintf('  Std Dev L_applied (Nm): [%.2e, %.2e, %.2e]\n', std_L_applied(1), std_L_applied(2), std_L_applied(3));
    fprintf('  Mean Torque Diff Norm (Nm): %.2e\n', mean_torque_diff);
    fprintf('  Std Dev Torque Diff Norm (Nm): %.2e\n', std_torque_diff);
    fprintf('%s\n', repmat('-',1,130));
end


%% Plotting Results (Ödevde istenen grafikler)

for k_case_plot = 1:4
    time_plot_L = time_vec(1:N_samples); 
    time_plot_state = time_vec; 

    figure_name_L_app = sprintf('Applied Control Torques - Case %d', k_case_plot);
    figure('Name', figure_name_L_app);
    plot(time_plot_L, results_L_applied{k_case_plot}(1,:), 'r', 'DisplayName', 'L_{applied,1}');
    hold on;
    plot(time_plot_L, results_L_applied{k_case_plot}(2,:), 'g', 'DisplayName', 'L_{applied,2}');
    plot(time_plot_L, results_L_applied{k_case_plot}(3,:), 'b', 'DisplayName', 'L_{applied,3}');
    hold off;
    xlabel('Time (s)');
    ylabel('Applied Control Torque (N.m)');
    title(sprintf('Applied Control Torques (L_{applied}) - Case %d', k_case_plot));
    legend show;
    grid on;

    figure_name_torque_diff = sprintf('Norm of Torque Difference - Case %d', k_case_plot);
    figure('Name', figure_name_torque_diff);
    plot(time_plot_L, results_torque_diff_norm{k_case_plot}, 'k');
    xlabel('Time (s)');
    ylabel('Norm of (L_{commanded} - L_{applied}) (N.m)');
    title(sprintf('Norm of Torque Difference - Case %d', k_case_plot));
    grid on;
    
    euler_angles_plot_deg = rad2deg_val * results_euler_angles{k_case_plot}; 
    euler_angles_plot_deg(1,:) = wrapTo180(euler_angles_plot_deg(1,:)); 
    euler_angles_plot_deg(2,:) = wrapTo180(euler_angles_plot_deg(2,:)); 
    euler_angles_plot_deg(3,:) = wrapTo180(euler_angles_plot_deg(3,:)); 

    figure_name_euler = sprintf('Euler Angles - Case %d', k_case_plot);
    figure('Name', figure_name_euler);
    plot(time_plot_state, euler_angles_plot_deg(1,:), 'r', 'DisplayName', '\psi (Yaw)');
    hold on;
    plot(time_plot_state, euler_angles_plot_deg(2,:), 'g', 'DisplayName', '\theta (Pitch)');
    plot(time_plot_state, euler_angles_plot_deg(3,:), 'b', 'DisplayName', '\phi (Roll)');
    hold off;
    xlabel('Time (s)');
    ylabel('Euler Angles (deg)');
    title(sprintf('Euler Angles - Case %d', k_case_plot));
    legend show;
    grid on;
    ylim([-180 180]);

    angular_velocities_plot_deg_s = rad2deg_val * results_angular_velocities{k_case_plot};
    figure_name_omega = sprintf('Angular Velocities - Case %d', k_case_plot);
    figure('Name', figure_name_omega);
    plot(time_plot_state, angular_velocities_plot_deg_s(1,:), 'r', 'DisplayName', '\omega_1');
    hold on;
    plot(time_plot_state, angular_velocities_plot_deg_s(2,:), 'g', 'DisplayName', '\omega_2');
    plot(time_plot_state, angular_velocities_plot_deg_s(3,:), 'b', 'DisplayName', '\omega_3');
    hold off;
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    title(sprintf('Angular Velocities - Case %d', k_case_plot));
    legend show;
    grid on;
end

disp('Simulations and plotting complete.');

%% Helper Functions

function C_BN = euler_to_dcm(psi, theta, phi)
    C_psi = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    C_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]; % Standart Ry(theta)
    C_phi = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    C_BN = C_phi * C_theta * C_psi; 
end
