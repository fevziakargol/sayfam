% UZB461E - Ay Görev Yörüngeleri - Dönem Projesi - Bölüm I
% Öğrenci: Ahmet Fevzi Akargöl
% Betik: visualize_best_lunar_transfer_AF
% Açıklama: Bu betik, parametre taraması ile bulunan en iyi TLI parametrelerini
%           kullanarak Dünya'dan Ay'a transfer yörüngesini simüle eder,
%           gerekli yakıt miktarını hesaplar ve detaylı yörünge grafiklerini çizer.
%
% En İyi Parametreler (run_parameter_sweep_AF.m çıktısından alınacak):
%   TLI Açısı: 230.00 derece
%   TLI DeltaV: 3090.00 m/s
%   Kavramsal LOI DeltaV: 830.93 m/s
% Hedef Ay Park İrtifası: 490.00 km
%
% Gerekli Dosyalar:
%   - SystemsOfEquations.m (Dinamik denklemler fonksiyonu, düzeltilmiş olmalıdır)
%   - lunarEvents.m (Olay fonksiyonu, güncellenmiş olmalıdır)

clear; clc; close all;

fprintf('En İyi Parametrelerle Detaylı Simülasyon, Yakıt Hesabı ve Görselleştirme Başlatılıyor...\n');

% --- BÖLÜM 1: SABİTLER VE BAŞLANGIÇ PARAMETRELERİ ---
G_const_vis = 6.6742867e-11;      % Yerçekimi Sabiti [m^3/(kg*s^2)]
R_earth_vis = 6.3781366e6;      % Dünya Yarıçapı [m]
M_earth_vis = 5.9721426e24;     % Dünya Kütlesi [kg]
mu_earth_vis = 3.9859792e14;    % Dünya Yerçekimi Parametresi [m^3/s^2]
R_moon_vis = 1.7374000e6;       % Ay Yarıçapı [m]
M_moon_vis = 7.3457576e22;      % Ay Kütlesi [kg]
mu_moon_vis = 4.9027692e12;     % Ay Yerçekimi Parametresi [m^3/s^2]
L_earth_moon_vis = 3.8440000e8; % Ortalama Dünya-Ay mesafesi [m]

% Ahmet Fevzi Akargöl için Proje Parametreleri
altitude_earth_park_vis = 430e3; % Dünya park yörüngesi irtifası [m]
altitude_moon_park_target_vis = 490e3;  % Hedef Ay park irtifası [m]
M_dry_spacecraft_vis = 1000;        % Uzay aracı KURU kütlesi [kg]

% Yakıt Hesabı için Parametreler
Isp_seconds = 310; % Özgül İtki [saniye] - Tipik bir değer, gerekirse güncelleyin
g0_ms2 = 9.80665;  % Standart yerçekimi ivmesi [m/s^2]
exhaust_velocity_ms = Isp_seconds * g0_ms2; % Etkin egzoz hızı [m/s]

% Dünya park yörüngesi hesaplamaları
r_D_park_vis = R_earth_vis + altitude_earth_park_vis;
v_D_park_mag_vis = sqrt(mu_earth_vis / r_D_park_vis);

fprintf('Sabitler ve temel parametreler tanımlandı.\n');
fprintf('  Kuru Kütle: %.2f kg\n', M_dry_spacecraft_vis);
fprintf('  Özgül İtki (Isp): %.1f s\n', Isp_seconds);

% --- EN İYİ BULUNAN TLI ve LOI PARAMETRELERİ ---
% Bu değerler `run_parameter_sweep_AF.m` çıktısından alınmıştır.
best_tli_alpha_deg = 230.00;
best_dV_TLI_magnitude_ms = 3090.00; % m/s
best_dV_LOI_magnitude_ms = 830.93;  % m/s (Kavramsal LOI DeltaV)

simulation_duration_days_vis = 7;

fprintf('Kullanılan En İyi TLI Parametreleri:\n');
fprintf('  TLI Açısı: %.2f derece\n', best_tli_alpha_deg);
fprintf('  TLI DeltaV: %.2f m/s\n', best_dV_TLI_magnitude_ms);
fprintf('Kullanılan Kavramsal LOI DeltaV: %.2f m/s\n\n', best_dV_LOI_magnitude_ms);

% --- YAKIT KÜTLESİ HESAPLAMALARI (Anlık Yanmalar) ---
% Adım 1: LOI Yanması için Yakıt Hesabı
% LOI sonrası kütle = Kuru kütle
m_final_LOI = M_dry_spacecraft_vis;
% LOI öncesi kütle (LOI yakıtı + kuru kütle)
m_initial_LOI = m_final_LOI * exp(best_dV_LOI_magnitude_ms / exhaust_velocity_ms);
m_propellant_LOI = m_initial_LOI - m_final_LOI;

% Adım 2: TLI Yanması için Yakıt Hesabı
% TLI sonrası kütle = LOI öncesi kütle (yani m_initial_LOI)
m_final_TLI = m_initial_LOI;
% TLI öncesi kütle (TLI yakıtı + LOI yakıtı + kuru kütle)
m_initial_TLI_wet_mass = m_final_TLI * exp(best_dV_TLI_magnitude_ms / exhaust_velocity_ms);
m_propellant_TLI = m_initial_TLI_wet_mass - m_final_TLI;

% Toplam Yakıt Kütlesi
m_propellant_total = m_propellant_TLI + m_propellant_LOI;
% Uzay aracının Dünya park yörüngesindeki toplam başlangıç (ıslak) kütlesi
M_spacecraft_initial_wet_vis = m_initial_TLI_wet_mass; 
% M_spacecraft_initial_wet_vis = M_dry_spacecraft_vis + m_propellant_total; % Alternatif hesaplama

fprintf('--- Yakıt Kütlesi Hesaplamaları ---\n');
fprintf('  LOI Yanması İçin Gerekli Yakıt (m_p_LOI): %.2f kg\n', m_propellant_LOI);
fprintf('  LOI Öncesi Toplam Kütle (m_initial_LOI): %.2f kg\n', m_initial_LOI);
fprintf('  TLI Yanması İçin Gerekli Yakıt (m_p_TLI): %.2f kg\n', m_propellant_TLI);
fprintf('  TLI Öncesi Toplam Başlangıç Kütlesi (Islak Kütle): %.2f kg\n', M_spacecraft_initial_wet_vis);
fprintf('  Toplam Yakıt Kütlesi (m_p_total): %.2f kg\n\n', m_propellant_total);

% --- BAŞLANGIÇ KOŞULLARININ AYARLANMASI ---
w_sys_vis = sqrt(G_const_vis * (M_earth_vis + M_moon_vis) / L_earth_moon_vis^3);
r1_com_vis = -(M_moon_vis / (M_earth_vis + M_moon_vis)) * L_earth_moon_vis;
r2_com_vis = (M_earth_vis / (M_earth_vis + M_moon_vis)) * L_earth_moon_vis;
pos0_earth_abs_vis = [r1_com_vis, 0, 0];
pos0_moon_abs_vis = [r2_com_vis, 0, 0];
vel0_earth_abs_vis = [0, r1_com_vis * w_sys_vis, 0];
vel0_moon_abs_vis = [0, r2_com_vis * w_sys_vis, 0];

tli_alpha_rad_vis = deg2rad(best_tli_alpha_deg);
pos_sc_rel_E_vis = [r_D_park_vis * cos(tli_alpha_rad_vis), r_D_park_vis * sin(tli_alpha_rad_vis), 0];
vel_sc_rel_E_vis = [-v_D_park_mag_vis * sin(tli_alpha_rad_vis), v_D_park_mag_vis * cos(tli_alpha_rad_vis), 0];
pos0_sc_abs_pre_tli_vis = pos_sc_rel_E_vis + pos0_earth_abs_vis;

if norm(vel_sc_rel_E_vis) > 1e-9
    unit_vec_vel_sc_vis = vel_sc_rel_E_vis / norm(vel_sc_rel_E_vis);
else
    unit_vec_vel_sc_vis = [1, 0, 0];
end
deltaV_TLI_vec_vis = unit_vec_vel_sc_vis * best_dV_TLI_magnitude_ms;
vel_sc_rel_E_post_tli_vis = vel_sc_rel_E_vis + deltaV_TLI_vec_vis;
vel0_sc_abs_post_tli_vis = vel_sc_rel_E_post_tli_vis + vel0_earth_abs_vis;

Y0_vis = [pos0_earth_abs_vis, vel0_earth_abs_vis, pos0_moon_abs_vis, vel0_moon_abs_vis, ...
          pos0_sc_abs_pre_tli_vis, vel0_sc_abs_post_tli_vis];

% --- Global Değişkenlerin Ayarlanması (SystemsOfEquations.m için) ---
global m G n; 
n = 3;
% ÖNEMLİ: SystemsOfEquations içindeki kütle, simülasyon boyunca sabit kalır.
% Anlık yanma kabulü ile, her bir coast fazı için kütle farklıdır.
% Ancak n-cisim çözücüsü için tek bir uzay aracı kütlesi tanımlanır.
% Bu kütle, genellikle ortalama bir değer veya kuru kütle alınabilir.
% Yakıt hesabı, bu dinamik simülasyondan elde edilen DeltaV'ler kullanılarak ayrı yapılır.
% Burada M_spacecraft_vis olarak kuru kütleyi kullanıyoruz, çünkü yakıt kütlesi
% dinamik denklemlerdeki kütle değişimini değil, sadece DeltaV ihtiyacını etkiler.
m = [M_earth_vis, M_moon_vis, M_dry_spacecraft_vis]; % Uzay aracı için KURU kütle kullanılır
G = G_const_vis;

% --- YÖRÜNGE SİMÜLASYONU ---
tspan_vis = [0, simulation_duration_days_vis * 24 * 3600];
ode_options_vis = odeset('RelTol',1e-9,'AbsTol',1e-10, 'Events', @lunarEvents, 'MaxStep', 1800);
fprintf('ODE45 simülasyonu çalıştırılıyor (detaylı görselleştirme için)...\n');
tic;
[T_vis, Y_vis, TE_vis, YE_vis, IE_vis] = ode45(@SystemsOfEquations, tspan_vis, Y0_vis, ode_options_vis);
computation_time_vis = toc;
fprintf('ODE45 simülasyonu tamamlandı. Hesaplama Süresi: %.2f saniye.\n', computation_time_vis);

if isempty(T_vis) || size(Y_vis,1) < 2
    error('Görselleştirme için ODE45 anlamlı bir yörünge hesaplayamadı. Parametreleri kontrol edin.');
end

altitude_at_event_vis = NaN;
if ~isempty(TE_vis)
    fprintf('\n`lunarEvents` olay fonksiyonu tetiklendi.\n');
    fprintf('  Olayın Gerçekleşme Zamanı: %.2f saniye (%.2f gün)\n', TE_vis(end), TE_vis(end)/(24*3600));
    pos_sc_at_event_abs_vis = YE_vis(end, 13:15);
    pos_moon_at_event_abs_vis = YE_vis(end, 7:9);
    r_sc_wrt_moon_at_event_vec_vis = pos_sc_at_event_abs_vis - pos_moon_at_event_abs_vis;
    altitude_at_event_vis = norm(r_sc_wrt_moon_at_event_vec_vis) - R_moon_vis;
    fprintf('  Olay Anındaki Ay Yüzeyine İrtifa: %.2f km (Hedef: %.0f km)\n', ...
            altitude_at_event_vis/1000, altitude_moon_park_target_vis/1000);
else
    warning('Görselleştirme simülasyonunda `lunarEvents` tetiklenmedi!');
end
fprintf('\n');

% --- VERİ ANALİZİ VE GÖRSELLEŞTİRME ---
fprintf('Simülasyon sonuçları analiz ediliyor ve detaylı grafikler oluşturuluyor...\n');
pos_earth_traj = Y_vis(:,1:3);
pos_moon_traj = Y_vis(:,7:9);
pos_sc_traj = Y_vis(:,13:15);
vel_sc_traj = Y_vis(:,16:18);
vel_moon_traj = Y_vis(:,10:12);

num_time_points = length(T_vis);
altitude_from_moon_series_vis = zeros(num_time_points, 1);
speed_wrt_moon_series_vis = zeros(num_time_points, 1);

for idx = 1:num_time_points
    r_sc_wrt_moon_vec_inst = pos_sc_traj(idx,:) - pos_moon_traj(idx,:);
    v_sc_wrt_moon_vec_inst = vel_sc_traj(idx,:) - vel_moon_traj(idx,:);
    altitude_from_moon_series_vis(idx) = norm(r_sc_wrt_moon_vec_inst) - R_moon_vis;
    speed_wrt_moon_series_vis(idx) = norm(v_sc_wrt_moon_vec_inst);
end
[min_altitude_overall_vis, ~] = min(altitude_from_moon_series_vis);

% --- GRAFİK 1: Genel 3D Yörünge (Barisantrik Çerçeve) ---
figure('Name', 'Genel Yörünge (Barisantrik Çerçeve) - En İyi Sonuç');
hold on;
plot3(pos_earth_traj(:,1)/1e3, pos_earth_traj(:,2)/1e3, pos_earth_traj(:,3)/1e3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Dünya Yörüngesi');
plot3(pos_moon_traj(:,1)/1e3, pos_moon_traj(:,2)/1e3, pos_moon_traj(:,3)/1e3, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ay Yörüngesi');
plot3(pos_sc_traj(:,1)/1e3, pos_sc_traj(:,2)/1e3, pos_sc_traj(:,3)/1e3, 'r-', 'LineWidth', 1.2, 'DisplayName', 'Uzay Aracı Yörüngesi');
plot3(pos_earth_traj(1,1)/1e3, pos_earth_traj(1,2)/1e3, pos_earth_traj(1,3)/1e3, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'c', 'MarkerSize', 10, 'DisplayName', 'Dünya Başlangıç');
plot3(pos_moon_traj(1,1)/1e3, pos_moon_traj(1,2)/1e3, pos_moon_traj(1,3)/1e3, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerSize', 8, 'DisplayName', 'Ay Başlangıç');
plot3(pos_sc_traj(1,1)/1e3, pos_sc_traj(1,2)/1e3, pos_sc_traj(1,3)/1e3, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'm', 'MarkerSize', 6, 'DisplayName', 'SC Başlangıç (TLI sonrası)');
if ~isempty(TE_vis)
    plot3(YE_vis(end,13)/1e3, YE_vis(end,14)/1e3, YE_vis(end,15)/1e3, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'MarkerSize', 14, 'DisplayName', sprintf('Ay''a Varış (%.0fkm İrtifa)', altitude_moon_park_target_vis/1e3));
end
title_str1 = sprintf('Dünya-Ay Sistemi ve Uzay Aracı Yörüngesi (En İyi Sonuçlu TLI)');
title_str2 = sprintf('TLI Açısı: %.2f derece, TLI DeltaV: %.2f m/s', best_tli_alpha_deg, best_dV_TLI_magnitude_ms);
title({title_str1, title_str2});
xlabel('X Konumu (Barisantrik Çerçeve) [km]'); ylabel('Y Konumu (Barisantrik Çerçeve) [km]'); zlabel('Z Konumu (Barisantrik Çerçeve) [km]');
legend('show', 'Location', 'northeastoutside'); axis equal; grid on; view(3);
hold off;

% --- GRAFİK 2: Ay Merkezli Yörünge (Yakın Geçiş/Park Yörüngesi) ---
pos_sc_wrt_moon_trajectory = zeros(size(pos_sc_traj));
for k_idx = 1:num_time_points
    pos_sc_wrt_moon_trajectory(k_idx,:) = pos_sc_traj(k_idx,:) - pos_moon_traj(k_idx,:);
end
figure('Name', 'Ay Merkezli Yörünge (Yakın Geçiş / Park Yörüngesi Girişi) - En İyi Sonuç');
hold on;
[x_m_sphere,y_m_sphere,z_m_sphere] = sphere(40);
surf(x_m_sphere*R_moon_vis/1e3, y_m_sphere*R_moon_vis/1e3, z_m_sphere*R_moon_vis/1e3,...
     'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.7, 'DisplayName', 'Ay');
plot3(pos_sc_wrt_moon_trajectory(:,1)/1e3, pos_sc_wrt_moon_trajectory(:,2)/1e3, pos_sc_wrt_moon_trajectory(:,3)/1e3, ...
      'r-', 'LineWidth', 1.5, 'DisplayName', 'Uzay Aracı Yörüngesi (Ay''a Göre)');
theta_park_orbit_plot = linspace(0, 2*pi, 200);
radius_park_orbit_plot_km = (R_moon_vis + altitude_moon_park_target_vis) / 1e3;
x_target_park_orbit_km = radius_park_orbit_plot_km * cos(theta_park_orbit_plot);
y_target_park_orbit_km = radius_park_orbit_plot_km * sin(theta_park_orbit_plot);
plot3(x_target_park_orbit_km, y_target_park_orbit_km, zeros(size(x_target_park_orbit_km)), ...
      'g--', 'LineWidth', 2, 'DisplayName', sprintf('Hedef Park Yör. (%.0fkm İrtifa)', altitude_moon_park_target_vis/1e3));
if ~isempty(TE_vis)
    pos_sc_at_event_wrt_moon_vis = YE_vis(end,13:15) - YE_vis(end,7:9);
    plot3(pos_sc_at_event_wrt_moon_vis(1)/1e3, pos_sc_at_event_wrt_moon_vis(2)/1e3, pos_sc_at_event_wrt_moon_vis(3)/1e3, ...
          'p', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'g', 'MarkerSize', 14, ...
          'DisplayName', 'Hedef İrtifaya Varış (LOI Noktası)');
end
title_str_moon1 = sprintf('Uzay Aracının Ay Merkezli Yörüngesi (En İyi Sonuçlu TLI)');
title_str_moon2 = sprintf('Min. İrtifa: %.2f km, Hesaplanan LOI dV: %.2f m/s', min_altitude_overall_vis/1000, best_dV_LOI_magnitude_ms);
title({title_str_moon1, title_str_moon2});
xlabel('X_{Ay} Konumu (Ay Merkezli) [km]'); ylabel('Y_{Ay} Konumu (Ay Merkezli) [km]'); zlabel('Z_{Ay} Konumu (Ay Merkezli) [km]');
legend('show', 'Location', 'northeast'); axis equal; grid on; view(3);
max_plot_range_km = (R_moon_vis + altitude_moon_park_target_vis + 7000e3) / 1e3;
xlim([-max_plot_range_km, max_plot_range_km]); ylim([-max_plot_range_km, max_plot_range_km]); zlim([-max_plot_range_km/2, max_plot_range_km/2]);
hold off;

fprintf('\nDetaylı görselleştirme tamamlandı.\n');
fprintf('Hesaplanan Toplam Yakıt Kütlesi: %.2f kg\n', m_propellant_total);
fprintf('  TLI için yakıt: %.2f kg\n', m_propellant_TLI);
fprintf('  LOI için yakıt: %.2f kg\n', m_propellant_LOI);
fprintf('Uzay Aracının Başlangıç Islak Kütlesi: %.2f kg\n', M_spacecraft_initial_wet_vis);
disp('Bu DeltaV değerleri ve yakıt miktarları, Bölüm I için anlık yanma varsayımıyla hesaplanmıştır.');
