% ------------------------------------------------------------------------
% UZB461E - Ay Görev Yörüngeleri - Dönem Projesi
% AY'DAN YÜKSEK İRTİFAYA KALKIŞ VE TEK NOKTADA YÖRÜNGE DAİRESELLEŞTİRME (Rev. 2)
% Öğrenci: Ahmet Fevzi Akargöl
%
% Bu betik:
% 1. Ay yüzeyinden, hedef dairesel yörünge irtifasına (490km) ulaşmayı
%    hedefleyen bir kalkışı simüle eder (Artırılmış yakıtla).
% 2. Bu kalkışın sonunda ulaşılan yörünge ("Nokta A") muhtemelen eliptiktir.
% 3. "Nokta A"da, yörüngeyi dairesel hale getirmek için tek bir anlık
%    düzeltme itkisi hesaplar.
% 4. Tüm aşamaları detaylı olarak görselleştirir.
% ------------------------------------------------------------------------
clear; clc; close all;
fprintf('AY''DAN YÜKSEK İRTİFAYA KALKIŞ VE YÖRÜNGE DAİRESELLEŞTİRME SİMÜLASYONU (Rev. 2) BAŞLATILIYOR...\n\n');

% --- SABİTLER VE GENEL PROJE PARAMETRELERİ ---
G_const = 6.6742867e-11;      
Rm_asc = 1.7374000e6;         % Ay'ın yarıçapı [m]
mu_moon_asc = 4.9027692e12;   % Ay kütle çekim parametresi [m^3/s^2]
g0_asc = 9.80665;             % Standart yerçekimi ivmesi [m/s^2]
m_dry_asc = 1000;             % Uzay aracının kuru kütlesi [kg]
Isp_asc = 310;                % Özgül itki [s]
exhaust_velocity = Isp_asc * g0_asc; % [m/s]
T_asc = 13000;                % [N] Ana Yükseliş (Kalkış) İtkisi

% --- HEDEF DAİRESEL YÖRÜNGE (ÖNCEDEN TANIMLI) ---
target_altitude_circ_km = 490; % km
r_circ_target_final = Rm_asc + target_altitude_circ_km * 1000; % m
v_circ_target_final_speed = sqrt(mu_moon_asc / r_circ_target_final); % m/s

% --- KALKIŞ (ANA YÜKSELİŞ - PART IIa) PARAMETRELERİ ---
initial_fuel_load_main_ascent_to_use = 4000; % kg (ARTIRILDI - Daha yüksek irtifa için)
m0_ascent = m_dry_asc + initial_fuel_load_main_ascent_to_use; % [kg] Kalkıştaki toplam kütle

% Apollo benzeri rehberlik parametreleri (yaklaşık değerler)
H_VERTICAL_LIFTOFF_END = 1000; % m (Dikey kalkışın bittiği irtifa)
PITCH_INITIAL_ASCENT = 75;   % derece (Yunuslama başlangıç açısı, yataya göre)
PITCH_FINAL_AT_INSERTION = 1; % derece (Yörüngeye girişteki son yunuslama açısı, yataya çok yakın)
INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT = target_altitude_circ_km * 1000; % m (örn. 490 km)
RADIAL_VEL_TARGET_FOR_APSIS = 10; % m/s (Bu irtifada hedeflenen düşük radyal hız - Event fonksiyonunda doğrudan kullanılmıyor ama konsept olarak var)

% Global değişkenlerin ODE fonksiyonları için tanımlanması
global GLOBAL_mu_moon_asc GLOBAL_Rm_asc GLOBAL_m_dry_asc GLOBAL_T_asc GLOBAL_Isp_asc GLOBAL_g0_asc ...
       GLOBAL_H_VERTICAL_LIFTOFF_END GLOBAL_PITCH_INITIAL_ASCENT GLOBAL_PITCH_FINAL_AT_INSERTION ...
       GLOBAL_INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT GLOBAL_RADIAL_VEL_TARGET_FOR_APSIS;

GLOBAL_mu_moon_asc = mu_moon_asc;
GLOBAL_Rm_asc = Rm_asc;
GLOBAL_m_dry_asc = m_dry_asc;
GLOBAL_T_asc = T_asc;
GLOBAL_Isp_asc = Isp_asc;
GLOBAL_g0_asc = g0_asc;
GLOBAL_H_VERTICAL_LIFTOFF_END = H_VERTICAL_LIFTOFF_END;
GLOBAL_PITCH_INITIAL_ASCENT = PITCH_INITIAL_ASCENT;
GLOBAL_PITCH_FINAL_AT_INSERTION = PITCH_FINAL_AT_INSERTION;
GLOBAL_INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT = INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT;
GLOBAL_RADIAL_VEL_TARGET_FOR_APSIS = RADIAL_VEL_TARGET_FOR_APSIS;

% --- AY YÜZEYİNDEN KALKIŞ SİMÜLASYONU (PART IIa) ---
fprintf('--- AY YÜZEYİNDEN KALKIŞ SİMÜLASYONU (HEDEF İRTİFA: %.0f km, BAŞLANGIÇ YAKIT: %.0f kg) BAŞLATILIYOR ---\n', INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT/1000, initial_fuel_load_main_ascent_to_use);
Y0_ascent = [Rm_asc; 0; 0;  0; 0; 0;  m0_ascent]; 
t_start_ascent = 0;
t_end_ascent = 3000; % s (Maksimum simülasyon süresi, artırılmış yakıt için gerekebilir)
tspan_ascent = [t_start_ascent t_end_ascent];

options_ascent = odeset('Events', @events_for_target_altitude_v3, 'RelTol', 1e-8, 'AbsTol', 1e-8);
[T_sol_ma, Y_sol_ma, TE_ma, YE_ma, IE_ma] = ode45(@ascentRates_to_Intermediate_v2, tspan_ascent, Y0_ascent, options_ascent);

if isempty(TE_ma)
    fprintf('UYARI: Kalkış simülasyonu tanımlanan olaylarla sonlanmadı.\n');
    if Y_sol_ma(end,7) <= GLOBAL_m_dry_asc + 1e-2 % Son noktada yakıt bittiyse (küçük tolerans)
        fprintf('SEBEP: Yakıt tükendi (Maksimum simülasyon süresine ulaşılmadan önce).\n');
    else
        fprintf('SEBEP: Maksimum simülasyon süresine (%.0f s) ulaşıldı.\n', t_end_ascent);
    end
else
    fprintf('Kalkış simülasyonu olay ile sonlandı (Olay Indeksi: %d, Zaman: %.2f s).\n', IE_ma(1), TE_ma(1));
     if IE_ma(1) == 2 % Yakıt bitme olayı
        fprintf('SEBEP: Yakıt tükendi.\n');
    elseif IE_ma(1) == 1 % Hedef irtifa olayı
        fprintf('SEBEP: Hedef irtifaya (%.0f km) ulaşıldı (veya yaklaşıldı).\n', (norm(YE_ma(1,1:3))-Rm_asc)/1000);
    end
end

pos_A_vec = Y_sol_ma(end, 1:3)';
vel_A_vec = Y_sol_ma(end, 4:6)';
mass_at_A = Y_sol_ma(end, 7);

r_A = norm(pos_A_vec);
alt_A_km = (r_A - Rm_asc)/1000;
Vr_A_initial = dot(vel_A_vec, pos_A_vec/r_A); 
vel_mag_sq_A = vel_A_vec(1)^2 + vel_A_vec(2)^2 + vel_A_vec(3)^2;
if (vel_mag_sq_A - Vr_A_initial^2) < -1e-6 
    Vt_A_initial = sqrt(vel_mag_sq_A); 
    fprintf('UYARI: Vt_A_initial hesaplanırken Vr > V! Vt_A_initial = Vtotal olarak ayarlandı.\n');
else
    Vt_A_initial = sqrt(max(0, vel_mag_sq_A - Vr_A_initial^2));
end

fuel_consumed_main_ascent = initial_fuel_load_main_ascent_to_use - (mass_at_A - m_dry_asc);
% Yakıt tüketimi negatif olamaz veya başlangıçtan fazla olamaz
fuel_consumed_main_ascent = max(0, min(fuel_consumed_main_ascent, initial_fuel_load_main_ascent_to_use));


fprintf('\n--- KALKIŞ SONRASI DURUM (YENİ NOKTA A) ---\n');
fprintf('  Ulaşılan İrtifa (alt_A): %.3f km\n', alt_A_km);
fprintf('  Yarıçap (r_A): %.0f m\n', r_A);
fprintf('  Radyal Hız (Vr_A_initial): %.4f m/s\n', Vr_A_initial);
fprintf('  Teğetsel Hız (Vt_A_initial): %.2f m/s\n', Vt_A_initial);
fprintf('  Kütle (mass_A): %.2f kg\n', mass_at_A);
fprintf('  Ana yükselişte harcanan yakıt: %.2f kg\n\n', fuel_consumed_main_ascent);

% --- NOKTA A'DAKİ YÖRÜNGENİN PARAMETRELERİ (Görselleştirme için) ---
a_initial_ellipse = NaN; e_initial_ellipse = NaN; omega_initial_ellipse = NaN; nu_A_at_PointA = NaN;
v_A_sq = Vr_A_initial^2 + Vt_A_initial^2;
eps_energy_A = v_A_sq/2 - mu_moon_asc/r_A;

if eps_energy_A >= -1e-3 % Neredeyse parabolik veya hiperbolik için küçük bir tolerans
    fprintf('UYARI: Nokta A''daki enerjiye göre yörünge eliptik değil veya sınıra çok yakın (Enerji=%.2e J/kg).\n', eps_energy_A);
    if eps_energy_A ~= 0
       a_initial_ellipse = -mu_moon_asc / (2*eps_energy_A); 
    else
       a_initial_ellipse = inf; 
    end
    h_A_val = r_A * Vt_A_initial;
    e_initial_ellipse = sqrt(max(0, 1 + (2*eps_energy_A*h_A_val^2)/(mu_moon_asc^2 + eps))); % eps to avoid div by zero if mu is tiny (not here)
     if e_initial_ellipse < 1 && a_initial_ellipse > 0 && isfinite(a_initial_ellipse)
        fprintf('  Durum eliptik olarak değerlendirildi: a=%.0fkm, e=%.4f\n', a_initial_ellipse/1000, e_initial_ellipse);
    else
        fprintf('  Yörünge eliptik değil: e=%.4f. Başlangıç elipsi çizilemeyecek.\n', e_initial_ellipse);
        a_initial_ellipse = NaN; 
    end
else 
    a_initial_ellipse = -mu_moon_asc / (2*eps_energy_A);
    h_A_val = r_A * Vt_A_initial; 
    e_initial_ellipse_sq = 1 + (2*eps_energy_A*h_A_val^2)/(mu_moon_asc^2);
    if e_initial_ellipse_sq < 0; e_initial_ellipse_sq = 0; end 
    e_initial_ellipse = sqrt(e_initial_ellipse_sq);

    if e_initial_ellipse >= 1
        fprintf('UYARI: Nokta A''daki yörünge için hesaplanan eksantriklik >= 1 (e=%.4f). Başlangıç elipsi çizilemeyecek.\n', e_initial_ellipse);
        a_initial_ellipse = NaN; 
    else
        cos_nu_A = (h_A_val^2 / (mu_moon_asc * r_A) - 1) / (e_initial_ellipse + eps);
        sin_nu_A = (Vr_A_initial * h_A_val) / (mu_moon_asc * (e_initial_ellipse + eps) );
        cos_nu_A = max(-1, min(1, cos_nu_A)); 
        sin_nu_A = max(-1, min(1, sin_nu_A)); 
        nu_A_at_PointA = atan2(sin_nu_A, cos_nu_A);
        fprintf('  Nokta A''daki Yörünge: a=%.0fkm, e=%.4f, nu_A=%.2f rad (%.1f deg)\n', a_initial_ellipse/1000, e_initial_ellipse, nu_A_at_PointA, rad2deg(nu_A_at_PointA));
        fprintf('    Periselen İrt.: %.1f km, Aposelen İrt.: %.1f km\n', (a_initial_ellipse*(1-e_initial_ellipse)-Rm_asc)/1000, (a_initial_ellipse*(1+e_initial_ellipse)-Rm_asc)/1000 );
    end
end

% --- DAİRESELLEŞTİRME İÇİN ANLIK DÜZELTME İTKİSİ (TEK NOKTADA) ---
deltaV_correction = NaN; fuel_correction = NaN;
mass_final_circular = mass_at_A; 
correction_possible = false;
total_additional_fuel = 0; 

% Düzeltme itkisi, Nokta A hedef irtifaya yakınsa ve yörünge eliptikse (veya düzeltilebilirse) ve yakıt varsa anlamlı.
% Hedef irtifaya ulaşıldığını varsayarak (veya yaklaşıldığında) düzeltme yapılır.
if mass_at_A > m_dry_asc + 1e-2 % Kullanılabilir yakıt var mı (çok küçük bir eşik)
    fprintf('\n--- DAİRESELLEŞTİRME DÜZELTME İTKİSİ HESAPLANIYOR (Nokta A''da, %.1fkm irtifada) ---\n', alt_A_km);
    
    v_target_tangential_at_A = sqrt(mu_moon_asc / r_A); 
    
    fprintf('  Nokta A Mevcut Durum: Vr=%.2f m/s, Vt=%.2f m/s\n', Vr_A_initial, Vt_A_initial);
    fprintf('  Nokta A Hedef Durum (r_A''da Dairesel): Vr=0 m/s, Vt_req=%.2f m/s\n', v_target_tangential_at_A);
    
    deltaV_correction = sqrt(Vr_A_initial^2 + (v_target_tangential_at_A - Vt_A_initial)^2);
    
    mass_after_correction = mass_at_A / exp(deltaV_correction / exhaust_velocity);
    fuel_correction = mass_at_A - mass_after_correction;
    if fuel_correction < 0; fuel_correction = 0; end
    
    fprintf('  Düzeltme İtkisi (DeltaV_corr): %.2f m/s\n', deltaV_correction);
    fprintf('  DeltaV_corr için Harcanan Yakıt: %.2f kg\n', fuel_correction);
    
    if (mass_at_A - fuel_correction) >= (m_dry_asc - 1e-3) % Kuru kütleden fazla mı? (Yakıt yetti mi? küçük tolerans)
        mass_final_circular = mass_after_correction;
        fprintf('  Nihai Kütle (Dairesel Yörüngede): %.2f kg\n', mass_final_circular);
        correction_possible = true;
        total_additional_fuel = fuel_correction; 
    else
        fprintf('  UYARI: Düzeltme itkisi için yeterli yakıt kalmadı (kalan kütle=%.2f kg, kuru kütle=%.0f kg).\n', mass_after_correction, m_dry_asc);
        total_additional_fuel = fuel_correction; % Harcanan yakıt bu kadar ama görev başarısız olabilir
        mass_final_circular = mass_after_correction; 
        correction_possible = false; % Görev başarısız sayılabilir
    end
else
    fprintf('\nUYARI: Kalkış sonrası Nokta A''da düzeltme itkisi için yeterli başlangıç yakıtı bulunmuyor (Kalan Kütle: %.2f kg).\n', mass_at_A);
    total_additional_fuel = 0; 
end


fprintf('\n--- NİHAİ GÖREV SONUCU (KALKIŞ + DÜZELTME) ---\n');
if correction_possible && isfinite(total_additional_fuel)
    fprintf('  Toplam Harcanan Yakıt (Ana Yükseliş + Düzeltme): %.2f kg\n', fuel_consumed_main_ascent + total_additional_fuel);
    fprintf('  Son Yörünge: %.0f km dairesel (r_A = %.1f km''de daireselleştirildi)\n', (r_A-Rm_asc)/1000, (r_A-Rm_asc)/1000);
    fprintf('  Son Hız (Teğetsel): %.2f m/s\n', sqrt(mu_moon_asc / r_A)); 
    fprintf('  Son Kütle: %.2f kg\n', mass_final_circular);
else
    fprintf('  Hedeflenen dairesel yörüngeye ulaşılamadı (düzeltme mümkün değil veya yapılmadı).\n');
    fprintf('  Sadece Ana Yükselişte Harcanan Yakıt: %.2f kg\n', fuel_consumed_main_ascent);
    fprintf('  Ana Yükseliş Sonu Ulaşılan İrtifa: %.1f km\n', alt_A_km);
    if mass_at_A <= m_dry_asc + 1e-2
        fprintf('  Ana Yükseliş Sonunda Yakıt Tamamen Tükendi.\n');
    end
end

% --- GÖRSELLEŞTİRME HAZIRLIKLARI ---
pos_A_2D = pos_A_vec(1:2); 

if ~isnan(a_initial_ellipse) && ~isnan(e_initial_ellipse) && ~isnan(nu_A_at_PointA) && ~any(isnan(pos_A_2D))
    theta_A_angle = atan2(pos_A_2D(2), pos_A_2D(1)); 
    omega_initial_ellipse = theta_A_angle - nu_A_at_PointA;
else
    omega_initial_ellipse = NaN; 
end

% --- GÖRSELLEŞTİRME ---
figure('Name', 'Ay''dan Yüksek İrtifaya Kalkış ve Yörünge Daireselleştirme (Rev. 2)');
hold on;
max_r_for_plot = r_A; 
if ~isempty(Y_sol_ma)
    altitudes_ascent_km = (sqrt(Y_sol_ma(:,1).^2 + Y_sol_ma(:,2).^2 + Y_sol_ma(:,3).^2) - Rm_asc)/1000;
    max_r_ascent = max(sqrt(Y_sol_ma(:,1).^2 + Y_sol_ma(:,2).^2 + Y_sol_ma(:,3).^2));
    max_r_for_plot = max(max_r_for_plot, max_r_ascent);
end
if ~isnan(a_initial_ellipse) && ~isnan(e_initial_ellipse) && e_initial_ellipse < 1 && isfinite(a_initial_ellipse)
    max_r_for_plot = max(max_r_for_plot, a_initial_ellipse * (1 + e_initial_ellipse)); 
end
axis_limit_plot = (max_r_for_plot + 200e3)/1000; 


plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8); 
title_main_plot_str = sprintf('Ay Yörünge Manevraları (Ana İtki T=%.0fN)', T_asc);

theta_moon_plot = linspace(0, 2*pi, 200);
x_moon_plot = Rm_asc * cos(theta_moon_plot) / 1000; 
y_moon_plot = Rm_asc * sin(theta_moon_plot) / 1000; 
fill(x_moon_plot, y_moon_plot, [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', [0.5 0.5 0.5], 'DisplayName', 'Ay');

if ~isempty(T_sol_ma) && ~isempty(Y_sol_ma) && size(Y_sol_ma,2) >=2
    plot(Y_sol_ma(:,1)/1000, Y_sol_ma(:,2)/1000, 'r-', 'LineWidth', 2, 'DisplayName', 'Ana Yükseliş (Sürekli İtki)');
    plot(Y_sol_ma(1,1)/1000, Y_sol_ma(1,2)/1000, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize',10, 'DisplayName', 'Kalkış Noktası'); 
end

if ~isnan(a_initial_ellipse) && ~isnan(e_initial_ellipse) && e_initial_ellipse < 1 && ~isnan(omega_initial_ellipse) && isfinite(a_initial_ellipse)
    nu_plot_initial = linspace(0, 2*pi, 400); 
    r_val_initial = (a_initial_ellipse * (1 - e_initial_ellipse^2)) ./ (1 + e_initial_ellipse * cos(nu_plot_initial));
    x_initial_ellipse = r_val_initial .* cos(nu_plot_initial + omega_initial_ellipse) / 1000; 
    y_initial_ellipse = r_val_initial .* sin(nu_plot_initial + omega_initial_ellipse) / 1000; 
    plot(x_initial_ellipse, y_initial_ellipse, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Nokta A Yörüngesi (Düzeltme Öncesi)');
end

if ~any(isnan(pos_A_2D)) 
    plot(pos_A_2D(1)/1000, pos_A_2D(2)/1000, 'ms', 'MarkerFaceColor', 'm', 'MarkerSize', 10, 'DisplayName', sprintf('Nokta A (%.0fkm İrt, Düzeltme İtkisi)', alt_A_km)); 
    text_offset = axis_limit_plot * 0.02; 
    text(pos_A_2D(1)/1000 + text_offset, pos_A_2D(2)/1000 + text_offset, ...
        sprintf(' A (Düzeltme Yeri)\n Vr=%.1fm/s, Vt=%.1fm/s\n dV_{corr}=%.1fm/s', Vr_A_initial, Vt_A_initial, deltaV_correction), ...
        'Color', 'm', 'FontSize',8, 'VerticalAlignment','bottom'); 
end

x_target_orbit_plot = r_circ_target_final * cos(theta_moon_plot) / 1000; 
y_target_orbit_plot = r_circ_target_final * sin(theta_moon_plot) / 1000; 
plot(x_target_orbit_plot, y_target_orbit_plot, 'g:', 'LineWidth', 2, 'DisplayName', sprintf('Hedef Nihai Dairesel Yör. (%.0fkm)',target_altitude_circ_km));

% Düzeltme sonrası yörünge (eğer başarılıysa, hedef yörünge ile çakışır)
if correction_possible && ~isnan(deltaV_correction)
    % Düzeltme r_A'da yapıldığı için, yeni yörünge r_A yarıçapında dairesel olmalı
    r_corrected_circ = r_A;
    x_corrected_orbit_plot = r_corrected_circ * cos(theta_moon_plot) / 1000;
    y_corrected_orbit_plot = r_corrected_circ * sin(theta_moon_plot) / 1000;
    plot(x_corrected_orbit_plot, y_corrected_orbit_plot, 'b-', 'LineWidth', 0.5, 'DisplayName', sprintf('Düzeltilmiş Dairesel Yör. (%.0fkm)', (r_corrected_circ-Rm_asc)/1000));
end


hold off;
axis equal; grid on;
if isnan(axis_limit_plot) || axis_limit_plot <= Rm_asc/1000; axis_limit_plot = (Rm_asc + 500e3)/1000; end % Güvenlik için
xlim([-axis_limit_plot axis_limit_plot]); ylim([-axis_limit_plot axis_limit_plot]);
xlabel('X Konumu (Ay Merkezli) [km]');
ylabel('Y Konumu (Ay Merkezli) [km]');

title_sub_plot_str = sprintf('Ana Yüks. Harcanan Yakıt: %.2fkg, Düzeltme Yakıtı: %.2fkg, Toplam Yakıt: %.2fkg', ...
    fuel_consumed_main_ascent, total_additional_fuel, fuel_consumed_main_ascent + total_additional_fuel);
if isnan(fuel_consumed_main_ascent) && isnan(total_additional_fuel)
    title_sub_plot_str = 'Yakıt bilgileri tam hesaplanamadı.';
elseif isnan(fuel_consumed_main_ascent)
    title_sub_plot_str = sprintf('Düzeltme Yakıtı: %.2fkg', total_additional_fuel);
elseif isnan(total_additional_fuel)
     title_sub_plot_str = sprintf('Ana Yüks. Harcanan Yakıt: %.2fkg (Düzeltme Yapılamadı/Gerekmedi)', fuel_consumed_main_ascent);
end

title({title_main_plot_str; title_sub_plot_str});
legend('show','Location','northeastoutside');


% --- ZAMAN GRAFİKLERİ (ANA YÜKSELİŞ FAZI) ---
if ~isempty(T_sol_ma) && ~isempty(Y_sol_ma) && size(Y_sol_ma,2) >=7
    figure('Name', sprintf('Ana Yükseliş Zaman Grafikleri (T=%.0fN, Başl.Yakıt=%.0fkg)', T_asc, initial_fuel_load_main_ascent_to_use));
    % İrtifa
    subplot(3,1,1);
    plot(T_sol_ma, altitudes_ascent_km, 'LineWidth',1.5); % altitudes_ascent_km yukarıda hesaplandı
    hold on; 
    yline(INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT/1000, 'm--', 'DisplayName', 'Hedef Kalkış İrt.');
    yline(alt_A_km, 'b:', 'DisplayName', 'Ulaşılan "Nokta A" İrt.');
    ylabel('İrtifa (km)'); title(sprintf('İrtifa vs. Zaman (Kalkış Hedef İrt. %.0fkm)', INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT/1000)); grid on; legend('show','Location','best');
    
    % Hız
    subplot(3,1,2);
    velocities_ma = sqrt(Y_sol_ma(:,4).^2 + Y_sol_ma(:,5).^2 + Y_sol_ma(:,6).^2);
    plot(T_sol_ma, velocities_ma, 'LineWidth',1.5);
    hold on;
    v_at_A = sqrt(Vt_A_initial^2 + Vr_A_initial^2);
    yline(v_at_A, 'b:', 'DisplayName', 'Ulaşılan "Nokta A" Hızı');
    ylabel('Hız (m/s)'); title('Hız Büyüklüğü vs. Zaman'); grid on; legend('show','Location','best');
    
    % Kütle
    subplot(3,1,3);
    plot(T_sol_ma, Y_sol_ma(:,7), 'LineWidth',1.5);
    hold on; 
    yline(m_dry_asc, 'r--', 'DisplayName', 'Kuru Kütle'); 
    yline(mass_at_A, 'b:', 'DisplayName', 'Ulaşılan "Nokta A" Kütlesi');
    ylabel('Kütle (kg)'); title('Kütle vs. Zaman'); grid on; xlabel('Zaman (s)'); legend('show','Location','best');
    
    % Global pitch parametrelerinin varlığını kontrol et
    if ~exist('GLOBAL_H_VERTICAL_LIFTOFF_END','var'); GLOBAL_H_VERTICAL_LIFTOFF_END=NaN; end
    if ~exist('GLOBAL_PITCH_INITIAL_ASCENT','var'); GLOBAL_PITCH_INITIAL_ASCENT=NaN; end
    if ~exist('GLOBAL_PITCH_FINAL_AT_INSERTION','var'); GLOBAL_PITCH_FINAL_AT_INSERTION=NaN; end

    sgtitle_text_timeplots = sprintf('Ana Yükseliş Fazı Detayları (T=%.0fN, Başl.Yakıt=%.0fkg)\nHvt=%.1fkm, Pi=%.0f°, Pf_ins=%.0f°',...
                        T_asc, initial_fuel_load_main_ascent_to_use, ...
                        GLOBAL_H_VERTICAL_LIFTOFF_END/1000, GLOBAL_PITCH_INITIAL_ASCENT, GLOBAL_PITCH_FINAL_AT_INSERTION);
    if any(isnan([GLOBAL_H_VERTICAL_LIFTOFF_END, GLOBAL_PITCH_INITIAL_ASCENT, GLOBAL_PITCH_FINAL_AT_INSERTION]))
        sgtitle_text_timeplots = sprintf('Ana Yükseliş Zaman Grafikleri (T=%.0fN, Başl.Yakıt=%.0fkg, Bazı pitch param. eksik)', T_asc, initial_fuel_load_main_ascent_to_use);
    end
    sgtitle(sgtitle_text_timeplots);
end


% --- YARDIMCI FONKSİYONLAR (ODE için) ---
function dy_asc = ascentRates_to_Intermediate_v2(t, y_asc) 
    global GLOBAL_mu_moon_asc GLOBAL_Rm_asc GLOBAL_m_dry_asc GLOBAL_T_asc GLOBAL_Isp_asc GLOBAL_g0_asc ...
           GLOBAL_H_VERTICAL_LIFTOFF_END GLOBAL_PITCH_INITIAL_ASCENT GLOBAL_PITCH_FINAL_AT_INSERTION ...
           GLOBAL_INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT;
    
    pos_vec = y_asc(1:3); 
    vel_vec = y_asc(4:6); 
    m_current = y_asc(7); 
    
    r_norm = norm(pos_vec); 
    if r_norm < eps; r_norm = eps; end 
    current_altitude = r_norm - GLOBAL_Rm_asc;
    
    dy_asc = zeros(7,1);
    dy_asc(1:3) = vel_vec; 
    
    a_gravity_vec = -GLOBAL_mu_moon_asc / r_norm^3 * pos_vec;
    
    unit_thrust_vec = [0;0;0]; 
    actual_thrust_force = 0;   
    
    if m_current > (GLOBAL_m_dry_asc + 1e-3) 
        actual_thrust_force = GLOBAL_T_asc; 
        
        if current_altitude < GLOBAL_H_VERTICAL_LIFTOFF_END 
            if r_norm > 1e-6 
                 unit_thrust_vec = pos_vec / r_norm; 
            else
                 unit_thrust_vec = [1;0;0]; 
            end
        else 
            denominator = (GLOBAL_INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT) - GLOBAL_H_VERTICAL_LIFTOFF_END; 
            if denominator < eps; denominator = eps; end 
            
            fraction_alt_covered = (current_altitude - GLOBAL_H_VERTICAL_LIFTOFF_END) / denominator;
            fraction_alt_covered = min(max(fraction_alt_covered, 0), 1); 
            
            current_pitch_deg_calc = GLOBAL_PITCH_INITIAL_ASCENT - ...
                                (GLOBAL_PITCH_INITIAL_ASCENT - GLOBAL_PITCH_FINAL_AT_INSERTION) * fraction_alt_covered;
            
            phi_pitch_rad = deg2rad(current_pitch_deg_calc); 
            
            % Basit 2D X-Y düzlemi için yerel radyal ve teğetsel vektörler
            pos_xy_norm = norm(pos_vec(1:2) + eps); % XY düzlemindeki izdüşümün normu
            u_r_xy = pos_vec(1:2) / pos_xy_norm; 
            u_theta_xy = [-u_r_xy(2); u_r_xy(1)]; 
            
            u_r_3d = [u_r_xy(1); u_r_xy(2); 0]; % Z=0 varsayımı
            u_theta_simple_3d = [u_theta_xy(1); u_theta_xy(2); 0]; % Z=0 varsayımı
                                                                       
            unit_thrust_vec_pitched = cos(phi_pitch_rad) * u_theta_simple_3d + sin(phi_pitch_rad) * u_r_3d;
            unit_thrust_vec = unit_thrust_vec_pitched / (norm(unit_thrust_vec_pitched) + eps) ;
        end
            
        if norm(unit_thrust_vec) < eps 
            if r_norm > 1e-6; unit_thrust_vec = pos_vec / r_norm; else; unit_thrust_vec = [1;0;0]; end
        end
    end
    
    thrust_force_vec = actual_thrust_force * unit_thrust_vec;
    total_acceleration_vec = a_gravity_vec + thrust_force_vec / m_current;
    dy_asc(4:6) = total_acceleration_vec;
    
    if actual_thrust_force > 0 && m_current > (GLOBAL_m_dry_asc + 1e-3)
        dy_asc(7) = -actual_thrust_force / (GLOBAL_Isp_asc * GLOBAL_g0_asc);
    else
        dy_asc(7) = 0; 
    end
end

function [value, isterminal, direction] = events_for_target_altitude_v3(t, y_asc)
    global GLOBAL_Rm_asc GLOBAL_m_dry_asc ... 
           GLOBAL_INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT; % GLOBAL_RADIAL_VEL_TARGET_FOR_APSIS burada kullanılmıyor
    
    pos = y_asc(1:3);
    m_current = y_asc(7);
    
    r_norm = norm(pos);
    if r_norm < eps; r_norm = eps; end
    current_altitude = r_norm - GLOBAL_Rm_asc;
    
    value = ones(2,1);      
    isterminal = ones(2,1); 
    direction = zeros(2,1); 
    
    % Olay 1: Hedef irtifaya ulaşma
    value(1) = current_altitude - GLOBAL_INTERMEDIATE_TARGET_ALTITUDE_FOR_MAIN_ASCENT;
    direction(1) = 1;  % Sadece irtifa artarken bu irtifayı geçtiğinde tetikle
    % isterminal(1) = 1; % Bu olay her zaman sonlandırıcı

    % Olay 2: Yakıtın bitmesi
    value(2) = m_current - (GLOBAL_m_dry_asc + 1e-3); % Küçük bir eşik yakıt için
    direction(2) = -1; % Sadece kütle azalırken (yakıt biterken) tetikle
    % isterminal(2) = 1; % Bu olay her zaman sonlandırıcı
end