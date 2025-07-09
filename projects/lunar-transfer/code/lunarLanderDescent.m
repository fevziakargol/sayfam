clear, clc, close all;
% Bu betik, Ay park yörüngesinden Ay yüzeyine inişi simüle eder.
% İki aşamalı simülasyon:
% 1. Deorbit sonrası itkisiz alçalma (belirli irtifaya kadar).
% 2. Frenleme yanması ile yüzeye iniş.

% Global değişkenler Rates ve Events fonksiyonları tarafından kullanılacak
global mu_moon Rm m_dry T Isp g0 target_landing_speed landing_velocity braking_altitude
global rates_call_counter event_check_counter % Olay kontrol sayacı eklendi

% --- SABİTLER VE PROJE PARAMETRELERİ ---
G = 6.6742867e-11;  % Evrensel Kütle Çekim Sabiti [N*m^2/kg^2]
Rm = 1.7374000e6;   % Ay'ın yarıçapı [m]
Mm = 7.3457576e22;  % Ay'ın kütlesi [kg]
mu_moon = 4.9027692e12; % Proje değerini kullan [m^3/s^2]
g0 = 9.80665;       % Standart yerçekimi ivmesi [m/s^2] (Isp tanımı için)

m_dry = 1000;       % Uzay aracının kuru kütlesi [kg]
altitude_moon_park = 490e3; % Ay park yörüngesi irtifası [m] (Ahmet Fevzi Akargöl için)
target_landing_speed = 3; % Hedef iniş hızı [m/s]
landing_velocity = inf; % İniş anındaki hızı saklamak için başlangıç değeri

% --- İNİŞ PARAMETRELERİ (Ayarlanabilir) ---
braking_altitude = 63059; % Frenlemeye başlama irtifası [m] (50 km)
Isp = 310;          % Özgül itki [s] (Varsayım)
initial_fuel_guess = 1000; % Başlangıç yakıt tahmini [kg]
T = 6500;           % Sabit İtki [N] (Ayarlanabilir)

m_initial = m_dry + initial_fuel_guess; % Toplam başlangıç kütlesi

% --- BAŞLANGIÇ KOŞULLARI (Ay Park Yörüngesi) ---
r_park = Rm + altitude_moon_park;
v_park_mag = sqrt(mu_moon / r_park); % Dairesel hız büyüklüğü
pos0_circ = [r_park, 0, 0];
vel0_circ = [0, v_park_mag, 0]; % Dairesel hız vektörü

% --- YÖRÜNGEDEN ÇIKIŞ YANMASI (DEORBIT BURN - DeltaV1) ---
% rp_target = Rm; % Hedef periseleneyi yüzeye indirelim
% ra = r_park;
% a_transfer = (ra + rp_target) / 2;
% va_transfer_mag = sqrt(mu_moon * (2/ra - 1/a_transfer));
% deltaV1_mag = v_park_mag - va_transfer_mag; % Hesaplanan değer ~94.7 m/s

% *** DeltaV1 Manuel Olarak Artırıldı ***
deltaV1_mag = 90.0; % m/s (Deneme değeri)

vel0_deorbit = vel0_circ - deltaV1_mag * (vel0_circ / norm(vel0_circ));
y0_coast = [pos0_circ, vel0_deorbit, m_initial]; % Kütle değişimi yok bu aşamada

% --- AŞAMA 1: İTKİSİZ ALÇALMA (FRENLEME İRTİFASINA KADAR) ---
t_orbit = 2*pi*sqrt(r_park^3/mu_moon); % Yaklaşık yörünge periyodu
tspan1 = [0, t_orbit * 1.5]; % Süreyi uzun tutalım (1.5 periyot)
fprintf('Aşama 1 Zaman Aralığı [0, %.2f] saniye\n', tspan1(2));
% *** options basitleştirildi (MaxStep kaldırıldı, toleranslar varsayılan) ***
options1 = odeset('Events', @startBrakingEvent);

fprintf('\n--- Başlangıç Durumu Kontrolü (DeltaV1 Sonrası) ---\n');
fprintf('Başlangıç İrtifası: %.2f km\n', (norm(y0_coast(1:3))-Rm) / 1000);
fprintf('DeltaV1 Miktarı (Manuel): %.4f m/s\n', deltaV1_mag);
fprintf('Başlangıç Hızı (DeltaV1 Sonrası): %.2f m/s\n', norm(y0_coast(4:6)));
fprintf('Başlangıç Kütlesi: %.2f kg\n', y0_coast(7));
fprintf('Frenleme Başlama İrtifası: %.2f km\n', braking_altitude / 1000);
fprintf('------------------------------------\n');

disp('Aşama 1 Başlatılıyor: İtkisiz Alçalma (ode45 kullanılıyor)...');
rates_call_counter = 0; tic; event_check_counter = 0; % Sayaçları sıfırla
[T1, Y1, TE1, YE1, IE1] = ode45(@coastRates, tspan1, y0_coast, options1);
computation_time1 = toc;
fprintf('Aşama 1 Tamamlandı. Süre: %.2f saniye. Olay İndeksi (IE1): %s\n', computation_time1, mat2str(IE1));

% Aşama 1 sonuçlarını kontrol et
if isempty(YE1) || isempty(IE1)
    disp('Aşama 1 Son Durum (Y1(end,:)):');
    disp(Y1(end,:));
    % Grafiği çizdirip bakalım
     if ~isempty(Y1)
        figure('Name', 'Aşama 1 Yörünge (Hata)');
        theta_moon = linspace(0, 2*pi, 100); x_moon = Rm * cos(theta_moon); y_moon = Rm * sin(theta_moon);
        fill(x_moon, y_moon, [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', [0.5 0.5 0.5]); hold on;
        plot(Y1(:,1), Y1(:,2), 'r.-'); % Noktaları da gösterelim
        plot(Y1(1,1), Y1(1,2), 'bo', 'MarkerFaceColor', 'b'); % Başlangıç
        plot(Y1(end,1), Y1(end,2), 'mo', 'MarkerSize', 8); % Bitiş
        title('Aşama 1 Yörünge (Hata)'); xlabel('X Konumu (m)'); ylabel('Y Konumu (m)');
        axis equal; grid on; legend('Ay', 'Yörünge', 'Başlangıç (dV1 sonrası)', 'Simülasyon Sonu'); hold off;
    end
    error('Hata: Aşama 1 frenleme irtifasına ulaşamadı. tspan1 çok kısa veya DeltaV1/braking_altitude yanlış.');
end
y0_brake = YE1(end, :); % Frenleme başlangıç durumu
t_start_brake = TE1(end); % Frenleme başlangıç zamanı

% --- AŞAMA 2: FRENLEME YANMASI (YÜZEYE KADAR) ---
% (Bu kısım Aşama 1 başarılı olursa çalışacak)
t_burn_duration_max = (y0_brake(7) - m_dry) / (T / (Isp * g0));
tspan2 = [t_start_brake, t_start_brake + t_burn_duration_max * 1.5];
options2 = odeset('RelTol',1e-8,'AbsTol', 1e-8, 'Events', @stopAtMoonSurface);
max_step_size = 5;
options2 = odeset(options2, 'MaxStep', max_step_size);

fprintf('\n--- Frenleme Başlangıç Durumu ---\n');
fprintf('Zaman: %.2f s\n', t_start_brake);
fprintf('İrtifa: %.2f m\n', norm(y0_brake(1:3))-Rm);
fprintf('Hız: %.2f m/s\n', norm(y0_brake(4:6)));
fprintf('Kütle: %.2f kg\n', y0_brake(7));
fprintf('------------------------------------\n');

disp('Aşama 2 Başlatılıyor: Frenleme Yanması (ode45 kullanılıyor)...');
rates_call_counter = 0; tic; event_check_counter = 0; % Sayaçları sıfırla
[T2, Y2, TE2, YE2, IE2] = ode45(@descentRatesWithThrust, tspan2, y0_brake, options2);
computation_time2 = toc;
fprintf('Aşama 2 Tamamlandı. Süre: %.2f saniye. Olay İndeksi (IE2): %s\n', computation_time2, mat2str(IE2));

% --- SONUÇLARIN BİRLEŞTİRİLMESİ VE İŞLENMESİ ---
T_sol = [T1; T2(2:end)];
Y_sol = [Y1; Y2(2:end,:)];

if ~isempty(YE2)
    final_state = YE2(end, :); final_time = TE2(end);
    final_pos = final_state(1:3); final_vel = final_state(4:6);
    final_mass = final_state(7);
    final_altitude = norm(final_pos) - Rm;
    final_speed = landing_velocity;
    fuel_consumed = m_initial - final_mass;

    fprintf('\n--- Simülasyon Sonuçları (İniş) ---\n');
    fprintf('Toplam Süre: %.2f saniye (%.2f dakika)\n', final_time, final_time/60);
    fprintf('Son İrtifa: %.2f m\n', final_altitude);
    fprintf('Son Hız (iniş anı): %.2f m/s\n', final_speed);
    fprintf('Harcanan Yakıt: %.2f kg\n', fuel_consumed);
    fprintf('Kalan Kütle: %.2f kg\n', final_mass);

    if final_altitude < 1 && final_speed <= target_landing_speed
        fprintf('İniş Başarılı! Hedef hıza ulaşıldı.\n');
    elseif final_altitude < 1 && final_speed > target_landing_speed
        fprintf('UYARI: İniş yapıldı ancak hız çok yüksek! (%.2f m/s)\n', final_speed);
    else
        fprintf('UYARI: Yüzeye iniş yapılamadı veya başka bir sorun oluştu (İrtifa: %.2f m).\n', final_altitude);
    end

    % --- GRAFİK ÇİZDİRME ---
    figure('Name', 'Ay İniş Yörüngesi (Frenlemeli)');
    theta_moon = linspace(0, 2*pi, 100);
    x_moon = Rm * cos(theta_moon); y_moon = Rm * sin(theta_moon);
    fill(x_moon, y_moon, [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', [0.5 0.5 0.5]);
    hold on;
    plot(Y_sol(:,1), Y_sol(:,2), 'r');
    plot(Y1(1,1), Y1(1,2), 'bo', 'MarkerFaceColor', 'b');
    plot(Y1(end,1), Y1(end,2), 'ms', 'MarkerFaceColor', 'm');
    plot(Y2(end,1), Y2(end,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    title('Ay İniş Yörüngesi (Frenlemeli)'); xlabel('X Konumu (m)'); ylabel('Y Konumu (m)');
    axis equal; grid on;
    legend('Ay', 'Yörünge', 'Başlangıç (dV1 sonrası)', 'Frenleme Başlangıcı', 'İniş Noktası'); hold off;

    figure('Name', 'İniş Zaman Grafikleri (Frenlemeli)');
    subplot(3,1,1); plot(T_sol, sqrt(Y_sol(:,1).^2 + Y_sol(:,2).^2 + Y_sol(:,3).^2) - Rm);
    title('İrtifa Zaman Grafiği'); xlabel('Zaman (s)'); ylabel('İrtifa (m)'); grid on;
    vline(t_start_brake, '--' , 'Frenleme Başlangıcı');
    subplot(3,1,2); plot(T_sol, sqrt(Y_sol(:,4).^2 + Y_sol(:,5).^2 + Y_sol(:,6).^2));
    title('Hız Zaman Grafiği'); xlabel('Zaman (s)'); ylabel('Hız (m/s)'); grid on;
    vline(t_start_brake, '--', 'Frenleme Başlangıcı');
    subplot(3,1,3); plot(T_sol, Y_sol(:,7));
    title('Kütle Zaman Grafiği'); xlabel('Zaman (s)'); ylabel('Kütle (kg)'); grid on;
    vline(t_start_brake, '--', 'Frenleme Başlangıcı');

else
    disp('Simülasyon (Aşama 2) bir olayla durdurulmadan tamamlandı veya hata oluştu.');
    fprintf('UYARI: Yüzeye iniş olayı tetiklenmedi.\n');
    if ~isempty(Y_sol) && size(Y_sol,1) > 1
        figure('Name', 'Ay İniş Yörüngesi (Aşama 2 Olay Yok)');
        theta_moon = linspace(0, 2*pi, 100);
        x_moon = Rm * cos(theta_moon); y_moon = Rm * sin(theta_moon);
        fill(x_moon, y_moon, [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', [0.5 0.5 0.5]); hold on;
        plot(Y_sol(:,1), Y_sol(:,2), 'r');
        plot(Y1(1,1), Y1(1,2), 'bo', 'MarkerFaceColor', 'b');
        plot(Y1(end,1), Y1(end,2), 'ms', 'MarkerFaceColor', 'm');
        plot(Y_sol(end,1), Y_sol(end,2), 'mo', 'MarkerSize', 8);
        title('Ay İniş Yörüngesi (Aşama 2 Olay Yok)'); xlabel('X Konumu (m)'); ylabel('Y Konumu (m)');
        axis equal; grid on;
        legend('Ay', 'Yörünge', 'Başlangıç (dV1 sonrası)', 'Frenleme Başlangıcı', 'Simülasyon Sonu'); hold off;
    end
end

% === YARDIMCI FONKSİYONLAR ===

% Aşama 1 için Rates Fonksiyonu (İtkisiz)
function dy = coastRates(t, y)
    global mu_moon Rm rates_call_counter
    rates_call_counter = rates_call_counter + 1;
    pos = y(1:3); vel = y(4:6); m = y(7); r = norm(pos);
    dy = zeros(7,1); dy(1:3) = vel;
    F_gravity = -mu_moon * m / r^3 * pos;
    accel = F_gravity / m; dy(4:6) = accel; dy(7) = 0;
    if mod(rates_call_counter, 1000) == 0
         fprintf('Aşama 1: t = %.2f s, İrtifa = %.2f km, Hız = %.2f m/s\n', ...
                 t, (r-Rm)/1000, norm(vel));
    end
end

% Aşama 2 için Rates Fonksiyonu (İtkili)
function dy = descentRatesWithThrust(t, y)
    global mu_moon Rm m_dry T Isp g0 rates_call_counter
    rates_call_counter = rates_call_counter + 1;
    pos = y(1:3); vel = y(4:6); m = y(7); r = norm(pos);
    dy = zeros(7,1); dy(1:3) = vel;
    F_gravity = -mu_moon * m / r^3 * pos;
    F_thrust = [0; 0; 0]; T_effective = 0; current_thrust_status = 0;
    if m > m_dry % Sadece yakıt varsa itki uygula
        T_effective = T;
        vel_norm = norm(vel);
        if vel_norm > 1e-9
            u_T = -vel / vel_norm; % Retro-thrust
            current_thrust_status = 1;
        else
            u_T = [0; 0; 0]; T_effective = 0;
            current_thrust_status = 0;
        end
        F_thrust = T_effective * u_T;
    end
    F_net = F_gravity + F_thrust; accel = F_net / m; dy(4:6) = accel;
    if current_thrust_status > 0
        dy(7) = -T_effective / (Isp * g0); else dy(7) = 0; end
    if mod(rates_call_counter, 500) == 0
         fprintf('Aşama 2: t = %.2f s, İrtifa = %.2f km, Hız = %.2f m/s, Kütle = %.2f kg, İtki: %d\n', ...
                 t, (r-Rm)/1000, norm(vel), m, current_thrust_status);
    end
end

% Aşama 1 için Olay Fonksiyonu (Frenleme Başlangıcı)
function [value, isterminal, direction] = startBrakingEvent(t, y)
    global Rm braking_altitude event_check_counter
    event_check_counter = event_check_counter + 1; % Sayaç
    pos = y(1:3); r = norm(pos);
    current_altitude = r - Rm;
    value = current_altitude - braking_altitude; % Frenleme irtifasına ulaşıldığında sıfır olur
    isterminal = 1; % Durdur
    direction = -1; % Azalırken

    % Her 100 olay kontrolünde bir yazdır
    if mod(event_check_counter, 100) == 0
        fprintf('Olay Kontrol 1: t = %.2f s, Mevcut İrtifa = %.2f km, Hedef = %.2f km, Value = %.2f m\n', ...
                t, current_altitude/1000, braking_altitude/1000, value);
    end
end

% Aşama 2 için Olay Fonksiyonu (Yüzeye İniş)
function [value, isterminal, direction] = stopAtMoonSurface(t, y)
    global Rm target_landing_speed landing_velocity
    pos = y(1:3); vel = y(4:6); r = norm(pos); v = norm(vel);
    value = r - Rm; % İrtifa
    isterminal = 1; % Durdur
    direction = -1; % Azalırken
    landing_velocity = v; % Hızı kaydet
end

% Zaman grafiğine dikey çizgi eklemek için yardımcı fonksiyon
function vline(x, style, label)
    ax = gca;
    if ~isgraphics(ax, 'axes') || isempty(get(ax, 'Children'))
        return;
    end
    ylim = get(ax, 'YLim');
    if numel(ylim) ~= 2, ylim = [-1 1]; end
    line([x x], ylim, 'LineStyle', style, 'Color', 'k');
    text_y_pos = ylim(2) + 0.01 * diff(ylim);
    if ~isfinite(text_y_pos)
        text_y_pos = ylim(1) + 0.99 * diff(ylim);
    end
    text(x, text_y_pos, label, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

% Ana betik dosyasının sonu
