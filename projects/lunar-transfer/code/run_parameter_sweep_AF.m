% UZB461E - Ay Görev Yörüngeleri - Dönem Projesi - Bölüm I
% Öğrenci: Ahmet Fevzi Akargöl
% Betik: run_parameter_sweep_AF
% Açıklama: Bu betik, Ay'a transfer yörüngesi için Translunar Injection (TLI)
%           parametrelerini (TLI açısı ve TLI deltaV büyüklüğü) belirlenen
%           aralıklarda sistematik olarak dener. Her bir parametre kombinasyonu
%           için `simulate_lunar_transfer_AF` fonksiyonunu çağırarak Ay'a minimum
%           yaklaşma irtifasını hesaplar. Amaç, öğrenciye özel hedef Ay park
%           irtifasına (490 km) en yakın sonucu veren TLI parametrelerini bulmaktır.
%
% Gerekli Dosyalar:
%   - simulate_lunar_transfer_AF.m (Bu yardımcı fonksiyon aynı klasörde olmalıdır)
%   - SystemsOfEquations.m (Dinamik denklemler fonksiyonu, düzeltilmiş olmalıdır)
%   - lunarEvents.m (Olay fonksiyonu, güncellenmiş olmalıdır)

clear; clc; close all;

fprintf('Parametre Tarama Betiği Başlatılıyor...\n');
disp('Bu betik, en uygun TLI parametrelerini bulmak için birden fazla simülasyon çalıştıracaktır.');
disp('Çalışma süresi, denenen parametre sayısına ve bilgisayar hızına bağlı olarak değişebilir.');

% --- PARAMETRE ARALIKLARI VE ADIMLARI ---
% Bu değerler, daha önceki denemelerden elde edilen "en iyi" sonuçlar
% etrafında daha hassas bir arama yapmak üzere ayarlanmıştır.
% Kullanıcı bu aralıkları ve adım büyüklüklerini daha da daraltabilir veya genişletebilir.

% TLI Açısı (tli_alpha_deg) için denenecek aralık ve adım
alpha_start_deg_sweep = 233;  % Başlangıç açısı [derece]
alpha_end_deg_sweep = 237;    % Bitiş açısı [derece]
alpha_step_deg_sweep = 0.5;   % Açı adımı [derece]
tli_alpha_range_deg_sweep = alpha_start_deg_sweep : alpha_step_deg_sweep : alpha_end_deg_sweep;

% TLI DeltaV Büyüklüğü (dV_TLI_magnitude) için denenecek aralık ve adım
deltaV_start_m_s_sweep = 3080.0; % Başlangıç deltaV [m/s]
deltaV_end_m_s_sweep = 3083.0;   % Bitiş deltaV [m/s]
deltaV_step_m_s_sweep = 0.25;    % DeltaV adımı [m/s]
dV_TLI_range_m_s_sweep = deltaV_start_m_s_sweep : deltaV_step_m_s_sweep : deltaV_end_m_s_sweep;

% Her bir deneme için kullanılacak sabit simülasyon süresi [gün]
% Olay fonksiyonu (`lunarEvents`) simülasyonu daha erken bitirebilir.
simulation_duration_days_sweep = 6;

% Hedef Ay Park İrtifası (Ahmet Fevzi Akargöl için) [m]
target_moon_altitude_m_sweep = 490e3;

fprintf('\nDenenecek TLI Açıları (Derece): [%.2f : %.2f : %.2f]\n', alpha_start_deg_sweep, alpha_step_deg_sweep, alpha_end_deg_sweep);
fprintf('Denenecek TLI DeltaV Değerleri (m/s): [%.2f : %.2f : %.2f]\n', deltaV_start_m_s_sweep, deltaV_step_m_s_sweep, deltaV_end_m_s_sweep);
fprintf('Her simülasyon için maksimum süre: %.1f gün\n', simulation_duration_days_sweep);
fprintf('Hedef Ay Park İrtifası: %.0f km\n\n', target_moon_altitude_m_sweep/1000);

% --- Sonuçların Saklanması İçin Yapılar ---
num_alpha_points = length(tli_alpha_range_deg_sweep);
num_deltaV_points = length(dV_TLI_range_m_s_sweep);
total_simulations = num_alpha_points * num_deltaV_points;

if total_simulations == 0
    error('Denenecek parametre aralığı boş. Lütfen başlangıç, bitiş ve adım değerlerini kontrol edin.');
end

% Sonuçları saklamak için matris: [Alfa, DeltaV, Minİrtifa_km, Hata_km, LOI_dV_kavramsal]
results_log_matrix = NaN(total_simulations, 5);
current_simulation_index = 0;

% En iyi sonucu saklamak için yapı
best_simulation_result = struct(...
    'alpha_deg', NaN, ...
    'deltaV_m_s', NaN, ...
    'min_altitude_m', inf, ...          % Minimum irtifayı bulmak için sonsuzla başlat
    'error_to_target_m', inf, ...       % Hedefe olan hatayı minimize etmek için sonsuzla başlat
    'conceptual_LOI_dV_m_s', NaN, ...
    'event_triggered_flag', false, ...
    'event_time_s', NaN ...
);

fprintf('Parametre tarama döngüleri başlatılıyor. Toplam %d simülasyon çalıştırılacak...\n', total_simulations);
overall_sweep_start_time = tic;

% --- PARAMETRE TARAMA DÖNGÜLERİ ---
for alpha_idx = 1:num_alpha_points
    current_tli_alpha = tli_alpha_range_deg_sweep(alpha_idx);
    
    for deltaV_idx = 1:num_deltaV_points
        current_tli_deltaV = dV_TLI_range_m_s_sweep(deltaV_idx);
        current_simulation_index = current_simulation_index + 1;
        
        fprintf('Çalışan Simülasyon %d / %d: TLI Açısı=%.2f derece, TLI DeltaV=%.2f m/s\n', ...
                current_simulation_index, total_simulations, current_tli_alpha, current_tli_deltaV);
        
        % Yardımcı simülasyon fonksiyonunu çağır (metin çıktılarını bastırarak)
        [sim_min_alt_m, ~, ~, sim_loi_dv, sim_event_info] = ...
            simulate_lunar_transfer_AF(current_tli_alpha, current_tli_deltaV, simulation_duration_days_sweep, true); % suppress_text_output = true
        
        % Sonuçları log matrisine kaydet
        results_log_matrix(current_simulation_index, 1) = current_tli_alpha;
        results_log_matrix(current_simulation_index, 2) = current_tli_deltaV;
        results_log_matrix(current_simulation_index, 3) = sim_min_alt_m / 1000; % km
        
        sim_current_error_m = abs(sim_min_alt_m - target_moon_altitude_m_sweep);
        results_log_matrix(current_simulation_index, 4) = sim_current_error_m / 1000; % km
        results_log_matrix(current_simulation_index, 5) = sim_loi_dv;

        fprintf('  -> Sonuç: Min İrtifa=%.2f km, Hedefe Uzaklık=%.2f km, Kavramsal LOI dV=%.2f m/s, Olay Tetiklendi=%s (Olay Zamanı=%.2fs)\n', ...
                sim_min_alt_m/1000, sim_current_error_m/1000, sim_loi_dv, mat2str(sim_event_info.triggered), sim_event_info.time);
        
        % En iyi sonucu güncelle:
        % - Minimum irtifa pozitif olmalı (Ay'a çarpmamalı).
        % - Hedefe olan hata mevcut en iyi hatadan daha düşük olmalı.
        if sim_min_alt_m > 0 && sim_current_error_m < best_simulation_result.error_to_target_m
            best_simulation_result.alpha_deg = current_tli_alpha;
            best_simulation_result.deltaV_m_s = current_tli_deltaV;
            best_simulation_result.min_altitude_m = sim_min_alt_m;
            best_simulation_result.error_to_target_m = sim_current_error_m;
            best_simulation_result.conceptual_LOI_dV_m_s = sim_loi_dv;
            best_simulation_result.event_triggered_flag = sim_event_info.triggered;
            best_simulation_result.event_time_s = sim_event_info.time;
            
            fprintf('  >>>> YENİ EN İYİ SONUÇ BULUNDU! <<<<\n');
        end
    end
end

overall_sweep_end_time = toc(overall_sweep_start_time);
fprintf('\nParametre taraması tamamlandı. Toplam geçen süre: %.2f saniye (yaklaşık %.2f dakika).\n', ...
        overall_sweep_end_time, overall_sweep_end_time/60);

% --- EN İYİ SONUCUN RAPORLANMASI ---
fprintf('\n----------- EN İYİ BULUNAN PARAMETRELER VE SONUÇLAR -----------\n');
if ~isnan(best_simulation_result.alpha_deg)
    fprintf('En İyi TLI Açısı (Derece):                %.2f\n', best_simulation_result.alpha_deg);
    fprintf('En İyi TLI DeltaV (m/s):                 %.2f\n', best_simulation_result.deltaV_m_s);
    fprintf('Ulaşılan Minimum İrtifa (Ay yüzeyine, km): %.2f\n', best_simulation_result.min_altitude_m / 1000);
    fprintf('Hedef İrtifaya (%.0f km) Uzaklık (km):      %.2f\n', target_moon_altitude_m_sweep/1000, best_simulation_result.error_to_target_m / 1000);
    fprintf('Kavramsal LOI DeltaV (m/s):              %.2f\n', best_simulation_result.conceptual_LOI_dV_m_s);
    fprintf('Olay Fonksiyonu Tetiklendi mi:             %s\n', mat2str(best_simulation_result.event_triggered_flag));
    if best_simulation_result.event_triggered_flag
        fprintf('Olayın Tetiklenme Zamanı (saniye):         %.2f (%.2f gün)\n', best_simulation_result.event_time_s, best_simulation_result.event_time_s/(24*3600));
    end
    
    disp('-----------------------------------------------------------------');
    disp('Bu en iyi parametrelerle detaylı bir simülasyon ve grafik çizimi için');
    disp('`visualize_best_lunar_transfer_AF.m` betiğini veya benzer bir');
    disp('görselleştirme betiğini bu değerlerle çalıştırabilirsiniz.');
else
    fprintf('Tarama sonucunda uygun bir çözüm bulunamadı.\n');
    fprintf('Lütfen parametre aralıklarını, adım büyüklüklerini ve\n');
    fprintf('`SystemsOfEquations.m` ile `lunarEvents.m` dosyalarınızın doğruluğunu kontrol edin.\n');
end

% İsteğe bağlı: Tüm sonuçları bir MAT dosyasına kaydetme
results_output_filename = sprintf('TaramaSonuclari_Alfa_%.1f-%.1f_dV_%.1f-%.1f.mat', ...
                           alpha_start_deg_sweep, alpha_end_deg_sweep, deltaV_start_m_s_sweep, deltaV_end_m_s_sweep);
try
    save(results_output_filename, 'results_log_matrix', 'best_simulation_result', ...
         'tli_alpha_range_deg_sweep', 'dV_TLI_range_m_s_sweep', 'target_moon_altitude_m_sweep');
    fprintf('\nTüm tarama sonuçları `%s` dosyasına kaydedildi.\n', results_output_filename);
catch ME_save
    fprintf('\nTarama sonuçları dosyaya kaydedilemedi: %s\n', ME_save.message);
end

fprintf('\n`run_parameter_sweep_AF.m` Betiği Çalışmasını Tamamladı.\n');
