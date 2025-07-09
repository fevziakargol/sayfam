% UZB461E - Ay Görev Yörüngeleri - Dönem Projesi
% Öğrenci: Ahmet Fevzi Akargöl
% Fonksiyon: lunarEvents
% Açıklama: Bu olay fonksiyonu, ODE45 çözücüsü tarafından kullanılır.
%           Uzay aracının Ay yüzeyine olan irtifasını izler ve
%           önceden belirlenmiş bir hedef irtifaya ulaşıldığında
%           simülasyonu durdurmak için bir olay tetikler.

function [value, isterminal, direction] = lunarEvents(t, y_state_vector)
    % Girdiler:
    %   t: Mevcut zaman (skaler).
    %   y_state_vector: O andaki sistem durum vektörü (sütun vektörü).
    %                   Sıralama: [Dünya(konum,hız), Ay(konum,hız), UzayAracı(konum,hız)]
    %                   (Dünya: y(1:6), Ay: y(7:12), Uzay Aracı: y(13:18))
    % Çıktılar:
    %   value: Olayın tetiklenme koşulu. value=0 olduğunda olay gerçekleşir.
    %   isterminal: Olay gerçekleştiğinde simülasyon durdurulsun mu? (1: Evet, 0: Hayır).
    %   direction: Olay 'value'nun sıfıra hangi yönde yaklaşırken tespit edilsin?
    %              (-1: Azalırken, 0: Her iki yönde, 1: Artarken).

    % Gerekli fiziksel sabitler ve proje parametreleri
    R_moon_local = 1.7374000e6;   % Ay Yarıçapı [m]
    altitude_moon_park_target_local = 490e3;  % Hedef Ay park irtifası [m] (Ahmet Fevzi Akargöl için)

    % Durum vektöründen Ay ve Uzay Aracı mutlak konumlarını al
    pos_sc_abs = y_state_vector(13:15);   % Uzay aracının mutlak konumu [x;y;z]
    pos_moon_abs = y_state_vector(7:9);     % Ay'ın mutlak konumu [x;y;z]
    
    % Uzay aracının Ay'a göre konum vektörü ve Ay merkezine olan uzaklığı
    r_sc_wrt_moon_vec = pos_sc_abs - pos_moon_abs;
    dist_to_moon_center = norm(r_sc_wrt_moon_vec);
    
    % Uzay aracının Ay yüzeyine olan anlık irtifası
    current_altitude_from_moon_surface = dist_to_moon_center - R_moon_local;

    % Olayın tetiklenme koşulu: Anlık irtifa, hedef park irtifasına eşitlendiğinde.
    % value = 0 olduğunda, current_altitude_from_moon_surface == altitude_moon_park_target_local.
    value = current_altitude_from_moon_surface - altitude_moon_park_target_local; 
    
    isterminal = 1; % Olay gerçekleştiğinde simülasyonu durdur.
    direction = -1; % 'value' değeri azalırken (yani irtifa hedef irtifaya doğru azalırken)
                    % sıfıra yaklaştığında olayı tetikle. Bu, aracın hedefe ilk ulaştığı anı yakalar.
end
