function f = bandingkan_log(file_robot, file_sim, run_robot, file_png)
%BANDINGKAN_LOG  Gambar log robot asli (unduhan /log.csv) dan log simulasi bertumpuk.
%   bandingkan_log('log_robot.csv', 'hasil/log_lurus_fuzzy.csv')
%   bandingkan_log(..., run_robot)  : pilih nomor percobaan (kolom run). Bawaan: run terakhir
%                                     yang memakai mode otomatis (metode F/P).
%   bandingkan_log(..., run_robot, 'hasil/banding.png') : simpan gambar.
%   Kedua file boleh berformat Excel Indonesia (sep=;) maupun koma biasa.
R = baca_log(file_robot); S = baca_log(file_sim);
if nargin < 3 || isempty(run_robot)
    oto = ismember(R.metode, {'F', 'P'});
    run_robot = max(R.run(oto));
end
R = R(R.run == run_robot, :);
S = S(S.run == S.run(end), :);
tR = (R.ms - R.ms(1)) / 1000; tS = (S.ms - S.ms(1)) / 1000;

f = figure('Name', 'Robot vs simulasi', 'Color', 'w', 'Position', [40 40 1200 900]);
tl = tiledlayout(f, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Log robot asli (run %d, metode %s) vs simulasi (metode %s)', run_robot, R.metode{1}, S.metode{1}));
kol = {'s_kiri', 'e', 'de', 'Kp', 'u', 'sudut', 'pwm', 'depan'};
jud = {'s\_kiri (cm)', 'error e (cm)', '\Deltae (cm)', 'Kp', 'u', 'sudut kemudi (\circ)', 'PWM', 'jarak depan (cm)'};
for i = 1:numel(kol)
    nexttile; a = R.(kol{i}); b = S.(kol{i});
    a(a >= 999) = NaN; b(b >= 999) = NaN;
    if any(strcmp(kol{i}, {'sudut', 'pwm'}))
        stairs(tR, a, 'r', 'LineWidth', 1.2); hold on; stairs(tS, b, 'b', 'LineWidth', 1.2);
    else
        plot(tR, a, 'r', tS, b, 'b', 'LineWidth', 1.2);
    end
    grid on; title(jud{i}); xlabel('waktu (s)');
    if i == 1, legend('robot asli', 'simulasi', 'Location', 'best'); end
end
fprintf('Ts robot  : %.3f s (rata-rata)\n', mean(diff(R.ms)) / 1000);
fprintf('Ts simulasi: %.3f s (rata-rata)\n', mean(diff(S.ms)) / 1000);
if nargin > 3 && ~isempty(file_png)
    exportgraphics(f, file_png, 'Resolution', 130);
end
end
