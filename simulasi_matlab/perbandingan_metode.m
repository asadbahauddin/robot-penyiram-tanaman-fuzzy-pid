function [ringkas, lengkap] = perbandingan_metode(folder, n_benih)
%PERBANDINGAN_METODE  Uji PID-Fuzzy vs PID tetap (Kp=Ki=Kd=9,25) pada lorong lurus selebar lorong
%   lintasan asli (jalur.dxf), dengan 5 jenis gangguan awal dan beberapa benih derau sensor.
%   Keluaran (di folder/perbandingan): metrik tiap run, ringkasan rata-rata +/- simpangan baku,
%   grafik respons tiap skenario (benih 1), dan grafik batang metrik.
if nargin < 1 || isempty(folder), folder = fullfile(fileparts(mfilename('fullpath')), 'hasil'); end
if nargin < 2, n_benih = 5; end
out = fullfile(folder, 'perbandingan'); if ~exist(out, 'dir'), mkdir(out); end
cfg0 = config_robot();
Wj = lintasan_jalur(); lebar = Wj.lebar_lorong(1);         % lebar lorong asli (~97,6 cm)
panjang = 1200;                                             % lorong uji 12 m agar respons tenang

%           nama              judul                                   y0(cm) th0(deg)
sk = { 'S1_geser_kiri',   'S1: mulai bergeser 10 cm ke kiri',          10,   0
       'S2_geser_kanan',  'S2: mulai bergeser 10 cm ke kanan',        -10,   0
       'S3_miring_kiri',  'S3: mulai menghadap 5\circ ke kiri',          0,   5
       'S4_miring_kanan', 'S4: mulai menghadap 5\circ ke kanan',         0,  -5
       'S5_gabungan',     'S5: bergeser 10 cm kiri & menghadap 5\circ kanan', 10, -5 };
metode = {'PID-Fuzzy', 'PID tetap'};
kol = {'puncak_cm', 'overshoot_cm', 'overshoot_pct', 'settling_s', 'ess_cm', 'IAE', 'ISE', 'RMSE_cm'};
lengkap = table();
for i = 1:size(sk, 1)
    W = lintasan_lurus(sk{i,3}, sk{i,4}, lebar, panjang);
    W.nama = sk{i,1}; W.judul = sprintf('%s (lorong %.1f cm)', sk{i,2}, lebar);
    W.T_maks = 80;
    H1 = struct([]);
    for b = 1:n_benih
        cfg = cfg0; cfg.seed = b;
        for m = 1:2
            H = simulasi(W, cfg, m == 1);
            M = hitung_metrik(H, 5);
            baris = table(string(sk{i,1}), string(metode{m}), b, 'VariableNames', {'skenario', 'metode', 'benih'});
            for c = kol, baris.(c{1}) = M.(c{1}); end
            baris.tertahan_ms = H.nTabrak;
            baris.status = string(unique([H.log.status{:}], 'stable'));
            lengkap = [lengkap; baris]; %#ok<AGROW>
            if b == 1, H1(m).H = H; end %#ok<AGROW>
        end
    end
    plot_perbandingan(H1(1).H, H1(2).H, out);
    close all;
    fprintf('%s selesai\n', sk{i,1});
end
writetable(lengkap, fullfile(out, 'metrik_lengkap.csv'));

% ringkasan rata-rata +/- simpangan baku per skenario & metode
ringkas = table();
for i = 1:size(sk, 1)
    for m = 1:2
        r = lengkap(lengkap.skenario == sk{i,1} & lengkap.metode == metode{m}, :);
        baris = table(string(sk{i,1}), string(metode{m}), 'VariableNames', {'skenario', 'metode'});
        for c = kol
            v = r.(c{1});
            baris.(c{1}) = mean(v, 'omitnan');
            baris.([c{1} '_sd']) = std(v, 'omitnan');
        end
        baris.tidak_tenang = sum(isnan(r.settling_s));
        ringkas = [ringkas; baris]; %#ok<AGROW>
    end
end
writetable(ringkas, fullfile(out, 'metrik_ringkas.csv'));

% tabel siap laporan: rata-rata (sd) dan selisih (%) PID-Fuzzy terhadap PID tetap
lap = table();
for i = 1:size(sk, 1)
    F = ringkas(ringkas.skenario == sk{i,1} & ringkas.metode == "PID-Fuzzy", :);
    P = ringkas(ringkas.skenario == sk{i,1} & ringkas.metode == "PID tetap", :);
    for c = {'puncak_cm', 'overshoot_cm', 'settling_s', 'ess_cm', 'IAE', 'RMSE_cm'}
        lap = [lap; table(string(sk{i,1}), string(c{1}), ...
            string(sprintf('%.2f (%.2f)', F.(c{1}), F.([c{1} '_sd']))), ...
            string(sprintf('%.2f (%.2f)', P.(c{1}), P.([c{1} '_sd']))), ...
            round((F.(c{1}) - P.(c{1})) / P.(c{1}) * 100, 1), ...
            'VariableNames', {'skenario', 'metrik', 'PID_Fuzzy', 'PID_tetap', 'selisih_pct'})]; %#ok<AGROW>
    end
end
writetable(lap, fullfile(out, 'tabel_laporan.csv'));
disp(lap);

% grafik batang
f = figure('Color', 'w', 'Position', [40 40 1200 800]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
title(tl, sprintf('PID-Fuzzy vs PID tetap: rata-rata %d benih derau (garis = simpangan baku)', n_benih));
jud = {'IAE (cm.s)', 'RMSE (cm)', 'Settling time \pm5 cm (s)', 'Simpangan puncak (cm)'};
kk = {'IAE', 'RMSE_cm', 'settling_s', 'puncak_cm'};
lbl = categorical(strrep(sk(:,1), '_', ' ')); lbl = reordercats(lbl, strrep(sk(:,1), '_', ' '));
for j = 1:4
    nexttile; Y = zeros(size(sk,1), 2); E = Y;
    for i = 1:size(sk, 1)
        for m = 1:2
            r = ringkas(ringkas.skenario == sk{i,1} & ringkas.metode == metode{m}, :);
            Y(i,m) = r.(kk{j}); E(i,m) = r.([kk{j} '_sd']);
        end
    end
    hb = bar(lbl, Y); hold on;
    for m = 1:2
        errorbar(hb(m).XEndPoints, Y(:,m), E(:,m), 'k.', 'LineWidth', 1);
    end
    hb(1).FaceColor = [0.2 0.4 0.9]; hb(2).FaceColor = [0.9 0.3 0.3];
    grid on; title(jud{j}); if j == 1, legend(metode, 'Location', 'best'); end
end
exportgraphics(f, fullfile(out, 'ringkasan_batang.png'), 'Resolution', 130);
close(f);
end
