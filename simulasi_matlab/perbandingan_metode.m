function [ringkas, lengkap] = perbandingan_metode(folder, n_benih)
%PERBANDINGAN_METODE  Uji PID-Fuzzy vs PID tetap pada lorong lurus selebar lorong lintasan asli
%   (jalur.dxf), 5 jenis gangguan awal x n_benih benih derau sensor.
%   Metode yang dibandingkan:
%     A  PID-Fuzzy            : rentang keluaran Kp 0-20, Ki 0-20, Kd 0-300 (config_robot)
%     B  PID-Fuzzy 0-20       : rentang asli dosen (Kp, Ki, Kd 0-20)
%     C  PID tetap 9,25       : gain hasil tuning manual di robot asli
%     D  PID tetap (tuning)   : gain terbaik tuning_pid_tetap.m (prosedur & kriteria sama dengan A)
%   Perbandingan utama yang adil: A vs D. Keluaran di folder/perbandingan.
if nargin < 1 || isempty(folder), folder = fullfile(fileparts(mfilename('fullpath')), 'hasil'); end
if nargin < 2, n_benih = 5; end
out = fullfile(folder, 'perbandingan'); if ~exist(out, 'dir'), mkdir(out); end
cfg0 = config_robot();
Wj = lintasan_jalur(); lebar = Wj.lebar_lorong(1);         % lebar lorong asli (~97,6 cm)
panjang = 1200;                                             % lorong uji 12 m agar respons tenang

% ---- daftar metode ----
cfgB = cfg0; cfgB.fz = fuzzy_param([0 20], [0 20], [0 20]);
cfgD = cfg0; ft = fullfile(out, 'tuning_pid_tetap.csv');
if exist(ft, 'file')
    tp = readtable(ft); tp = sortrows(tp, 'J');
    cfgD.k.kpTetap = tp.Kp(1); cfgD.k.kiTetap = tp.Ki(1); cfgD.k.kdTetap = tp.Kd(1);
else
    warning('tuning_pid_tetap.csv belum ada: jalankan tuning_pid_tetap dulu. Dipakai gain robot.');
end
met = struct('nama', {'PID-Fuzzy', 'PID-Fuzzy 0-20', 'PID tetap 9,25', ...
                      sprintf('PID tetap (%g/%g/%g)', cfgD.k.kpTetap, cfgD.k.kiTetap, cfgD.k.kdTetap)}, ...
             'fuzzy', {true, true, false, false}, 'cfg', {cfg0, cfgB, cfg0, cfgD});
warnaM = [0.15 0.35 0.9; 0.55 0.7 1; 0.95 0.55 0.55; 0.85 0.1 0.1];

%  S1-S5: kondisi nominal (sama dengan kondisi tuning). S6-S7: uji ketahanan, kondisi berubah
%  tanpa menyetel ulang gain / rentang.
%           nama              judul                                          y0(cm) th0(deg) ubah-cfg
sk = { 'S1_geser_kiri',   'S1: mulai bergeser 10 cm ke kiri',                10,   0, @(c) c
       'S2_geser_kanan',  'S2: mulai bergeser 10 cm ke kanan',              -10,   0, @(c) c
       'S3_miring_kiri',  'S3: mulai menghadap 5\circ ke kiri',               0,   5, @(c) c
       'S4_miring_kanan', 'S4: mulai menghadap 5\circ ke kanan',              0,  -5, @(c) c
       'S5_gabungan',     'S5: geser 10 cm kiri & menghadap 5\circ kanan',   10,  -5, @(c) c
       'S6_derau_dm556',  'S6: geser 10 cm + bacaan semu DM556 (15%)',       10,   0, @(c) setfield(c, 'sensor', setfield(c.sensor, 'anomali_dm556', true))
       'S7_lebih_cepat',  'S7: geser 10 cm + PWM jelajah 150 (+-40 cm/s)',   10,   0, @(c) setfield(c, 'k', setfield(c.k, 'KECEPATAN_JELAJAH', 150)) };
kol = {'puncak_cm', 'overshoot_cm', 'overshoot_pct', 'settling_s', 'ess_cm', 'IAE', 'ISE', 'RMSE_cm'};
lengkap = table();
for i = 1:size(sk, 1)
    W = lintasan_lurus(sk{i,3}, sk{i,4}, lebar, panjang);
    W.nama = sk{i,1}; W.judul = sprintf('%s (lorong %.1f cm)', sk{i,2}, lebar);
    W.T_maks = 80;
    H1 = cell(1, numel(met));
    for b = 1:n_benih
        for m = 1:numel(met)
            cfg = sk{i,5}(met(m).cfg); cfg.seed = b;
            H = simulasi(W, cfg, met(m).fuzzy); H.metode = met(m).nama;
            M = hitung_metrik(H, 5);
            baris = table(string(sk{i,1}), string(met(m).nama), b, 'VariableNames', {'skenario', 'metode', 'benih'});
            for c = kol, baris.(c{1}) = M.(c{1}); end
            baris.tertahan_ms = H.nTabrak;
            baris.status = string(unique([H.log.status{:}], 'stable'));
            lengkap = [lengkap; baris]; %#ok<AGROW>
            if b == 1, H1{m} = H; end
        end
    end
    plot_perbandingan(H1{1}, H1{4}, out);                   % detail: A vs D
    gambar_semua(H1, met, warnaM, W.judul, fullfile(out, ['respons4_' sk{i,1} '.png']));
    close all;
    fprintf('%s selesai\n', sk{i,1});
end
writetable(lengkap, fullfile(out, 'metrik_lengkap.csv'));

% ---- ringkasan rata-rata +/- simpangan baku ----
ringkas = table();
for i = 1:size(sk, 1)
    for m = 1:numel(met)
        r = lengkap(lengkap.skenario == sk{i,1} & lengkap.metode == met(m).nama, :);
        baris = table(string(sk{i,1}), string(met(m).nama), 'VariableNames', {'skenario', 'metode'});
        for c = kol
            v = r.(c{1});
            baris.(c{1}) = mean(v, 'omitnan'); baris.([c{1} '_sd']) = std(v, 'omitnan');
        end
        baris.tidak_tenang = sum(isnan(r.settling_s));
        ringkas = [ringkas; baris]; %#ok<AGROW>
    end
end
writetable(ringkas, fullfile(out, 'metrik_ringkas.csv'));

% ---- tabel siap laporan: rata-rata (sd) per metode ----
km = {'puncak_cm', 'overshoot_cm', 'settling_s', 'ess_cm', 'IAE', 'RMSE_cm'};
lap = table();
for i = 1:size(sk, 1)
    for c = km
        baris = table(string(sk{i,1}), string(c{1}), 'VariableNames', {'skenario', 'metrik'});
        for m = 1:numel(met)
            r = ringkas(ringkas.skenario == sk{i,1} & ringkas.metode == met(m).nama, :);
            s = sprintf('%.2f (%.2f)', r.(c{1}), r.([c{1} '_sd']));
            if r.tidak_tenang > 0 && strcmp(c{1}, 'settling_s')
                s = sprintf('%s [%d/%d tak tenang]', s, r.tidak_tenang, n_benih);
            end
            baris.(sprintf('M%c', 'A' + m - 1)) = string(s);
        end
        lap = [lap; baris]; %#ok<AGROW>
    end
end
lap.Properties.VariableDescriptions = {'', '', met.nama};
writetable(lap, fullfile(out, 'tabel_laporan.csv'));
fprintf('\nMA = %s | MB = %s | MC = %s | MD = %s\n', met.nama);
disp(lap);

% ---- rata-rata semua skenario (satu angka per metode) ----
tot = table();
kelompok = {'nominal (S1-S5)', startsWith(ringkas.skenario, ["S1" "S2" "S3" "S4" "S5"])
            'ketahanan (S6-S7)', startsWith(ringkas.skenario, ["S6" "S7"])};
for g = 1:size(kelompok, 1)
    for m = 1:numel(met)
        r = ringkas(ringkas.metode == met(m).nama & kelompok{g,2}, :);
        tot = [tot; table(string(kelompok{g,1}), string(met(m).nama), mean(r.IAE), mean(r.RMSE_cm), ...
            mean(r.settling_s, 'omitnan'), sum(r.tidak_tenang), mean(r.overshoot_cm), mean(r.ess_cm), ...
            'VariableNames', {'kelompok', 'metode', 'IAE', 'RMSE_cm', 'settling_s', 'run_tak_tenang', ...
            'overshoot_cm', 'ess_cm'})]; %#ok<AGROW>
    end
end
writetable(tot, fullfile(out, 'rata_rata_semua_skenario.csv'));
disp(tot);

% ---- grafik batang ----
f = figure('Color', 'w', 'Position', [40 40 1300 820]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
title(tl, sprintf('Perbandingan metode kendali: rata-rata %d benih derau (garis = simpangan baku)', n_benih));
jud = {'IAE (cm.s)', 'RMSE (cm)', 'Settling time \pm5 cm (s)', 'Overshoot (cm)'};
kk = {'IAE', 'RMSE_cm', 'settling_s', 'overshoot_cm'};
lbl = categorical(strrep(sk(:,1), '_', ' ')); lbl = reordercats(lbl, strrep(sk(:,1), '_', ' '));
for j = 1:4
    nexttile; Y = zeros(size(sk,1), numel(met)); E = Y;
    for i = 1:size(sk, 1)
        for m = 1:numel(met)
            r = ringkas(ringkas.skenario == sk{i,1} & ringkas.metode == met(m).nama, :);
            Y(i,m) = r.(kk{j}); E(i,m) = r.([kk{j} '_sd']);
        end
    end
    hb = bar(lbl, Y); hold on;
    for m = 1:numel(met)
        hb(m).FaceColor = warnaM(m,:);
        errorbar(hb(m).XEndPoints, Y(:,m), E(:,m), 'k.', 'LineWidth', 0.8);
    end
    grid on; title(jud{j}); if j == 1, legend({met.nama}, 'Location', 'northoutside', 'NumColumns', 2); end
end
exportgraphics(f, fullfile(out, 'ringkasan_batang.png'), 'Resolution', 130);
close(f);
end

function gambar_semua(H1, met, warnaM, judul, file)
f = figure('Color', 'w', 'Position', [40 40 1100 450]); hold on;
for m = 1:numel(H1)
    T = H1{m}.traj; ok = ismember(T.status, [1 2]) & abs(sin(T.th)) < sin(deg2rad(30));
    y = T.y(ok) + H1{m}.cfg.sensor.s_kiri(1) * sin(T.th(ok));
    plot(T.t(ok), y, 'Color', warnaM(m,:), 'LineWidth', 1.6);
end
yline(0, 'k--'); yline([-5 5], 'k:'); grid on;
xlabel('waktu (s)'); ylabel('simpangan dari tengah lorong (cm)');
legend([{met.nama}, {'setpoint', 'pita \pm5 cm'}], 'Location', 'northeast');
title(['Posisi lateral 4 metode - ' strrep(judul, '_', ' ')]);
exportgraphics(f, file, 'Resolution', 130);
end
