% MAIN  Simulasi Robot Penyiram Tanaman Otomatis dengan Fuzzy-PID (Ackermann, wall-following).
%   Jalankan dari folder simulasi_matlab:  >> main
%   atau dari terminal: matlab -batch "run('simulasi_matlab/main.m')"
%
%   Atur pilihan di bawah. Semua parameter robot ada di config_robot.m.
clc; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
cd(here);
folder = fullfile(here, 'hasil'); if ~exist(folder, 'dir'), mkdir(folder); end

%% ================= PILIHAN =================
skenario = {'lurus', 'tidak_di_tengah', 'miring', 'belokan', 'belokan_sempit', 'rintangan', 'jalur'};
% (pilihan di bawah bisa ditimpa dengan mendefinisikan variabelnya sebelum menjalankan main)
if ~exist('jalankan_perbandingan', 'var'), jalankan_perbandingan = true; end  % hasil/perbandingan/
if ~exist('n_benih', 'var'), n_benih = 5; end              % benih derau tiap skenario perbandingan
if ~exist('jalankan_tuning', 'var'), jalankan_tuning = false; end  % ulangi tuning (lama, +-20 menit)
buat_animasi = true;          % simpan GIF animasi (metode PID-Fuzzy)
anomali_dm556 = false;        % true: tambahkan pembacaan semu ~50 cm (gangguan driver DM556)
%% ==========================================

modeBatch = batchStartupOptionUsed;
cfg = config_robot();
cfg.sensor.anomali_dm556 = anomali_dm556;

fprintf('== Uji fuzzy terhadap hitungan tangan ==\n');
uji_fuzzy;
gambar_fuzzy(folder);
gambar_rule(folder);
close all;

ringkas = table();
for s = skenario
    W = buat_lintasan(s{1});
    for useFuzzy = [true false]
        H = simulasi(W, cfg, useFuzzy);
        km = 'fuzzy'; if ~useFuzzy, km = 'pid'; end
        ekspor_log(H, fullfile(folder, sprintf('log_%s_%s.csv', W.nama, km)), true);
        % data grafik: lintasan & kemudi aktual tiap 10 ms (pemisah ';', desimal ',' untuk Excel)
        tulis_csv_id(H.traj, fullfile(folder, sprintf('data_lintasan_%s_%s.csv', W.nama, km)));
        plot_hasil(H, folder);
        if buat_animasi && useFuzzy
            animasi(H, fullfile(folder, sprintf('animasi_%s.gif', W.nama)), ~modeBatch);
        end
        if buat_animasi && strcmp(W.nama, 'jalur')      % video start -> finish, kecepatan asli
            animasi(H, fullfile(folder, sprintf('video_%s_%s.mp4', W.nama, km)), false, 1);
        end
        L = H.log; st = [L.status{:}];
        ringkas = [ringkas; table(string(W.nama), string(H.metode), H.traj.t(end), ...
            mean(diff(L.ms))/1000, sum(~isnan(H.tSiram)), numel(H.tSiram), H.nTabrak, ...
            string(unique(st, 'stable')), ...
            'VariableNames', {'skenario', 'metode', 'durasi_s', 'Ts_rata_s', 'disiram', ...
            'jml_tanaman', 'ms_tertahan', 'status_dilalui'})]; %#ok<AGROW>
        hasil.(W.nama).(km) = H;
        if modeBatch, close all; end
    end
end
fprintf('\n== Ringkasan skenario ==\n'); disp(ringkas);
writetable(ringkas, fullfile(folder, 'ringkasan_skenario.csv'));

if jalankan_tuning                     % lama (+-20 menit): cari rentang fuzzy & gain PID tetap
    tuning_rentang(folder);
    tuning_pid_tetap(folder);
end
if jalankan_perbandingan
    fprintf('\n== Perbandingan PID-Fuzzy vs PID tetap (%d benih) ==\n', n_benih);
    perbandingan_metode(folder, n_benih);
    if modeBatch, close all; end
end
save(fullfile(folder, 'hasil_simulasi.mat'), 'hasil', 'ringkas', '-v7.3');
fprintf('\nSelesai. Semua gambar, GIF, dan CSV ada di %s\n', folder);

function W = buat_lintasan(nama)
switch nama
    case 'lurus',           W = lintasan_lurus(0, 0);
    case 'tidak_di_tengah', W = lintasan_lurus(15, 0);
    case 'miring',          W = lintasan_lurus(0, 8);
    case 'belokan',         W = lintasan_belokan(150);
    case 'belokan_sempit',  W = lintasan_belokan(100);
    case 'rintangan',       W = lintasan_rintangan();
    case 'uturn',           W = lintasan_uturn(540, 100, 40, 100);   % lintasan asumsi lama
    case 'jalur',           W = lintasan_jalur('', 5);              % lintasan asli dari dxf/jalur.dxf
    otherwise, error('Skenario tidak dikenal: %s', nama);
end
end
