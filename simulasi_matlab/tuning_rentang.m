function T = tuning_rentang(folder)
%TUNING_RENTANG  Cari rentang output Kp, Ki, Kd (9 kategori tetap, rule dosen tetap) lewat simulasi.
%   Dosen: "output Kp, Ki, Kd dibuat 9 kategori dalam rentang 0-20, kalau rentang angkanya kurang
%   sesuai tolong diperbaiki". Rentang tiap gain diuji pada grid nilai maksimum; batas bawah 0.
%   Kriteria: J = rata-rata IAE (cm.s) skenario S1 (geser 10 cm) & S3 (miring 5 deg) di lorong
%   selebar lorong asli, ditambah penalti 100 bila robot tidak tenang (settling NaN).
if nargin < 1 || isempty(folder), folder = fullfile(fileparts(mfilename('fullpath')), 'hasil'); end
out = fullfile(folder, 'perbandingan'); if ~exist(out, 'dir'), mkdir(out); end
cfg0 = config_robot(); Wj = lintasan_jalur(); lb = Wj.lebar_lorong(1);
W1 = lintasan_lurus(10, 0, lb, 1200); W1.T_maks = 80;
W3 = lintasan_lurus(0, 5, lb, 1200);  W3.T_maks = 80;
gP = [10 20 30 40]; gI = [1 5 10 20]; gD = [20 50 100 150];
T = table();
for p = gP
    for i = gI
        for d = gD
            cfg = cfg0; cfg.fz = fuzzy_param([0 p], [0 i], [0 d]);
            M1 = hitung_metrik(simulasi(W1, cfg, true), 5);
            M3 = hitung_metrik(simulasi(W3, cfg, true), 5);
            J = mean([M1.IAE M3.IAE]) + 100 * (isnan(M1.settling_s) + isnan(M3.settling_s));
            T = [T; table(p, i, d, J, M1.IAE, M3.IAE, M1.settling_s, M3.settling_s, M1.overshoot_cm, M3.overshoot_cm, ...
                'VariableNames', {'Kp_maks', 'Ki_maks', 'Kd_maks', 'J', 'IAE_S1', 'IAE_S3', 'ts_S1', 'ts_S3', 'os_S1', 'os_S3'})]; %#ok<AGROW>
            fprintf('Kp[0 %g] Ki[0 %g] Kd[0 %g] J=%.1f\n', p, i, d, J);
        end
    end
end
T = sortrows(T, 'J');
writetable(T, fullfile(out, 'tuning_rentang_fuzzy.csv'));
disp(T(1:10, :));
end
