function T = tuning_pid_tetap(folder)
%TUNING_PID_TETAP  Setel gain PID tetap dengan prosedur & kriteria yang SAMA dengan tuning_rentang
%   (J = rata-rata IAE S1 & S3 + penalti 100 bila tidak tenang), agar perbandingan adil.
if nargin < 1 || isempty(folder), folder = fullfile(fileparts(mfilename('fullpath')), 'hasil'); end
out = fullfile(folder, 'perbandingan'); if ~exist(out, 'dir'), mkdir(out); end
cfg0 = config_robot(); Wj = lintasan_jalur(); lb = Wj.lebar_lorong(1);
W1 = lintasan_lurus(10, 0, lb, 1200); W1.T_maks = 80;
W3 = lintasan_lurus(0, 5, lb, 1200);  W3.T_maks = 80;
gP = [5 9.25 15 20]; gI = [0 9.25 20]; gD = [9.25 50 100 150 200 300];
T = table();
for p = gP
    for i = gI
        for d = gD
            cfg = cfg0; cfg.k.kpTetap = p; cfg.k.kiTetap = i; cfg.k.kdTetap = d;
            M1 = hitung_metrik(simulasi(W1, cfg, false), 5);
            M3 = hitung_metrik(simulasi(W3, cfg, false), 5);
            J = mean([M1.IAE M3.IAE]) + 100 * (isnan(M1.settling_s) + isnan(M3.settling_s));
            T = [T; table(p, i, d, J, M1.IAE, M3.IAE, M1.settling_s, M3.settling_s, M1.overshoot_cm, M3.overshoot_cm, ...
                'VariableNames', {'Kp', 'Ki', 'Kd', 'J', 'IAE_S1', 'IAE_S3', 'ts_S1', 'ts_S3', 'os_S1', 'os_S3'})]; %#ok<AGROW>
            fprintf('Kp=%g Ki=%g Kd=%g J=%.1f\n', p, i, d, J);
        end
    end
end
T = sortrows(T, 'J');
writetable(T, fullfile(out, 'tuning_pid_tetap.csv'));
disp(T(1:10, :));
end
