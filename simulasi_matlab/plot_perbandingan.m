function [tab, f] = plot_perbandingan(HF, HP, folder)
%PLOT_PERBANDINGAN  PID-Fuzzy vs PID tetap pada skenario yang sama: grafik + tabel metrik.
MF = hitung_metrik(HF); MP = hitung_metrik(HP);
nm = {'Simpangan puncak (cm)', 'Overshoot (cm)', 'Overshoot (%)', 'Settling time +/-5 cm (s)', ...
      'Steady-state error (cm)', 'IAE (cm.s)', 'ISE (cm^2.s)', 'RMSE (cm)'};
kk = {'puncak_cm', 'overshoot_cm', 'overshoot_pct', 'settling_s', 'ess_cm', 'IAE', 'ISE', 'RMSE_cm'};
vF = cellfun(@(c) MF.(c), kk)'; vP = cellfun(@(c) MP.(c), kk)';
tab = table(nm', round(vF, 3), round(vP, 3), 'VariableNames', {'Metrik', 'PID_Fuzzy', 'PID_tetap'});

f = figure('Name', 'Perbandingan', 'Color', 'w', 'Position', [40 40 1200 820]);
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nmP = sprintf('PID tetap (Kp=%g, Ki=%g, Kd=%g)', HP.cfg.k.kpTetap, HP.cfg.k.kiTetap, HP.cfg.k.kdTetap);
title(tl, sprintf('%s vs %s - %s', HF.metode, nmP, strrep(HF.world.judul, '_', ' ')));
[tF, yF] = lateral(HF); [tP, yP] = lateral(HP);
nexttile([1 2]); plot(tF, yF, 'b', tP, yP, 'r', 'LineWidth', 1.5); hold on;
yline(0, 'k--'); yline([-5 5], 'k:'); grid on; ylabel('simpangan dari tengah (cm)'); xlabel('waktu (s)');
legend(HF.metode, nmP, 'setpoint (tengah)', 'pita \pm5 cm'); title('Posisi lateral robot');
LF = HF.log; LP = HP.log; t1 = (LF.ms - LF.ms(1))/1000; t2 = (LP.ms - LP.ms(1))/1000;
nexttile; plot(t1, LF.e, 'b', t2, LP.e, 'r', 'LineWidth', 1.2); grid on; ylabel('e (cm)'); xlabel('waktu (s)'); title('Error (log)');
nexttile; stairs(t1, LF.sudut, 'b', 'LineWidth', 1.2); hold on; stairs(t2, LP.sudut, 'r', 'LineWidth', 1.2);
grid on; ylabel('derajat'); xlabel('waktu (s)'); title('Sudut kemudi target');
nexttile; plot(t1, LF.u, 'b', t2, LP.u, 'r', 'LineWidth', 1.2); grid on; ylabel('u'); xlabel('waktu (s)'); title('Keluaran PID u');
ax = nexttile; axis(ax, 'off');
teks = sprintf('%-28s %10s %10s\n', 'Metrik', 'PID-Fuzzy', 'PID tetap');
for i = 1:numel(nm), teks = [teks sprintf('%-28s %10.3f %10.3f\n', nm{i}, vF(i), vP(i))]; end %#ok<AGROW>
text(ax, 0, 0.5, teks, 'FontName', 'Consolas', 'FontSize', 10, 'Interpreter', 'none');
title(ax, 'Tabel metrik');
if nargin > 2 && ~isempty(folder)
    exportgraphics(f, fullfile(folder, ['perbandingan_' HF.world.nama '.png']), 'Resolution', 130);
    writetable(tab, fullfile(folder, ['metrik_' HF.world.nama '.csv']));
end
end

function [t, y] = lateral(H)
T = H.traj; ok = ismember(T.status, [1 2]) & abs(sin(T.th)) < sin(deg2rad(30));
xs = H.cfg.sensor.s_kiri(1);
t = T.t(ok); y = T.y(ok) + xs*sin(T.th(ok));
y = y - H.world.y_ref(T.x(ok), T.y(ok));
end
