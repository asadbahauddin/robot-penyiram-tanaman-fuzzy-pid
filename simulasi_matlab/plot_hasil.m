function figs = plot_hasil(H, folder)
%PLOT_HASIL  Grafik respons satu simulasi (data dari log, format sama dengan robot).
%   Disimpan ke folder sebagai PNG bila folder diberikan.
L = H.log; T = H.traj;
t = (L.ms - L.ms(1)) / 1000;
nama = sprintf('%s_%s', H.world.nama, kode_metode(H));
judul = sprintf('%s - %s', strrep(H.world.judul, '_', ' '), H.metode);
sp = L.setpoint; sp(sp < 0) = NaN;
skiri = L.s_kiri; skiri(skiri >= 999) = NaN;
skanan = L.s_kanan; skanan(skanan >= 999) = NaN;

f1 = figure('Name', ['Respons ' nama], 'Color', 'w', 'Position', [50 50 1200 900]);
tl = tiledlayout(f1, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, judul);

nexttile; plot(t, skiri, 'b', t, sp, 'k--', 'LineWidth', 1.2); grid on;
ylabel('cm'); legend('s\_kiri (posisi lateral)', 'setpoint', 'Location', 'best');
title('Posisi lateral terhadap setpoint');

nexttile; yyaxis left; plot(t, L.e, 'LineWidth', 1.2); ylabel('e (cm)');
yyaxis right; plot(t, L.de, 'LineWidth', 1); ylabel('\Deltae (cm)'); grid on;
title('Error dan delta error');

nexttile; plot(t, L.Kp, t, L.Ki, t, L.Kd, 'LineWidth', 1.2); grid on;
legend('Kp', 'Ki', 'Kd', 'Location', 'best'); ylim([-1 21]); title('Gain PID');

nexttile; plot(t, L.P, t, L.I, t, L.D, t, L.u, 'k', 'LineWidth', 1.1); grid on;
legend('P', 'I', 'D', 'u', 'Location', 'best'); title('Komponen PID dan keluaran u');

nexttile; stairs(t, L.sudut, 'LineWidth', 1.2); hold on;
plot(T.t - L.ms(1)/1000, T.delta_deg, 'LineWidth', 1); grid on;
ylabel('derajat'); legend('target (log)', 'aktual stepper', 'Location', 'best');
title('Sudut kemudi');

nexttile; stairs(t, L.pwm, 'LineWidth', 1.2); grid on; ylabel('PWM'); ylim([-160 160]);
title('PWM motor');

nexttile; D = [L.depan L.s_kiri L.s_kanan L.kiri L.kanan]; D(D >= 999) = NaN;
plot(t, D, 'LineWidth', 1); grid on; ylabel('cm'); ylim([0 420]);
legend('depan', 's\_kiri', 's\_kanan', 'kiri', 'kanan', 'Location', 'best');
title('Jarak sensor (999 = tak terbaca, tidak digambar)');

nexttile; kode = 'JLBKRMSC';
[~, sn] = ismember([L.status{:}], kode);
stairs(t, sn, 'LineWidth', 1.2); hold on; stairs(t, L.pompa * 0.8 + 0.1, 'g', 'LineWidth', 1.5);
yticks(1:8); yticklabels({'JELAJAH', 'MELAMBAT', 'BERHENTI', 'BELOK KIRI', 'BELOK KANAN', 'MUNDUR', 'LIHAT', 'CARI CELAH'});
ylim([0 8.5]); grid on; xlabel('waktu (s)'); legend('status', 'pompa (0/1)', 'Location', 'best');
title('Mesin keadaan dan pompa');
for ax = findall(f1, 'Type', 'axes')', xlabel(ax, 'waktu (s)'); xlim(ax, [0 max(t(end), 1)]); end

f2 = gambar_lintasan(H);
figs = [f1 f2];
if nargin > 1 && ~isempty(folder)
    exportgraphics(f1, fullfile(folder, ['respons_' nama '.png']), 'Resolution', 130);
    exportgraphics(f2, fullfile(folder, ['lintasan_' nama '.png']), 'Resolution', 130);
end
end

function f = gambar_lintasan(H)
W = H.world; T = H.traj;
f = figure('Name', 'Lintasan', 'Color', 'w', 'Position', [60 60 1100 520]);
hold on; axis equal; grid on;
plot(W.seg(:, [1 3])', W.seg(:, [2 4])', 'Color', [0.45 0.3 0.15], 'LineWidth', 2);
th = linspace(0, 2*pi, 30);
for i = 1:size(W.tanaman, 1)
    c = W.tanaman(i, :);
    if isnan(H.tSiram(i)), col = [0.2 0.7 0.2]; else, col = [0.1 0.4 1]; end
    fill(c(1) + c(3)*cos(th), c(2) + c(3)*sin(th), col, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end
a = H.cfg.sensor.s_kiri(1);
xs = T.x + a*cos(T.th); ys = T.y + a*sin(T.th);      % titik sensor samping
h1 = plot(T.x, T.y, 'r', 'LineWidth', 1.5);
h2 = plot(xs, ys, 'm--', 'LineWidth', 1);
P = robot_badan([T.x(1) T.y(1) T.th(1)], H.cfg);
patch(P(:,1), P(:,2), 'w', 'FaceColor', 'none', 'EdgeColor', [0.5 0.5 0.5]);
P = robot_badan([T.x(end) T.y(end) T.th(end)], H.cfg);
patch(P(:,1), P(:,2), 'w', 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 1.5);
axis(W.batas); xlabel('x (cm)'); ylabel('y (cm)');
title(sprintf('Lintasan robot - %s - %s  (biru = tanaman disiram: %d/%d)', strrep(W.nama, '_', ' '), ...
    H.metode, sum(~isnan(H.tSiram)), numel(H.tSiram)));
legend([h1 h2], {'poros belakang', 'titik sensor samping'}, 'Location', 'northeastoutside');
end

function k = kode_metode(H)
if H.useFuzzy, k = 'fuzzy'; else, k = 'pid'; end
end
