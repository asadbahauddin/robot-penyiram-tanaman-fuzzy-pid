function gambar_rule(folder)
%GAMBAR_RULE  Tabel rule base Kp, Ki, Kd (7x7) sebagai PNG berwarna + gabungan fungsi keanggotaan.
%   Baris = Error, kolom = Delta Error (sesuai tabel dosen). Warna = besar keluaran (ZE..EVB).
if nargin < 1, folder = ''; end
fz = fuzzy_param();
R = {fz.RULE_KP, fz.RULE_KI, fz.RULE_KD};
nm = {'Kp (Proportional)', 'Ki (Integral)', 'Kd (Derivative)'};
fn = {'rule_Kp.png', 'rule_Ki.png', 'rule_Kd.png'};
cmap = parula(9);
for k = 1:3
    f = figure('Color', 'w', 'Position', [100 100 640 560]);
    ax = axes(f); hold(ax, 'on'); axis(ax, 'ij', 'equal', 'off');
    for i = 1:7
        for j = 1:7
            v = R{k}(i, j);
            rectangle(ax, 'Position', [j i 1 1], 'FaceColor', [cmap(v,:) 0.55], 'EdgeColor', 'k');
            text(ax, j+0.5, i+0.5, fz.lbl_out{v}, 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
        end
        text(ax, 0.9, i+0.5, fz.lbl_in{i}, 'HorizontalAlignment', 'right', 'FontWeight', 'bold');
        text(ax, i+0.5, 0.85, fz.lbl_in{i}, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    text(ax, 4.5, 0.35, '\Delta Error', 'HorizontalAlignment', 'center', 'FontSize', 12);
    text(ax, 0.1, 4.5, 'Error', 'HorizontalAlignment', 'center', 'Rotation', 90, 'FontSize', 12);
    xlim(ax, [-0.3 8.1]); ylim(ax, [0 8.1]);
    title(ax, ['Rule Base ' nm{k}], 'FontSize', 14);
    if ~isempty(folder), exportgraphics(f, fullfile(folder, fn{k}), 'Resolution', 150); end
end

% gabungan MF input & output dalam satu gambar
f = figure('Color', 'w', 'Position', [100 100 1100 420]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
x = linspace(-6, 6, 601); z = linspace(0, 20, 801); w = lines(9);
nexttile; hold on;
for k = 1:7
    plot(x, arrayfun(@(v) trapmf_fw(v, fz.MF_IN(k,:)), x), 'LineWidth', 2, 'Color', w(k,:));
    text(fz.MF_IN(k,2), 1.06, fz.lbl_in{k}, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
ylim([0 1.15]); grid on; xlabel('e / \Deltae (cm)'); ylabel('\mu'); title('Input: error dan delta error');
nexttile; hold on;
for k = 1:9
    plot(z, arrayfun(@(v) trapmf_fw(v, fz.MF_OUT(k,:)), z), 'LineWidth', 2, 'Color', w(k,:));
    text(fz.OUT_CENTER(k), 1.06, fz.lbl_out{k}, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
ylim([0 1.15]); grid on; xlabel('Kp / Ki / Kd'); title('Output: Kp, Ki, Kd (rentang 0-20)');
title(tl, 'Fungsi keanggotaan fuzzy Mamdani');
if ~isempty(folder), exportgraphics(f, fullfile(folder, 'mf_gabungan.png'), 'Resolution', 150); end
end
