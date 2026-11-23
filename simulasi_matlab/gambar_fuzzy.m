function gambar_fuzzy(folder)
%GAMBAR_FUZZY  Fungsi keanggotaan e, de, Kp/Ki/Kd dan permukaan Kp/Ki/Kd(e, de) untuk Bab 3/4.
%   Permukaan dihitung dengan fuzzy_mamdani() (identik firmware v12), bukan toolbox.
if nargin < 1, folder = ''; end
fz = fuzzy_param();
warna = lines(9);

% --- MF input (e dan de sama)
x = linspace(-6, 6, 601);
for nmIn = {'e', 'de'}
    f = figure('Color', 'w', 'Position', [100 100 760 360]); hold on;
    for k = 1:7
        mu = arrayfun(@(v) trapmf_fw(v, fz.MF_IN(k,:)), x);
        plot(x, mu, 'LineWidth', 2, 'Color', warna(k,:));
        text(fz.MF_IN(k,2), 1.06, fz.lbl_in{k}, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    ylim([0 1.15]); grid on; ylabel('\mu');
    if strcmp(nmIn{1}, 'e'), xlabel('error e (cm)'); title('Fungsi keanggotaan error (e)');
    else, xlabel('delta error \Deltae (cm)'); title('Fungsi keanggotaan delta error (\Deltae)'); end
    simpan(f, folder, ['mf_' nmIn{1} '.png']);
end

% --- MF output
z = linspace(0, 20, 801);
for nmOut = {'Kp', 'Ki', 'Kd'}
    f = figure('Color', 'w', 'Position', [100 100 760 360]); hold on;
    for k = 1:9
        mu = arrayfun(@(v) trapmf_fw(v, fz.MF_OUT(k,:)), z);
        plot(z, mu, 'LineWidth', 2, 'Color', warna(k,:));
        text(fz.OUT_CENTER(k), 1.06, fz.lbl_out{k}, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    ylim([0 1.15]); xlim([0 20]); grid on; ylabel('\mu'); xlabel(nmOut{1});
    title(['Fungsi keanggotaan keluaran ' nmOut{1} ' (rentang 0-20)']);
    simpan(f, folder, ['mf_' nmOut{1} '.png']);
end

% --- permukaan
[E, DE] = meshgrid(linspace(-6, 6, 61));
KP = zeros(size(E)); KI = KP; KD = KP;
for i = 1:numel(E)
    [KP(i), KI(i), KD(i)] = fuzzy_mamdani(E(i), DE(i), 0, 0, 0, fz);
end
Z = {KP, KI, KD}; nm = {'Kp', 'Ki', 'Kd'};
for j = 1:3
    f = figure('Color', 'w', 'Position', [100 100 700 520]);
    surf(E, DE, Z{j}, 'EdgeAlpha', 0.3); colormap(f, parula); colorbar;
    xlabel('e (cm)'); ylabel('\Deltae (cm)'); zlabel(nm{j}); zlim([0 20]);
    title(['Permukaan fuzzy ' nm{j} '(e, \Deltae)']); view(-35, 30);
    simpan(f, folder, ['surface_' nm{j} '.png']);
end
end

function simpan(f, folder, nama)
if ~isempty(folder), exportgraphics(f, fullfile(folder, nama), 'Resolution', 150); end
end
