function animasi(H, file_gif, tampil, langkah)
%ANIMASI  Animasi robot di lorong: badan, roda kemudi, sinar 5 sensor, tanaman menyala saat disiram.
%   animasi(H)                       : tampilkan di layar
%   animasi(H, 'hasil/x.gif')        : tampilkan & simpan GIF
%   animasi(H, file, false)          : hanya simpan GIF (mode -batch)
%   langkah : ambil tiap n frame (1 frame = 0,1 s). Bawaan 3.
if nargin < 2, file_gif = ''; end
if nargin < 3, tampil = true; end
if nargin < 4, langkah = 3; end
W = H.world; cfg = H.cfg; FR = H.frame;
vis = 'on'; if ~tampil, vis = 'off'; end
f = figure('Color', 'w', 'Position', [40 80 1150 520], 'Visible', vis);
ax = axes(f); hold(ax, 'on'); axis(ax, 'equal'); axis(ax, W.batas); grid(ax, 'on');
xlabel(ax, 'x (cm)'); ylabel(ax, 'y (cm)');
plot(ax, W.seg(:, [1 3])', W.seg(:, [2 4])', 'Color', [0.45 0.3 0.15], 'LineWidth', 3);
th = linspace(0, 2*pi, 24); nT = size(W.tanaman, 1); hT = gobjects(nT, 1);
for i = 1:nT
    c = W.tanaman(i, :);
    hT(i) = fill(ax, c(1) + c(3)*cos(th), c(2) + c(3)*sin(th), [0.2 0.7 0.2], ...
        'FaceAlpha', 0.6, 'EdgeColor', [0 0.4 0]);
end
hJejak = plot(ax, NaN, NaN, 'r:', 'LineWidth', 1);
hBadan = patch(ax, NaN, NaN, [0.85 0.85 0.95], 'EdgeColor', 'k', 'LineWidth', 1.5);
hRoda = gobjects(4, 1);
for j = 1:4, hRoda(j) = patch(ax, NaN, NaN, 'k'); end
nS = numel(H.namaSensor);
warnaS = [0.9 0.5 0; 0.6 0 0.8; 0 0.6 0.9; 0.6 0 0.8; 0.9 0 0];   % s_kanan kanan s_kiri kiri depan
hRay = gobjects(nS, 1); hKotak = gobjects(nS, 1);
for j = 1:nS
    hRay(j) = plot(ax, NaN, NaN, '-', 'Color', warnaS(j,:), 'LineWidth', 1.2);
    hKotak(j) = patch(ax, NaN, NaN, warnaS(j,:), 'EdgeColor', 'k');   % kotak sensor 2 x 5 cm (robot.dxf)
end
hAir = plot(ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.2 0.5 1], 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
namaStatus = {'JELAJAH', 'MELAMBAT', 'BERHENTI', 'BELOK KIRI', 'BELOK KANAN', 'MUNDUR', 'LIHAT', 'CARI CELAH'};
pertama = true;
for n = 1:langkah:numel(FR)
    F = FR(n); p = F.pose;
    P = robot_badan(p, cfg);
    set(hBadan, 'XData', P(:,1), 'YData', P(:,2));
    % roda: belakang lurus, depan berbelok delta
    J = robot_jejak(p, cfg, F.delta);
    for j = 1:4, set(hRoda(j), 'XData', J{j+1}(:,1), 'YData', J{j+1}(:,2)); end
    for j = 1:nS
        sd = cfg.sensor.(H.namaSensor{j});
        o = lok2w(sd(1:2), p);
        set(hRay(j), 'XData', [o(1) F.ray(j,1)], 'YData', [o(2) F.ray(j,2)]);
        a = deg2rad(sd(3));
        kt = [-1 -2.5; 1 -2.5; 1 2.5; -1 2.5] * [cos(a) sin(a); -sin(a) cos(a)] + sd(1:2);
        Kw = cell2mat(arrayfun(@(i) lok2w(kt(i,:), p), (1:4)', 'UniformOutput', false));
        set(hKotak(j), 'XData', Kw(:,1), 'YData', Kw(:,2));
    end
    sudah = H.tSiram <= F.t;
    for i = find(sudah)', set(hT(i), 'FaceColor', [0.1 0.4 1]); end
    if F.pompa
        o = lok2w([20 0], p); set(hAir, 'XData', o(1), 'YData', o(2));
    else
        set(hAir, 'XData', NaN, 'YData', NaN);
    end
    k = H.traj.t <= F.t;
    set(hJejak, 'XData', H.traj.x(k), 'YData', H.traj.y(k));
    pompa = 'OFF'; if F.pompa, pompa = 'ON'; end
    title(ax, sprintf('%s | %s | t = %.1f s | %s | kemudi %+.0f\\circ | pompa %s', ...
        strrep(W.nama, '_', ' '), H.metode, F.t, namaStatus{F.status}, rad2deg(F.delta), pompa));
    drawnow limitrate;
    if ~isempty(file_gif)
        img = print(f, '-RGBImage', '-r70');
        [A, map] = rgb2ind(img, 64, 'nodither');
        if pertama
            imwrite(A, map, file_gif, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1*langkah/2);
            pertama = false;
        else
            imwrite(A, map, file_gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1*langkah/2);
        end
    end
end
if ~tampil, close(f); end
end

function w = lok2w(l, p)
w = [p(1) + cos(p(3))*l(1) - sin(p(3))*l(2), p(2) + sin(p(3))*l(1) + cos(p(3))*l(2)];
end
