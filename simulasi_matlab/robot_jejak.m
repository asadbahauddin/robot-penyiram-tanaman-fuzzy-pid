function poly = robot_jejak(pose, cfg, delta)
%ROBOT_JEJAK  Poligon badan + 4 roda (koordinat dunia) untuk cek tabrakan & animasi.
%   poly{1} = badan, poly{2..5} = roda belakang kiri/kanan, roda depan kiri/kanan.
if nargin < 3, delta = 0; end
poly = cell(1, 5);
poly{1} = robot_badan(pose, cfg);
r = cfg.d_roda/2; t = cfg.tebal_roda/2;
pusat = [0  cfg.track_b/2; 0 -cfg.track_b/2; cfg.L cfg.track_d/2; cfg.L -cfg.track_d/2];
sud = [0 0 delta delta];
kotak = [-r -t; r -t; r t; -r t];
c = cos(pose(3)); s = sin(pose(3));
for j = 1:4
    cj = cos(sud(j)); sj = sin(sud(j));
    lok = kotak * [cj sj; -sj cj] + pusat(j,:);            % putar roda lalu geser ke posisinya
    poly{j+1} = [pose(1) + c*lok(:,1) - s*lok(:,2), pose(2) + s*lok(:,1) + c*lok(:,2)];
end
end
