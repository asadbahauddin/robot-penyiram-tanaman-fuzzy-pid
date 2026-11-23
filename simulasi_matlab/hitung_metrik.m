function M = hitung_metrik(H, pita_cm)
%HITUNG_METRIK  Metrik kinerja kendali lateral dari posisi SEBENARNYA robot di lorong.
%   Simpangan diukur di titik sensor samping (cfg.sensor.s_kiri) terhadap garis tengah.
%   Hanya bagian lurus saat status JELAJAH/MELAMBAT yang dihitung.
%   overshoot  : simpangan terjauh di sisi seberang posisi awal, % dari simpangan awal
%   settling   : waktu sejak start sampai simpangan masuk dan tetap di pita +/- pita_cm
%   ess        : rata-rata |simpangan| pada 3 s terakhir
%   IAE, ISE   : integral |y| dt dan y^2 dt;  RMSE : akar rata-rata kuadrat simpangan
if nargin < 2, pita_cm = 2; end
T = H.traj;
ok = ismember(T.status, [1 2]) & abs(T.th) < deg2rad(30);
xs = H.cfg.sensor.s_kiri(1);            % posisi sensor samping di depan poros belakang
t = T.t(ok); y = T.y(ok) + xs*sin(T.th(ok));
y = y - H.world.y_ref(T.x(ok), T.y(ok));
dt = median(diff(T.t));
y0 = y(1);
if abs(y0) > pita_cm
    seberang = -sign(y0) * y;
    M.overshoot_pct = max(0, max(seberang)) / abs(y0) * 100;
else
    M.overshoot_pct = NaN;
end
M.overshoot_cm = max(0, max(-sign(y0 + eps) * y));
luar = find(abs(y) > pita_cm, 1, 'last');
if isempty(luar), M.settling_s = 0;
elseif luar == numel(y), M.settling_s = NaN;           % tidak pernah tenang
else, M.settling_s = t(luar + 1) - t(1); end
akhir = t >= t(end) - 3;
M.ess_cm = mean(abs(y(akhir)));
M.IAE = sum(abs(y)) * dt;
M.ISE = sum(y.^2) * dt;
M.RMSE_cm = sqrt(mean(y.^2));
M.y0_cm = y0;
end
