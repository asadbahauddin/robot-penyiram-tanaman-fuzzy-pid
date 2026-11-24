function M = hitung_metrik(H, pita_cm)
%HITUNG_METRIK  Metrik kinerja kendali lateral dari posisi SEBENARNYA robot di lorong.
%   Simpangan y diukur di titik sensor samping (cfg.sensor.s_kiri) terhadap garis tengah lorong,
%   hanya saat status JELAJAH/MELAMBAT dan robot sejajar lorong (|sin th| < sin 30 deg).
%   puncak_cm     : simpangan terbesar |y|
%   overshoot_cm  : simpangan terjauh di SEBERANG garis tengah setelah robot pertama kali melewatinya
%   overshoot_pct : overshoot_cm / |simpangan awal| x 100 (hanya bila simpangan awal >= 2 cm)
%   settling_s    : waktu sampai |y| masuk dan tetap di pita +/- pita_cm (NaN = tidak tenang)
%   ess_cm        : rata-rata |y| pada 5 s terakhir
%   IAE, ISE      : integral |y| dt dan y^2 dt;  RMSE_cm : akar rata-rata kuadrat y
if nargin < 2, pita_cm = 5; end
T = H.traj;
ok = ismember(T.status, [1 2]) & abs(sin(T.th)) < sin(deg2rad(30));
xs = H.cfg.sensor.s_kiri(1);
t = T.t(ok); y = T.y(ok) + xs*sin(T.th(ok));
y = y - H.world.y_ref(T.x(ok), T.y(ok));
dt = median(diff(T.t));
y0 = y(1);
M.y0_cm = y0;
M.puncak_cm = max(abs(y));
% arah simpangan awal: dari posisi awal, atau dari arah gerak awal bila mulai di tengah
s0 = sign(y0); if abs(y0) < 2, s0 = sign(y(find(abs(y) >= 2, 1))); end
if isempty(s0) || s0 == 0, s0 = 1; end
lewat = find(s0 * y < 0, 1);
if isempty(lewat), M.overshoot_cm = 0; else, M.overshoot_cm = max(0, max(-s0 * y(lewat:end))); end
if abs(y0) >= 2, M.overshoot_pct = M.overshoot_cm / abs(y0) * 100; else, M.overshoot_pct = NaN; end
luar = find(abs(y) > pita_cm, 1, 'last');
if isempty(luar), M.settling_s = 0;
elseif luar == numel(y), M.settling_s = NaN;
else, M.settling_s = t(luar + 1) - t(1); end
akhir = t >= t(end) - 5;
M.ess_cm = mean(abs(y(akhir)));
M.IAE = sum(abs(y)) * dt;
M.ISE = sum(y.^2) * dt;
M.RMSE_cm = sqrt(mean(y.^2));
M.durasi_s = t(end) - t(1);
end
