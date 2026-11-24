function w = lintasan_lurus(offset_cm, sudut_awal_deg, lebar, panjang)
%LINTASAN_LURUS  Lorong lurus dengan deretan tanaman kiri & kanan, ujung depan terbuka.
%   offset_cm      : posisi awal robot dari tengah lorong (+ = ke kiri). Bawaan 0.
%   sudut_awal_deg : arah awal robot (+ = menghadap kiri). Bawaan 0.
%   lebar          : lebar lorong (cm). Bawaan 100.
%   panjang        : panjang lorong (cm). Bawaan 800.
if nargin < 1, offset_cm = 0; end
if nargin < 2, sudut_awal_deg = 0; end
if nargin < 3 || isempty(lebar), lebar = 100; end
if nargin < 4 || isempty(panjang), panjang = 800; end
w.nama = 'lurus';
if offset_cm ~= 0, w.nama = 'tidak_di_tengah';
elseif sudut_awal_deg ~= 0, w.nama = 'miring'; end
w.judul = sprintf('Lorong lurus %g cm (awal y = %+g cm, \\theta = %+g\\circ)', lebar, offset_cm, sudut_awal_deg);
w.lebar = lebar;
h = lebar/2;
xa = -100; xb = panjang;
w.seg = [xa  h xb  h                % tepi bedengan kiri
         xa -h xb -h];              % tepi bedengan kanan
w.tanaman = [tanaman_baris(40, xb-40, h); tanaman_baris(70, xb-40, -h)];
w.pose0 = [0, offset_cm, deg2rad(sudut_awal_deg)];
w.x_selesai = panjang - 100;        % simulasi berhenti saat poros belakang melewati x ini
w.T_maks = max(40, panjang / 25);
w.y_ref = @(x, y) zeros(size(x));   % garis tengah lorong (untuk metrik)
w.batas = [xa-20 xb+20 -h-60 h+60];
end
