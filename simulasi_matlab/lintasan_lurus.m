function w = lintasan_lurus(offset_cm, sudut_awal_deg)
%LINTASAN_LURUS  Lorong lurus 8 m selebar w.lebar dengan deretan tanaman kiri & kanan.
%   offset_cm      : posisi awal robot dari tengah lorong (+ = ke kiri). Bawaan 0.
%   sudut_awal_deg : arah awal robot (+ = menghadap kiri). Bawaan 0.
if nargin < 1, offset_cm = 0; end
if nargin < 2, sudut_awal_deg = 0; end
w.nama = 'lurus';
if offset_cm ~= 0, w.nama = 'tidak_di_tengah';
elseif sudut_awal_deg ~= 0, w.nama = 'miring'; end
w.judul = sprintf('Lorong lurus (awal y = %+g cm, \\theta = %+g\\circ)', offset_cm, sudut_awal_deg);
w.lebar = 100;                      % cm  [ASUMSI]
h = w.lebar/2;
xa = -100; xb = 800;
w.seg = [xa  h xb  h                % tepi bedengan kiri
         xa -h xb -h];              % tepi bedengan kanan
w.tanaman = [tanaman_baris(40, 760, h); tanaman_baris(70, 760, -h)];
w.pose0 = [0, offset_cm, deg2rad(sudut_awal_deg)];
w.x_selesai = 700;                  % simulasi berhenti saat poros belakang melewati x ini
w.T_maks = 40;
w.y_ref = @(x, y) zeros(size(x));   % garis tengah lorong (untuk metrik)
w.batas = [xa-20 xb+20 -h-60 h+60];
end
