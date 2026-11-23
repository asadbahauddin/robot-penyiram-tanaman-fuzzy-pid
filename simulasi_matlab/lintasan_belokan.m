function w = lintasan_belokan(lebar_setelah)
%LINTASAN_BELOKAN  Lorong lurus 6 m lalu belok KANAN 90 derajat (dinding di ujung).
%   lebar_setelah : lebar lorong setelah belokan (cm). Bawaan 100.
if nargin < 1, lebar_setelah = 150; end
w.nama = 'belokan';
if lebar_setelah < 150, w.nama = 'belokan_sempit'; end
w.judul = sprintf('Lorong dengan belokan kanan 90\\circ (lebar setelah belokan %g cm)', lebar_setelah);
w.lebar = 100; h = w.lebar/2;
xb = 600;                           % dinding ujung (luar)
xi = xb - lebar_setelah;            % tepi dalam belokan
yb = -500;
w.seg = [-100  h  xb  h             % tepi kiri (luar)
          xb  h   xb  yb            % dinding ujung / tepi kiri lorong baru
         -100 -h  xi -h             % tepi kanan (dalam)
          xi  -h  xi  yb];          % tepi kanan lorong baru
w.tanaman = [tanaman_baris(40, xb-60, h); tanaman_baris(70, xi-40, -h)];
w.pose0 = [0 0 0];
w.x_selesai = Inf;
w.y_selesai = yb + 150;             % selesai bila robot sudah masuk lorong baru
w.T_maks = 50;
w.y_ref = @(x, y) zeros(size(x));
w.batas = [-120 xb+60 yb-20 h+60];
end
