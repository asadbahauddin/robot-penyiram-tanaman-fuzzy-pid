function w = lintasan_uturn(panjang, lebar, bedengan, ruang_putar)
%LINTASAN_UTURN  Lintasan serpentin: 3 lorong lurus sepanjang `panjang` dihubungkan 2 U-turn.
%   Lorong 1 (y = 0) ke arah +x -> U-turn kanan -> lorong 2 ke arah -x -> U-turn kiri -> lorong 3.
%   panjang     : panjang lurusan (cm)                       [UKUR: 540]
%   lebar       : lebar lorong (cm)                          [ASUMSI: 100]
%   bedengan    : lebar bedengan tanaman pemisah lorong (cm) [ASUMSI: 40]
%   ruang_putar : lebar area putar di ujung (cm)             [ASUMSI: 100]
if nargin < 1, panjang = 540; end
if nargin < 2, lebar = 100; end
if nargin < 3, bedengan = 40; end
if nargin < 4, ruang_putar = 100; end
w.nama = 'uturn';
w.judul = sprintf('Lintasan 2 U-turn (lurus %g cm, lorong %g cm)', panjang, lebar);
w.lebar = lebar; h = lebar/2; p = lebar + bedengan;  R = ruang_putar;
yAtas = h; yBawah = -2*p - h;
xKiri = -R; xKanan = panjang + R;
w.seg = [ xKiri  yAtas  xKanan yAtas           % dinding / bedengan luar atas
          xKiri  yBawah xKanan yBawah          % bawah
          xKanan yAtas  xKanan yBawah          % ujung kanan
          xKiri  yAtas  xKiri  yBawah];        % ujung kiri
% bedengan 1 (antara lorong 1 & 2): menempel dinding kiri, terbuka di ujung kanan (x = panjang)
b1 = [-h-bedengan, -h];  w.seg = [w.seg; kotak(xKiri, panjang, b1(1), b1(2))];
% bedengan 2 (antara lorong 2 & 3): menempel dinding kanan, terbuka di ujung kiri (x = 0)
b2 = [-p-h-bedengan, -p-h]; w.seg = [w.seg; kotak(0, xKanan, b2(1), b2(2))];
w.tanaman = [tanaman_baris(40, panjang-40, yAtas, [], [], +1)
             tanaman_baris(70, panjang-40, b1(2), [], [], -1)
             tanaman_baris(70, panjang-40, b1(1), [], [], +1)
             tanaman_baris(40, panjang-40, b2(2), [], [], -1)
             tanaman_baris(40, panjang-40, b2(1), [], [], +1)
             tanaman_baris(70, panjang-40, yBawah, [], [], -1)];
w.pose0 = [0 0 0];
w.x_selesai = Inf;
w.selesai = @(q) q(2) < -2*p + h && q(1) > panjang - 60 && abs(q(3)) < pi/4;  % ujung lorong 3
w.T_maks = 150;
w.y_ref = @(x, y) -p * round(-y / p);          % tengah lorong terdekat
w.batas = [xKiri-20 xKanan+20 yBawah-20 yAtas+20];
end

function S = kotak(x0, x1, y0, y1)
S = [x0 y0 x1 y0; x1 y0 x1 y1; x1 y1 x0 y1; x0 y1 x0 y0];
end
