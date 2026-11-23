function w = lintasan_jalur(file, tebal_sekat)
%LINTASAN_JALUR  Lintasan asli dari jalur.dxf (SolidWorks, satuan cm): 3 lorong + 2 U-turn.
%   Dinding luar = bedengan menempel tembok. Garis mendatar di dalam (bukan dinding luar)
%   dianggap SEKAT/bedengan pemisah dengan tebal `tebal_sekat` (bawaan 5 cm), berpusat di garisnya.
%   Tanaman diletakkan di atas setiap bedengan (dinding panjang & sekat) tiap 60 cm [ASUMSI],
%   dan hanya terlihat oleh sensor kiri/kanan (pendeteksi tanaman).
if nargin < 1 || isempty(file)
    file = fullfile(fileparts(mfilename('fullpath')), 'dxf', 'jalur.dxf');
end
if nargin < 2, tebal_sekat = 5; end
L = baca_dxf(file);
L = L(hypot(L(:,3)-L(:,1), L(:,4)-L(:,2)) > 1, :);             % buang garis nol
xmin = min([L(:,1); L(:,3)]); xmax = max([L(:,1); L(:,3)]);
ymin = min([L(:,2); L(:,4)]); ymax = max([L(:,2); L(:,4)]);
datar = abs(L(:,4) - L(:,2)) < 0.5;                             % termasuk garis hampir datar
yl = (L(:,2) + L(:,4)) / 2;
dalam = datar & yl > ymin + 1 & yl < ymax - 1;

seg = L(~dalam, :);
% sekat: kelompokkan garis dalam yang berdekatan (gambar memuat garis ganda/diagonal tipis)
D = L(dalam, :); sudah = false(size(D,1), 1); sekat = zeros(0, 3);  % [x0 x1 y]
for i = 1:size(D, 1)
    if sudah(i), continue; end
    sama = abs((D(:,2)+D(:,4))/2 - (D(i,2)+D(i,4))/2) < 2;
    sudah = sudah | sama;
    xs = [D(sama,1); D(sama,3)]; ys = [D(sama,2); D(sama,4)];
    sekat(end+1, :) = [min(xs) max(xs) mean(ys)]; %#ok<AGROW>
end
h = tebal_sekat / 2;
for i = 1:size(sekat, 1)
    x0 = sekat(i,1); x1 = sekat(i,2); y = sekat(i,3);
    seg = [seg; x0 y-h x1 y-h; x1 y-h x1 y+h; x1 y+h x0 y+h; x0 y+h x0 y-h]; %#ok<AGROW>
end

% tanaman di atas bedengan
r = 10; T = zeros(0, 3);
for yb = [ymin ymax]
    xs = (xmin + 40 : 60 : xmax - 30)';
    T = [T; xs, yb*ones(size(xs)), r*ones(size(xs))]; %#ok<AGROW>
end
for i = 1:size(sekat, 1)
    xs = (sekat(i,1) + 40 : 60 : sekat(i,2) - 30)';
    T = [T; xs, sekat(i,3)*ones(size(xs)), r*ones(size(xs))]; %#ok<AGROW>
end

% batas lorong (untuk metrik & selesai)
ysk = sort(sekat(:,3))';
tepiB = [ymin, ysk + h]; tepiA = [ysk - h, ymax];
w.tengah = (tepiB + tepiA) / 2;                 % garis tengah tiap lorong
w.lebar_lorong = tepiA - tepiB;
w.nama = 'jalur';
w.judul = sprintf('Lintasan asli jalur.dxf (%d lorong, %.0f cm, sekat %g cm)', numel(w.tengah), xmax - xmin, tebal_sekat);
w.lebar = w.lebar_lorong(1);
w.seg = seg;
w.tanaman = T;
w.pose0 = [xmin + 5 + 10, w.tengah(1), 0];      % 5 cm dari dinding, poros belakang 10 cm dari ekor
yc = w.tengah; xe = xmax;
w.selesai = @(q) q(2) > (yc(end) + yc(end-1))/2 && q(1) > xe - 120 && abs(q(3)) < pi/4;
w.x_selesai = Inf;
w.T_maks = 180;
w.y_ref = @(x, y) lorong_terdekat(y, yc);
w.batas = [xmin-20 xmax+20 ymin-20 ymax+20];
end

function yr = lorong_terdekat(y, yc)
[~, i] = min(abs(y(:) - yc), [], 2);
yr = reshape(yc(i), size(y));
end
