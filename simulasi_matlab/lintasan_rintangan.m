function w = lintasan_rintangan()
%LINTASAN_RINTANGAN  Lorong lurus dengan rintangan (kotak 40 cm) menutup lorong di x = 450 cm.
w.nama = 'rintangan';
w.judul = 'Lorong lurus dengan rintangan di depan';
w.lebar = 100; h = w.lebar/2;
xr = 450; pr = 40;
w.seg = [-100  h  800  h
         -100 -h  800 -h
          xr  -h  xr   h            % sisi depan rintangan
          xr  -h  xr+pr -h
          xr   h  xr+pr  h
          xr+pr -h xr+pr h];
w.tanaman = [tanaman_baris(40, 760, h); tanaman_baris(70, 760, -h)];
w.pose0 = [0 0 0];
w.x_selesai = Inf;
w.T_maks = 40;
w.y_ref = @(x, y) zeros(size(x));
w.batas = [-120 820 -h-60 h+60];
end
