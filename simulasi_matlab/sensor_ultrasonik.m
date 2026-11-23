function [d, titik, idxTanaman] = sensor_ultrasonik(pose, sdef, world, lihatTanaman, sc)
%SENSOR_ULTRASONIK  Ray-casting dalam kerucut +/- sc.cone_deg, ambil jarak terdekat.
%   pose  : [x y th] robot;  sdef : [x y arah_deg] sensor pada badan robot
%   lihatTanaman : true -> sensor hanya melihat tanaman (sensor kiri/kanan, dipasang tinggi)
%                  false -> sensor melihat dinding/bedengan & rintangan (navigasi & depan)
%   d     : jarak bersih (cm), Inf bila tidak ada pantulan dalam jangkauan maks
%   titik : [x y] ujung sinar terdekat (untuk animasi)
%   idxTanaman : indeks tanaman yang terkena (0 bila bukan tanaman)
c = cos(pose(3)); s = sin(pose(3));
o = [pose(1) + c*sdef(1) - s*sdef(2), pose(2) + s*sdef(1) + c*sdef(2)];
a0 = pose(3) + deg2rad(sdef(3));
sud = a0 + deg2rad(linspace(-sc.cone_deg, sc.cone_deg, sc.n_ray));
d = Inf; idxTanaman = 0; titik = o + sc.maks_cm * [cos(a0) sin(a0)];
for a = sud
    r = [cos(a) sin(a)];
    if ~lihatTanaman && ~isempty(world.seg)
        q = world.seg(:,1:2); sv = world.seg(:,3:4) - q;
        den = r(1)*sv(:,2) - r(2)*sv(:,1);
        qp = q - o;
        t = (qp(:,1).*sv(:,2) - qp(:,2).*sv(:,1)) ./ den;
        u = (qp(:,1)*r(2) - qp(:,2)*r(1)) ./ den;
        ok = abs(den) > 1e-12 & t > 0 & u >= 0 & u <= 1;
        if any(ok)
            tm = min(t(ok));
            if tm < d, d = tm; titik = o + tm*r; idxTanaman = 0; end
        end
    end
    if lihatTanaman && ~isempty(world.tanaman)
        C = world.tanaman(:,1:2); R = world.tanaman(:,3);
        oc = o - C;
        b = oc * r';
        cc = sum(oc.^2, 2) - R.^2;
        disc = b.^2 - cc;
        ok = disc >= 0;
        t = -b - sqrt(max(disc, 0));
        ok = ok & t > 0;
        if any(ok)
            [tm, im] = min(t(ok));
            idx = find(ok); idx = idx(im);
            if tm < d, d = tm; titik = o + tm*r; idxTanaman = idx; end
        end
    end
end
if d > sc.maks_cm
    d = Inf; idxTanaman = 0; titik = o + sc.maks_cm * [cos(a0) sin(a0)];
end
end
