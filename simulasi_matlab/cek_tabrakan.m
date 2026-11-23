function nabrak = cek_tabrakan(pose, cfg, world)
%CEK_TABRAKAN  true bila badan robot memotong dinding / rintangan.
P = robot_badan(pose, cfg);
E = [P, P([2 3 4 1], :)];             % 4 sisi badan: [x1 y1 x2 y2]
S = world.seg;
nabrak = false;
for i = 1:4
    if any(potong(E(i,:), S)), nabrak = true; return; end
end
% ujung segmen di dalam badan (mis. sudut rintangan kecil)
inside = inpolygon(S(:,1), S(:,2), P(:,1), P(:,2)) | inpolygon(S(:,3), S(:,4), P(:,1), P(:,2));
nabrak = any(inside);
end

function h = potong(e, S)
% perpotongan segmen e dengan setiap baris S (vektorisasi)
p = e(1:2); r = e(3:4) - p;
q = S(:,1:2); s = S(:,3:4) - q;
den = r(1)*s(:,2) - r(2)*s(:,1);
qp = q - p;
t = (qp(:,1).*s(:,2) - qp(:,2).*s(:,1)) ./ den;
u = (qp(:,1)*r(2) - qp(:,2)*r(1)) ./ den;
h = abs(den) > 1e-12 & t >= 0 & t <= 1 & u >= 0 & u <= 1;
end
