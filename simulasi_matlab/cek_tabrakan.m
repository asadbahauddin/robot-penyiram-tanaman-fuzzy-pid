function nabrak = cek_tabrakan(pose, cfg, world, delta)
%CEK_TABRAKAN  true bila badan atau roda robot memotong dinding / tepi bedengan / rintangan.
if nargin < 4, delta = 0; end
S = world.seg;
poly = robot_jejak(pose, cfg, delta);
nabrak = false;
for k = 1:numel(poly)
    P = poly{k};
    E = [P, P([2:end 1], :)];
    for i = 1:size(E, 1)
        if any(potong(E(i,:), S)), nabrak = true; return; end
    end
    % ujung segmen di dalam poligon (mis. sudut rintangan kecil)
    if any(inpolygon(S(:,1), S(:,2), P(:,1), P(:,2)) | inpolygon(S(:,3), S(:,4), P(:,1), P(:,2)))
        nabrak = true; return;
    end
end
end

function h = potong(e, S)
p = e(1:2); r = e(3:4) - p;
q = S(:,1:2); s = S(:,3:4) - q;
den = r(1)*s(:,2) - r(2)*s(:,1);
qp = q - p;
t = (qp(:,1).*s(:,2) - qp(:,2).*s(:,1)) ./ den;
u = (qp(:,1)*r(2) - qp(:,2)*r(1)) ./ den;
h = abs(den) > 1e-12 & t >= 0 & t <= 1 & u >= 0 & u <= 1;
end
