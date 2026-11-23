function pose = model_ackermann(pose, v, delta_rad, dt, L)
%MODEL_ACKERMANN  Model sepeda kinematik (titik acuan: tengah poros belakang).
%   x' = v cos(th),  y' = v sin(th),  th' = v tan(delta) / L
%   pose = [x y th] (cm, cm, rad), v (cm/s), delta (rad, + = roda ke kiri), dt (s), L (cm).
x = pose(1); y = pose(2); th = pose(3);
w = v * tan(delta_rad) / L;
if abs(w) < 1e-9
    x = x + v * cos(th) * dt;
    y = y + v * sin(th) * dt;
else                                    % integrasi eksak busur lingkaran
    th2 = th + w * dt;
    R = v / w;
    x = x + R * (sin(th2) - sin(th));
    y = y - R * (cos(th2) - cos(th));
    th = th2;
end
pose = [x y th];
end
