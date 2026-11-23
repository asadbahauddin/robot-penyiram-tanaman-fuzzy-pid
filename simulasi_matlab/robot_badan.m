function P = robot_badan(pose, cfg)
%ROBOT_BADAN  Titik sudut badan robot (koordinat dunia). Memakai cfg.badan_poly (dari robot.dxf)
%   bila ada, selain itu kotak panjang x lebar.
if isfield(cfg, 'badan_poly')
    lok = cfg.badan_poly;
else
    xb = -cfg.overhang_b; xf = cfg.panjang - cfg.overhang_b; w = cfg.lebar / 2;
    lok = [xb -w; xf -w; xf w; xb w];
end
c = cos(pose(3)); s = sin(pose(3));
P = [pose(1) + c*lok(:,1) - s*lok(:,2), pose(2) + s*lok(:,1) + c*lok(:,2)];
end
