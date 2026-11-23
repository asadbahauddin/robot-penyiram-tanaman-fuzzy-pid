function P = robot_badan(pose, cfg)
%ROBOT_BADAN  Titik sudut badan robot (4x2, koordinat dunia) berlawanan jarum jam.
xb = -cfg.overhang_b; xf = cfg.panjang - cfg.overhang_b; w = cfg.lebar / 2;
lok = [xb -w; xf -w; xf w; xb w];
P = ke_dunia(lok, pose);
end

function W = ke_dunia(lok, pose)
c = cos(pose(3)); s = sin(pose(3));
W = [pose(1) + c*lok(:,1) - s*lok(:,2), pose(2) + s*lok(:,1) + c*lok(:,2)];
end
