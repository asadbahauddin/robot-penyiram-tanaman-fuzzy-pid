function [L, P] = baca_dxf(file)
%BACA_DXF  Baca entitas LINE dan POINT dari file DXF ASCII (SolidWorks, satuan sesuai file).
%   L : [x1 y1 x2 y2] tiap LINE,  P : [x y] tiap POINT.
t = regexp(fileread(file), '\r?\n', 'split');
t = strtrim(t);
n = floor(numel(t)/2);
kode = t(1:2:2*n); nilai = t(2:2:2*n);
L = zeros(0, 4); P = zeros(0, 2);
diEnt = false; jenis = ''; v = struct('x1',0,'y1',0,'x2',0,'y2',0);
for i = 1:n
    k = kode{i}; s = nilai{i};
    if strcmp(k, '2') && strcmp(s, 'ENTITIES'), diEnt = true; end
    if ~diEnt, continue; end
    if strcmp(k, '0')
        simpan();
        jenis = s;
        if strcmp(s, 'ENDSEC'), diEnt = false; end
        continue;
    end
    switch k
        case '10', v.x1 = str2double(s);
        case '20', v.y1 = str2double(s);
        case '11', v.x2 = str2double(s);
        case '21', v.y2 = str2double(s);
    end
end
simpan();

    function simpan()
        if strcmp(jenis, 'LINE'), L(end+1, :) = [v.x1 v.y1 v.x2 v.y2];
        elseif strcmp(jenis, 'POINT'), P(end+1, :) = [v.x1 v.y1]; end
        jenis = '';
    end
end
