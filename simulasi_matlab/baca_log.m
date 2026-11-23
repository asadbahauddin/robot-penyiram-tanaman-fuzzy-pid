function L = baca_log(file)
%BACA_LOG  Baca log robot / simulasi. Mendukung format Excel Indonesia (sep=; ; desimal ,)
%   dan format biasa (koma, titik). Kolom metode/status menjadi teks, lainnya angka.
txt = fileread(file);
baris = regexp(txt, '\r?\n', 'split');
baris = baris(~cellfun(@isempty, strtrim(baris)));
excel = startsWith(strtrim(baris{1}), 'sep=');
if excel, baris(1) = []; end
if excel || contains(baris{1}, ';'), sep = ';'; else, sep = ','; end
hdr = strsplit(strtrim(baris{1}), sep);
n = numel(baris) - 1;
data = cell(n, numel(hdr));
for i = 1:n
    p = strsplit(strtrim(baris{i+1}), sep);
    if numel(p) ~= numel(hdr), continue; end      % baris rusak / judul ulang
    data(i, :) = p;
end
data = data(~cellfun(@isempty, data(:, 1)), :);
L = table();
for j = 1:numel(hdr)
    kol = data(:, j);
    if any(strcmp(hdr{j}, {'metode', 'status'}))
        L.(hdr{j}) = kol;
    else
        if sep == ';', kol = strrep(kol, ',', '.'); end
        L.(hdr{j}) = str2double(kol);
    end
end
end
