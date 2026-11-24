function T = tanaman_baris(x0, x1, ytepi, jarak, r, arah)
%TANAMAN_BARIS  Deretan tanaman [cx cy r] di atas bedengan pada garis ytepi (sejajar sumbu x).
%   Bedengan = tembok tepi lorong, tanaman berada tepat di atas garis bedengan (sama dengan
%   lintasan_jalur). Bawaan jarak antar tanaman 60 cm, jari-jari daun 10 cm [ASUMSI].
%   arah : dipertahankan untuk kompatibilitas (tidak lagi menggeser tanaman).
if nargin < 4 || isempty(jarak), jarak = 60; end
if nargin < 5 || isempty(r), r = 10; end
if nargin < 6, arah = []; end %#ok<NASGU>
cx = (x0:jarak:x1)';
T = [cx, ytepi * ones(size(cx)), r * ones(size(cx))];
end
