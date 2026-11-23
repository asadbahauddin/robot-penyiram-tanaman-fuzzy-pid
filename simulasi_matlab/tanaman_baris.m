function T = tanaman_baris(x0, x1, ytepi, jarak, r, arah)
%TANAMAN_BARIS  Deretan tanaman [cx cy r] di balik tepi bedengan ytepi (sejajar sumbu x).
%   arah : +1 tanaman di atas tepi (y > ytepi), -1 di bawah. Bawaan: menjauhi garis y = 0.
%   Daun menjorok 5 cm ke lorong (asumsi). Bawaan jarak 60 cm, jari-jari daun 15 cm.
if nargin < 4 || isempty(jarak), jarak = 60; end
if nargin < 5 || isempty(r), r = 15; end
if nargin < 6 || isempty(arah), arah = sign(ytepi); end
cx = (x0:jarak:x1)';
cy = ytepi + arah * (r - 5) * ones(size(cx));
T = [cx cy r*ones(size(cx))];
end
