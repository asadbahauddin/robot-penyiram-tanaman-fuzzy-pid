function T = tanaman_baris(x0, x1, ytepi, jarak, r)
%TANAMAN_BARIS  Deretan tanaman [cx cy r] di luar tepi bedengan ytepi.
%   Daun menjorok 5 cm ke lorong (asumsi). jarak antar tanaman 60 cm, jari-jari daun 15 cm.
if nargin < 4, jarak = 60; end
if nargin < 5, r = 15; end
cx = (x0:jarak:x1)';
cy = ytepi + sign(ytepi) * (r - 5) * ones(size(cx));
T = [cx cy r*ones(size(cx))];
end
