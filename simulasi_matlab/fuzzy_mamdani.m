function [kp, ki, kd] = fuzzy_mamdani(e, de, kp, ki, kd, fz)
%FUZZY_MAMDANI  Salinan persis fuzzyTuning() firmware v12.
%   Fuzzifikasi (7 segitiga) -> inferensi min-max (49 aturan) -> COA titik pusat.
%   Bila tidak ada aturan aktif, nilai kp/ki/kd lama dipertahankan.
if nargin < 6, fz = fuzzy_param(); end
e  = min(max(e,  fz.range_in(1)), fz.range_in(2));
de = min(max(de, fz.range_in(1)), fz.range_in(2));

nIn = size(fz.MF_IN, 1);  nOut = numel(fz.OUT_CENTER);
muE = zeros(1, nIn); muDE = zeros(1, nIn);
for k = 1:nIn
    muE(k)  = trapmf_fw(e,  fz.MF_IN(k,:));
    muDE(k) = trapmf_fw(de, fz.MF_IN(k,:));
end

aggP = zeros(1, nOut); aggI = aggP; aggD = aggP;
for i = 1:nIn            % i = error (baris)
    for j = 1:nIn        % j = delta error (kolom)
        alpha = min(muE(i), muDE(j));
        if alpha <= 0, continue; end
        aggP(fz.RULE_KP(i,j)) = max(aggP(fz.RULE_KP(i,j)), alpha);
        aggI(fz.RULE_KI(i,j)) = max(aggI(fz.RULE_KI(i,j)), alpha);
        aggD(fz.RULE_KD(i,j)) = max(aggD(fz.RULE_KD(i,j)), alpha);
    end
end
z = fz.OUT_CENTER;
if sum(aggP) > 1e-4, kp = sum(aggP .* z) / sum(aggP); end
if sum(aggI) > 1e-4, ki = sum(aggI .* z) / sum(aggI); end
if sum(aggD) > 1e-4, kd = sum(aggD .* z) / sum(aggD); end
end
