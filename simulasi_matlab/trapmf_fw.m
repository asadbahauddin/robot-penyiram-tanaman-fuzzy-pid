function mu = trapmf_fw(x, p)
%TRAPMF_FW  Salinan persis trapmf() firmware (segitiga = trapesium dengan b = c).
a = p(1); b = p(2); c = p(3); d = p(4);
if x < a || x > d
    mu = 0;
elseif x >= b && x <= c
    mu = 1;
elseif x < b
    mu = (x - a) / (b - a);
else
    mu = (d - x) / (d - c);
end
end
