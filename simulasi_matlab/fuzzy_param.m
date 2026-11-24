function fz = fuzzy_param(range_kp, range_ki, range_kd)
%FUZZY_PARAM  Parameter fuzzy Mamdani sesuai gambar dosen = MF_IN, OUT_CENTER_*, RULE_K* firmware.
%   Input e & de : 7 segitiga NB NM NS Z PS PM PB pada [-6, 6] (disimpan sebagai trapesium a b c d).
%   Output Kp/Ki/Kd: 9 segitiga ZE VS S MS M MB B VB EVB berjarak sama pada rentang masing-masing.
%   Rentang bawaan dosen [0 20] untuk ketiganya; dosen mempersilakan rentang diperbaiki, maka
%   rentang tiap gain bisa diberikan sendiri: fuzzy_param([0 20], [0 5], [0 80]).
%   Rule: baris = ERROR (NB..PB), kolom = DELTA ERROR (NB..PB).
if nargin < 1 || isempty(range_kp), range_kp = [0 20]; end
if nargin < 2 || isempty(range_ki), range_ki = [0 20]; end
if nargin < 3 || isempty(range_kd), range_kd = [0 20]; end

fz.lbl_in  = {'NB','NM','NS','Z','PS','PM','PB'};
fz.lbl_out = {'ZE','VS','S','MS','M','MB','B','VB','EVB'};
fz.range_in  = [-6 6];
fz.MF_IN = [ -6 -6 -6 -4
             -6 -4 -4 -2
             -4 -2 -2  0
             -2  0  0  2
              0  2  2  4
              2  4  4  6
              4  6  6  6 ];
fz.range_kp = range_kp; fz.range_ki = range_ki; fz.range_kd = range_kd;
fz.CP = linspace(range_kp(1), range_kp(2), 9);   % titik pusat ZE..EVB untuk Kp
fz.CI = linspace(range_ki(1), range_ki(2), 9);   % ... untuk Ki
fz.CD = linspace(range_kd(1), range_kd(2), 9);   % ... untuk Kd
fz.OUT_CENTER = fz.CP;                            % (kompatibel dengan kode lama)
fz.range_out = range_kp;
fz.MF_OUT = segitiga(fz.CP);
fz.MF_OUT_I = segitiga(fz.CI);
fz.MF_OUT_D = segitiga(fz.CD);

ZE=1; VS=2; S=3; MS=4; M=5; MB=6; B=7; VB=8; EVB=9;
fz.RULE_KP = [ EVB VB  B   MB  B   VB  EVB
               VB  B   MB  M   MB  B   VB
               B   MB  M   MS  M   MB  B
               MB  M   MS  ZE  MS  M   MB
               B   MB  M   MS  M   MB  B
               VB  B   MB  M   MB  B   VB
               EVB VB  B   MB  B   VB  EVB ];
fz.RULE_KI = [ ZE  VS  S   MS  S   VS  ZE
               VS  S   MS  M   MS  S   VS
               S   MS  M   MB  M   MS  S
               MS  M   MB  EVB MB  M   MS
               S   MS  M   MB  M   MS  S
               VS  S   MS  M   MS  S   VS
               ZE  VS  S   MS  S   VS  ZE ];
fz.RULE_KD = [ EVB VB  B   MB  B   VB  EVB
               VB  B   MB  B   MB  B   VB
               B   MB  M   MS  M   MB  B
               MB  B   MS  ZE  MS  B   MB
               B   MB  M   MS  M   MB  B
               VB  B   MB  B   MB  B   VB
               EVB VB  B   MB  B   VB  EVB ];
end

function T = segitiga(c)
% segitiga (a b b d) untuk gambar; ujung ZE & EVB berupa bahu
d = c(2) - c(1);
T = [max(c - d, c(1)); c; c; min(c + d, c(end))]';
end
