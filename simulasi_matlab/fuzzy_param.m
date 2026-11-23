function fz = fuzzy_param()
%FUZZY_PARAM  Parameter fuzzy Mamdani sesuai gambar dosen = MF_IN, OUT_CENTER, RULE_K* firmware v12.
%   Input e & de : 7 segitiga NB NM NS Z PS PM PB pada [-6, 6] (disimpan sebagai trapesium a b c d).
%   Output Kp/Ki/Kd: 9 segitiga ZE VS S MS M MB B VB EVB pada [0, 20], pusat tiap 2,5.
%   Rule: baris = ERROR (NB..PB), kolom = DELTA ERROR (NB..PB).

fz.lbl_in  = {'NB','NM','NS','Z','PS','PM','PB'};
fz.lbl_out = {'ZE','VS','S','MS','M','MB','B','VB','EVB'};
fz.range_in  = [-6 6];
fz.range_out = [0 20];
fz.MF_IN = [ -6 -6 -6 -4
             -6 -4 -4 -2
             -4 -2 -2  0
             -2  0  0  2
              0  2  2  4
              2  4  4  6
              4  6  6  6 ];
fz.OUT_CENTER = 0:2.5:20;                     % ZE..EVB
c = fz.OUT_CENTER;
fz.MF_OUT = [max(c-2.5,0); c; c; min(c+2.5,20)]';   % segitiga (a b b d) untuk gambar

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
