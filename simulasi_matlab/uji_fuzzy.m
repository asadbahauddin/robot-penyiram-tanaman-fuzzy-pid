% UJI_FUZZY  Bandingkan fuzzy_mamdani() dengan hitungan tangan dari tabel rule dosen.
fz = fuzzy_param();
% kolom: e, de, Kp, Ki, Kd (harapan, dihitung tangan)
%  (0,0)  : hanya aturan Z-Z aktif (mu=1)            -> Kp ZE=0,   Ki EVB=20, Kd ZE=0
%  (1,0)  : Z 0,5 & PS 0,5 ; de Z=1                  -> Kp (0+7,5)/2=3,75  Ki (20+12,5)/2=16,25  Kd 3,75
%  (2,2)  : PS-PS mu=1                               -> Kp M=10,  Ki M=10,  Kd M=10
%  (-4,0) : NM-Z mu=1                                -> Kp M=10,  Ki M=10,  Kd B=15
%  (6,6)  : PB-PB                                    -> Kp EVB=20, Ki ZE=0, Kd EVB=20
%  (9,-9) : disaturasi ke (6,-6) PB-NB               -> Kp EVB=20, Ki ZE=0, Kd EVB=20
%  (3,-1) : e PS .5 PM .5 ; de NS .5 Z .5
%           Kp: PS-NS M .5, PS-Z MS .5, PM-NS MB .5, PM-Z M .5 -> (10+7,5+12,5)/3 = 10
%           Ki: PS-NS M, PS-Z MB, PM-NS MS, PM-Z M -> (10+12,5+7,5)/3 = 10
%           Kd: PS-NS M, PS-Z MS, PM-NS MB, PM-Z B -> (10+7,5+12,5+15)/4 = 11,25
T = [ 0  0   0    20     0
      1  0   3.75 16.25  3.75
      2  2  10    10    10
     -4  0  10    10    15
      6  6  20     0    20
      9 -9  20     0    20
      3 -1  10    10    11.25];
lulus = true;
fprintf('   e     de  |   Kp     Ki     Kd   | harapan\n');
for i = 1:size(T,1)
    [kp, ki, kd] = fuzzy_mamdani(T(i,1), T(i,2), -1, -1, -1, fz);
    ok = all(abs([kp ki kd] - T(i,3:5)) < 1e-9);
    lulus = lulus && ok;
    fprintf('%5.1f %5.1f  | %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f  %s\n', ...
        T(i,1), T(i,2), kp, ki, kd, T(i,3:5), ifelse_str(ok));
end
if ~lulus, error('Uji fuzzy GAGAL'); end
fprintf('Uji fuzzy LULUS (%d kasus)\n', size(T,1));

function s = ifelse_str(ok)
if ok, s = 'OK'; else, s = 'SALAH'; end
end
