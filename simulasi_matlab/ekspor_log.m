function ekspor_log(H, file, excelID)
%EKSPOR_LOG  Tulis log simulasi dengan header & format angka SAMA PERSIS dengan robot (LOG_HEADER).
%   excelID = true (bawaan): format unduhan /log.csv robot (baris 'sep=;', pemisah ';', desimal ',').
%   excelID = false        : format Serial / log.txt (pemisah ',', desimal '.').
if nargin < 3, excelID = true; end
HEADER = 'ms,run,metode,status,depan,s_kiri,s_kanan,kiri,kanan,setpoint,e,de,Kp,Ki,Kd,P,I,D,u,sudut,pwm,pompa,air_habis';
L = H.log;
fid = fopen(file, 'w');
if fid < 0, error('Tidak bisa menulis %s', file); end
if excelID, fprintf(fid, 'sep=;\r\n'); end
fprintf(fid, '%s\r\n', konversi(HEADER, excelID));
for i = 1:height(L)
    s = sprintf('%d,%u,%c,%c,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d', ...
        L.ms(i), L.run(i), L.metode{i}, L.status{i}, L.depan(i), L.s_kiri(i), L.s_kanan(i), ...
        L.kiri(i), L.kanan(i), L.setpoint(i), L.e(i), L.de(i), L.Kp(i), L.Ki(i), L.Kd(i), ...
        L.P(i), L.I(i), L.D(i), L.u(i), L.sudut(i), L.pwm(i), L.pompa(i), L.air_habis(i));
    fprintf(fid, '%s\r\n', konversi(s, excelID));
end
fclose(fid);
end

function s = konversi(s, excelID)
% sama dengan toExcelID() firmware: ',' -> ';' lalu '.' -> ','
if excelID
    s = strrep(s, ',', ';');
    s = strrep(s, '.', ',');
end
end
