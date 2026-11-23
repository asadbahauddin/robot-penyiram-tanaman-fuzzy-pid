function tulis_csv_id(T, file)
%TULIS_CSV_ID  Tulis tabel angka ke CSV format Excel Indonesia (sep=; , pemisah ';', desimal ',').
fid = fopen(file, 'w');
fprintf(fid, 'sep=;\r\n%s\r\n', strjoin(T.Properties.VariableNames, ';'));
A = T{:, :};
fmt = [strjoin(repmat({'%.4f'}, 1, size(A, 2)), ';') '\r\n'];
s = sprintf(fmt, A');
fprintf(fid, '%s', strrep(s, '.', ','));
fclose(fid);
end
