function H = simulasi(world, cfg, useFuzzy)
%SIMULASI  Jalankan robot pada lintasan world dengan metode PID-Fuzzy (useFuzzy=true) atau PID tetap.
%   Pewaktuan meniru firmware:
%   - sensorTaskFn: sensor dibaca berurutan (level air tiap 8 siklus, lalu s_kanan, kanan,
%     s_kiri, kiri, depan). Tiap sensor = waktu gema (timeout 25 ms) + jeda 35 ms.
%   - autoModeStep dipanggil sekali per siklus sensor, dt = selisih millis() antar panggilan.
%   - steerLoop: stepper 1 langkah tiap 700 us menuju target (10 langkah/derajat).
%   Fisika diintegrasikan tiap 1 ms (model sepeda kinematik + motor orde-1).
rng(cfg.seed);
if isfield(world, 'T_maks'), Tmaks = world.T_maks; else, Tmaks = cfg.T_maks; end
sc = cfg.sensor; k = cfg.k; spd = cfg.steer.stepsPerDeg;
namaS = cfg.sensor.urutan;                 % {'s_kanan','kanan','s_kiri','kiri','depan'}
v13 = isfield(cfg.k, 'versi') && cfg.k.versi >= 13;
if v13   % v13: kelima sensor = sensor jarak navigasi (melihat dinding/bedengan & rintangan)
    tanamanS = struct('s_kanan', false, 'kanan', false, 's_kiri', false, 'kiri', false, 'depan', false);
else     % v12: sensor kiri/kanan hanya melihat tanaman (pemicu pompa)
    tanamanS = struct('s_kanan', false, 'kanan', true, 's_kiri', false, 'kiri', true, 'depan', false);
end
arahKemudi = 1; if ~cfg.steer.kiri_positif, arahKemudi = -1; end

st = kendali_init(cfg, useFuzzy);
pose = world.pose0; v = 0; cur = 0; stepAcc = 0;
g = struct('s_kanan', 999, 'kanan', 999, 's_kiri', 999, 'kiri', 999, 'depan', 999, 'air', 999);
titik = struct(); idxT = struct();
for n = namaS, titik.(n{1}) = pose(1:2); idxT.(n{1}) = 0; end

nT = size(world.tanaman, 1);
tSiram = nan(nT, 1);                       % waktu pertama tanaman disiram

nAir = 0; daftar = {}; pos = 0; tNext = 0; lastAutoMs = 0;
LOG = [];
nMax = Tmaks*1000;
TR = nan(ceil(nMax/10)+1, 9);  nTR = 0;    % [t x y th delta_deg v pompa status sudut_target]
FR = struct('t', {}, 'pose', {}, 'delta', {}, 'ray', {}, 'pompa', {}, 'status', {});
nTabrak = 0;
dtf = cfg.dt_ms / 1000;

for t = 0:cfg.dt_ms:nMax
    %% ---------- tugas sensor (FreeRTOS) ----------
    while t >= tNext
        if pos == 0                                     % siklus baru
            nAir = mod(nAir + 1, sc.baca_air_tiap);
            if nAir == 0, daftar = [{'air'}, namaS]; else, daftar = namaS; end
            pos = 1;
        end
        if pos <= numel(daftar)
            nm = daftar{pos};
            if strcmp(nm, 'air')
                d = sc.air_cm + sc.derau_sd*randn; ti = 0;
            else
                [d, pt, ti] = sensor_ultrasonik(pose, cfg.sensor.(nm), world, tanamanS.(nm), sc);
                titik.(nm) = pt;
                if isfinite(d), d = d + sc.derau_sd*randn; end
                if sc.anomali_dm556 && ~st.jedaStepper && rand < sc.p_anomali
                    d = 50 + 6*rand;                    % pembacaan semu derau DM556
                end
            end
            if ~isfinite(d) || d > sc.maks_cm || d <= sc.min_cm
                g.(nm) = 999; tg = sc.timeout_ms;      % pulseIn timeout -> getDist() = 999
                if ~strcmp(nm, 'air'), idxT.(nm) = 0; end
            else
                g.(nm) = d; tg = d / sc.cm_per_us / 1000;
                if ~strcmp(nm, 'air'), idxT.(nm) = ti; end
            end
            tNext = tNext + tg + sc.jeda_ms;
            pos = pos + 1;
        else
            % siklus selesai -> loop() memanggil autoModeStep(dt)
            now = floor(t);
            if lastAutoMs == 0, dt = 0.25; else, dt = (now - lastAutoMs) / 1000; end
            lastAutoMs = now;
            [st, baris] = kendali_otomatis(st, g, dt, now, cur, cfg);
            if isempty(LOG), LOG = baris; else, LOG(end+1, 1) = baris; end %#ok<AGROW>
            pos = 0;
        end
    end

    %% ---------- penyiraman ----------
    if st.pumpOn && v13 && nT > 0
        % v13: pompa menyala saat jalan lurus; tanaman tersiram bila berada di samping robot
        % (sepanjang badan, sampai cfg.k.jangkauSiram cm dari garis tengah robot)
        dx = world.tanaman(:,1) - pose(1); dy = world.tanaman(:,2) - pose(2);
        lx = cos(pose(3))*dx + sin(pose(3))*dy; ly = -sin(pose(3))*dx + cos(pose(3))*dy;
        kena = lx > -cfg.overhang_b & lx < cfg.panjang - cfg.overhang_b & abs(ly) < k.jangkauSiram & isnan(tSiram);
        tSiram(kena) = t/1000;
    elseif st.pumpOn
        for nm = {'kiri', 'kanan'}
            i = idxT.(nm{1});
            if i > 0 && g.(nm{1}) < k.jarakTanaman && isnan(tSiram(i)), tSiram(i) = t/1000; end
        end
    end

    %% ---------- steerLoop (stepper) ----------
    tgt = st.tgtAngle * spd;
    if ~st.jedaStepper
        stepAcc = stepAcc + cfg.dt_ms * 1000 / cfg.steer.stepUs;
        while stepAcc >= 1 && cur ~= tgt
            cur = cur + sign(tgt - cur); stepAcc = stepAcc - 1;
        end
        if cur == tgt, stepAcc = min(stepAcc, 1); end
    end
    delta = arahKemudi * deg2rad(cur / spd);

    %% ---------- motor BTS7960 + kinematika ----------
    pwm = st.motorSpeed;
    vT = sign(pwm) * interp1(cfg.motor.pwm_tab, cfg.motor.v_tab, min(abs(pwm), 255));
    v = v + (vT - v) * dtf / cfg.motor.tau;
    if abs(v) > 1e-6
        p2 = model_ackermann(pose, v, delta, dtf, cfg.L);
        if cek_tabrakan(p2, cfg, world, delta)
            v = 0; nTabrak = nTabrak + 1;          % tertahan dinding (roda selip)
        else
            pose = p2;
        end
    end

    %% ---------- rekam ----------
    if mod(t, 10) == 0
        nTR = nTR + 1;
        TR(nTR, :) = [t/1000, pose, rad2deg(delta), v, st.pumpOn, st.state, st.tgtAngle];
    end
    if mod(t, 100) == 0
        R = zeros(numel(namaS), 2);
        for j = 1:numel(namaS), R(j,:) = titik.(namaS{j}); end
        FR(end+1) = struct('t', t/1000, 'pose', pose, 'delta', delta, 'ray', R, ...
                           'pompa', st.pumpOn, 'status', st.state); %#ok<AGROW>
    end

    %% ---------- selesai? ----------
    if pose(1) > world.x_selesai, break; end
    if isfield(world, 'y_selesai') && pose(2) < world.y_selesai, break; end
    if isfield(world, 'selesai') && world.selesai(pose), break; end
end

H.world = world; H.cfg = cfg; H.useFuzzy = useFuzzy;
H.metode = 'PID-Fuzzy'; if ~useFuzzy, H.metode = 'PID tetap'; end
H.log = struct2table(LOG);
H.traj = array2table(TR(1:nTR, :), 'VariableNames', ...
    {'t','x','y','th','delta_deg','v','pompa','status','sudut_target'});
H.frame = FR;
H.tSiram = tSiram;
H.nTabrak = nTabrak;
H.namaSensor = namaS;
end
