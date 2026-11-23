function [st, baris] = kendali_otomatis(st, d, dt, now, curSteps, cfg)
%KENDALI_OTOMATIS  Salinan autoModeStep() firmware v12 (dipanggil sekali per siklus sensor).
%   st       : keadaan kendali (dari kendali_init)
%   d        : struct getDist() mentah: depan, s_kiri, s_kanan, kiri, kanan (999 = tak terbaca)
%   dt       : waktu antar siklus (s), now : millis() (ms)
%   curSteps : posisi stepper saat ini (langkah)
%   baris    : struct satu baris log (kolom = LOG_HEADER)
k = cfg.k; spd = cfg.steer.stepsPerDeg; maxA = cfg.steer.maxAngle;
J=1; L=2; B=3; BK=4; BN=5; MU=6; LI=7;   % JELAJAH MELAMBAT BERHENTI BELOK_KIRI BELOK_KANAN MUNDUR LIHAT

%% 0) Baca sensor + filter
st.hL(st.hi+1) = d.s_kiri;
st.hR(st.hi+1) = d.s_kanan;
st.hi = mod(st.hi + 1, 3);
sKiri  = median3(st.hL);
sKanan = median3(st.hR);
[sKiri,  st.fL, st.nL] = saring(sKiri,  st.fL, st.nL, k.navMax);
[sKanan, st.fR, st.nR] = saring(sKanan, st.fR, st.nR, k.navMax);
kiri = d.kiri; kanan = d.kanan;
st.hF(st.fi+1) = d.depan; st.fi = mod(st.fi + 1, 3);
front = median3(st.hF);
if front >= 999 && st.lastFront < k.jarakAman && (st.lastFront < 30 || st.nHilang < 4)
    front = st.lastFront;
    st.nHilang = st.nHilang + 1;
else
    st.lastFront = front;
    st.nHilang = 0;
end

sp = -1;
i1 = mod(st.hi + 2, 3) + 1; i2 = mod(st.hi + 1, 3) + 1;
kiriBuka  = st.hL(i1) >= k.jarakBuka && st.hL(i2) >= k.jarakBuka;
kananBuka = st.hR(i1) >= k.jarakBuka && st.hR(i2) >= k.jarakBuka;

% deteksi macet
macet = false;
maju = any(st.state == [J L BK BN]) && st.motorSpeed > 0;
if ~maju || front >= k.jarakMundur + 15 || abs(front - st.fRef) > 3
    st.fRef = front; st.tRef = now;
elseif now - st.tRef > 1500
    macet = true; st.fRef = 999; st.tRef = now;
end

%% 1) Tentukan keadaan
tgtSteps = st.tgtAngle * spd;
kemudiSampai = abs(tgtSteps - curSteps) <= 3 * spd;
if st.state == MU
    t = now - st.faseStart;
    if st.fase == 0 && t >= k.JEDA_MS && (kemudiSampai || t >= 3000)
        st.fase = 1; st.faseStart = now;
    elseif st.fase == 1 && t >= k.waktuMundur
        st.fase = 2; st.faseStart = now;
    elseif st.fase == 2 && t >= k.JEDA_MS && (kemudiSampai || t >= 3000)
        st.turnStartMs = now;
        if st.turnDir > 0, st.state = BN;
        elseif st.turnDir < 0, st.state = BK;
        else, st.state = L; end
    end
elseif st.state == BK || st.state == BN
    t = now - st.turnStartMs;
    if front < k.jarakMundur || macet
        st = mulaiMundur(st, now, k);
    elseif t >= k.waktuBelok && front > k.jarakBelok
        st.state = J; st.turnDir = 0; st.mundurCount = 0;
        st.integral = 0; st.errorSeb = 0;
    elseif t > k.BELOK_MAKS_MS
        st.state = B;
    end
elseif k.LIHAT_SEBELUM_BELOK && st.state == LI
    t = now - st.faseStart;
    if ~st.jedaStepper
        if (curSteps == tgtSteps && t >= 300) || t >= 3000
            st.jedaStepper = true;
            st.lihatN = 0; st.lihatL = 0; st.lihatR = 0; st.lihatF = 999;
        end
    else
        st.lihatN = st.lihatN + 1;
        if st.lihatN >= 2
            st.lihatL = st.lihatL + min(d.s_kiri, 400);
            st.lihatR = st.lihatR + min(d.s_kanan, 400);
            if d.depan < st.lihatF, st.lihatF = d.depan; end
        end
        if st.lihatN >= 1 + k.LIHAT_SIKLUS
            rl = st.lihatL / k.LIHAT_SIKLUS; rr = st.lihatR / k.LIHAT_SIKLUS;
            st.jedaStepper = false;
            bukaL = rl >= k.jarakBuka; bukaR = rr >= k.jarakBuka;
            if bukaL, keKiri = ~bukaR || rl >= rr;
            elseif bukaR, keKiri = false;
            else, keKiri = rl >= rr; end
            st.turnDir = ifelse(keKiri, -1, 1);
            st.turnStartMs = now;
            if st.lihatF >= k.jarakBelok && st.lihatF >= k.jarakPaksa
                st.state = J; st.turnDir = 0;
            elseif st.lihatF < k.jarakMundur + k.MUNDUR_DULU_CM
                st = mulaiMundur(st, now, k);
            else
                st.state = ifelse(keKiri, BK, BN);
            end
        end
    end
elseif k.LIHAT_SEBELUM_BELOK && (front < k.jarakMundur || macet || front < k.jarakBelok || front < k.jarakPaksa)
    st.state = LI; st.lihatN = 0; st.faseStart = now;
elseif front < k.jarakMundur || macet
    if kiriBuka, keKiri = ~kananBuka || sKiri >= sKanan;
    elseif kananBuka, keKiri = false;
    else, keKiri = sKiri >= sKanan; end
    st.turnDir = ifelse(keKiri, -1, 1); st.turnStartMs = now;
    st = mulaiMundur(st, now, k);
elseif front < k.jarakBelok && (kiriBuka || kananBuka)
    keKiri = kiriBuka && (~kananBuka || sKiri >= sKanan);
    st.state = ifelse(keKiri, BK, BN); st.turnDir = ifelse(keKiri, -1, 1); st.turnStartMs = now;
elseif front < k.jarakPaksa
    keKiri = sKiri >= sKanan;
    st.state = ifelse(keKiri, BK, BN); st.turnDir = ifelse(keKiri, -1, 1); st.turnStartMs = now;
elseif st.state == B && front <= k.jarakAman
    st.state = B;
elseif front > k.jarakAman
    st.state = J; st.mundurCount = 0;
else
    st.state = L;
end

%% 2) Aksi
if st.state == LI
    st.pumpOn = false; st.motorSpeed = 0; st.integral = 0; st.errorSeb = 0;
elseif st.state == B
    st.pumpOn = false; st.motorSpeed = 0; st.integral = 0; st.errorSeb = 0;
    st.tgtAngle = batasRate(0, st.tgtAngle, k.steerRate, maxA);
elseif st.state == MU
    st.pumpOn = false;
    a = min(k.sudutBelok, maxA);
    arah = ifelse(st.fase == 2, st.turnDir, -st.turnDir);
    st.tgtAngle = clampA(fix(k.steerSign * arah * a), maxA);
    st.motorSpeed = ifelse(st.fase == 1, -k.pwmMundur, 0);
    st.integral = 0; st.errorSeb = 0;
elseif st.state == BK || st.state == BN
    st.pumpOn = false;
    a = min(k.sudutBelok, maxA);
    st.tgtAngle = clampA(fix(k.steerSign * ifelse(st.state == BN, a, -a)), maxA);
    siap = abs(st.tgtAngle * spd - curSteps) <= 5 * spd;
    st.motorSpeed = ifelse(siap || (now - st.turnStartMs) > 2000, k.KECEPATAN_MENDEKATI, 0);
    st.integral = 0; st.errorSeb = 0;
else  % JELAJAH / MELAMBAT : PID-Fuzzy
    okL = sKiri < k.navMax; okR = sKanan < k.navMax; ada = true; e = 0;
    if okL && okR
        tengah = (sKiri + sKanan) * 0.5;
        sp = ifelse(k.spTengah, tengah, k.SETPOINT_KIRI_CM);
        st.lastHalf = 0.8 * st.lastHalf + 0.2 * tengah;
        e = sp - sKiri;
    elseif okL
        sp = ifelse(k.spTengah, st.lastHalf, k.SETPOINT_KIRI_CM); e = sp - sKiri;
    elseif okR
        sp = ifelse(k.spTengah, st.lastHalf, k.SETPOINT_KIRI_CM); e = sKanan - sp;
    else
        ada = false;
    end
    if ada
        st.error = e;
        st.deltaError = st.error - st.errorSeb;
        if st.useFuzzy
            [st.Kp, st.Ki, st.Kd] = fuzzy_mamdani(st.error, st.deltaError, st.Kp, st.Ki, st.Kd, cfg.fz);
        else
            st.Kp = k.kpTetap; st.Ki = k.kiTetap; st.Kd = k.kdTetap;
        end
        pid.integral = st.integral;
        [output, pid] = pid_diskrit(st.error, st.deltaError, dt, st.Kp, st.Ki, st.Kd, pid, k);
        st.integral = pid.integral; st.P = pid.P; st.I = pid.I; st.D = pid.D; st.u = pid.u;
        batas = min(k.autoMaxAngle, maxA);
        sudut = min(max(k.steerSign * k.steerScale * output, -batas), batas);
        target = fix(sudut);
        if abs(st.error) < k.zonaMati
            target = 0; st.integral = 0;
        end
        st.tgtAngle = batasRate(target, st.tgtAngle, k.steerRate, maxA);
        st.errorSeb = st.error;
    else
        st.tgtAngle = batasRate(0, st.tgtAngle, k.steerRate, maxA);
        st.integral = 0;
    end
    st.motorSpeed = ifelse(st.state == J, k.KECEPATAN_JELAJAH, k.KECEPATAN_MENDEKATI);
    tanaman = kiri < k.jarakTanaman || kanan < k.jarakTanaman;
    st.pumpOn = tanaman && ~st.waterLow;
end

%% 4) Log satu baris (format LOG_HEADER)
kode = 'JLBKRMS';
baris = struct('ms', now, 'run', st.run, 'metode', ifelse(st.useFuzzy, 'F', 'P'), ...
    'status', kode(st.state), 'depan', front, 's_kiri', sKiri, 's_kanan', sKanan, ...
    'kiri', kiri, 'kanan', kanan, 'setpoint', sp, 'e', st.error, 'de', st.deltaError, ...
    'Kp', st.Kp, 'Ki', st.Ki, 'Kd', st.Kd, 'P', st.P, 'I', st.I, 'D', st.D, 'u', st.u, ...
    'sudut', st.tgtAngle, 'pwm', st.motorSpeed, 'pompa', double(st.pumpOn), 'air_habis', double(st.waterLow));
end

%% ---------- fungsi bantu (salinan perilaku firmware) ----------
function m = median3(h)
a = h(1); b = h(2); c = h(3);
m = max(min(a, b), min(max(a, b), c));
end

function [y, f, n] = saring(x, f, n, navMax)
LONJAK_CM = 15; ALFA = 0.6;
if x >= navMax, f = -1; n = 0; y = x; return; end
if f < 0, f = x; n = 0; y = f; return; end
if abs(x - f) > LONJAK_CM
    n = n + 1;
    if n < 3, y = f; return; end
end
n = 0;
f = ALFA * x + (1 - ALFA) * f;
y = f;
end

function st = mulaiMundur(st, now, k)
st.mundurCount = st.mundurCount + 1;
if st.mundurCount > k.MUNDUR_MAKS, st.state = 3; return; end   % BERHENTI
st.state = 6; st.fase = 0; st.faseStart = now;
end

function a = batasRate(target, sekarang, rate, maxA)
a = clampA(min(max(target, sekarang - rate), sekarang + rate), maxA);   % + steerRecalcTarget()
end

function a = clampA(a, maxA)
a = min(max(a, -maxA), maxA);
end

function y = ifelse(c, a, b)
if c, y = a; else, y = b; end
end
