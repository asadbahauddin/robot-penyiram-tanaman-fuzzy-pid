function cfg = config_robot()
%CONFIG_ROBOT  Semua parameter simulasi robot penyiram (satuan: cm, detik, derajat).
%   Nilai kendali disalin dari firmware program_robot_final_v12.ino.
%   Nilai FISIK yang ditandai [ASUMSI] belum diukur dari robot asli: ganti di sini.

%% ---------------- Dimensi robot [ASUMSI] ----------------
% Kerangka robot: titik asal di tengah poros roda BELAKANG, x ke depan, y ke kiri.
cfg.L          = 50;    % cm, jarak sumbu roda depan-belakang (wheelbase)      [ASUMSI]
cfg.panjang    = 80;    % cm, panjang badan                                    [ASUMSI]
cfg.lebar      = 60;    % cm, lebar badan                                      [ASUMSI]
cfg.overhang_b = 15;    % cm, badan di belakang poros belakang                 [ASUMSI]
cfg.lebar_roda = 50;    % cm, jarak roda kiri-kanan (untuk gambar)             [ASUMSI]

%% ---------------- Posisi & arah sensor [ASUMSI] ----------------
% [x y arah_deg] pada kerangka robot. Arah 0 = depan, +90 = kiri, -90 = kanan.
xd = cfg.panjang - cfg.overhang_b;                  % ujung depan badan
cfg.sensor.depan   = [xd,  0,              0];      % JSN depan
cfg.sensor.s_kiri  = [50,  cfg.lebar/2,   90];      % HC samping kiri  (navigasi)
cfg.sensor.s_kanan = [50, -cfg.lebar/2,  -90];      % HC samping kanan (navigasi)
cfg.sensor.kiri    = [20,  cfg.lebar/2,   90];      % HC kiri  (deteksi tanaman)
cfg.sensor.kanan   = [20, -cfg.lebar/2,  -90];      % HC kanan (deteksi tanaman)
cfg.sensor.urutan  = {'s_kanan','kanan','s_kiri','kiri','depan'};  % urutan baca firmware (ECHO_PIN 1..5)
cfg.sensor.cone_deg  = 15;     % setengah sudut pancar
cfg.sensor.n_ray     = 7;      % jumlah sinar per kerucut
cfg.sensor.maks_cm   = 400;    % di atas ini -> 999 (timeout)
cfg.sensor.min_cm    = 1;      % <= ini -> tidak valid -> 999 (firmware: d <= 1)
cfg.sensor.derau_sd  = 0.5;    % cm, derau Gaussian
cfg.sensor.anomali_dm556 = false;  % true: pembacaan semu ~50-56 cm saat DM556 aktif
cfg.sensor.p_anomali     = 0.15;   % peluang tiap pembacaan terganggu bila anomali aktif
cfg.sensor.jeda_ms       = 35;     % SEQ_INTER_MS
cfg.sensor.timeout_ms    = 25;     % SEQ_ECHO_TIMEOUT_US
cfg.sensor.cm_per_us     = 0.01715;% SOUND_CM_PER_US
cfg.sensor.baca_air_tiap = 8;      % sensor level air dibaca tiap 8 siklus
cfg.sensor.air_cm        = 20;     % jarak sensor->air (air cukup)

%% ---------------- Motor penggerak [ASUMSI: tanpa sensor kecepatan] ----------------
% Pemetaan PWM -> kecepatan (cm/s), interpolasi linear. PWM < 65 robot tidak bergerak.
cfg.motor.pwm_tab = [0  65  80  110  140  200  255];
cfg.motor.v_tab   = [0   0  20   30   35   45   50];   % cm/s  [ASUMSI]
cfg.motor.tau     = 0.30;  % s, konstanta waktu orde-1 percepatan/pengereman [ASUMSI]

%% ---------------- Kemudi stepper DM556 (firmware) ----------------
cfg.steer.maxAngle    = 45;     % g_maxAngle
cfg.steer.stepsPerDeg = 10;     % g_stepsPerDeg
cfg.steer.stepUs      = 700;    % STEP_INTERVAL_US
% Dengan "Balik arah kemudi" bawaan (g_steerSign = -1), sudut POSITIF = roda ke KIRI.
cfg.steer.kiri_positif = true;

%% ---------------- Parameter kendali otomatis (firmware) ----------------
k.jarakAman    = 150;   k.jarakBelok  = 100;  k.jarakMundur = 15;
k.waktuMundur  = 1500;  k.pwmMundur   = 140;  k.jarakBuka   = 100;
k.jarakPaksa   = 40;    k.navMax      = 80;   k.jarakTanaman = 40;
k.sudutBelok   = 40;    k.waktuBelok  = 1500; k.zonaMati    = 2.0;
k.BELOK_MAKS_MS = 20000; k.JEDA_MS = 400; k.MUNDUR_MAKS = 6;
k.LIHAT_SEBELUM_BELOK = true; k.LIHAT_SIKLUS = 3; k.MUNDUR_DULU_CM = 25;
k.KECEPATAN_JELAJAH = 110; k.KECEPATAN_MENDEKATI = 80;
k.spTengah = true; k.SETPOINT_KIRI_CM = 20;
k.kpTetap = 9.25; k.kiTetap = 9.25; k.kdTetap = 9.25;
k.INTEGRAL_MAX = 0.5; k.OUTPUT_MAX = 100;
k.steerScale = 0.08; k.autoMaxAngle = 10; k.steerRate = 4; k.steerSign = -1;
k.BATAS_AIR_HABIS = 33;
k.Kp0 = 0; k.Ki0 = 20; k.Kd0 = 0;   % nilai awal = fuzzy(0,0) v12
cfg.k = k;

%% ---------------- Fuzzy Mamdani (parameter dosen, sama dengan firmware v12) ----------------
cfg.fz = fuzzy_param();

%% ---------------- Simulasi ----------------
cfg.dt_ms   = 1;      % langkah fisika 1 ms
cfg.T_maks  = 40;     % s
cfg.seed    = 1;      % benih derau (hasil bisa diulang)
end
