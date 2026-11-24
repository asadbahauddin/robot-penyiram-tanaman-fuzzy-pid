/*
 * ============================================================================
 *  robot_final_manual_otomatis.ino  -  Firmware Robot Wall-Follower Penyiram
 * ============================================================================
 *
 *  Ini adalah PENGGABUNGAN dari:
 *   1) robot_diagnostic.ino kamu (pin mapping, sensor task FreeRTOS,
 *      motor BTS7960, steering DM556 lewat BC547 inverting, web UI) -
 *      SEMUA PIN DI FILE INI MENGIKUTI robot_diagnostic.ino KARENA SUDAH
 *      TERBUKTI BEKERJA DI HARDWARE KAMU.
 *   2) Logika Fuzzy (depan) + Fuzzy-PID adaptif (kiri-kanan) dari program
 *      "manual + otomatis" sebelumnya, sekarang dipetakan ke sistem
 *      TARGET SUDUT stepper (bukan pulsa mentah per-loop) supaya konsisten
 *      dengan kalibrasi max-angle & steps/derajat yang sudah kamu punya.
 *
 *  CATATAN SOAL "STEPPER HANYA BELOK KIRI":
 *  Logika arah (dirPos) di kode ini SIMETRIS untuk kedua arah - artinya
 *  ini kemungkinan besar BUKAN bug software, melainkan masalah di jalur
 *  DIR menuju DM556 (resistor base BC547 lepas/salah nilai, transistor
 *  rusak, kolektor-emitor tertukar, atau tidak ada pull-up di DIR+).
 *  Ukur tegangan di pin DIR yang masuk ke DM556 saat slider steering
 *  digeser kiri vs kanan - kalau tidak berubah, itu konfirmasi masalah
 *  di rangkaian BC547/DIR, bukan di firmware ini.
 *
 *  v9: perbaikan navigasi otomatis (dari analisis log 49/51/52):
 *      - jarak depan tidak lagi "terkunci" pada pembacaan semu (mis. 36/56 cm) saat sensor
 *        depan tidak menerima pantulan (999) -> robot tidak belok/mundur padahal depan jauh;
 *      - riwayat filter & pewaktu macet dikosongkan setiap run baru -> tidak langsung MUNDUR
 *        begitu OTOMATIS ditekan;
 *      - deteksi macet hanya bila depan < jarak mundur + 15 cm (dulu < 60 cm);
 *      - opsi PIN_TRIG_DEPAN: kabel TRIG terpisah untuk JSN depan (anti-crosstalk), bawaan mati;
 *      - log dikirim juga lewat WebSocket untuk logger_robot.py di laptop.
 *      - JEDA_STEPPER_UKUR: driver stepper DM556 dimatikan sesaat selama setiap sensor
 *        mengukur. Uji membuktikan DM556 aktif membuat semua sensor membaca ~50-56 cm semu.
 *        (PWM motor 25 kHz; JEDA_MOTOR_UKUR_DEPAN ada tetapi dimatikan - bukan penyebab.)
 *      - reset pengaman BTS7960 (enable di-LOW-kan sesaat tiap motor mulai berputar);
 *      - LIHAT_SEBELUM_BELOK: berhenti & baca sensor saat DM556 mati (bersih) sebelum memilih arah
 *        belok (status 'S' di log). Arah kemudi otomatis diatur lewat "Balik arah kemudi".
 *  v13: perbaikan U-turn (diuji di simulasi MATLAB dengan lintasan jalur.dxf & robot.dxf):
 *       - keadaan baru CARI_CELAH ('C'): bila saat LIHAT tak ada sisi terbuka, robot maju pelan
 *         sampai sensor samping melihat celah, lalu MUNDUR LURUS (g_waktuMundurCari) dan belok penuh;
 *       - deteksi macet saat manuver: depan & kedua samping tidak berubah 1,5 s (berapa pun jaraknya);
 *       - belok selesai bila depan lapang DAN salah satu samping melihat dinding; kemudi langsung lurus;
 *       - sudut belok bawaan 45 derajat;
 *       - rentang keluaran fuzzy Kd diperlebar ke 0-300 (Kp, Ki tetap 0-20; 9 label & rule dosen tetap);
 *       - kelima sensor untuk navigasi: sensor kiri/kanan (sudut moncong) menjadi pengaman sudut
 *         depan (< 15 cm -> dianggap macet). Pompa menyala selama jalan lurus, mati saat belok.
 *  v12: fuzzy Mamdani diganti sesuai parameter dosen: input e & delta e 7 segitiga
 *       (NB, NM, NS, Z, PS, PM, PB) pada [-6, 6]; output Kp, Ki, Kd 9 segitiga
 *       (ZE, VS, S, MS, M, MB, B, VB, EVB) pada [0, 20]; rule base 7x7 (baris = error,
 *       kolom = delta error). Inferensi min-max & defuzzifikasi COA titik pusat tetap.
 *       Nilai awal Kp/Ki/Kd = hasil fuzzy di e = 0, de = 0 (0 / 20 / 0). Lainnya identik v10.
 *  v11: tampilan web dirapikan: Mode Operasi paling atas, JOYSTICK analog (maju/mundur/belok
 *       di MANUAL; tambah daya sementara di OTOMATIS untuk tanjakan/batu), pemetaan sensor
 *       paling bawah, keterangan-keterangan kecil dihapus.
 *  v10: mode REMOTE KONTROL dihapus (kartu remote, perintah "rc", pengaman sinyal & soft-start).
 *       Mode MANUAL tetap lewat slider motor & steering. Selebihnya identik dengan v9.
 *
 *  MODE OPERASI (dipilih dari tombol di halaman web):
 *    - MANUAL   : kontrol motor & steering langsung via slider/tombol,
 *                 identik dengan robot_diagnostic.ino kamu.
 *    - OTOMATIS : wall-following. Depan dicek dengan ambang jarak;
 *                 kiri-kanan dikendalikan Fuzzy-PID adaptif yang
 *                 hasilnya diubah jadi TARGET SUDUT stepper, lalu
 *                 stepper bergerak halus ke sana lewat steerLoop()
 *                 yang sudah ada. Saat dekat tanaman, pompa ON dan
 *                 robot berhenti sebentar untuk menyiram.
 *
 *  Pompa & Buzzer BELUM ada di robot_diagnostic.ino, jadi ditambahkan
 *  di pin bebas: PUMP_PIN = GPIO4, BUZZER_PIN = GPIO5 (sesuaikan jika
 *  wiring aslinya berbeda).
 *
 *  REVISI: perhitungan fuzzy diganti menjadi MAMDANI sesuai Bab 3 skripsi
 *  (5 trapesium, 25 aturan, min-max, COA) + PID diskrit sesuai Bab 4.
 *  Yang lain (pin, motor, stepper, web, sensor, pompa) TIDAK diubah.
 *  TAMBAHAN (pengujian Bab 4):
 *   - NAVIGASI pakai sensor SAMPING kiri/kanan (jaga robot di tengah lorong, PID-Fuzzy);
 *     sensor KIRI/KANAN dipakai mendeteksi tanaman (pemicu pompa).
 *   - Depan < 150 cm -> MELAMBAT. Depan < 100 cm -> belok ke sisi yang TERBUKA (> 1 m / tak
 *     terbaca). Depan < 15 cm -> berhenti, MUNDUR (kemudi dibalik), lalu maju lagi sambil
 *     belok (belok 3 titik) sampai depan lapang. Semua jarak bisa diubah dari web.
 *   - Semua ambang, sudut belok, skala kemudi & pemetaan sensor bisa diatur dari web.
 *   - metode kendali PID-Fuzzy / PID tetap bisa dipilih dari web (tanpa upload ulang).
 *   - log data kendali disimpan di RAM dan bisa diunduh lewat HP
 *  (WiFi RobotFinal -> 192.168.4.1 -> tombol "Unduh CSV").
 *
 *  Board   : ESP32 (Arduino-ESP32 core 2.x / 3.x)
 *  Mode WiFi: Access Point - SSID "RobotFinal", Password "robot1234",
 *             IP 192.168.4.1
 *  Library : ESPAsyncWebServer (fork ESP32Async) + AsyncTCP (fork ESP32Async)
 *            + ArduinoJson v7.x
 * ============================================================================
 */

// PENTING: ArduinoJson HARUS di-include SEBELUM ESPAsyncWebServer.
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_system.h>   // esp_reset_reason(): penyebab restart terakhir (diagnosa)
#include <Preferences.h>   // simpan setelan web ke flash (tidak hilang saat robot dimatikan)

// ---------------------------------------------------------------------------
//  LAPISAN KOMPATIBILITAS (ArduinoJson v6/v7, ESP32 core 2.x/3.x)
// ---------------------------------------------------------------------------
#if ARDUINOJSON_VERSION_MAJOR < 7
  #define JSON_DOC(name) DynamicJsonDocument name(512)
#else
  #define JSON_DOC(name) JsonDocument name
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ===========================================================================
//  KONFIGURASI ACCESS POINT
// ===========================================================================
static const char *AP_SSID = "RobotFinal";
static const char *AP_PASS = "robot1234";
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

// ===========================================================================
//  PIN - PERSIS SEPERTI robot_diagnostic.ino (sudah terbukti di hardware)
// ===========================================================================
#define PIN_TRIG        13
// TRIG terpisah untuk JSN depan (anti-crosstalk). -1 = ikut PIN_TRIG bersama (bawaan).
// Jika kabel TRIG JSN depan dipindah ke pin sendiri (mis. GPIO15), isi nomor pinnya di sini.
#define PIN_TRIG_DEPAN  -1

#define PIN_ECHO_JSN_F  14   // JSN depan
#define PIN_ECHO_HC_SR  27   // HC samping kanan
#define PIN_ECHO_HC_R   26   // HC kanan
#define PIN_ECHO_HC_SL  25   // HC samping kiri
#define PIN_ECHO_HC_L   33   // HC kiri
#define PIN_ECHO_JSN_WL 32   // JSN water level

#define PIN_RPWM        16
#define PIN_LPWM        17
#define PIN_R_EN        18
#define PIN_L_EN        21

#define PIN_STEP        23   // INVERTING (lewat BC547)
#define PIN_DIR         19   // INVERTING (lewat BC547)
#define PIN_EN          22   // INVERTING (lewat BC547)

// --- PIN BARU: pompa & buzzer (belum ada di diagnostic, sesuaikan jika perlu) ---
#define PUMP_PIN         4
#define BUZZER_PIN       5

// ===========================================================================
//  SENSOR - definisi (identik dengan robot_diagnostic.ino)
// ===========================================================================
#define N_SENSOR 6

static const uint8_t ECHO_PIN[N_SENSOR] = {
  PIN_ECHO_JSN_F,   // 0 - depan
  PIN_ECHO_HC_SR,   // 1 - samping kanan
  PIN_ECHO_HC_R,    // 2 - kanan
  PIN_ECHO_HC_SL,   // 3 - samping kiri
  PIN_ECHO_HC_L,    // 4 - kiri
  PIN_ECHO_JSN_WL   // 5 - level air
};

static const char *SENSOR_KEY[N_SENSOR] = {
  "jsn_wl", "hc_sl", "hc_r", "hc_l", "hc_sr", "jsn_f"
};

// indeks langsung ke ECHO_PIN[] / g_dist[] untuk logika otomatis
// Indeks sensor yang dipakai logika otomatis (0..5 = urutan ECHO_PIN di atas).
// Bisa dipetakan ulang dari halaman web (kartu "Pemetaan Sensor") tanpa upload.
static int IDX_FRONT  = 5;   // GPIO32  JSN depan       : rintangan & keputusan belok (hasil uji: depan = GPIO32)
static int IDX_SRIGHT = 1;   // GPIO27  samping kanan   : NAVIGASI (jaga posisi di lorong)
static int IDX_RIGHT  = 2;   // GPIO26  kanan           : DETEKSI TANAMAN (pemicu pompa)
static int IDX_SLEFT  = 3;   // GPIO25  samping kiri    : NAVIGASI
static int IDX_LEFT   = 4;   // GPIO33  kiri            : DETEKSI TANAMAN
static int IDX_WATER  = 0;   // GPIO14  JSN level air   (hasil uji: level air = GPIO14)

static float g_dist[N_SENSOR] = { -1, -1, -1, -1, -1, -1 };

static SemaphoreHandle_t g_distMutex  = NULL;
static TaskHandle_t      g_sensorTask = NULL;

#define TRIG_SELFTEST   0
#define SENSOR_DEBUG    0   // 0 agar Serial hanya berisi log CSV kendali

#define SENSOR_TASK_CORE     0
#define SEQ_ECHO_TIMEOUT_US  25000UL
#define SEQ_INTER_MS         35UL
#define SOUND_CM_PER_US      0.01715f
#define DIST_MAX_CM          500.0f

// ===========================================================================
//  MOTOR PENGGERAK (BTS7960) - identik dengan robot_diagnostic.ino
// ===========================================================================
// 25 kHz (dulu 20 kHz): harmonik ke-2 PWM 20 kHz = 40 kHz = frekuensi kerja sensor ultrasonik,
// terbukti sensor depan membaca ~56 cm semu saat motor berputar. 25 kHz = batas atas BTS7960.
// Bila pembacaan semu tetap ada, kembalikan ke 20000 (gangguan berarti dari jalur listrik).
#define MOTOR_PWM_FREQ  20000   // dikembalikan: 25 kHz terbukti tidak berpengaruh
#define MOTOR_PWM_RES   8
#define MOTOR_CH_RPWM   0
#define MOTOR_CH_LPWM   1

static inline void pwmSetup(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
#else
  ledcSetup(ch, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(pin, ch);
#endif
}
static inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

static int  g_motorSpeed = 0;          // -255..255 (neg = mundur)
static bool g_motorEn    = false;
// ---- JOYSTICK WEB ----
// MANUAL  : y = maju/mundur (PWM = y% x g_joyPwm), x = belok (sudut = x% x g_joySudut).
//           Stik dilepas -> motor berhenti & kemudi lurus.
// OTOMATIS: y menambah daya sementara (tanjakan / batu). Saat maju, stik atas menaikkan PWM dari
//           PWM dasar menuju 255; saat mundur, stik bawah. Dilepas -> kembali ke PWM dasar.
//           x diabaikan (kemudi tetap dari PID-Fuzzy).
static int  g_joyPwm   = 200;          // PWM saat stik penuh (MANUAL), bisa diubah dari web
static int  g_joySudut = 30;           // sudut kemudi saat stik penuh ke samping (MANUAL)
static volatile int      g_joyX = 0, g_joyY = 0;   // -100..100
static volatile uint32_t g_joyLastMs = 0;
static volatile bool     g_joyBaru = false;
#define JOY_TIMEOUT_MS  600            // tak ada pesan stik > ini (WiFi putus) -> dianggap dilepas
static int  g_autoBase = 0;            // PWM keputusan mode OTOMATIS sebelum ditambah stik
// Jeda motor saat JSN depan mengukur (anti-derau). Uji: motor berputar membuat sensor depan
// membaca ~50-56 cm semu walau 5 V stabil. Selama pengukuran depan (3-25 ms per siklus sensor)
// driver BTS7960 dimatikan (R_EN/L_EN LOW -> motor meluncur bebas, tidak mengerem), lalu
// langsung dinyalakan lagi. 0 = matikan fitur ini.
// HASIL UJI: penyebab pembacaan semu ternyata DM556 (bukan motor), jadi fitur ini dimatikan.
#define JEDA_MOTOR_UKUR_DEPAN  0
#define JEDA_MOTOR_TENANG_MS   3       // tunggu derau mereda sebelum sensor depan dipicu
static volatile bool g_jedaMotor = false;
// Jeda STEPPER saat sensor mengukur. Uji (robot diangkat, motor diam): begitu Steering Enable
// dinyalakan, JSN depan langsung membaca ~50-56 cm semu -> derau berasal dari driver DM556
// (arus penahan kumparan stepper). Selama setiap sensor mengukur (maks. 25 ms) EN DM556
// dimatikan dan langkah stepper ditahan, lalu dinyalakan lagi. 0 = matikan fitur ini.
// HASIL UJI: mematikan-menyalakan DM556 berulang membuat stepper bergetar, kehilangan langkah,
// dan kemudi tidak sampai sudut penuh -> fitur dimatikan. Derau DM556 diatasi di perangkat keras.
#define JEDA_STEPPER_UKUR      0
#define JEDA_STEPPER_TENANG_MS 5       // tunggu derau DM556 mereda sebelum sensor dipicu
// Jeda hanya bila kemudi DIAM dan hampir lurus (|sudut| <= batas ini). Saat berbelok tajam
// stepper harus tetap menahan roda: bila dimatikan, roda didorong lantai kembali ke lurus
// (uji log 63/65: mode otomatis tidak bisa belok, padahal remote saat diam bisa).
#define JEDA_STEPPER_MAKS_DEG  10
static volatile bool g_jedaStepper = false;

// ===========================================================================
//  STEERING (DM556 stepper via BC547 inverting) - identik diagnostic
// ===========================================================================
static int  g_maxAngle    = 45;
static int  g_stepsPerDeg = 10;
static bool g_steerEn     = false;
static const char *resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWER-ON";
    case ESP_RST_BROWNOUT: return "BROWNOUT (tegangan drop)";
    case ESP_RST_PANIC:    return "PANIC (program crash)";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_EXT:      return "EXTERNAL (tombol EN)";
    default:               return "LAIN";
  }
}

static long g_curSteps    = 0;
static long g_tgtSteps    = 0;
static int  g_tgtAngle    = 0;

static uint32_t g_lastStepUs = 0;
#define STEP_INTERVAL_US  700

static inline void drvWrite(uint8_t gpio, uint8_t logic) {
  digitalWrite(gpio, logic ? LOW : HIGH);   // dibalik oleh BC547
}
static inline void drvStepPulse() {
  drvWrite(PIN_STEP, HIGH);
  delayMicroseconds(20);   // lewat BC547 + opto DM556, pulsa 6 us terlalu sempit
  drvWrite(PIN_STEP, LOW);
  delayMicroseconds(20);
}

// ===========================================================================
//  MODE OPERASI: MANUAL / OTOMATIS
// ===========================================================================
enum ModeOperasi { MODE_MANUAL, MODE_OTOMATIS };
static ModeOperasi g_mode = MODE_MANUAL;   // default MANUAL saat baru nyala

// Keadaan mode otomatis (mengikuti flowchart Gambar 3.4 Bab 3):
//   JELAJAH  : jarak depan > 50 cm  -> PID-Fuzzy aktif, PWM 160
//   MELAMBAT : 30 <= depan <= 50 cm -> PID-Fuzzy aktif, PWM 90
//   BERHENTI : depan < 30 cm        -> motor & pompa mati, tunggu sampai depan > 50 cm
enum RobotState { JELAJAH, MELAMBAT, BERHENTI, BELOK_KIRI, BELOK_KANAN, MUNDUR, LIHAT, CARI_CELAH };
// v13 CARI_CELAH (status 'C' di log): setelah LIHAT tidak ada sisi terbuka -> maju pelan sambil
// menjaga tengah lorong sampai sensor samping melihat celah, lalu mundur lurus & belok penuh.
#define CARI_MIN_CM  20                // CARI_CELAH: maju paling dekat sejauh ini dari dinding depan
static const int8_t URUTAN_BELOK[] = { -1, 1 };   // cadangan bila celah tak terlihat: -1 kiri, +1 kanan
#define N_URUTAN_BELOK (sizeof(URUTAN_BELOK) / sizeof(URUTAN_BELOK[0]))
static float   g_jarakSudut  = 15.0f;  // v13: cm, sensor kiri/kanan (sudut moncong) < ini -> hampir menabrak
static uint8_t g_urutanIdx   = 0;
static bool    g_mundurLurus = false;  // v13: mundur dengan kemudi lurus (setelah CARI_CELAH)
static int     g_waktuMundurCari = 300; // ms mundur lurus sebelum belok penuh (+-10-15 cm)
// LIHAT SEBELUM BELOK: derau DM556 membuat semua sensor membaca ~50-56 cm semu sehingga sisi
// yang lapang tak terlihat dan arah belok salah dipilih. Saat depan terhalang, robot BERHENTI
// (roda diam & tidak bergerak), DM556 dimatikan sesaat (aman: robot diam, tak ada gaya lantai),
// sensor dibaca bersih beberapa siklus, lalu arah belok dipilih dan DM556 dinyalakan lagi.
// Bila depan ternyata lapang (pembacaan semu), robot lanjut jalan tanpa belok. 0 = matikan.
#define LIHAT_SEBELUM_BELOK  1
#define LIHAT_SIKLUS         3        // jumlah siklus sensor bersih yang dirata-rata (~0,2 s/siklus)
// Setelah melihat: bila dinding depan < (Jarak mundur + ini) cm, robot MUNDUR dulu sambil kemudi
// dibalik (belok 3 titik), baru maju sambil belok. Log 80: belok langsung dari jarak +-38 cm
// membuat robot menabrak dinding saat berbelok; setelah mundur dulu belokan berhasil.
// 0 = hanya mundur bila depan < Jarak mundur (perilaku lama).
#define MUNDUR_DULU_CM       25
static RobotState g_state = JELAJAH;

// ===========================================================================
//  PARAMETER FUZZY MAMDANI + PID (mode OTOMATIS) - SESUAI BAB 3 SKRIPSI
//  Tabel 3.1 (e, de), Tabel 3.2 (Kp, Ki, Kd), Tabel 3.3-3.5 (rule base),
//  inferensi Mamdani (min-max), defuzzifikasi COA titik pusat.
// ===========================================================================
// Ambang jarak depan (bisa diubah dari web tanpa upload).
static float g_jarakAman    = 150.0f; // cm: depan < ini -> MELAMBAT
static float g_jarakBelok   = 100.0f; // cm: depan < ini -> belok ke sisi yang terbuka
static float g_jarakMundur  = 15.0f;  // cm: depan < ini -> berhenti, MUNDUR, lalu maju lagi
static int   g_waktuMundur  = 1500;   // ms: lama mundur tiap manuver
static int   g_pwmMundur    = 140;    // PWM saat mundur (>= 65 agar robot bergerak)
static float g_jarakBuka    = 100.0f; // cm: sisi dianggap TERBUKA bila > ini (atau tidak terbaca)
static float g_jarakPaksa   = 40.0f;  // cm: depan < ini & tak ada sisi terbuka -> tetap belok ke sisi yang lebih lapang
static float g_navMax       = 80.0f;  // cm: sensor samping dipakai navigasi bila < ini
static float g_jarakTanaman = 40.0f;  // cm: kiri/kanan < ini -> ada tanaman -> pompa ON
static int   g_sudutBelok   = 45;     // derajat kemudi saat berbelok (v13: 45, U-turn muat di celah 99 cm)
static int   g_waktuBelok   = 1500;   // ms: lama belok minimum sebelum kembali lurus
static float g_zonaMati     = 2.0f;   // cm: |e| < ini dianggap sudah di tengah -> kemudi lurus
#define BELOK_MAKS_MS  20000           // pengaman: belok (termasuk mundur) > 20 s -> berhenti
#define JEDA_MS        400             // jeda diam sebelum & sesudah mundur (kemudi berputar)
#define MUNDUR_MAKS    6               // pengaman: maksimum manuver mundur berturut-turut
static uint32_t g_turnStartMs = 0;
static int      g_turnDir     = 0;     // arah belok aktif: +1 kanan, -1 kiri, 0 lurus
static uint8_t  g_fase        = 0;     // fase mundur: 0 jeda, 1 mundur, 2 jeda
static uint32_t g_faseStart   = 0;
static uint8_t  g_mundurCount = 0;
static uint8_t  g_lihatN = 0;          // hitungan siklus dalam keadaan LIHAT
static float    g_lihatL = 0, g_lihatR = 0, g_lihatF = 999;   // jumlah s_kiri, s_kanan; depan min
static const float BATAS_AIR_HABIS   = 33.0f;  // cm: air hampir habis bila jarak sensor->air > 33 cm
                                                // (jeriken 38 cm, sisa air < 5 cm). Sesuaikan tangki.

static int   KECEPATAN_JELAJAH   = 110;  // PWM jalan biasa (bisa diubah dari web)
static int   KECEPATAN_MENDEKATI = 80;   // PWM saat pelan/belok (bisa diubah dari web)

// --- Setpoint (Persamaan 4 Bab 3: e = d_setpoint - d_aktual, d_aktual = sensor kiri)
//     SETPOINT_TENGAH 0 : setpoint tetap = SETPOINT_KIRI_CM (lorong 100 cm, robot 60 cm -> 20 cm)
//     SETPOINT_TENGAH 1 : setpoint = (kiri + kanan)/2 -> otomatis di tengah untuk lebar lorong berapa pun
static bool g_spTengah = true;           // true: setpoint = tengah lorong (samping kiri+kanan)/2
static const float SETPOINT_KIRI_CM = 20.0f;

// --- Metode kendali, bisa diganti dari halaman web TANPA upload ulang:
//     true  = hybrid PID-Fuzzy (Bab 3)
//     false = PID konvensional gain tetap (pembanding Subbab 4.5 & 4.6)
static bool  g_useFuzzy = true;
static float g_kpTetap  = 9.25f;   // gain PID tetap, bisa diubah dari web (hasil tuning manual)
static float g_kiTetap  = 9.25f;
static float g_kdTetap  = 9.25f;
static uint16_t g_run   = 0;       // nomor percobaan, naik setiap masuk mode OTOMATIS

static float Kp = 0.0f, Ki = 20.0f, Kd = 0.0f;   // nilai awal = hasil fuzzy (v12) saat e = 0, de = 0

static const float INTEGRAL_MAX = 0.5f;    // batas akumulasi integral (anti-windup)
static const float OUTPUT_MAX   = 100.0f;

// ---- PENYETELAN KEMUDI OTOMATIS (bisa diubah dari halaman web, tanpa upload) ----
// theta_target = trunc(sat(g_steerSign * g_steerScale * u ; +/- g_autoMaxAngle))
// lalu perubahan sudut per siklus dibatasi g_steerRate derajat agar belok bertahap.
static float g_steerScale    = 0.08f;  // derajat per satuan u (dulu 0,45 -> terlalu agresif)
static int   g_autoMaxAngle  = 10;     // batas sudut kemudi di mode otomatis (derajat)
static int   g_steerRate     = 4;      // perubahan sudut maksimum per siklus sensor (derajat)
// Arah kemudi. Dengan e = setpoint - d_kiri, e > 0 berarti robot TERLALU DEKAT
// tanaman KIRI, jadi robot harus belok KANAN. Kalau saat diuji robot malah
// belok mendekati tanaman kiri, centang "Balik arah kemudi" di web.
static float g_steerSign     = -1.0f;

// --- v12 (parameter dosen): 7 fungsi keanggotaan SEGITIGA error & delta error, semesta [-6, 6].
//     Disimpan sebagai trapesium (a, b, c, d) dengan b = c sehingga trapmf() lama tetap dipakai.
//     NB dan PB berupa bahu (a = b = c = -6 / b = c = d = 6) karena masukan disaturasi ke [-6, 6].
#define N_IN   7
#define N_OUT  9
#define FZ_MIN -6.0f
#define FZ_MAX  6.0f
enum { IN_NB, IN_NM, IN_NS, IN_Z, IN_PS, IN_PM, IN_PB };   // awalan IN_ agar tidak bentrok dengan makro ESP32
static const float MF_IN[N_IN][4] = {
  { -6, -6, -6, -4 },   // NB
  { -6, -4, -4, -2 },   // NM
  { -4, -2, -2,  0 },   // NS
  { -2,  0,  0,  2 },   // Z
  {  0,  2,  2,  4 },   // PS
  {  2,  4,  4,  6 },   // PM
  {  4,  6,  6,  6 }    // PB
};
// --- v12 (parameter dosen): 9 label keluaran Kp, Ki, Kd segitiga pada [0, 20], jarak 2,5.
//     Titik pusat COA = puncak segitiga.
enum { OUT_ZE, OUT_VS, OUT_S, OUT_MS, OUT_M, OUT_MB, OUT_B, OUT_VB, OUT_EVB };
static const float OUT_CENTER[N_OUT] = {
  0.0f,   // ZE  (0, 0, 2.5)
  2.5f,   // VS  (0, 2.5, 5)
  5.0f,   // S   (2.5, 5, 7.5)
  7.5f,   // MS  (5, 7.5, 10)
  10.0f,  // M   (7.5, 10, 12.5)
  12.5f,  // MB  (10, 12.5, 15)
  15.0f,  // B   (12.5, 15, 17.5)
  17.5f,  // VB  (15, 17.5, 20)
  20.0f   // EVB (17.5, 20, 20)
};
// --- v13: rentang keluaran Kd diperlebar ke [0, 300] (9 label sama, jarak 37,5), Kp & Ki tetap [0, 20].
//     Dosen mempersilakan rentang diperbaiki. Hasil simulasi (tuning_rentang.m): dengan Kd 0-20 suku D
//     terlalu lemah karena delta e per sampel hanya +-0,5 cm -> robot berayun terus di lorong
//     (settling 56 s). Kd 0-300: settling +-3 s, IAE turun +-60 %.
static const float OUT_CENTER_KD[N_OUT] = {
  0.0f, 37.5f, 75.0f, 112.5f, 150.0f, 187.5f, 225.0f, 262.5f, 300.0f   // ZE .. EVB
};
// --- v12 (parameter dosen): rule base 7x7. Baris = ERROR (NB..PB), kolom = DELTA ERROR (NB..PB)
static const uint8_t RULE_KP[N_IN][N_IN] = {
  { OUT_EVB, OUT_VB, OUT_B,  OUT_MB, OUT_B,  OUT_VB, OUT_EVB },   // NB
  { OUT_VB,  OUT_B,  OUT_MB, OUT_M,  OUT_MB, OUT_B,  OUT_VB  },   // NM
  { OUT_B,   OUT_MB, OUT_M,  OUT_MS, OUT_M,  OUT_MB, OUT_B   },   // NS
  { OUT_MB,  OUT_M,  OUT_MS, OUT_ZE, OUT_MS, OUT_M,  OUT_MB  },   // Z
  { OUT_B,   OUT_MB, OUT_M,  OUT_MS, OUT_M,  OUT_MB, OUT_B   },   // PS
  { OUT_VB,  OUT_B,  OUT_MB, OUT_M,  OUT_MB, OUT_B,  OUT_VB  },   // PM
  { OUT_EVB, OUT_VB, OUT_B,  OUT_MB, OUT_B,  OUT_VB, OUT_EVB }    // PB
};
static const uint8_t RULE_KI[N_IN][N_IN] = {
  { OUT_ZE, OUT_VS, OUT_S,  OUT_MS,  OUT_S,  OUT_VS, OUT_ZE },
  { OUT_VS, OUT_S,  OUT_MS, OUT_M,   OUT_MS, OUT_S,  OUT_VS },
  { OUT_S,  OUT_MS, OUT_M,  OUT_MB,  OUT_M,  OUT_MS, OUT_S  },
  { OUT_MS, OUT_M,  OUT_MB, OUT_EVB, OUT_MB, OUT_M,  OUT_MS },
  { OUT_S,  OUT_MS, OUT_M,  OUT_MB,  OUT_M,  OUT_MS, OUT_S  },
  { OUT_VS, OUT_S,  OUT_MS, OUT_M,   OUT_MS, OUT_S,  OUT_VS },
  { OUT_ZE, OUT_VS, OUT_S,  OUT_MS,  OUT_S,  OUT_VS, OUT_ZE }
};
static const uint8_t RULE_KD[N_IN][N_IN] = {
  { OUT_EVB, OUT_VB, OUT_B,  OUT_MB, OUT_B,  OUT_VB, OUT_EVB },
  { OUT_VB,  OUT_B,  OUT_MB, OUT_B,  OUT_MB, OUT_B,  OUT_VB  },
  { OUT_B,   OUT_MB, OUT_M,  OUT_MS, OUT_M,  OUT_MB, OUT_B   },
  { OUT_MB,  OUT_B,  OUT_MS, OUT_ZE, OUT_MS, OUT_B,  OUT_MB  },
  { OUT_B,   OUT_MB, OUT_M,  OUT_MS, OUT_M,  OUT_MB, OUT_B   },
  { OUT_VB,  OUT_B,  OUT_MB, OUT_B,  OUT_MB, OUT_B,  OUT_VB  },
  { OUT_EVB, OUT_VB, OUT_B,  OUT_MB, OUT_B,  OUT_VB, OUT_EVB }
};

static float g_error = 0.0f, g_errorSebelumnya = 0.0f, g_deltaError = 0.0f;
static float g_integral = 0.0f;
static float g_P = 0.0f, g_I = 0.0f, g_D = 0.0f, g_u = 0.0f;   // untuk log data

// siklus sensor: dinaikkan oleh sensorTaskFn setiap selesai membaca 6 sensor.
// Kendali otomatis hanya dihitung saat ada data baru -> dt = periode sensor (Ts)
static volatile uint32_t g_sensorCycle = 0;

#define LOG_CSV 1   // 1 = cetak data kendali ke Serial (CSV) saat mode OTOMATIS

// --- LOG DATA KE HP (tanpa kabel) ---
// Setiap baris data kendali juga disimpan di RAM ESP32 (buffer melingkar).
// Unduh dari HP: sambung ke WiFi "RobotFinal", buka 192.168.4.1, tekan "Unduh CSV".
#define ROBOT_LOG_MAX      330    // jumlah baris tersimpan (data terlama ditimpa)
#define ROBOT_LOG_LEN 180
static char g_logBuf[ROBOT_LOG_MAX][ROBOT_LOG_LEN];
static int  g_logHead  = 0;
static int  g_logCount = 0;
static SemaphoreHandle_t g_logMutex = NULL;
static const char *LOG_HEADER = "ms,run,metode,status,depan,s_kiri,s_kanan,kiri,kanan,setpoint,e,de,Kp,Ki,Kd,P,I,D,u,sudut,pwm,pompa,air_habis";

static bool g_pumpOn   = false;
static bool g_waterLow = false;

// ===========================================================================
//  WEB SERVER / WEBSOCKET
// ===========================================================================
AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

static uint32_t g_lastBroadcast = 0;
static uint32_t g_lastWsClean   = 0;
static bool     g_forceSend     = false;
#define BROADCAST_MS  100

// ===========================================================================
//  PROTOTIPE FUNGSI
//  (manual, karena string HTML besar di bawah membuat auto-prototype
//  Arduino IDE gagal mendeteksi fungsi setelahnya)
// ===========================================================================
void sensorTaskFn(void *arg);
static void motorApply();
static void steerRecalcTarget();
static void steerApplyEnable();
static int  steerReportAngle();
static void steerLoop();
static void pompaOn();
static void pompaOff();
static void cekLevelAir();
static float getDist(int idx);
static float trapmf(float x, const float p[4]);
static void fuzzyTuning(float e, float de, float &kp, float &ki, float &kd);
static float hitungPID(float e, float de, float dt);
static void switchToOtomatis();
static void logAdd(const char *line);
static void logClear();
static void switchToManual();
static void autoModeStep(float dt);
static void broadcastSensors();
static void handleCommand(const uint8_t *data, size_t len);
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len);

// ===========================================================================
//  UI HTML (PROGMEM)
// ===========================================================================
static const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="id"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Robot Penyiram - Final</title>
<style>
:root{
  --bg:#0e1116; --panel:#161b22; --panel2:#1c2330; --line:#2a3240;
  --tx:#e6edf3; --tx2:#9aa7b4; --acc:#3b82f6; --ok:#22c55e; --warn:#f59e0b;
  --danger:#ef4444;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif;padding:12px;max-width:920px;margin:0 auto}
h1{font-size:16px}
h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--tx2);margin-bottom:10px}
header{display:flex;align-items:center;justify-content:space-between;gap:10px;
  background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px 14px;margin-bottom:12px}
.badge{display:flex;align-items:center;gap:7px;font-size:12px;color:var(--tx2)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--danger);box-shadow:0 0 0 3px rgba(239,68,68,.15)}
.dot.on{background:var(--ok);box-shadow:0 0 0 3px rgba(34,197,94,.15)}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.sensors{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
.sensor{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.sensor .lbl{font-size:11px;color:var(--tx2);text-transform:uppercase;letter-spacing:.04em}
.sensor .val{font-size:22px;font-weight:600;margin-top:4px}
.sensor .val small{font-size:12px;color:var(--tx2);font-weight:400}
.sensor.warn{border-color:var(--warn);background:rgba(245,158,11,.10)}
.sensor.warn .val{color:var(--warn)}
.sensor.dead .val{color:var(--tx2)}
.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.row+.row{margin-top:12px}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:6px;border-radius:6px;background:var(--line);outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:20px;height:20px;border-radius:50%;background:var(--acc);cursor:pointer;border:2px solid #0e1116}
input[type=range]::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:var(--acc);cursor:pointer;border:2px solid #0e1116}
input[disabled],button[disabled]{opacity:.35;pointer-events:none}
.readout{min-width:74px;text-align:right;font-variant-numeric:tabular-nums;font-weight:600}
button{background:var(--panel2);color:var(--tx);border:1px solid var(--line);border-radius:9px;padding:8px 12px;font-size:13px;cursor:pointer}
button:hover{border-color:var(--acc)}
button:active{transform:translateY(1px)}
button.primary{background:var(--acc);border-color:var(--acc);color:#fff}
button.stop{background:var(--danger);border-color:var(--danger);color:#fff}
button.mode{flex:1}
a.btn{display:inline-block;background:var(--panel2);color:var(--tx);border:1px solid var(--line);border-radius:9px;padding:8px 12px;font-size:13px;text-decoration:none}
a.btn.primary{background:var(--acc);border-color:var(--acc);color:#fff}
button.mode.aktif{background:var(--acc);border-color:var(--acc);color:#fff}
.btns{display:flex;gap:8px;flex-wrap:wrap}
label.chk{display:flex;align-items:center;gap:7px;cursor:pointer;user-select:none;font-size:13px}
input[type=number]{background:var(--panel2);color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:7px 9px;width:90px;font-size:13px}
select{background:var(--panel2);color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:7px 9px;font-size:13px}
.telemetry{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;font-size:12px}
.telemetry div{background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:7px 9px}
.telemetry b{display:block;font-size:15px;font-weight:600;margin-top:2px}
.proc{font-size:12.5px;color:var(--tx2);background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.proc ol{margin-left:16px}
.proc li{margin:3px 0}
.muted{color:var(--tx2);font-size:12px}
.row .lbl{min-width:150px;color:var(--tx2);font-size:12px}
.lbl2{min-width:80px;color:var(--tx2);font-size:12px}
.joywrap{display:flex;flex-direction:column;align-items:center;gap:10px;margin-bottom:12px}
#joy{position:relative;width:230px;height:230px;border-radius:50%;background:var(--panel2);border:2px solid var(--line);touch-action:none;user-select:none;-webkit-user-select:none}
#joy::before{content:'';position:absolute;left:50%;top:10%;bottom:10%;border-left:1px dashed var(--line)}
#joy::after{content:'';position:absolute;top:50%;left:10%;right:10%;border-top:1px dashed var(--line)}
#knob{position:absolute;left:50%;top:50%;width:84px;height:84px;margin:-42px 0 0 -42px;border-radius:50%;background:var(--acc);box-shadow:0 4px 14px rgba(0,0,0,.5);z-index:1;pointer-events:none}
#joy.auto #knob{background:var(--warn)}
.joyinfo{display:flex;gap:16px;font-variant-numeric:tabular-nums;font-weight:600}
.boost{color:var(--warn)}
</style></head><body>

<header>
  <h1>🌱 Robot Penyiram Tanaman</h1>
  <div class="badge"><span id="dot" class="dot"></span><span id="wsTxt">Terputus</span></div>
</header>

<div class="card">
  <h2>Mode Operasi</h2>
  <div class="row btns">
    <button class="mode" id="btnManual" onclick="setMode('manual')">MANUAL</button>
    <button class="mode" id="btnOtomatis" onclick="setMode('otomatis')">OTOMATIS</button>
  </div>
</div>

<div class="card">
  <h2 id="joyTitle">Kendali Manual</h2>
  <div class="joywrap">
    <div id="joy"><div id="knob"></div></div>
    <div class="joyinfo">
      <span>PWM <span id="jPwm">0</span></span>
      <span>Sudut <span id="jSdt">0</span>&deg;</span>
      <span id="jBoost" class="boost" style="display:none">BOOST</span>
    </div>
  </div>
  <div class="row"><span class="lbl2">Kecepatan</span><input type="range" id="jyP" min="0" max="255" step="1"><span class="readout" id="jyPv">-</span></div>
  <div class="row"><span class="lbl2">Steering</span><input type="range" id="jyS" min="1" max="45" step="1"><span class="readout" id="jySv">-</span></div>
  <div class="row btns">
    <label class="chk"><input type="checkbox" id="mEn"> Motor Enable</label>
    <label class="chk"><input type="checkbox" id="sEn"> Steering Enable</label>
  </div>
</div>

<div class="card">
  <h2>Sensor Jarak</h2>
  <div class="sensors" id="sensors"></div>
  <div class="row" style="margin-top:10px">
    <span class="muted">Pompa: <span id="pumpTxt">OFF</span></span>
    <span class="muted" id="waterWarn" style="display:none;color:var(--warn)">⚠ AIR HAMPIR HABIS</span>
  </div>
</div>

<div class="card">
  <h2>Penyetelan Mode Otomatis</h2>
  <div class="row btns"><span class="lbl">PWM jalan biasa</span><input type="number" id="pJalan" step="1" min="0" max="255"></div>
  <div class="row btns"><span class="lbl">PWM pelan / belok</span><input type="number" id="pPelan" step="1" min="0" max="255"></div>
  <div class="row btns"><span class="lbl">Skala sudut (&deg;/u)</span><input type="number" id="sScale" step="0.01" min="0.01" max="2"></div>
  <div class="row btns"><span class="lbl">Batas sudut (&deg;)</span><input type="number" id="sMaxA" step="1" min="1" max="45"></div>
  <div class="row btns"><span class="lbl">Laju belok (&deg;/siklus)</span><input type="number" id="sRate" step="1" min="1" max="45"></div>
  <div class="row btns"><span class="lbl">Jarak mulai pelan (cm)</span><input type="number" id="jAman" step="1" min="20" max="400"></div>
  <div class="row btns"><span class="lbl">Jarak mundur (cm)</span><input type="number" id="jStop" step="1" min="5" max="395"></div>
  <div class="row btns"><span class="lbl">Lama mundur (ms)</span><input type="number" id="wMundur" step="100" min="200" max="5000"></div>
  <div class="row btns"><span class="lbl">PWM mundur</span><input type="number" id="pMundur" step="1" min="0" max="255"></div>
  <div class="row btns"><span class="lbl">Jarak mulai belok (cm)</span><input type="number" id="jBelok" step="1" min="10" max="400"></div>
  <div class="row btns"><span class="lbl">Sisi terbuka (cm)</span><input type="number" id="jBuka" step="1" min="30" max="400"></div>
  <div class="row btns"><span class="lbl">Belok paksa &lt; (cm)</span><input type="number" id="jPaksa" step="1" min="15" max="150"></div>
  <div class="row btns"><span class="lbl">Sudut belok (&deg;)</span><input type="number" id="sBelok" step="1" min="5" max="45"></div>
  <div class="row btns"><span class="lbl">Lama belok min. (ms)</span><input type="number" id="wBelok" step="1" min="200" max="7000"></div>
  <div class="row btns"><span class="lbl">Zona mati lurus (cm)</span><input type="number" id="zMati" step="0.1" min="0" max="10"></div>
  <div class="row btns"><span class="lbl">Jarak tanaman (cm)</span><input type="number" id="jTanam" step="1" min="5" max="200"></div>
  <div class="row"><label class="chk"><input type="checkbox" id="sInv"> Balik arah kemudi</label></div>
  <div class="row"><label class="chk"><input type="checkbox" id="sSpT"> Setpoint tengah lorong</label></div>
  <div class="row btns"><button id="sSet" class="primary">Simpan Penyetelan</button>
    <button id="sReset" class="stop">Kembalikan ke Bawaan</button></div>
</div>

<div class="card">
  <h2>Metode Kendali</h2>
  <div class="row btns">
    <button class="mode" id="btnFuzzy" onclick="setCtrl('fuzzy')">PID-FUZZY</button>
    <button class="mode" id="btnPid" onclick="setCtrl('pid')">PID TETAP</button>
  </div>
  <div class="row btns">
    <input type="number" id="gKp" step="0.01" placeholder="Kp">
    <input type="number" id="gKi" step="0.01" placeholder="Ki">
    <input type="number" id="gKd" step="0.01" placeholder="Kd">
    <button id="gSet">Simpan Gain</button>
  </div>
</div>

<div class="card">
  <h2>Kalibrasi Kemudi</h2>
  <div class="row btns">
    <input type="number" id="calMax" min="1" max="180" step="1" placeholder="max&deg;">
    <button id="setMax">Set Max Angle</button>
  </div>
  <div class="row btns">
    <input type="number" id="calSpd" min="1" max="1000" step="1" placeholder="steps/&deg;">
    <button id="setSpd">Set Steps / Derajat</button>
  </div>
  <div class="row btns">
    <button id="zero" class="primary">Zero Position</button>
  </div>
</div>

<div class="card">
  <h2>Data Log</h2>
  <div class="row btns">
    <span class="muted"><b id="logN">0</b> baris</span>
    <a class="btn primary" href="/log.csv">Unduh CSV</a>
    <a class="btn" href="/log.txt" target="_blank">Lihat Log</a>
    <button id="logClr" class="stop">Hapus Log</button>
  </div>
</div>

<div class="card">
  <h2>Telemetry</h2>
  <div class="telemetry">
    <div>mode <b id="tMode">manual</b></div>
    <div>ESP32 menyala <b id="tUp">-</b></div>
    <div>restart terakhir <b id="tRst">-</b></div>
    <div>motor_speed <b id="tSpeed">0</b></div>
    <div>steer_angle <b id="tAngle">0&deg;</b></div>
    <div>max_angle <b id="tMax">45</b></div>
    <div>steps_per_deg <b id="tSpd">10</b></div>
    <div>motor_en <b id="tMEn">false</b></div>
    <div>steer_en <b id="tSEn">false</b></div>
    <div>percobaan (run) <b id="tRun">0</b></div>
    <div>status <b id="tStat">-</b></div>
    <div>metode <b id="tCtrl">fuzzy</b></div>
    <div>error e (cm) <b id="tE">0</b></div>
    <div>Kp / Ki / Kd <b id="tK">-</b></div>
  </div>
</div>

<div class="card">
  <h2>Pemetaan Sensor</h2>
  <p class="muted" style="margin-bottom:8px">depan <b id="pDepan">-</b> &middot; s.kiri <b id="pSKiri">-</b> &middot; s.kanan <b id="pSKanan">-</b> &middot; kiri <b id="pKiri">-</b> &middot; kanan <b id="pKanan">-</b> &middot; air <b id="pAir">-</b> cm</p>
  <div class="row btns"><span class="lbl">Depan</span><select id="mF"></select></div>
  <div class="row btns"><span class="lbl">Samping kiri</span><select id="mSL"></select></div>
  <div class="row btns"><span class="lbl">Samping kanan</span><select id="mSR"></select></div>
  <div class="row btns"><span class="lbl">Kiri (tanaman)</span><select id="mL"></select></div>
  <div class="row btns"><span class="lbl">Kanan (tanaman)</span><select id="mR"></select></div>
  <div class="row btns"><span class="lbl">Level air</span><select id="mW"></select></div>
  <div class="row btns"><button id="mSet" class="primary">Simpan Pemetaan</button></div>
</div>

<script>
const SENS = [
  ["jsn_f","JSN Depan"],
  ["hc_sr","HC Samping Kanan"],
  ["hc_r","HC Kanan"],
  ["hc_sl","HC Samping Kiri"],
  ["hc_l","HC Kiri"],
  ["jsn_wl","JSN Water Level"]
];
const $ = id => document.getElementById(id);
const grid = $('sensors');
SENS.forEach(([k,label])=>{
  const d = document.createElement('div');
  d.className='sensor'; d.id='s_'+k;
  d.innerHTML = '<div class="lbl">'+label+'</div><div class="val">--<small> cm</small></div>';
  grid.appendChild(d);
});
const GPIO = [14,27,26,25,33,32];
['mF','mSL','mSR','mL','mR','mW'].forEach(id=>{
  $(id).innerHTML = GPIO.map((g,i)=>'<option value="'+i+'">'+i+' &middot; GPIO'+g+'</option>').join('');
});

let ws, curMax = 45, curMode = '';

function setStatus(ok){
  $('dot').classList.toggle('on', ok);
  $('wsTxt').textContent = ok ? 'Terhubung' : 'Terputus';
}
function send(obj){
  if(ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}
function connect(){
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen  = ()=>{ setStatus(true); send({cmd:'read_sensors'}); };
  ws.onclose = ()=>{ setStatus(false); setTimeout(connect, 1000); };
  ws.onerror = ()=>{ ws.close(); };
  ws.onmessage = ev=>{
    let m; try{ m = JSON.parse(ev.data); }catch(e){ return; }
    if(m.type === 'sensor') applySensor(m);
  };
}

function setMode(m){ send({cmd:'set_mode', value:m}); }
function setCtrl(c){ send({cmd:'set_ctrl', value:c}); }

function applyModeUI(mode){
  curMode = mode;
  const manual = (mode === 'manual');
  $('btnManual').className = 'mode' + (manual ? ' aktif' : '');
  $('btnOtomatis').className = 'mode' + (!manual ? ' aktif' : '');
  $('joy').classList.toggle('auto', !manual);
  $('joyTitle').textContent = manual ? 'Kendali Manual' : 'Tambah Daya (Otomatis)';
  $('tMode').textContent = mode;
}

const pct = v => Math.round(v / 255 * 100) + '%';
const f1 = v => (v >= 999 || v < 0) ? '--' : (+v).toFixed(1);

function applySensor(m){
  SENS.forEach(([k])=>{
    const el = $('s_'+k);
    const v  = m[k];
    const val = el.querySelector('.val');
    el.classList.remove('warn','dead');
    if(v === undefined) return;
    if(v < 0){
      val.innerHTML = '&mdash;<small> cm</small>';
      el.classList.add('dead');
    }else{
      val.innerHTML = v.toFixed(1) + '<small> cm</small>';
      if(v < 20) el.classList.add('warn');
    }
  });

  if(typeof m.max_angle === 'number' && m.max_angle !== curMax){
    curMax = m.max_angle;
    $('jyS').max = curMax;
  }
  if(m.mode && m.mode !== curMode) applyModeUI(m.mode);

  $('pumpTxt').textContent = m.pump ? 'ON' : 'OFF';
  if(typeof m.log_n === 'number') $('logN').textContent = m.log_n;
  $('waterWarn').style.display = m.water_low ? 'inline' : 'none';

  $('jPwm').textContent = m.motor_speed;
  $('jSdt').textContent = m.steer_angle;
  const ubah = (m.mode === 'otomatis' && m.motor_speed !== m.auto_base);
  $('jBoost').style.display = ubah ? 'inline' : 'none';
  $('jBoost').textContent = (m.motor_speed === 0) ? 'REM' : 'BOOST';
  if(typeof m.joy_pwm === 'number' && document.activeElement !== $('jyP')){
    $('jyP').value = m.joy_pwm; $('jyPv').textContent = pct(m.joy_pwm);
  }
  if(typeof m.joy_sudut === 'number' && document.activeElement !== $('jyS')){
    $('jyS').value = m.joy_sudut; $('jySv').innerHTML = m.joy_sudut + '&deg;';
  }

  $('tSpeed').textContent = m.motor_speed;
  if(m.up !== undefined){
    $('tUp').textContent = m.up + ' s';
    $('tRst').textContent = m.rst;
    $('tRst').style.color = /BROWNOUT|PANIC|WATCHDOG/.test(m.rst) ? 'var(--danger)' : '';
    if(window._lastUp !== undefined && m.up + 2 < window._lastUp) window._nRestart = (window._nRestart||0) + 1;
    window._lastUp = m.up;
    if(window._nRestart) $('tRst').textContent = m.rst + ' (' + window._nRestart + 'x restart)';
  }
  $('tAngle').innerHTML   = m.steer_angle + '&deg;';
  $('tMax').textContent   = m.max_angle;
  $('tSpd').textContent   = m.steps_per_deg;
  $('tMEn').textContent   = m.motor_en;
  $('tSEn').textContent   = m.steer_en;
  if(m.run !== undefined){
    $('tRun').textContent  = m.run;
    $('tStat').textContent = m.status;
    $('tCtrl').textContent = m.ctrl;
    $('tE').textContent    = (+m.e).toFixed(2);
    $('tK').textContent    = (+m.kp).toFixed(2)+' / '+(+m.ki).toFixed(2)+' / '+(+m.kd).toFixed(2);
    $('btnFuzzy').className = 'mode' + (m.ctrl === 'fuzzy' ? ' aktif' : '');
    $('btnPid').className   = 'mode' + (m.ctrl === 'pid'   ? ' aktif' : '');
    if(!$('gKp').value) $('gKp').placeholder = 'Kp ' + m.kp_t;
    if(!$('gKi').value) $('gKi').placeholder = 'Ki ' + m.ki_t;
    if(!$('gKd').value) $('gKd').placeholder = 'Kd ' + m.kd_t;
    [['sScale','s_scale'],['sMaxA','s_max'],['sRate','s_rate'],['jAman','j_aman'],['jStop','j_stop'],
     ['pJalan','p_jalan'],['pPelan','p_pelan'],['wMundur','w_mundur'],['pMundur','p_mundur'],['jBelok','j_belok'],
     ['jBuka','j_buka'],['sBelok','s_belok'],['wBelok','w_belok'],['zMati','z_mati'],['jTanam','j_tanam'],['jPaksa','j_paksa']]
      .forEach(([id,k])=>{ if(document.activeElement !== $(id)) $(id).value = m[k]; });
    $('pDepan').textContent = f1(m.depan);
    $('pKiri').textContent  = f1(m.kiri);
    $('pKanan').textContent = f1(m.kanan);
    $('pSKiri').textContent = f1(m.s_kiri);
    $('pSKanan').textContent= f1(m.s_kanan);
    $('pAir').textContent   = f1(m.air);
    if(!mDirty){ $('mF').value = m.i_f; $('mL').value = m.i_l; $('mR').value = m.i_r; $('mW').value = m.i_w; $('mSL').value = m.i_sl; $('mSR').value = m.i_sr; }
    if(!sDirty) $('sSpT').checked = !!m.sp_t;
    if(!sDirty) $('sInv').checked = !!m.s_inv;
  }
  $('mEn').checked = !!m.motor_en;
  $('sEn').checked = !!m.steer_en;
  if(!$('calMax').value) $('calMax').placeholder = m.max_angle + '°';
  if(!$('calSpd').value) $('calSpd').placeholder = m.steps_per_deg + ' steps/°';
}

/* ---- Joystick analog ----
   x, y = -100..100 (y positif = maju). Dikirim tiap 100 ms selama ditahan (sekaligus
   sinyal hidup: bila ESP32 tidak menerima > 0,6 s, stik dianggap dilepas). */
const joy = $('joy'), knob = $('knob');
let jx = 0, jy = 0, jAct = false, jTimer = null;
function joyPos(e){
  const r = joy.getBoundingClientRect();
  const R = r.width / 2, lim = R - knob.offsetWidth / 2;
  let dx = e.clientX - (r.left + R), dy = e.clientY - (r.top + R);
  const d = Math.hypot(dx, dy);
  if(d > lim){ dx *= lim / d; dy *= lim / d; }
  knob.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
  let x = dx / lim, y = -dy / lim;
  if(Math.abs(x) < 0.08) x = 0;
  if(Math.abs(y) < 0.08) y = 0;
  jx = Math.round(x * 100); jy = Math.round(y * 100);
}
function joySend(){ send({cmd:'joy', x:jx, y:jy}); }
joy.addEventListener('pointerdown', e=>{
  e.preventDefault();
  joy.setPointerCapture(e.pointerId);
  jAct = true; joyPos(e); joySend();
  clearInterval(jTimer); jTimer = setInterval(joySend, 100);
});
joy.addEventListener('pointermove', e=>{ if(jAct) joyPos(e); });
function joyUp(){
  if(!jAct) return;
  jAct = false; clearInterval(jTimer);
  jx = jy = 0; knob.style.transform = '';
  joySend();
}
joy.addEventListener('pointerup', joyUp);
joy.addEventListener('pointercancel', joyUp);
window.addEventListener('blur', joyUp);

function joyCfg(){ send({cmd:'joy_cfg', pwm:+$('jyP').value, sudut:+$('jyS').value}); }
$('jyP').addEventListener('input', e=>{ $('jyPv').textContent = pct(+e.target.value); });
$('jyS').addEventListener('input', e=>{ $('jySv').innerHTML = e.target.value + '&deg;'; });
$('jyP').addEventListener('change', joyCfg);
$('jyS').addEventListener('change', joyCfg);

$('mEn').addEventListener('change', e=> send({cmd:'motor_enable', value:e.target.checked}));
$('sEn').addEventListener('change', e=> send({cmd:'steer_enable', value:e.target.checked}));

/* ---- Kalibrasi ---- */
$('setMax').addEventListener('click', ()=>{
  const v = parseInt($('calMax').value, 10);
  if(v > 0) send({cmd:'set_max_angle', value:v});
});
$('setSpd').addEventListener('click', ()=>{
  const v = parseInt($('calSpd').value, 10);
  if(v > 0) send({cmd:'set_steps_per_deg', value:v});
});
$('zero').addEventListener('click', ()=> send({cmd:'zero_position'}));

let sDirty = false, mDirty = false;
$('sReset').addEventListener('click', ()=>{
  if (!$('sReset').dataset.yakin) {           // tekan dua kali untuk konfirmasi
    $('sReset').dataset.yakin = '1';
    $('sReset').textContent = 'Tekan lagi untuk yakin';
    setTimeout(()=>{ delete $('sReset').dataset.yakin; $('sReset').textContent = 'Kembalikan ke Bawaan'; }, 3000);
    return;
  }
  send({cmd:'reset_setelan'});
});
$('sSpT').addEventListener('change', ()=>{ sDirty = true; });
$('sInv').addEventListener('change', ()=>{ sDirty = true; });
['mF','mL','mR','mW','mSL','mSR'].forEach(id=> $(id).addEventListener('change', ()=>{ mDirty = true; }));
$('mSet').addEventListener('click', ()=>{
  send({cmd:'set_map', f:+$('mF').value, l:+$('mL').value, r:+$('mR').value, w:+$('mW').value, sl:+$('mSL').value, sr:+$('mSR').value});
  mDirty = false;
});
$('sSet').addEventListener('click', ()=>{
  send({cmd:'set_steer',
        scale: parseFloat($('sScale').value),
        maxa:  parseInt($('sMaxA').value, 10),
        rate:  parseInt($('sRate').value, 10),
        inv:   $('sInv').checked,
        jaman: parseFloat($('jAman').value),
        jstop: parseFloat($('jStop').value),
        sptengah: $('sSpT').checked,
        pjalan: parseInt($('pJalan').value, 10),
        ppelan: parseInt($('pPelan').value, 10),
        wmundur: parseInt($('wMundur').value, 10),
        pmundur: parseInt($('pMundur').value, 10),
        jbelok: parseFloat($('jBelok').value),
        jbuka:  parseFloat($('jBuka').value),
        jpaksa: parseFloat($('jPaksa').value),
        sbelok: parseInt($('sBelok').value, 10),
        wbelok: parseInt($('wBelok').value, 10),
        zmati:  parseFloat($('zMati').value),
        jtanam: parseFloat($('jTanam').value)});
  sDirty = false;
});
$('gSet').addEventListener('click', ()=>{
  const o = {cmd:'set_gain'};
  ['Kp','Ki','Kd'].forEach(k=>{ const v = parseFloat($('g'+k).value); if(!isNaN(v)) o[k.toLowerCase()] = v; });
  send(o);
});
$('logClr').addEventListener('click', ()=>{
  fetch('/log_clear').then(()=>{ $('logN').textContent = '0'; });
});

applyModeUI('manual');
connect();
</script>
</body></html>
)HTMLPAGE";

// ===========================================================================
//  SENSOR - task FreeRTOS (sekuensial, anti-crosstalk) - identik diagnostic
// ===========================================================================
#if SENSOR_DEBUG
static void sensorDebugPrint() {
  Serial.print("[SENS] ");
  for (int i = 0; i < N_SENSOR; i++) {
    Serial.print(SENSOR_KEY[i]); Serial.print('=');
    if (g_dist[i] < 0) Serial.print("--   ");
    else { Serial.print(g_dist[i], 1); Serial.print("cm "); }
  }
  Serial.println();
}
#endif

void sensorTaskFn(void *arg) {
  (void)arg;
  for (;;) {
    static uint8_t nAir = 0;
    nAir = (nAir + 1) % 8;
    for (int i = 0; i < N_SENSOR; i++) {
      // sensor level air berubah sangat lambat -> cukup dibaca tiap 8 siklus,
      // sehingga siklus sensor navigasi lebih cepat (kemudi lebih responsif)
      if (i == IDX_WATER && nAir != 0) continue;
      bool jedaSt = false;
#if JEDA_STEPPER_UKUR
      long batasJeda = (long)JEDA_STEPPER_MAKS_DEG * g_stepsPerDeg;
      if (i != IDX_WATER && g_steerEn &&                // sensor tangki tidak terganggu
          g_curSteps == g_tgtSteps && labs(g_curSteps) <= batasJeda) {
        jedaSt = true;
        g_jedaStepper = true;                      // steerLoop berhenti mengirim langkah
        steerApplyEnable();                        // EN DM556 mati -> kumparan tanpa arus
        vTaskDelay(pdMS_TO_TICKS(JEDA_STEPPER_TENANG_MS));
      }
#endif
      bool jeda = false;
#if JEDA_MOTOR_UKUR_DEPAN
      if (i == IDX_FRONT && g_motorEn && g_motorSpeed != 0) {
        jeda = true;
        g_jedaMotor = true;
        motorApply();                              // driver mati -> motor meluncur
        vTaskDelay(pdMS_TO_TICKS(JEDA_MOTOR_TENANG_MS));
      }
#endif
      int trig = PIN_TRIG;
#if PIN_TRIG_DEPAN >= 0
      if (i == IDX_FRONT) trig = PIN_TRIG_DEPAN;   // JSN depan dipicu sendirian
#endif
      digitalWrite(trig, LOW);
      delayMicroseconds(4);
      digitalWrite(trig, HIGH);
      delayMicroseconds(12);
      digitalWrite(trig, LOW);

      unsigned long dur = pulseIn(ECHO_PIN[i], HIGH, SEQ_ECHO_TIMEOUT_US);
      if (jeda) {
        g_jedaMotor = false;
        motorApply();                              // motor jalan lagi
      }
      if (jedaSt) {
        g_jedaStepper = false;
        steerApplyEnable();                        // stepper aktif lagi
      }
      float d = (dur > 0) ? (float)dur * SOUND_CM_PER_US : -1.0f;
      if (d > DIST_MAX_CM || d <= 1.0f) d = -1.0f;

      xSemaphoreTake(g_distMutex, portMAX_DELAY);
      g_dist[i] = d;
      xSemaphoreGive(g_distMutex);

      vTaskDelay(pdMS_TO_TICKS(SEQ_INTER_MS));
    }
    g_sensorCycle++;          // satu siklus 6 sensor selesai -> data baru untuk kendali
#if SENSOR_DEBUG
    sensorDebugPrint();
#endif
  }
}

// baca 1 sensor secara thread-safe; -1 (invalid) dipetakan ke 999 supaya
// logika fuzzy/threshold otomatis tidak salah menganggap "dekat"
static float getDist(int idx) {
  float v;
  xSemaphoreTake(g_distMutex, portMAX_DELAY);
  v = g_dist[idx];
  xSemaphoreGive(g_distMutex);
  return (v < 0) ? 999.0f : v;
}

// ===========================================================================
//  MOTOR - terapkan state ke hardware (identik diagnostic)
// ===========================================================================
static void motorApply() {
  if (g_jedaMotor) {                   // sensor depan sedang mengukur: driver mati sesaat
    digitalWrite(PIN_R_EN, LOW);
    digitalWrite(PIN_L_EN, LOW);
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);
    return;
  }
  // Reset pengaman BTS7960: bila arus berlebih / panas (motor tertahan saat belok), driver
  // mengunci keluarannya sampai R_EN/L_EN di-LOW-kan. Uji: robot macet tak bisa maju/mundur,
  // pulih setelah Motor Enable dimatikan-dinyalakan. Maka setiap motor MULAI berputar (dari 0
  // atau ganti arah) enable di-LOW-kan 2 ms dulu agar kunci pengaman terlepas.
  static int lastSpeed = 0;
  int sekarang = g_motorEn ? g_motorSpeed : 0;
  if (sekarang != 0 && (lastSpeed == 0 || (lastSpeed > 0) != (sekarang > 0))) {
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);
    digitalWrite(PIN_R_EN, LOW);
    digitalWrite(PIN_L_EN, LOW);
    delay(2);
  }
  lastSpeed = sekarang;

  digitalWrite(PIN_R_EN, g_motorEn ? HIGH : LOW);
  digitalWrite(PIN_L_EN, g_motorEn ? HIGH : LOW);

  if (!g_motorEn || g_motorSpeed == 0) {
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);
    return;
  }
  if (g_motorSpeed > 0) {
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, g_motorSpeed);
  } else {
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, -g_motorSpeed);
  }
}

// ===========================================================================
//  STEERING - identik diagnostic (dipakai bersama MANUAL & OTOMATIS)
// ===========================================================================
static void steerRecalcTarget() {
  g_tgtAngle = constrain(g_tgtAngle, -g_maxAngle, g_maxAngle);
  g_tgtSteps = (long)g_tgtAngle * (long)g_stepsPerDeg;
}
// Polaritas EN DM556. Uji putar poros: dengan nilai 0, Enable OFF -> poros terkunci, Enable ON ->
// poros bebas (terbalik), maka dipakai 1. Benar bila: Enable OFF = poros bebas, ON = terkunci.
#define DM556_EN_DIBALIK  1
// Arah putar stepper. Uji: stepper berputar terbalik (CW <-> CCW), maka dipakai 1.
// Benar bila: stik / sudut ke KANAN -> roda kemudi berbelok ke KANAN.
#define DM556_DIR_DIBALIK 1
static void steerApplyEnable() {
  bool aktif = g_steerEn && !g_jedaStepper;
  if (DM556_EN_DIBALIK) aktif = !aktif;
  drvWrite(PIN_EN, aktif ? LOW : HIGH);
}
static int steerReportAngle() {
  if (g_stepsPerDeg <= 0) return 0;
  return (int)lroundf((float)g_curSteps / (float)g_stepsPerDeg);
}
static void steerLoop() {
  if (!g_steerEn || g_jedaStepper) return;   // jangan melangkah saat DM556 dimatikan sesaat
  if (g_curSteps == g_tgtSteps) return;

  uint32_t now = micros();
  if (now - g_lastStepUs < STEP_INTERVAL_US) return;
  g_lastStepUs = now;

  bool dirPos = (g_tgtSteps > g_curSteps);
  drvWrite(PIN_DIR, (dirPos != (bool)DM556_DIR_DIBALIK) ? LOW : HIGH);
  delayMicroseconds(50);   // DM556 butuh DIR stabil >= 5 us sebelum STEP; BC547 + opto menambah tunda
  drvStepPulse();
  g_curSteps += dirPos ? 1 : -1;
}

// ===========================================================================
//  JOYSTICK: kendali MANUAL & tambah daya OTOMATIS
// ===========================================================================
static int pwmDenganBoost(int base) {
  if (g_mode != MODE_OTOMATIS || base == 0) return base;   // robot diputuskan diam -> tetap diam
  int y = g_joyY;
  if (base > 0 && y < 0) return 0;     // maju + stik ditarik ke bawah -> REM (PWM 0, BTS7960 mengerem)
  if (base > 0 && y > 0) return base + (255 - base) * y / 100;
  if (base < 0 && y < 0) return base - (255 + base) * (-y) / 100;
  return base;
}
static void setAutoSpeed(int base) {
  g_autoBase   = base;
  g_motorSpeed = pwmDenganBoost(base);
  motorApply();
}
static void applyJoy() {
  if (g_mode == MODE_MANUAL) {
    g_motorSpeed = constrain(g_joyY * g_joyPwm / 100, -255, 255);
    g_tgtAngle   = g_joyX * g_joySudut / 100;
    steerRecalcTarget();
  } else {
    g_motorSpeed = pwmDenganBoost(g_autoBase);
  }
  motorApply();
}

// ===========================================================================
//  POMPA & BUZZER
// ===========================================================================
static void pompaOn()  { digitalWrite(PUMP_PIN, HIGH); g_pumpOn = true;  }
static void pompaOff() { digitalWrite(PUMP_PIN, LOW);  g_pumpOn = false; }

static void cekLevelAir() {
  float w = getDist(IDX_WATER);
  // w = 999 -> tidak terbaca (air penuh, di zona buta JSN) -> dianggap air masih ada
  g_waterLow = (w < 999.0f) && (w > BATAS_AIR_HABIS);
  digitalWrite(BUZZER_PIN, g_waterLow ? HIGH : LOW);
}

// ===========================================================================
//  FUZZY MAMDANI (Bab 3): Fuzzifikasi -> Inferensi (min-max) -> Defuzzifikasi COA
// ===========================================================================
// Persamaan (5) Bab 3: fungsi keanggotaan trapesium
static float trapmf(float x, const float p[4]) {
  const float a = p[0], b = p[1], c = p[2], d = p[3];
  if (x < a || x > d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;            // daerah datar (termasuk bahu kiri/kanan)
  if (x < b) return (x - a) / (b - a);          // sisi naik
  return (d - x) / (d - c);                     // sisi turun
}

static void fuzzyTuning(float e, float de, float &kp, float &ki, float &kd) {
  // saturasi ke semesta pembicaraan [-6, 6]
  e  = constrain(e,  FZ_MIN, FZ_MAX);
  de = constrain(de, FZ_MIN, FZ_MAX);

  // 1) Fuzzifikasi
  float muE[N_IN], muDE[N_IN];
  for (int k = 0; k < N_IN; k++) {
    muE[k]  = trapmf(e,  MF_IN[k]);
    muDE[k] = trapmf(de, MF_IN[k]);
  }

  // 2) Inferensi Mamdani: alpha = min(mu_e, mu_de); agregasi per label = max(alpha)
  float aggKp[N_OUT] = {0}, aggKi[N_OUT] = {0}, aggKd[N_OUT] = {0};
  for (int i = 0; i < N_IN; i++) {       // i = error (baris tabel dosen)
    for (int j = 0; j < N_IN; j++) {     // j = delta error (kolom tabel dosen)
      float alpha = min(muE[i], muDE[j]);
      if (alpha <= 0.0f) continue;
      aggKp[RULE_KP[i][j]] = max(aggKp[RULE_KP[i][j]], alpha);
      aggKi[RULE_KI[i][j]] = max(aggKi[RULE_KI[i][j]], alpha);
      aggKd[RULE_KD[i][j]] = max(aggKd[RULE_KD[i][j]], alpha);
    }
  }

  // 3) Defuzzifikasi COA, Persamaan (11): Vo = sum(mu_i * z_i) / sum(mu_i)
  float nP = 0, dP = 0, nI = 0, dI = 0, nD = 0, dD = 0;
  for (int k = 0; k < N_OUT; k++) {
    nP += aggKp[k] * OUT_CENTER[k];  dP += aggKp[k];
    nI += aggKi[k] * OUT_CENTER[k];  dI += aggKi[k];
    nD += aggKd[k] * OUT_CENTER_KD[k];  dD += aggKd[k];   // v13: rentang Kd 0-300
  }
  if (dP > 0.0001f) kp = nP / dP;   // jika tidak ada aturan aktif, nilai lama dipertahankan
  if (dI > 0.0001f) ki = nI / dI;
  if (dD > 0.0001f) kd = nD / dD;
}

// ===========================================================================
//  PID DISKRIT (Bab 4): I(k) = I(k-1) + e*Ts ; u = Kp*e + Ki*I + Kd*de
// ===========================================================================
static float hitungPID(float e, float de, float dt) {
  // anti-windup: integral di-reset saat error berganti tanda (robot melewati setpoint)
  if ((e > 0 && g_integral < 0) || (e < 0 && g_integral > 0)) g_integral = 0.0f;
  g_integral += e * dt;
  g_integral = constrain(g_integral, -INTEGRAL_MAX, INTEGRAL_MAX);

  g_P = Kp * e;
  g_I = Ki * g_integral;
  g_D = Kd * de;                    // delta error per sampling (sama dengan input fuzzy)
  float output = g_P + g_I + g_D;
  g_u = constrain(output, -OUTPUT_MAX, OUTPUT_MAX);
  return g_u;
}

// ===========================================================================
//  LOG DATA (buffer melingkar di RAM, diunduh lewat web)
// ===========================================================================
static void logAdd(const char *line) {
  if (!g_logMutex) return;
  xSemaphoreTake(g_logMutex, portMAX_DELAY);
  strncpy(g_logBuf[g_logHead], line, ROBOT_LOG_LEN - 1);
  g_logBuf[g_logHead][ROBOT_LOG_LEN - 1] = '\0';
  g_logHead = (g_logHead + 1) % ROBOT_LOG_MAX;
  if (g_logCount < ROBOT_LOG_MAX) g_logCount++;
  xSemaphoreGive(g_logMutex);
  // kirim baris yang sama ke laptop (skrip Python) lewat WebSocket:
  // {"type":"log","row":"ms,run,metode,..."} - satu pesan per siklus kendali
  if (ws.count() > 0) {
    char m[ROBOT_LOG_LEN + 32];
    snprintf(m, sizeof(m), "{\"type\":\"log\",\"row\":\"%s\"}", line);
    ws.textAll(m);
  }
}

static void logClear() {
  if (!g_logMutex) return;
  xSemaphoreTake(g_logMutex, portMAX_DELAY);
  g_logHead = 0;
  g_logCount = 0;
  xSemaphoreGive(g_logMutex);
}

// format Excel Indonesia: pemisah kolom ';' dan desimal ','
static void toExcelID(const char *src, char *dst, size_t n) {
  size_t i = 0;
  for (; src[i] && i < n - 1; i++) {
    char c = src[i];
    dst[i] = (c == ',') ? ';' : (c == '.') ? ',' : c;
  }
  dst[i] = '\0';
}

// Pengisi unduhan bertahap: log dikirim baris per baris tanpa menyusun seluruh file
// di RAM (mencegah ERR_CONNECTION_RESET karena memori ESP32 tidak cukup).
// Hanya satu unduhan pada satu waktu; unduh saat robot di mode MANUAL.
static struct {
  int    start, count, line;            // line: -2 = "sep=;", -1 = header, 0.. = data
  bool   excel;
  char   pend[ROBOT_LOG_LEN + 4];
  size_t pendLen, pendPos;
} g_dl;

static size_t dlFill(uint8_t *buf, size_t maxLen, size_t index, bool excel) {
  if (index == 0) {                     // permintaan baru -> ambil potret posisi log
    xSemaphoreTake(g_logMutex, portMAX_DELAY);
    g_dl.count = g_logCount;
    g_dl.start = (g_logHead - g_logCount + ROBOT_LOG_MAX) % ROBOT_LOG_MAX;
    xSemaphoreGive(g_logMutex);
    g_dl.excel = excel;
    g_dl.line  = excel ? -2 : -1;
    g_dl.pendLen = g_dl.pendPos = 0;
  }
  size_t n = 0;
  while (n < maxLen) {
    if (g_dl.pendPos >= g_dl.pendLen) {             // siapkan baris berikutnya
      if (g_dl.line >= g_dl.count) break;           // selesai
      char raw[ROBOT_LOG_LEN];
      if (g_dl.line == -2) {
        strcpy(raw, "sep=;");
      } else if (g_dl.line == -1) {
        strncpy(raw, LOG_HEADER, sizeof(raw) - 1); raw[sizeof(raw) - 1] = '\0';
      } else {
        xSemaphoreTake(g_logMutex, portMAX_DELAY);
        memcpy(raw, g_logBuf[(g_dl.start + g_dl.line) % ROBOT_LOG_MAX], ROBOT_LOG_LEN);
        xSemaphoreGive(g_logMutex);
        raw[ROBOT_LOG_LEN - 1] = '\0';
      }
      if (g_dl.excel && g_dl.line != -2) toExcelID(raw, g_dl.pend, ROBOT_LOG_LEN);
      else { strncpy(g_dl.pend, raw, ROBOT_LOG_LEN - 1); g_dl.pend[ROBOT_LOG_LEN - 1] = '\0'; }
      g_dl.pendLen = strlen(g_dl.pend);
      g_dl.pend[g_dl.pendLen++] = '\r';
      g_dl.pend[g_dl.pendLen++] = '\n';
      g_dl.pendPos = 0;
      g_dl.line++;
    }
    size_t k = g_dl.pendLen - g_dl.pendPos;
    if (k > maxLen - n) k = maxLen - n;
    memcpy(buf + n, g_dl.pend + g_dl.pendPos, k);
    g_dl.pendPos += k;
    n += k;
  }
  return n;                             // 0 = unduhan selesai
}

// ===========================================================================
//  LOGIKA MODE OTOMATIS (wall-following)
// ===========================================================================
static void switchToOtomatis() {
  g_mode = MODE_OTOMATIS;
  g_state = JELAJAH;
  g_urutanIdx = 0;          // v13: urutan belok cadangan mulai dari awal tiap run
  g_mundurLurus = false;
  g_integral = 0.0f;
  g_errorSebelumnya = 0.0f;
  g_motorEn = true;
  g_steerEn = true;
  g_jedaStepper = false;
  steerApplyEnable();
  g_autoBase = 0;
  g_run++;
#if LOG_CSV
  Serial.println(LOG_HEADER);
#endif
}

static bool g_rekamManual = false;   // true setelah motor digerakkan di mode MANUAL (satu run)

static void switchToManual() {
  g_mode = MODE_MANUAL;
  g_rekamManual = false;               // run manual baru dimulai saat motor pertama kali jalan
  if (g_jedaStepper) { g_jedaStepper = false; steerApplyEnable(); }   // keluar dari LIHAT
  g_autoBase = 0;
  g_motorSpeed = 0;
  motorApply();
  pompaOff();
  // steerEn/motorEn TIDAK diubah - biar user kendalikan lewat checkbox
}

// Dipanggil dari loop() HANYA saat ada siklus sensor baru, dt = waktu antar siklus
// filter median 3 sampel: membuang lonjakan sesaat pembacaan ultrasonik
static float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

// ---------------------------------------------------------------------------
// LOG MODE MANUAL (data demonstrasi: robot dikemudikan manual lewat web).
// Dicatat hanya saat motor berputar atau kemudi sedang bergerak, supaya buffer
// tidak penuh oleh baris diam. metode = 'H', status = 'H' (manual / human).
// Kolom sudut = posisi kemudi hasil hitungan step (bukan target).
// Kolom setpoint/e dihitung dengan rumus yang sama dengan mode otomatis (untuk
// analisis), Kp..u = 0 karena kendali PID-Fuzzy tidak bekerja.
// ---------------------------------------------------------------------------
static void manualLogStep() {
#if LOG_CSV
  bool kemudiGerak = (g_curSteps != g_tgtSteps);
  if (g_motorSpeed == 0 && !kemudiGerak) return;
  if (!g_rekamManual) {
    if (g_motorSpeed == 0) return;     // run manual dimulai saat motor mulai jalan
    g_rekamManual = true;
    g_run++;
  }
  float front  = getDist(IDX_FRONT);
  float sKiri  = getDist(IDX_SLEFT);
  float sKanan = getDist(IDX_SRIGHT);
  float kiri   = getDist(IDX_LEFT);
  float kanan  = getDist(IDX_RIGHT);
  float sp = -1.0f, e = 0.0f;
  if (sKiri < g_navMax && sKanan < g_navMax) {
    sp = (sKiri + sKanan) * 0.5f;
    e  = sp - sKiri;
  }
  char baris[ROBOT_LOG_LEN];
  snprintf(baris, sizeof(baris),
           "%lu,%u,H,H,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,0.00,0.000,0.000,0.000,0.00,0.00,0.00,0.00,%d,%d,%d,%d",
           millis(), (unsigned)g_run, front, sKiri, sKanan, kiri, kanan, sp, e,
           steerReportAngle(), g_motorSpeed, g_pumpOn ? 1 : 0, g_waterLow ? 1 : 0);
  Serial.println(baris);
  logAdd(baris);
#endif
}

static void autoModeStep(float dt) {
  // ---- 0) Baca sensor ----
  // Run baru (baru masuk OTOMATIS): semua riwayat filter dikosongkan. Tanpa ini, nilai
  // sisa run sebelumnya (mis. depan 22 cm dan pewaktu "macet" yang sudah lama) langsung
  // dianggap macet -> robot MUNDUR begitu tombol OTOMATIS ditekan.
  static uint16_t runLalu = 0;
  const bool runBaru = (g_run != runLalu);
  runLalu = g_run;
  static float hL[3] = {999, 999, 999}, hR[3] = {999, 999, 999};
  static uint8_t hi = 0;
  if (runBaru) {
    for (int k = 0; k < 3; k++) { hL[k] = 999; hR[k] = 999; }
    hi = 0;
  }
  hL[hi] = getDist(IDX_SLEFT);
  hR[hi] = getDist(IDX_SRIGHT);
  hi = (hi + 1) % 3;
  float sKiri  = median3(hL[0], hL[1], hL[2]);   // navigasi (disaring median 3)
  float sKanan = median3(hR[0], hR[1], hR[2]);
  // Penolakan lonjakan: robot tidak mungkin bergeser > LONJAK_CM dalam satu siklus,
  // jadi pembacaan yang melompat jauh dianggap pantulan salah (dipakai nilai lama),
  // kecuali lompatan itu bertahan 3 siklus berturut-turut (lorong memang berubah).
  // Setelah itu dihaluskan rata-rata bergerak eksponensial (EMA).
  {
    static float fL = -1, fR = -1;
    static uint8_t nL = 0, nR = 0;
    if (runBaru) { fL = fR = -1; nL = nR = 0; }
    const float LONJAK_CM = 15.0f, ALFA = 0.6f;
    auto saring = [&](float x, float &f, uint8_t &n) -> float {
      if (x >= g_navMax) { f = -1; n = 0; return x; }          // tak dipakai navigasi
      if (f < 0) { f = x; n = 0; return f; }
      if (fabsf(x - f) > LONJAK_CM && ++n < 3) return f;       // tolak lonjakan sesaat
      n = 0;
      f = ALFA * x + (1.0f - ALFA) * f;
      return f;
    };
    sKiri  = saring(sKiri,  fL, nL);
    sKanan = saring(sKanan, fR, nR);
  }
  float kiri   = getDist(IDX_LEFT);              // v13: pengaman sudut moncong kiri (navigasi)
  float kanan  = getDist(IDX_RIGHT);
  float front  = getDist(IDX_FRONT);
  // median 3 sampel: buang lonjakan sesaat sensor depan
  {
    static float hF[3] = {999, 999, 999};
    static uint8_t fi = 0;
    if (runBaru) { hF[0] = hF[1] = hF[2] = 999; fi = 0; }
    hF[fi] = front; fi = (fi + 1) % 3;
    front = median3(hF[0], hF[1], hF[2]);
  }
  // JSN depan tak terbaca (999) padahal sebelumnya sudah dekat -> anggap masih dekat.
  // Ditahan terus HANYA bila terakhir < 30 cm (zona buta JSN: halangan benar-benar dekat).
  // Selain itu ditahan maks. 4 siklus; setelah itu 999 diterima (depan memang lapang),
  // agar jarak depan tidak terkunci pada nilai lama (mis. 50 cm) padahal depan sudah jauh.
  static float lastFront = 999.0f;
  static uint8_t nHilang = 0;
  if (runBaru) { lastFront = 999.0f; nHilang = 0; }
  if (front >= 999.0f && lastFront < g_jarakAman && (lastFront < 30.0f || nHilang < 4)) {
    front = lastFront;
    nHilang++;
  } else {
    lastFront = front;
    nHilang = 0;
  }

  float sp = -1.0f;   // -1 di log = kendali kemudi PID-Fuzzy tidak dihitung pada siklus ini
  uint32_t now = millis();
  // Sisi TERBUKA bila 2 pembacaan terakhir berturut-turut > jarakBuka (999 = tak ada pantulan
  // juga dihitung terbuka). Memakai data mentah agar keputusan belok tidak tertunda filter.
  uint8_t i1 = (hi + 2) % 3, i2 = (hi + 1) % 3;   // indeks pembacaan terbaru & sebelumnya
  bool kiriBuka  = (hL[i1] >= g_jarakBuka) && (hL[i2] >= g_jarakBuka);
  bool kananBuka = (hR[i1] >= g_jarakBuka) && (hR[i2] >= g_jarakBuka);

  // Deteksi MACET: depan < (jarak mundur + 15 cm) dan tidak berubah > 3 cm selama 1,5 s
  // padahal motor maju (robot sudah menempel halangan walau sensor depan masih membaca
  // > jarak mundur). Dulu batasnya 60 cm: pembacaan semu yang tetap (mis. 36 / 56 cm)
  // membuat robot dianggap macet lalu mundur & belok padahal depan masih jauh.
  // v13: saat manuver (BELOK / CARI_CELAH) macet = motor maju tetapi depan, samping kiri &
  // samping kanan (mentah) tidak berubah > 3 cm selama 1,5 s, BERAPA PUN jaraknya. Uji simulasi:
  // sudut badan/roda menempel dinding sementara sensor depan masih membaca 40-140 cm, sehingga
  // aturan lama (depan < 30 cm) tidak pernah aktif dan robot mendorong dinding sampai 20 s.
  // Saat jalan biasa: depan < jarak belok dan tidak berubah 1,5 s.
  bool macet = false;
  {
    static float fRef = 999.0f, lRef = 999.0f, rRef = 999.0f;
    static uint32_t tRef = 0;
    if (runBaru) { fRef = lRef = rRef = 999.0f; tRef = now; }
    bool maju = (g_state == JELAJAH || g_state == MELAMBAT || g_state == BELOK_KIRI ||
                 g_state == BELOK_KANAN || g_state == CARI_CELAH)
                && g_motorSpeed > 0;   // sedang direm lewat stik -> bukan macet
    bool manuver = (g_state == BELOK_KIRI || g_state == BELOK_KANAN || g_state == CARI_CELAH);
    if (manuver) {
      if (!maju || fabsf(front - fRef) > 3.0f || fabsf(hL[i1] - lRef) > 3.0f || fabsf(hR[i1] - rRef) > 3.0f) {
        fRef = front; lRef = hL[i1]; rRef = hR[i1]; tRef = now;
      } else if (now - tRef > 1500UL) { macet = true; fRef = lRef = rRef = 999.0f; tRef = now; }
    } else {
      lRef = rRef = 999.0f;
      if (!maju || front >= g_jarakBelok || fabsf(front - fRef) > 3.0f) { fRef = front; tRef = now; }
      else if (now - tRef > 1500UL) { macet = true; fRef = 999.0f; tRef = now; }
    }
  }
  // v13: kelima sensor dipakai navigasi. Sensor KIRI/KANAN (miring di sudut moncong) menjadi
  // pengaman sudut depan: bila salah satu < g_jarakSudut, sudut robot hampir menabrak dinding/
  // bedengan -> diperlakukan seperti macet (LIHAT saat jalan lurus, MUNDUR saat belok/cari celah).
  if ((kiri < g_jarakSudut || kanan < g_jarakSudut) &&
      (g_state == JELAJAH || g_state == MELAMBAT || g_state == BELOK_KIRI ||
       g_state == BELOK_KANAN || g_state == CARI_CELAH)) {
    macet = true;
  }

  // ---- 1) Tentukan keadaan ----
  auto mulaiMundur = [&]() {
    g_mundurCount++;
    if (g_mundurCount > MUNDUR_MAKS) { g_state = BERHENTI; return; }   // terjebak
    g_state = MUNDUR;
    g_fase = 0;
    g_faseStart = now;
  };
  auto mulaiLihat = [&]() {             // berhenti & tunggu kemudi diam sebelum membaca bersih
    g_state = LIHAT;
    g_lihatN = 0;
    g_faseStart = now;
  };

  // kemudi dianggap sudah sampai bila selisih posisi stepper < 3 derajat
  bool kemudiSampai = labs(g_tgtSteps - g_curSteps) <= 3L * g_stepsPerDeg;
  if (g_state == MUNDUR) {
    uint32_t t = now - g_faseStart;
    // jeda diam sampai kemudi selesai berputar ke arah mundur (maks 3 s), baru motor mundur
    if (g_fase == 0 && t >= JEDA_MS && (kemudiSampai || t >= 3000UL)) {   // selesai jeda -> mulai mundur
      g_fase = 1; g_faseStart = now;
    } else if (g_fase == 1 && t >= (uint32_t)(g_mundurLurus ? g_waktuMundurCari : g_waktuMundur)) {
      g_fase = 2; g_faseStart = now;                   // selesai mundur -> jeda
    } else if (g_fase == 2 && t >= JEDA_MS && (kemudiSampai || t >= 3000UL)) {   // kemudi siap -> maju lagi
      g_turnStartMs = now;                             // waktu belok dihitung sejak maju lagi
      g_mundurLurus = false;
      if (g_turnDir > 0)      g_state = BELOK_KANAN;   // lanjut maju sambil belok
      else if (g_turnDir < 0) g_state = BELOK_KIRI;
      else                    g_state = MELAMBAT;
    }
  } else if (g_state == BELOK_KIRI || g_state == BELOK_KANAN) {
    uint32_t t = now - g_turnStartMs;
    if (front < g_jarakMundur || macet) {
      mulaiMundur();                                   // terlalu dekat / tertahan -> mundur dulu
    } else if (t >= (uint32_t)g_waktuBelok && front > g_jarakBelok &&
               (sKiri < g_navMax || sKanan < g_navMax)) {
      // v13: belokan selesai bila depan lapang DAN salah satu sisi sudah melihat dinding lorong
      g_state = JELAJAH;                               // jalan depan lapang -> belokan selesai
      g_turnDir = 0;
      // v13: kemudi langsung lurus (tanpa batas laju 4 derajat/siklus). Uji simulasi: dari 45
      // derajat ke 0 butuh +-2 s sehingga robot terus berputar +-15 derajat melewati arah lorong.
      g_tgtAngle = 0;
      steerRecalcTarget();
      g_mundurCount = 0;
      g_integral = 0.0f;
      g_errorSebelumnya = 0.0f;
    } else if (t > BELOK_MAKS_MS) {
      g_state = BERHENTI;                              // belok terlalu lama
    }
#if LIHAT_SEBELUM_BELOK
  } else if (g_state == LIHAT) {
    uint32_t t = now - g_faseStart;
    if (!g_jedaStepper) {
      // tunggu robot berhenti & stepper selesai melangkah (maks 3 s), lalu matikan DM556
      if ((g_curSteps == g_tgtSteps && t >= 300UL) || t >= 3000UL) {
        g_jedaStepper = true;
        steerApplyEnable();
        g_lihatN = 0;
        g_lihatL = g_lihatR = 0;
        g_lihatF = 999.0f;
      }
    } else {
      g_lihatN++;
      if (g_lihatN >= 2) {                             // siklus pertama masih bercampur derau
        float l = getDist(IDX_SLEFT), r = getDist(IDX_SRIGHT), f = getDist(IDX_FRONT);
        g_lihatL += min(l, 400.0f);                    // 999 (tak ada pantulan) = lapang
        g_lihatR += min(r, 400.0f);
        if (f < g_lihatF) g_lihatF = f;
      }
      if (g_lihatN >= 1 + LIHAT_SIKLUS) {
        float rl = g_lihatL / LIHAT_SIKLUS, rr = g_lihatR / LIHAT_SIKLUS;
        g_jedaStepper = false;                         // DM556 aktif lagi
        steerApplyEnable();
        bool bukaL = rl >= g_jarakBuka, bukaR = rr >= g_jarakBuka;
        bool keKiri = bukaL ? (!bukaR || rl >= rr) : (bukaR ? false : rl >= rr);
        g_turnDir = keKiri ? -1 : 1;
        g_turnStartMs = now;
        if (g_lihatF >= g_jarakBelok && g_lihatF >= g_jarakPaksa) {
          g_state = JELAJAH;                           // depan ternyata lapang (bacaan semu)
          g_turnDir = 0;
        } else if (!bukaL && !bukaR && g_lihatF >= CARI_MIN_CM) {
          g_state = CARI_CELAH;                        // v13: belum ada celah -> maju pelan mencari
          g_turnDir = 0;
        } else if (g_lihatF < g_jarakMundur + MUNDUR_DULU_CM) {
          mulaiMundur();                               // dinding dekat: mundur dulu (belok 3 titik), lalu belok
        } else {
          g_state = keKiri ? BELOK_KIRI : BELOK_KANAN;
        }
      }
    }
  } else if (g_state == CARI_CELAH) {
    // v13 CARI CELAH: maju pelan (PID-Fuzzy tetap menjaga tengah lorong) sampai sensor samping
    // melihat celah. Pada lintasan U-turn, celah baru terlihat saat moncong +-30 cm dari dinding
    // ujung, padahal keputusan LIHAT diambil di 100 cm -> dulu arah belok untung-untungan.
    if (front > g_jarakBelok) {
      // depan lapang lagi (mis. kerucut sensor depan tadi mengenai dinding samping saat robot
      // miring) -> kembali jalan biasa; tanpa ini robot tertahan di CARI_CELAH (pelan) terus
      g_state = MELAMBAT;
    } else if (kiriBuka || kananBuka) {
      bool keKiri = (kiriBuka && kananBuka) ? (hL[i1] >= hR[i1]) : kiriBuka;
      g_turnDir = keKiri ? -1 : 1;
      g_turnStartMs = now;
      if (front < g_jarakMundur + MUNDUR_DULU_CM) {
        mulaiMundur();                                 // dekat dinding: mundur LURUS dulu agar
        g_mundurLurus = true;                          // busur belok penuh muat di area putar
      } else {
        g_state = keKiri ? BELOK_KIRI : BELOK_KANAN;
      }
    } else if (front < CARI_MIN_CM || macet) {
      // tidak ada celah sampai dekat dinding: pakai urutan belok cadangan
      g_turnDir = URUTAN_BELOK[g_urutanIdx % N_URUTAN_BELOK];
      g_urutanIdx++;
      g_turnStartMs = now;
      mulaiMundur();
      g_mundurLurus = true;
    }
  } else if (front < g_jarakMundur || macet || front < g_jarakBelok || front < g_jarakPaksa) {
    mulaiLihat();                                      // depan terhalang: berhenti & lihat dulu
#endif
  } else if (front < g_jarakMundur || macet) {
    // terlalu dekat / tertahan: berhenti, mundur, lalu maju sambil belok ke sisi
    // terbuka atau yang lebih lapang (tidak mundur lurus lalu menabrak lagi)
    bool keKiri = kiriBuka ? (!kananBuka || sKiri >= sKanan) : (kananBuka ? false : sKiri >= sKanan);
    g_turnDir = keKiri ? -1 : 1;
    g_turnStartMs = now;
    mulaiMundur();
  } else if (front < g_jarakBelok && (kiriBuka || kananBuka)) {
    // depan terhalang -> belok ke sisi yang TERBUKA (> jarakBuka). Bila keduanya terbuka,
    // pilih sisi yang lebih lapang.
    bool keKiri = kiriBuka && (!kananBuka || sKiri >= sKanan);
    g_state = keKiri ? BELOK_KIRI : BELOK_KANAN;
    g_turnDir = keKiri ? -1 : 1;
    g_turnStartMs = now;
  } else if (front < g_jarakPaksa) {
    // depan sudah dekat tetapi belum ada sisi terbuka: belok ke sisi yang lebih lapang
    bool keKiri = (sKiri >= sKanan);
    g_state = keKiri ? BELOK_KIRI : BELOK_KANAN;
    g_turnDir = keKiri ? -1 : 1;
    g_turnStartMs = now;
  } else if (g_state == BERHENTI && front <= g_jarakAman) {
    g_state = BERHENTI;                                // tunggu sampai depan lapang
  } else if (front > g_jarakAman) {
    g_state = JELAJAH;
    g_mundurCount = 0;
  } else {
    g_state = MELAMBAT;
  }

  // ---- 2) Aksi ----
  if (g_state == LIHAT) {
    pompaOff();
    setAutoSpeed(0);                                  // diam; kemudi tetap di posisinya
    g_integral = 0.0f;
    g_errorSebelumnya = 0.0f;

  } else if (g_state == BERHENTI) {
    pompaOff();
    setAutoSpeed(0);
    g_integral = 0.0f;
    g_errorSebelumnya = 0.0f;
    g_tgtAngle = constrain(0, g_tgtAngle - g_steerRate, g_tgtAngle + g_steerRate);
    steerRecalcTarget();                                   // luruskan kemudi saat diam

  } else if (g_state == MUNDUR) {
    pompaOff();
    int a = min(g_sudutBelok, g_maxAngle);
    // saat mundur kemudi DIBALIK agar badan robot terus berputar ke arah belok;
    // pada jeda akhir kemudi disiapkan kembali ke arah belok untuk maju.
    int arah = (g_fase == 2) ? g_turnDir : -g_turnDir;
    if (g_mundurLurus && g_fase < 2) arah = 0;        // v13: mundur lurus sebelum belok penuh
    g_tgtAngle = (int)(g_steerSign * arah * a);
    steerRecalcTarget();
    setAutoSpeed((g_fase == 1) ? -g_pwmMundur : 0);
    g_integral = 0.0f;
    g_errorSebelumnya = 0.0f;

  } else if (g_state == BELOK_KIRI || g_state == BELOK_KANAN) {
    pompaOff();
    int a = min(g_sudutBelok, g_maxAngle);
    // arah KANAN = tanda g_steerSign (sama dengan koreksi saat terlalu dekat tanaman kiri)
    g_tgtAngle = (int)(g_steerSign * (g_state == BELOK_KANAN ? a : -a));
    steerRecalcTarget();
    // motor berhenti sebentar sampai roda kemudi selesai berbelok (maks 2 s),
    // supaya robot tidak terus lurus menuju halangan selama stepper masih berputar
    bool siap = labs(g_tgtSteps - g_curSteps) <= 5L * g_stepsPerDeg;
    setAutoSpeed((siap || (now - g_turnStartMs) > 2000UL) ? KECEPATAN_MENDEKATI : 0);
    g_integral = 0.0f;
    g_errorSebelumnya = 0.0f;

  } else {
    // JELAJAH / MELAMBAT: jaga posisi di lorong dengan sensor SAMPING (PID-Fuzzy)
    static float lastHalf = SETPOINT_KIRI_CM;
    bool okL = (sKiri  < g_navMax);
    bool okR = (sKanan < g_navMax);
    bool ada = true;
    float e = 0.0f;
    if (okL && okR) {
      float tengah = (sKiri + sKanan) * 0.5f;
      sp = g_spTengah ? tengah : SETPOINT_KIRI_CM;
      // setengah lebar lorong untuk mode satu sisi: diperbarui pelan agar tidak ikut melonjak
      lastHalf = 0.8f * lastHalf + 0.2f * tengah;
      e  = sp - sKiri;                                     // Persamaan (4)
    } else if (okL) {                                      // sisi kanan celah -> ikuti kiri
      sp = g_spTengah ? lastHalf : SETPOINT_KIRI_CM;
      e  = sp - sKiri;
    } else if (okR) {                                      // sisi kiri celah -> ikuti kanan
      sp = g_spTengah ? lastHalf : SETPOINT_KIRI_CM;
      e  = sKanan - sp;                                    // cermin: dekat kanan -> e < 0
    } else {
      ada = false;                                         // kedua sisi lapang
    }

    if (ada) {
      g_error      = e;
      g_deltaError = g_error - g_errorSebelumnya;          // Persamaan (6)
      if (g_useFuzzy) {
        fuzzyTuning(g_error, g_deltaError, Kp, Ki, Kd);    // Mamdani Bab 3
      } else {
        Kp = g_kpTetap; Ki = g_kiTetap; Kd = g_kdTetap;    // PID konvensional
      }
      float output = hitungPID(g_error, g_deltaError, dt);

      int batas = min(g_autoMaxAngle, g_maxAngle);
      float sudut = constrain(g_steerSign * g_steerScale * output, (float)-batas, (float)batas);
      int target = (int)sudut;
      if (fabsf(g_error) < g_zonaMati) {                  // sudah di tengah -> kemudi lurus
        target = 0;
        g_integral = 0.0f;
      }
      g_tgtAngle = constrain(target, g_tgtAngle - g_steerRate, g_tgtAngle + g_steerRate);
      steerRecalcTarget();
      g_errorSebelumnya = g_error;
    } else {
      g_tgtAngle = constrain(0, g_tgtAngle - g_steerRate, g_tgtAngle + g_steerRate);
      steerRecalcTarget();
      g_integral = 0.0f;
    }

    setAutoSpeed((g_state == JELAJAH) ? KECEPATAN_JELAJAH : KECEPATAN_MENDEKATI);

    // ---- 3) Penyiraman (v13): pompa menyala selama robot berjalan lurus (JELAJAH, MELAMBAT,
    //      CARI_CELAH) dan mati saat belok, mundur, melihat atau berhenti. Air habis -> mati.
    if (!g_waterLow) pompaOn(); else pompaOff();
  }

#if LOG_CSV
  // ---- 4) Log satu baris per siklus ----
  // status: J jelajah, L melambat, B berhenti, K belok kiri, R belok kanan, M mundur (H = manual, lihat manualLogStep)
  const char st = (g_state == JELAJAH) ? 'J' : (g_state == MELAMBAT) ? 'L' :
                  (g_state == BERHENTI) ? 'B' : (g_state == BELOK_KIRI) ? 'K' :
                  (g_state == BELOK_KANAN) ? 'R' : (g_state == LIHAT) ? 'S' :
                  (g_state == CARI_CELAH) ? 'C' : 'M';
  char baris[ROBOT_LOG_LEN];
  snprintf(baris, sizeof(baris),
           "%lu,%u,%c,%c,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d",
           millis(), (unsigned)g_run, g_useFuzzy ? 'F' : 'P', st,
           front, sKiri, sKanan, kiri, kanan, sp, g_error, g_deltaError,
           Kp, Ki, Kd, g_P, g_I, g_D, g_u, g_tgtAngle, g_motorSpeed,
           g_pumpOn ? 1 : 0, g_waterLow ? 1 : 0);
  Serial.println(baris);
  logAdd(baris);
#endif
}

// ===========================================================================
//  BROADCAST telemetry
// ===========================================================================
static void broadcastSensors() {
  JSON_DOC(doc);
  doc["type"] = "sensor";
  doc["up"]   = millis() / 1000;          // detik sejak ESP32 menyala (reset ke 0 bila restart)
  doc["rst"]  = resetReasonStr();         // penyebab restart terakhir

  xSemaphoreTake(g_distMutex, portMAX_DELAY);
  for (int i = 0; i < N_SENSOR; i++) doc[SENSOR_KEY[i]] = g_dist[i];
  xSemaphoreGive(g_distMutex);

  doc["motor_speed"]   = g_motorSpeed;
  doc["steer_angle"]   = steerReportAngle();
  doc["max_angle"]     = g_maxAngle;
  doc["steps_per_deg"] = g_stepsPerDeg;
  doc["motor_en"]      = g_motorEn;
  doc["steer_en"]      = g_steerEn;
  doc["mode"]          = (g_mode == MODE_OTOMATIS) ? "otomatis" : "manual";
  doc["pump"]          = g_pumpOn;
  doc["water_low"]     = g_waterLow;
  doc["log_n"]         = g_logCount;
  doc["run"]           = g_run;
  doc["ctrl"]          = g_useFuzzy ? "fuzzy" : "pid";
  doc["status"]        = (g_mode != MODE_OTOMATIS) ? "-" :
                         (g_state == JELAJAH) ? "jelajah" : (g_state == MELAMBAT) ? "melambat" :
                         (g_state == BERHENTI) ? "berhenti" : (g_state == BELOK_KIRI) ? "belok kiri" :
                         (g_state == BELOK_KANAN) ? "belok kanan" :
                         (g_state == LIHAT) ? "melihat" :
                         (g_state == CARI_CELAH) ? "cari celah" : "mundur";
  doc["e"]             = g_error;
  doc["kp"]            = Kp;
  doc["ki"]            = Ki;
  doc["kd"]            = Kd;
  doc["kp_t"]          = g_kpTetap;
  doc["ki_t"]          = g_kiTetap;
  doc["kd_t"]          = g_kdTetap;
  doc["s_scale"]       = g_steerScale;
  doc["j_aman"]        = g_jarakAman;
  doc["j_stop"]        = g_jarakMundur;
  doc["w_mundur"] = g_waktuMundur; doc["p_mundur"] = g_pwmMundur;
  doc["p_jalan"] = KECEPATAN_JELAJAH; doc["p_pelan"] = KECEPATAN_MENDEKATI;
  doc["depan"]         = getDist(IDX_FRONT);
  doc["kiri"]          = getDist(IDX_LEFT);
  doc["kanan"]         = getDist(IDX_RIGHT);
  doc["s_kiri"]        = getDist(IDX_SLEFT);
  doc["s_kanan"]       = getDist(IDX_SRIGHT);
  doc["j_belok"] = g_jarakBelok; doc["j_buka"] = g_jarakBuka; doc["j_paksa"] = g_jarakPaksa; doc["j_tanam"] = g_jarakTanaman;
  doc["s_belok"] = g_sudutBelok; doc["w_belok"] = g_waktuBelok; doc["z_mati"] = g_zonaMati;
  doc["sudut_cmd"]     = g_tgtAngle;
  doc["s_max"]         = g_autoMaxAngle;
  doc["s_rate"]        = g_steerRate;
  doc["s_inv"]         = (g_steerSign > 0);
  doc["sp_t"]          = g_spTengah;
  doc["air"]           = getDist(IDX_WATER);
  doc["i_f"] = IDX_FRONT; doc["i_l"] = IDX_LEFT; doc["i_r"] = IDX_RIGHT; doc["i_w"] = IDX_WATER;
  doc["i_sl"] = IDX_SLEFT; doc["i_sr"] = IDX_SRIGHT;
  doc["joy_pwm"] = g_joyPwm; doc["joy_sudut"] = g_joySudut; doc["auto_base"] = g_autoBase;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ===========================================================================
//  HANDLE COMMAND dari client
// ===========================================================================
// ===========================================================================
//  SIMPAN / MUAT SETELAN (flash ESP32, namespace "robot")
// ===========================================================================
static Preferences g_prefs;
static const int SETELAN_VERSI = 3;   // naikkan bila nilai bawaan kemudi diubah

static void simpanSetelan() {
  g_prefs.begin("robot", false);
  g_prefs.putInt("iF", IDX_FRONT);  g_prefs.putInt("iL", IDX_LEFT);   g_prefs.putInt("iR", IDX_RIGHT);
  g_prefs.putInt("iW", IDX_WATER);  g_prefs.putInt("iSL", IDX_SLEFT); g_prefs.putInt("iSR", IDX_SRIGHT);
  g_prefs.putFloat("sc", g_steerScale); g_prefs.putInt("sm", g_autoMaxAngle); g_prefs.putInt("sr", g_steerRate);
  g_prefs.putFloat("sg", g_steerSign);  g_prefs.putBool("spt", g_spTengah);
  g_prefs.putFloat("jA", g_jarakAman);  g_prefs.putFloat("jB", g_jarakBelok); g_prefs.putFloat("jM", g_jarakMundur);
  g_prefs.putInt("wM", g_waktuMundur);  g_prefs.putInt("pM", g_pwmMundur);
  g_prefs.putFloat("jK", g_jarakBuka); g_prefs.putFloat("jP", g_jarakPaksa);  g_prefs.putFloat("jT", g_jarakTanaman);
  g_prefs.putInt("sB", g_sudutBelok);   g_prefs.putInt("wB", g_waktuBelok);  g_prefs.putFloat("zM", g_zonaMati);
  g_prefs.putInt("pJ", KECEPATAN_JELAJAH); g_prefs.putInt("pP", KECEPATAN_MENDEKATI);
  g_prefs.putFloat("kp", g_kpTetap); g_prefs.putFloat("ki", g_kiTetap); g_prefs.putFloat("kd", g_kdTetap);
  g_prefs.putBool("fz", g_useFuzzy);
  g_prefs.putInt("mxA", g_maxAngle); g_prefs.putInt("spd", g_stepsPerDeg);
  g_prefs.putInt("jyP", g_joyPwm);   g_prefs.putInt("jyS", g_joySudut);
  g_prefs.putInt("ver", SETELAN_VERSI);
  g_prefs.end();
}

static void muatSetelan() {
  g_prefs.begin("robot", true);    // nilai kedua = nilai bawaan program bila belum pernah disimpan
  bool versiLama = g_prefs.getInt("ver", 0) != SETELAN_VERSI;
  IDX_FRONT = g_prefs.getInt("iF", IDX_FRONT);  IDX_LEFT  = g_prefs.getInt("iL", IDX_LEFT);
  IDX_RIGHT = g_prefs.getInt("iR", IDX_RIGHT);  IDX_WATER = g_prefs.getInt("iW", IDX_WATER);
  IDX_SLEFT = g_prefs.getInt("iSL", IDX_SLEFT); IDX_SRIGHT = g_prefs.getInt("iSR", IDX_SRIGHT);
  g_maxAngle = g_prefs.getInt("mxA", g_maxAngle); g_stepsPerDeg = g_prefs.getInt("spd", g_stepsPerDeg);
  g_steerSign = g_prefs.getFloat("sg", g_steerSign);
  g_joyPwm   = g_prefs.getInt("jyP", g_joyPwm);  g_joySudut = g_prefs.getInt("jyS", g_joySudut);
  if (versiLama) {   // setelan dari program lama: hanya pemetaan sensor & kalibrasi kemudi yang dipakai
    g_prefs.end();
    simpanSetelan();   // tulis ulang dengan nilai bawaan baru + nomor versi
    return;
  }
  g_steerScale = g_prefs.getFloat("sc", g_steerScale); g_autoMaxAngle = g_prefs.getInt("sm", g_autoMaxAngle);
  g_steerRate  = g_prefs.getInt("sr", g_steerRate);
  g_spTengah   = g_prefs.getBool("spt", g_spTengah);
  g_jarakAman  = g_prefs.getFloat("jA", g_jarakAman);  g_jarakBelok = g_prefs.getFloat("jB", g_jarakBelok);
  g_jarakMundur= g_prefs.getFloat("jM", g_jarakMundur);
  g_waktuMundur= g_prefs.getInt("wM", g_waktuMundur);  g_pwmMundur = g_prefs.getInt("pM", g_pwmMundur);
  g_jarakBuka  = g_prefs.getFloat("jK", g_jarakBuka); g_jarakPaksa = g_prefs.getFloat("jP", g_jarakPaksa);  g_jarakTanaman = g_prefs.getFloat("jT", g_jarakTanaman);
  g_sudutBelok = g_prefs.getInt("sB", g_sudutBelok);   g_waktuBelok = g_prefs.getInt("wB", g_waktuBelok);
  g_zonaMati   = g_prefs.getFloat("zM", g_zonaMati);
  KECEPATAN_JELAJAH   = g_prefs.getInt("pJ", KECEPATAN_JELAJAH);
  KECEPATAN_MENDEKATI = g_prefs.getInt("pP", KECEPATAN_MENDEKATI);
  g_kpTetap = g_prefs.getFloat("kp", g_kpTetap); g_kiTetap = g_prefs.getFloat("ki", g_kiTetap);
  g_kdTetap = g_prefs.getFloat("kd", g_kdTetap);
  g_useFuzzy = g_prefs.getBool("fz", g_useFuzzy);
  g_maxAngle = g_prefs.getInt("mxA", g_maxAngle); g_stepsPerDeg = g_prefs.getInt("spd", g_stepsPerDeg);
  g_prefs.end();
}

static void handleCommand(const uint8_t *data, size_t len) {
  JSON_DOC(doc);
  if (deserializeJson(doc, data, len)) return;

  const char *cmd = doc["cmd"];
  if (!cmd) return;

  if (!strcmp(cmd, "set_mode")) {
    const char *v = doc["value"];
    if (v && !strcmp(v, "otomatis")) switchToOtomatis();
    else if (v && !strcmp(v, "manual")) switchToManual();

  } else if (!strcmp(cmd, "motor_speed")) {
    if (g_mode != MODE_MANUAL) return;   // abaikan saat OTOMATIS
    g_motorSpeed = constrain((int)(doc["value"] | 0), -255, 255);
    motorApply();

  } else if (!strcmp(cmd, "motor_stop")) {
    if (g_mode != MODE_MANUAL) return;
    g_motorSpeed = 0;
    motorApply();

  } else if (!strcmp(cmd, "motor_enable")) {
    // enable selalu boleh - dipakai juga sebagai tombol darurat di OTOMATIS
    g_motorEn = doc["value"] | false;
    motorApply();

  } else if (!strcmp(cmd, "steer_angle")) {
    if (g_mode != MODE_MANUAL) return;
    g_tgtAngle = (int)(doc["value"] | 0);
    steerRecalcTarget();

  } else if (!strcmp(cmd, "steer_center")) {
    if (g_mode != MODE_MANUAL) return;
    g_tgtAngle = 0;
    steerRecalcTarget();

  } else if (!strcmp(cmd, "steer_enable")) {
    g_steerEn = doc["value"] | false;
    steerApplyEnable();

  } else if (!strcmp(cmd, "set_max_angle")) {
    g_maxAngle = constrain((int)(doc["value"] | g_maxAngle), 1, 180);
    steerRecalcTarget();
    simpanSetelan();

  } else if (!strcmp(cmd, "set_steps_per_deg")) {
    g_stepsPerDeg = constrain((int)(doc["value"] | g_stepsPerDeg), 1, 1000);
    steerRecalcTarget();
    simpanSetelan();

  } else if (!strcmp(cmd, "zero_position")) {
    g_curSteps = 0;
    g_tgtSteps = 0;
    g_tgtAngle = 0;

  } else if (!strcmp(cmd, "set_ctrl")) {
    const char *v = doc["value"];
    if (v) {
      g_useFuzzy = strcmp(v, "pid") != 0;
    }
    simpanSetelan();
    g_forceSend = true;

  } else if (!strcmp(cmd, "set_steer")) {
    g_steerScale   = constrain((float)(doc["scale"] | g_steerScale), 0.01f, 2.0f);
    g_autoMaxAngle = constrain((int)(doc["maxa"] | g_autoMaxAngle), 1, 45);
    g_steerRate    = constrain((int)(doc["rate"] | g_steerRate), 1, 45);
    if (!doc["inv"].isNull()) g_steerSign = (doc["inv"] | false) ? 1.0f : -1.0f;
    g_jarakAman = constrain((float)(doc["jaman"] | g_jarakAman), 20.0f, 400.0f);
    g_jarakMundur = constrain((float)(doc["jstop"] | g_jarakMundur), 5.0f, g_jarakAman - 5.0f);
    g_waktuMundur = constrain((int)(doc["wmundur"] | g_waktuMundur), 200, 5000);
    g_pwmMundur   = constrain((int)(doc["pmundur"] | g_pwmMundur), 0, 255);
    if (!doc["sptengah"].isNull()) g_spTengah = doc["sptengah"] | false;
    g_jarakBelok   = constrain((float)(doc["jbelok"] | g_jarakBelok), g_jarakMundur, g_jarakAman);
    g_jarakBuka    = constrain((float)(doc["jbuka"]  | g_jarakBuka), 30.0f, 400.0f);
    g_jarakPaksa   = constrain((float)(doc["jpaksa"] | g_jarakPaksa), 15.0f, 150.0f);
    g_jarakTanaman = constrain((float)(doc["jtanam"] | g_jarakTanaman), 5.0f, 200.0f);
    g_sudutBelok   = constrain((int)(doc["sbelok"] | g_sudutBelok), 5, 45);
    g_waktuBelok   = constrain((int)(doc["wbelok"] | g_waktuBelok), 200, 7000);
    g_zonaMati     = constrain((float)(doc["zmati"] | g_zonaMati), 0.0f, 10.0f);
    KECEPATAN_JELAJAH   = constrain((int)(doc["pjalan"] | KECEPATAN_JELAJAH), 0, 255);
    KECEPATAN_MENDEKATI = constrain((int)(doc["ppelan"] | KECEPATAN_MENDEKATI), 0, 255);
    simpanSetelan();
    g_forceSend = true;

  } else if (!strcmp(cmd, "set_map")) {
    IDX_FRONT = constrain((int)(doc["f"] | IDX_FRONT), 0, N_SENSOR - 1);
    IDX_LEFT  = constrain((int)(doc["l"] | IDX_LEFT),  0, N_SENSOR - 1);
    IDX_RIGHT = constrain((int)(doc["r"] | IDX_RIGHT), 0, N_SENSOR - 1);
    IDX_WATER = constrain((int)(doc["w"] | IDX_WATER), 0, N_SENSOR - 1);
    IDX_SLEFT  = constrain((int)(doc["sl"] | IDX_SLEFT),  0, N_SENSOR - 1);
    IDX_SRIGHT = constrain((int)(doc["sr"] | IDX_SRIGHT), 0, N_SENSOR - 1);
    simpanSetelan();
    g_forceSend = true;

  } else if (!strcmp(cmd, "set_gain")) {
    g_kpTetap = doc["kp"] | g_kpTetap;
    g_kiTetap = doc["ki"] | g_kiTetap;
    g_kdTetap = doc["kd"] | g_kdTetap;
    simpanSetelan();
    g_forceSend = true;

  } else if (!strcmp(cmd, "reset_setelan")) {
    g_prefs.begin("robot", false);
    g_prefs.clear();                   // hapus semua setelan tersimpan
    g_prefs.end();
    delay(100);
    ESP.restart();                     // mulai ulang dengan nilai bawaan program

  } else if (!strcmp(cmd, "joy")) {
    // hanya disimpan; diterapkan di loop() agar tidak berebut dengan kendali otomatis
    g_joyX = constrain((int)(doc["x"] | 0), -100, 100);
    g_joyY = constrain((int)(doc["y"] | 0), -100, 100);
    g_joyLastMs = millis();
    g_joyBaru = true;

  } else if (!strcmp(cmd, "joy_cfg")) {
    g_joyPwm   = constrain((int)(doc["pwm"] | g_joyPwm), 0, 255);
    g_joySudut = constrain((int)(doc["sudut"] | g_joySudut), 1, 180);
    simpanSetelan();
    g_forceSend = true;

  } else if (!strcmp(cmd, "read_sensors")) {
    g_forceSend = true;
  }
}

// ===========================================================================
//  WEBSOCKET EVENT
// ===========================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      g_forceSend = true;
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        handleCommand(data, len);
      }
      break;
    }

    default:
      break;
  }
}

// ===========================================================================
//  SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);

  // ---- Sensor pin ----
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
#if PIN_TRIG_DEPAN >= 0
  pinMode(PIN_TRIG_DEPAN, OUTPUT);
  digitalWrite(PIN_TRIG_DEPAN, LOW);
#endif
  for (int i = 0; i < N_SENSOR; i++) {
    pinMode(ECHO_PIN[i], INPUT);
  }

  // ---- Motor BTS7960 ----
  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_L_EN, OUTPUT);
  digitalWrite(PIN_R_EN, LOW);
  digitalWrite(PIN_L_EN, LOW);
  pwmSetup(PIN_RPWM, MOTOR_CH_RPWM);
  pwmSetup(PIN_LPWM, MOTOR_CH_LPWM);
  pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
  pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);

  // ---- DM556 (INVERTING via BC547) ----
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  drvWrite(PIN_STEP, LOW);
  drvWrite(PIN_DIR,  LOW);
  steerApplyEnable();

  // ---- Pompa & Buzzer ----
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ---- Access Point ----
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);   // matikan hemat daya WiFi agar perintah remote tidak tertunda
  Serial.printf("AP \"%s\"  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // ---- WebSocket + HTTP ----
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  // Unduh log untuk Excel (format Indonesia: ';' dan desimal ',')
  // Unduh log untuk Excel (format Indonesia: ';' dan desimal ','), dikirim bertahap (chunked)
  server.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest *req) {
    AsyncWebServerResponse *res = req->beginChunkedResponse("text/csv",
      [](uint8_t *buf, size_t maxLen, size_t index) -> size_t {
        return dlFill(buf, maxLen, index, true);
      });
    res->addHeader("Content-Disposition", "attachment; filename=\"log_robot.csv\"");
    req->send(res);
  });
  // Lihat log mentah (format titik, untuk Google Sheets / dibaca langsung)
  server.on("/log.txt", HTTP_GET, [](AsyncWebServerRequest *req) {
    AsyncWebServerResponse *res = req->beginChunkedResponse("text/plain",
      [](uint8_t *buf, size_t maxLen, size_t index) -> size_t {
        return dlFill(buf, maxLen, index, false);
      });
    req->send(res);
  });
  server.on("/log_clear", HTTP_GET, [](AsyncWebServerRequest *req) {
    logClear();
    g_forceSend = true;
    req->send(200, "text/plain", "Log dihapus");
  });
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });

  // ---- Task sensor (core terpisah) ----
  g_distMutex = xSemaphoreCreateMutex();
  g_logMutex  = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    sensorTaskFn, "sensor", 4096, NULL, 1, &g_sensorTask, SENSOR_TASK_CORE
  );

  muatSetelan();                       // ambil setelan terakhir yang disimpan dari web
  steerRecalcTarget();
  server.begin();

  g_lastBroadcast = millis();
  g_lastWsClean   = millis();
  g_lastStepUs    = micros();
}

// ===========================================================================
//  LOOP
// ===========================================================================
void loop() {
  uint32_t now = millis();

#if TRIG_SELFTEST
  digitalWrite(PIN_TRIG, (now / 250) & 1);
  return;
#endif

  cekLevelAir();

  // Kendali otomatis dihitung SEKALI per siklus sensor (bukan setiap putaran loop),
  // sehingga dt = periode sampling sensor (Ts) dan delta error bermakna.
  static uint32_t lastCycle  = 0;
  static uint32_t lastAutoMs = 0;
  uint32_t cyc = g_sensorCycle;
  if (cyc != lastCycle) {
    lastCycle = cyc;
    float dt = (lastAutoMs == 0) ? 0.25f : (now - lastAutoMs) / 1000.0f;
    lastAutoMs = now;
    if (g_mode == MODE_OTOMATIS) {
      autoModeStep(dt);
    } else {
      manualLogStep();                 // rekam data saat robot dikemudikan manual
    }
  }

  // ---- Joystick web ----
  if ((g_joyX != 0 || g_joyY != 0) && now - g_joyLastMs > JOY_TIMEOUT_MS) {
    g_joyX = 0; g_joyY = 0;            // sinyal stik hilang (WiFi putus) -> anggap dilepas
    g_joyBaru = true;
  }
  if (g_joyBaru) {
    g_joyBaru = false;
    applyJoy();
  }

  steerLoop();   // selalu jalan: dipakai baik oleh MANUAL maupun OTOMATIS

  if (g_forceSend || (now - g_lastBroadcast >= BROADCAST_MS)) {
    g_forceSend     = false;
    g_lastBroadcast = now;
    broadcastSensors();
  }

  if (now - g_lastWsClean >= 1000) {
    g_lastWsClean = now;
    ws.cleanupClients();
  }
}
