# Robot Penyiram Tanaman Otomatis dengan Metode Fuzzy-PID

Repo ini berisi firmware ESP32 dan simulasi MATLAB untuk penelitian
"Inovasi Teknologi Robot Penyiram Tanaman Otomatis Menggunakan Metode Fuzzy-PID".

Robot berkemudi Ackermann. Kemudinya digerakkan stepper DM556 dan roda belakang oleh motor DC
lewat driver BTS7960. Robot berjalan di lorong antara dua deret tanaman, menjaga posisinya di
tengah lorong dengan PID yang gain-nya diatur fuzzy Mamdani, dan menyiram selama berjalan lurus.
Kelima sensor ultrasonik (depan, samping kiri, samping kanan, kiri, kanan) dipakai untuk navigasi.
Sensor kiri/kanan yang miring di sudut moncong menjadi pengaman agar sudut robot tidak menabrak.

```
firmware/program_robot_final_v12/   firmware ESP32: fuzzy sesuai parameter dosen
firmware/program_robot_final_v13/   v12 + perbaikan U-turn (diuji di simulasi) <- pakai ini
simulasi_matlab/                     simulasi MATLAB (R2024a, tanpa toolbox)
  dxf/                               robot.dxf & jalur.dxf dari SolidWorks (satuan cm)
  hasil/                             gambar PNG, animasi GIF, video MP4, log & data CSV, metrik
```

Video robot dari start sampai finish di lintasan asli ada di
`simulasi_matlab/hasil/video_jalur_fuzzy.mp4` (PID-Fuzzy) dan `video_jalur_pid.mp4` (PID tetap).

## Firmware v13 (perbaikan U-turn)

Di simulasi, firmware v12 gagal U-turn di lintasan `jalur.dxf`. Ada dua penyebab:

- Celah U-turn baru terlihat oleh sensor samping saat moncong robot sekitar 30 cm dari dinding ujung.
  Padahal arah belok sudah diputuskan di jarak 100 cm.
- Deteksi macet hanya aktif bila sensor depan membaca < 30 cm, sehingga robot bisa terus mendorong dinding.

Perubahan di v13 (Fuzzy-PID identik dengan v12):

1. Keadaan baru `CARI_CELAH` (status `C` di log). Bila saat LIHAT tidak ada sisi yang terbuka,
   robot maju pelan sampai celah terlihat. Setelah itu robot mundur lurus sekitar 300 ms lalu
   berbelok penuh 45°.
2. Saat manuver, robot dianggap macet bila jarak depan dan kedua sisi tidak berubah selama 1,5 s,
   berapa pun jaraknya.
3. Belokan dianggap selesai bila depan sudah lapang dan salah satu sisi melihat dinding. Kemudi
   langsung diluruskan.
4. Sudut belok bawaan 45°. Kalau setelan lama (40°) tersimpan di flash, ubah "Sudut belok" di
   halaman web menjadi 45 lalu tekan Simpan.

Kedua versi sudah di-compile untuk ESP32 (core esp32 2.0.17).

## Fuzzy Mamdani (sama di firmware v12 dan simulasi)

| Bagian | Isi |
|---|---|
| Input | error e dan delta error Δe (cm). Masing-masing 7 segitiga NB, NM, NS, Z, PS, PM, PB pada [-6, 6], dengan puncak tiap 2. Nilai di luar rentang disaturasi. |
| Output | Kp, Ki, Kd. Masing-masing 9 segitiga ZE, VS, S, MS, M, MB, B, VB, EVB pada [0, 20], dengan puncak tiap 2,5. |
| Rule | 3 × 49 aturan (baris = e, kolom = Δe), lihat `simulasi_matlab/fuzzy_param.m` |
| Inferensi | min-max |
| Defuzzifikasi | COA titik pusat: Σ(μ·z)/Σμ |

Pada e = 0 dan Δe = 0 hasilnya Kp = 0, Ki = 20, Kd = 0. Nilai ini dipakai sebagai nilai awal di firmware.

## Menjalankan simulasi

Jalankan di MATLAB:

```matlab
cd simulasi_matlab
main
```

Atau dari terminal:

```
matlab -batch "run('simulasi_matlab/main.m')"
```

Skrip `main.m` menjalankan langkah berikut:

1. **Uji fuzzy** (`uji_fuzzy.m`): mencocokkan hasil fuzzy dengan hitungan tangan dari tabel rule.
2. **Gambar fuzzy** (`gambar_fuzzy.m`): fungsi keanggotaan e, Δe, Kp, Ki, Kd dan permukaan Kp/Ki/Kd terhadap (e, Δe).
3. **Simulasi semua skenario**, masing-masing dengan PID-Fuzzy dan PID tetap (Kp = Ki = Kd = 9,25):
   - `lurus`: lorong lurus, start di tengah.
   - `tidak_di_tengah`: start bergeser 15 cm ke kiri.
   - `miring`: start di tengah, menghadap 8°.
   - `belokan`: belok kanan 90°, lorong 150 cm setelah belokan.
   - `belokan_sempit`: sama, lorong 100 cm setelah belokan.
   - `rintangan`: lorong buntu karena rintangan di depan.
   - `jalur`: lintasan asli dari `dxf/jalur.dxf` (3 lorong, 2 U-turn, sekat 5 cm). Badan robot
     dan posisi kelima sensor diambil dari `dxf/robot.dxf`.
4. **Grafik, animasi, dan log**: grafik respons, gambar lintasan, animasi GIF, dan log CSV berformat sama dengan robot.
5. **Perbandingan PID-Fuzzy vs PID tetap**: grafik dan tabel metrik (overshoot, settling time, steady-state error, IAE, ISE, RMSE).

Semua parameter (dimensi robot, posisi sensor, kecepatan, ambang jarak, gain) ada di
`simulasi_matlab/config_robot.m`. Parameter yang bertanda `[ASUMSI]` belum diukur dari robot asli.

### Apa yang disalin dari firmware

- `fuzzy_mamdani.m` sama dengan `fuzzyTuning()`.
- `pid_diskrit.m` sama dengan `hitungPID()`.
- `kendali_otomatis.m` sama dengan `autoModeStep()`. Isinya:
  - filter median-3, penolakan lonjakan > 15 cm, dan EMA α = 0,6;
  - penahanan sensor depan saat tak terbaca dan deteksi macet;
  - keadaan JELAJAH, MELAMBAT, BERHENTI, BELOK, MUNDUR (3 fase), dan LIHAT;
  - pemetaan u ke sudut kemudi dengan pembatas laju 4°/siklus;
  - v13: pompa menyala selama jalan lurus dan mati saat belok; sensor kiri/kanan < 15 cm
    dianggap hampir menabrak (v12: pompa menyala bila kiri atau kanan < 40 cm).
- `simulasi.m` meniru pewaktuan `sensorTaskFn()`. Lima sensor dibaca berurutan, masing-masing dengan waktu gema (timeout 25 ms) ditambah jeda 35 ms. Kendali dihitung sekali per siklus (Ts ≈ 0,21 s). Stepper bergerak 1 langkah per 700 µs.
- `sensor_ultrasonik.m` memakai ray-casting kerucut ±15°. Jarak di atas 400 cm bernilai 999. Ada derau Gaussian dan opsi `anomali_dm556` untuk meniru pembacaan semu sekitar 50 cm dari driver stepper.

### Membandingkan dengan log robot asli

Unduh `/log.csv` dari robot (WiFi RobotFinal, alamat 192.168.4.1), lalu jalankan:

```matlab
bandingkan_log('log_robot.csv', 'hasil/log_tidak_di_tengah_fuzzy.csv')
```

Kedua format bisa dibaca: format Excel Indonesia (`sep=;`, desimal koma) dan format koma biasa.
