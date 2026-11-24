# Prompt: Poster Hasil Penelitian – Robot Penyiram Tanaman Fuzzy-PID

Tempel seluruh isi file ini ke sesi Claude Code baru (folder kerja: `c:\Asad\Kuliah\joki\MBKM BU VIVI`),
atau ketik: **"Baca dan kerjakan design/PROMPT_POSTER.md"**.

---

## Tugas

Buat (atau revisi) **poster hasil penelitian** ukuran **A1 portrait** untuk penelitian
"Inovasi Teknologi Robot Penyiram Tanaman Otomatis Menggunakan Metode Fuzzy-PID".
Poster mengikuti **Ketentuan Isian Poster** dari Buku Panduan Penelitian 2026 (Lampiran 2) LPPM.

Poster versi pertama sudah ada di artifact Design (canvas):
**https://claude.ai/artifact/587jcEUStLgXmaAmpq73rD**. Kalau diminta revisi, baca dulu file
`project/Main.dc.html` versi terbaru (pengguna kadang mengedit langsung di kanvas) lalu ubah
seperlunya dengan `edit_version`. Jangan menimpa editan pengguna. Jangan membuat artifact baru
kecuali diminta.

## Sumber data (semua di folder ini)

| File | Isi |
|---|---|
| `design/Ketentuan Isian Poster.docx` | Isi wajib poster a–k + foto robot di lapangan + grafik hasil |
| `design/referensi design.jpg` | Contoh gaya poster (PKM): judul besar hijau, bagian bernomor, garis penghubung |
| `design/robot1..9.jpeg` | Foto robot (robot3 = tampak samping, paling bagus untuk poster) |
| `design/Tut Wuri Handayani.png`, `LOGO UTM_BERWARNA.jpg.jpeg`, `logo BLU.png`, `logo-unggul-kuning.png`, `logo LPPM.png` | Logo resmi |
| `simulasi_matlab/hasil/perbandingan/` | Grafik & tabel hasil simulasi (ringkasan_batang.png, respons4_S1/S3/S6, perbandingan_S1, rata_rata_semua_skenario.csv) |
| `laporan/Laporan_Simulasi_Pengujian_Robot_Penyiram_FuzzyPID.docx` | Penjelasan skenario & hasil pengujian |

## Isi poster (ketentuan a–k)

- a. Judul: Inovasi Teknologi Robot Penyiram Tanaman Otomatis Menggunakan Metode Fuzzy-PID
- b. Perguruan tinggi: **Universitas Trunodjoyo Madura** (ejaan "Trunodjoyo" mengikuti logo & ketentuan)
- c. Skema: Penelitian Grup Riset
- d. Dana penelitian: Rp 29.000.000,-
- e. Tim: Ketua – Ir. Vivi Tri Widyaningrum, S.Kom., M.T.; Anggota – Ir. Hanifudin Sukri, S.Kom., M.Kom.
- f. Tahun: 2026
- g. TKT akhir: TKT 3
- h. Luaran: Jurnal Internasional Terindeks Scopus
- i. Kata kunci: Pertanian (agriculture); mobile robot; Fuzzy-PID; wall follower; pertanian cerdas (smart farming)
- j. Ringkasan: dari dokumen ketentuan, dibagi paragraf Urgensi / Tujuan / Metode / Kontribusi / Luaran, dengan penyesuaian:
  - "arduino" → "mikrokontroler ESP32 (Arduino)"; "fuel pump" tetap;
  - hapus kata dobel "meningkatkan meningkatkan";
  - paragraf Luaran diakhiri: "Kemudian untuk luaran tambahan berupa Hak Cipta sudah terbit (Granted)."
    (Hak Cipta TIDAK ditampilkan di kartu info, hanya kalimat ini).
- k. Gambar: foto robot di lintasan uji (dari dokumen ketentuan, 13/11/2026) + robot tampak samping;
  grafik hasil = Gambar 7–11 di ketentuan (batang IAE/RMSE/settling/overshoot; posisi lateral 4 metode
  S1, S3, S6; detail metode A vs D pada S1).

## Gaya yang sudah disetujui pengguna

- **A1 portrait = 2245 × 3179 px** (594 × 841 mm, 96 px/in), satu artboard fixed, ekspor PDF vektor.
- Elegan, **dengan gradasi** dan **garis-garis** seperti contoh: kepala bergradasi hijau gelap → hijau daun
  (#0A2B20 → #12503A → #1F7A4D → #3E9A5C) dengan garis sirkuit tipis kuning/putih; aksen kuning #F2C230.
- **Judul putih semua** (tanpa sorotan warna).
- Logo di kepala (di atas latar gelap): Tut Wuri, UTM, BLU, Unggul, | LPPM. Latar putih/kotak-kotak pada
  Tut Wuri, UTM, BLU harus dihapus dulu (flood-fill dari tepi, simpan salinan di scratchpad, jangan
  ubah file asli). Logo LPPM bertulisan putih → hanya cocok di latar gelap.
- Baris 5 kartu info: Skema, Dana, Tahun, TKT, Luaran.
- Kolom kiri: ① Tim Peneliti ② Kata Kunci ③ Ringkasan; kolom kanan: ④ Hasil Robot ⑤ Grafik Hasil.
  Lingkaran nomor kuning, garis putus-putus vertikal menghubungkan nomor (seperti contoh).
- Panel kecil "Metode yang dibandingkan" (A PID-Fuzzy Kd 0–300, B PID-Fuzzy 0–20, C PID tetap 9,25,
  D PID tetap tuning 15/0/300) + "Temuan simulasi".
- Pita bawah bergradasi: nama kampus, LPPM, dan **QR code ke https://github.com/asadbahauddin/robot-penyiram-tanaman-fuzzy-pid**.
- Font Google: Poppins (judul) + Inter (isi). Teks isi ≥ 19 px.
- **Tanpa foto tim** dan **tanpa kontak/email**.

## Aturan penting

1. **Jujur soal hasil.** Semua grafik adalah hasil **simulasi MATLAB**, bukan uji robot asli. Temuan yang
   boleh ditulis (rata-rata 5 run, skenario nominal):
   - rentang Kd 0–20 → 0–300: settling time 51,2 s → 1,4 s;
   - PID-Fuzzy menurunkan IAE 65 % dibanding PID tetap 9,25;
   - PID tetap yang disetel ulang (D) setara / sedikit lebih baik (IAE 66,7 vs 75,3).
   Jangan menulis klaim bahwa PID-Fuzzy lebih unggul dari PID tetap yang disetel ulang.
2. Jangan mengarang data, statistik, nama, atau logo. Data yang kurang → tanya pengguna atau tulis [ISI].
3. Jangan menggambar ulang logo resmi; pakai file dari folder `design/`.
4. **Jangan pernah commit `design/pass foto As'ad.jpg`** (foto pribadi, sudah di .gitignore) dan
   `design/referensi design.jpg`.
5. Setiap perubahan file di repo: commit kecil per perubahan, langsung `git push` ke `origin main`,
   **tanpa** baris "Co-Authored-By: Claude" (pengguna tidak mau Claude tercatat sebagai kontributor).
6. Gunakan bahasa Indonesia baku saat melapor ke pengguna; laporkan apa yang diubah dan apa yang
   masih perlu dicek (misalnya teks terpotong di kolom yang padat).

## Cara kerja

1. Baca `design/Ketentuan Isian Poster.docx` (teks + gambar di `word/media/`) dan file poster di artifact.
2. Ikuti petunjuk tipe Design (guide intent "design" → create_from_type hanya bila belum ada poster;
   kalau sudah ada, revisi artifact yang ada).
3. Unggah aset gambar dengan `upload_asset`, pakai URL `/_blob/...` di HTML.
4. Terbitkan/revisi, lalu beri pengguna tautan kanvas dan ringkasan perubahan.
