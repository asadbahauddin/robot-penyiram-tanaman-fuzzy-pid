// Membuat laporan Word simulasi & pengujian robot penyiram Fuzzy-PID dari file hasil simulasi.
const fs = require('fs');
const path = require('path');
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, ImageRun, AlignmentType,
  WidthType, ShadingType, BorderStyle, HeadingLevel, LevelFormat, PageBreak, Footer, PageNumber,
  TableOfContents,
} = require('docx');

const ROOT = 'C:/Asad/Kuliah/joki/MBKM BU VIVI';
const HASIL = path.join(ROOT, 'simulasi_matlab/hasil');
const PB = path.join(HASIL, 'perbandingan');
const OUT = path.join(ROOT, 'laporan/Laporan_Simulasi_Pengujian_Robot_Penyiram_FuzzyPID.docx');

// ---------- CSV ----------
function bacaCSV(f) {
  const t = fs.readFileSync(f, 'utf8').replace(/\r/g, '').trim().split('\n');
  const split = (s) => { const o = []; let c = '', q = false;
    for (const ch of s) { if (ch === '"') q = !q; else if (ch === ',' && !q) { o.push(c); c = ''; } else c += ch; }
    o.push(c); return o; };
  const h = split(t[0]);
  return t.slice(1).map((l) => { const v = split(l); const r = {}; h.forEach((k, i) => (r[k] = v[i])); return r; });
}
const n = (x) => Number(x);
const fmt = (x, d = 2) => (x === '' || x === undefined || isNaN(n(x))) ? '–' : n(x).toFixed(d).replace('.', ',');
const pct = (a, b) => ((a - b) / b * 100);
const fpct = (v) => (v > 0 ? '+' : '') + v.toFixed(0).replace('.', ',') + ' %';

const ringkas = bacaCSV(path.join(PB, 'metrik_ringkas.csv'));
const rata = bacaCSV(path.join(PB, 'rata_rata_semua_skenario.csv'));
const tunF = bacaCSV(path.join(PB, 'tuning_rentang_fuzzy.csv'));
const tunP = bacaCSV(path.join(PB, 'tuning_pid_tetap.csv'));
const fungsi = bacaCSV(path.join(HASIL, 'ringkasan_skenario.csv'));

const MET = ['PID-Fuzzy', 'PID-Fuzzy 0-20', 'PID tetap 9,25', 'PID tetap (15/0/300)'];
const LBL = {
  'PID-Fuzzy': 'A. PID-Fuzzy (Kd 0–300)',
  'PID-Fuzzy 0-20': 'B. PID-Fuzzy (rentang asli 0–20)',
  'PID tetap 9,25': 'C. PID tetap 9,25 (robot)',
  'PID tetap (15/0/300)': 'D. PID tetap hasil tuning',
};
const SK = [
  ['S1_geser_kiri', 'S1', 'Robot mulai 10 cm di kiri garis tengah, arah lurus', 'Respons terhadap posisi awal tidak di tengah'],
  ['S2_geser_kanan', 'S2', 'Robot mulai 10 cm di kanan garis tengah, arah lurus', 'Memeriksa kesimetrisan respons kiri–kanan'],
  ['S3_miring_kiri', 'S3', 'Robot mulai di tengah, menghadap 5° ke kiri', 'Respons terhadap kesalahan arah (heading)'],
  ['S4_miring_kanan', 'S4', 'Robot mulai di tengah, menghadap 5° ke kanan', 'Kesimetrisan respons terhadap arah'],
  ['S5_gabungan', 'S5', 'Mulai 10 cm di kiri dan menghadap 5° ke kanan', 'Gangguan posisi dan arah sekaligus'],
  ['S6_derau_dm556', 'S6', 'Seperti S1 + 15 % pembacaan sensor semu 50–56 cm (derau driver DM556)', 'Uji ketahanan terhadap gangguan sensor'],
  ['S7_lebih_cepat', 'S7', 'Seperti S1 + PWM jelajah 150 (± 40 cm/s, nominal 110)', 'Uji ketahanan terhadap perubahan kecepatan'],
];
const getR = (sk, m) => ringkas.find((r) => r.skenario === sk && r.metode === m);

// ---------- gaya ----------
const FONT = 'Times New Roman';
const W = 9072;                                   // lebar isi A4 dengan margin 2,5 cm (DXA)
const border = { style: BorderStyle.SINGLE, size: 4, color: '808080' };
const borders = { top: border, bottom: border, left: border, right: border };

const P = (text, opt = {}) => new Paragraph({
  alignment: opt.align ?? AlignmentType.JUSTIFIED, spacing: { before: 100, after: 120, line: 300 },
  children: (Array.isArray(text) ? text : [text]).map((t) => (typeof t === 'string' ? new TextRun(t) : t)),
  ...opt.p,
});
const B = (t) => new TextRun({ text: t, bold: true });
const I = (t) => new TextRun({ text: t, italics: true });
const H1 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun(t)] });
const H2 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun(t)] });
const bul = (t) => new Paragraph({ numbering: { reference: 'bullet', level: 0 }, spacing: { after: 60, line: 288 },
  alignment: AlignmentType.JUSTIFIED,
  children: (Array.isArray(t) ? t : [t]).map((x) => (typeof x === 'string' ? new TextRun(x) : x)) });
const num = (t, ref = 'angka') => new Paragraph({ numbering: { reference: ref, level: 0 }, spacing: { after: 60, line: 288 },
  alignment: AlignmentType.JUSTIFIED,
  children: (Array.isArray(t) ? t : [t]).map((x) => (typeof x === 'string' ? new TextRun(x) : x)) });

let nTabel = 0, nGambar = 0;
const capT = (t) => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 120, after: 80 }, keepNext: true,
  children: [B(`Tabel ${++nTabel}. `), new TextRun(t)] });
const capG = (t) => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 60, after: 200 },
  children: [B(`Gambar ${++nGambar}. `), new TextRun(t)] });

function tabel(header, rows, cw, opt = {}) {
  const tot = cw.reduce((a, b) => a + b, 0);
  const sel = (t, i, isH, shade) => new TableCell({
    borders, width: { size: cw[i], type: WidthType.DXA },
    shading: isH ? { fill: 'D9EAD3', type: ShadingType.CLEAR } : (shade ? { fill: 'F3F8F0', type: ShadingType.CLEAR } : undefined),
    margins: { top: 50, bottom: 50, left: 90, right: 90 },
    children: [new Paragraph({
      alignment: (i === 0 || (opt.kiri || []).includes(i)) ? AlignmentType.LEFT : AlignmentType.CENTER,
      children: (Array.isArray(t) ? t : [t]).map((x) => (typeof x === 'string'
        ? new TextRun({ text: x, bold: isH, size: opt.size ?? 20, font: FONT }) : x)),
    })],
  });
  return new Table({
    width: { size: tot, type: WidthType.DXA }, columnWidths: cw,
    rows: [new TableRow({ tableHeader: true, children: header.map((h, i) => sel(h, i, true)) }),
      ...rows.map((r, ri) => new TableRow({ cantSplit: true,
        children: r.map((c, i) => sel(c, i, false, (opt.sorot || []).includes(ri))) }))],
  });
}

function ukuranPNG(f) { const b = fs.readFileSync(f); return [b.readUInt32BE(16), b.readUInt32BE(20)]; }
function gambar(f, lebarCm, keterangan, wh) {
  const [w, h] = wh ?? ukuranPNG(f);
  const lw = Math.round(lebarCm / 2.54 * 96);
  return [new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 120 }, keepNext: true,
    children: [new ImageRun({ type: f.endsWith('.png') ? 'png' : 'jpg', data: fs.readFileSync(f),
      transformation: { width: lw, height: Math.round(lw * h / w) },
      altText: { title: keterangan, description: keterangan, name: path.basename(f) } })] }),
    capG(keterangan)];
}

// ---------- isi ----------
const isi = [];

// Halaman judul
isi.push(new Paragraph({ spacing: { before: 2400, after: 240 }, alignment: AlignmentType.CENTER,
  children: [new TextRun({ text: 'LAPORAN SIMULASI DAN PENGUJIAN', bold: true, size: 32, font: FONT })] }));
isi.push(new Paragraph({ spacing: { after: 480 }, alignment: AlignmentType.CENTER,
  children: [new TextRun({ text: 'Inovasi Teknologi Robot Penyiram Tanaman Otomatis Menggunakan Metode Fuzzy-PID',
    bold: true, size: 28, font: FONT })] }));
isi.push(P('Simulasi MATLAB R2024a: model robot Ackermann, lima sensor ultrasonik, logika firmware ESP32 v13, ' +
  'dan perbandingan PID-Fuzzy dengan PID tetap', { align: AlignmentType.CENTER }));
isi.push(new Paragraph({ spacing: { before: 1200 }, alignment: AlignmentType.CENTER, children: [new TextRun('Disusun oleh:')] }));
isi.push(P('[Nama Mahasiswa] – [NIM]', { align: AlignmentType.CENTER }));
isi.push(new Paragraph({ spacing: { before: 360 }, alignment: AlignmentType.CENTER, children: [new TextRun('Dosen Pembimbing:')] }));
isi.push(P('[Nama Dosen Pembimbing]', { align: AlignmentType.CENTER }));
isi.push(new Paragraph({ spacing: { before: 1200 }, alignment: AlignmentType.CENTER,
  children: [new TextRun('[Program Studi] – [Fakultas] – [Nama Kampus]')] }));
isi.push(P('2026', { align: AlignmentType.CENTER }));
isi.push(new Paragraph({ children: [new PageBreak()] }));

// Daftar isi
isi.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 240 },
  children: [new TextRun({ text: 'DAFTAR ISI', bold: true, size: 28, font: FONT })] }));
isi.push(new TableOfContents('Daftar Isi', { hyperlink: true, headingStyleRange: '1-2' }));
isi.push(P([I('Klik kanan pada daftar isi lalu pilih "Update Field" bila nomor halaman belum muncul.')],
  { align: AlignmentType.LEFT }));
isi.push(new Paragraph({ children: [new PageBreak()] }));

// 1. Pendahuluan
isi.push(H1('1. Pendahuluan'));
isi.push(P('Laporan ini menjelaskan simulasi dan pengujian sistem kendali robot penyiram tanaman otomatis. ' +
  'Robot berjalan di lorong di antara deretan tanaman, menjaga posisinya di tengah lorong dengan PID yang gain-nya ' +
  'diatur oleh fuzzy Mamdani, berbalik arah (U-turn) di ujung lorong, dan menyalakan pompa saat sensor mendeteksi tanaman.'));
isi.push(P('Tujuan simulasi:'));
isi.push(num('Menguji logika kendali firmware ESP32 (versi 13) pada model robot dan lintasan yang ukurannya diambil dari gambar SolidWorks (robot.dxf dan jalur.dxf).'));
isi.push(num('Menentukan rentang keluaran Kp, Ki, Kd fuzzy yang sesuai, sebagaimana diminta dosen pembimbing.'));
isi.push(num('Membandingkan kinerja PID-Fuzzy dengan PID tetap secara terukur dan adil pada skenario yang sama.'));
isi.push(P([B('Ruang lingkup. '), 'Seluruh hasil pada laporan ini adalah hasil simulasi. Data uji robot asli belum ' +
  'digunakan. Beberapa parameter fisik masih berupa asumsi dan ditandai pada Tabel 1.']));

// 2. Model simulasi
isi.push(H1('2. Model Simulasi'));
isi.push(H2('2.1 Robot'));
isi.push(P('Robot dimodelkan dengan model sepeda kinematik (Ackermann) dengan titik acuan di tengah poros roda belakang: ' +
  'ẋ = v cos θ, ẏ = v sin θ, θ̇ = v tan δ / L, dengan v kecepatan, θ arah hadap, δ sudut roda kemudi, dan L jarak sumbu roda. ' +
  'Persamaan diintegrasikan setiap 1 ms. Bentuk badan dan roda ikut dihitung saat memeriksa tabrakan dengan dinding.'));
isi.push(...gambar(path.join(ROOT, 'design/robot3.jpeg'), 12, 'Robot penyiram tanaman (tampak samping)', [1280, 720]));
isi.push(capT('Parameter robot pada simulasi'));
isi.push(tabel(['Parameter', 'Nilai', 'Sumber'], [
  ['Panjang × lebar badan', '70 × 45 cm (moncong depan 25 cm)', 'robot.dxf'],
  ['Jarak sumbu roda (L)', '50 cm', 'Ukur'],
  ['Roda', 'Ø 20 cm, tebal 5 cm', 'Ukur'],
  ['Jarak roda belakang / depan', '55 cm / 40 cm', 'Ukur'],
  ['Letak poros belakang', '10 cm dari ekor robot', 'Asumsi'],
  ['Kecepatan pada PWM 80 / 110 / 140', '20 / 30 / 35 cm/s, respons orde-1 τ = 0,3 s', 'Asumsi'],
  ['Kemudi', 'Stepper DM556, 10 langkah/derajat, 700 µs/langkah, maks. 45°', 'Firmware'],
], [2900, 4372, 1800], { kiri: [1] }));

isi.push(H2('2.2 Sensor'));
isi.push(P('Robot memakai lima sensor ultrasonik. Jarak dihitung dengan ray-casting dalam kerucut ±15° (jarak ' +
  'terdekat diambil), batas 400 cm (di luar itu bernilai 999 seperti fungsi getDist() firmware), dan ditambah derau ' +
  'Gaussian σ = 0,5 cm. Sensor dibaca bergiliran seperti firmware: tiap sensor membutuhkan waktu gema (maks. 25 ms) ' +
  'ditambah jeda 35 ms, sehingga satu siklus kendali (Ts) rata-rata ± 0,20–0,22 s.'));
isi.push(capT('Posisi dan fungsi sensor (diukur dari tengah poros belakang)'));
isi.push(tabel(['Sensor', 'Posisi (maju, ke kiri)', 'Arah', 'Fungsi'], [
  ['Depan', '60,0 cm ; 0 cm', '0°', 'Mendeteksi dinding/rintangan, keputusan belok'],
  ['Samping kiri', '11,0 cm ; 22,5 cm', '+90°', 'Navigasi: jarak ke tepi kiri'],
  ['Samping kanan', '11,0 cm ; −22,5 cm', '−90°', 'Navigasi: jarak ke tepi kanan'],
  ['Kiri', '38,5 cm ; 17,5 cm', '+51,4°', 'Mendeteksi tanaman (pompa)'],
  ['Kanan', '38,5 cm ; −17,5 cm', '−55,5°', 'Mendeteksi tanaman (pompa)'],
], [1700, 2400, 1100, 3872], { kiri: [3] }));

isi.push(H2('2.3 Lintasan'));
isi.push(P('Lintasan asli dibaca langsung dari jalur.dxf: tiga lorong sejajar sepanjang 381,8 cm dengan lebar ' +
  '± 97,6 cm, dipisahkan sekat/bedengan setebal 5 cm, dengan dua celah U-turn selebar 99 cm di ujung kanan dan kiri. ' +
  'Untuk perbandingan metode kendali dipakai lorong lurus sepanjang 12 m dengan lebar yang sama (97,6 cm), agar ' +
  'respons sempat mencapai keadaan tunak.'));

isi.push(H2('2.4 Algoritma kendali'));
isi.push(P('Setiap siklus sensor, firmware menghitung:'));
isi.push(bul(['Setpoint di tengah lorong: sp = (s_kiri + s_kanan)/2, dan error e = sp − s_kiri (cm). ' +
  'Nilai e positif berarti robot terlalu dekat sisi kiri.']));
isi.push(bul('Perubahan error per sampel: Δe = e(k) − e(k−1).'));
isi.push(bul('Fuzzy Mamdani menghasilkan Kp, Ki, Kd dari (e, Δe) (Bagian 3).'));
isi.push(bul('PID diskrit: u = Kp·e + Ki·I + Kd·Δe, dengan I = Σ e·Ts dibatasi ±0,5 dan di-reset saat tanda e berganti; u dibatasi ±100.'));
isi.push(bul('Sudut kemudi: sudut = trunc(sat(−0,08·u ; ±10°)), perubahan maksimum 4° per siklus, dan kemudi lurus bila |e| < 2 cm (zona mati).'));
isi.push(bul('Pompa menyala bila sensor kiri atau kanan membaca < 40 cm.'));
isi.push(P('Mesin keadaan firmware v13 mengatur gerak di ujung lorong:'));
isi.push(capT('Keadaan (status) robot pada firmware v13'));
isi.push(tabel(['Kode', 'Keadaan', 'Perilaku'], [
  ['J', 'JELAJAH', 'Depan > 150 cm: PWM 110, kemudi PID-Fuzzy'],
  ['L', 'MELAMBAT', 'Depan 100–150 cm: PWM 80, kemudi PID-Fuzzy'],
  ['S', 'LIHAT', 'Depan < 100 cm: berhenti, membaca sensor saat stepper dimatikan'],
  ['C', 'CARI CELAH', 'Belum ada sisi terbuka: maju pelan sampai celah terlihat (baru di v13)'],
  ['M', 'MUNDUR', 'Mundur (lurus sebelum belok penuh, atau 3 titik bila tersangkut)'],
  ['K / R', 'BELOK KIRI / KANAN', 'Kemudi 45° sampai depan lapang dan satu sisi melihat dinding'],
  ['B', 'BERHENTI', 'Terjebak / batas waktu manuver'],
], [900, 2300, 5872], { kiri: [2] }));

// 3. Fuzzy
isi.push(H1('3. Desain Fuzzy Mamdani'));
isi.push(P('Struktur fuzzy mengikuti rancangan dosen pembimbing: input e dan Δe masing-masing tujuh fungsi keanggotaan ' +
  'segitiga (NB, NM, NS, Z, PS, PM, PB) pada semesta [−6, 6] cm dengan puncak setiap 2 cm. Output Kp, Ki, Kd ' +
  'masing-masing sembilan fungsi keanggotaan segitiga (ZE, VS, S, MS, M, MB, B, VB, EVB) yang berjarak sama. ' +
  'Inferensi memakai metode min–max dengan 49 aturan per output, dan defuzzifikasi memakai COA titik pusat. ' +
  'Fungsi fuzzy simulasi dicocokkan dengan hitungan tangan pada tujuh pasangan (e, Δe), dan semuanya sama persis.'));
isi.push(...gambar(path.join(HASIL, 'mf_gabungan.png'), 15.5, 'Fungsi keanggotaan input (e, Δe) dan output Kp, Ki, Kd'));
isi.push(...gambar(path.join(HASIL, 'rule_Kp.png'), 9, 'Rule base Kp (baris = error, kolom = delta error)'));
isi.push(...gambar(path.join(HASIL, 'rule_Ki.png'), 9, 'Rule base Ki'));
isi.push(...gambar(path.join(HASIL, 'rule_Kd.png'), 9, 'Rule base Kd'));

isi.push(H2('3.1 Penentuan rentang output Kp, Ki, Kd'));
isi.push(P('Dosen meminta rentang output 0–20 diperbaiki bila kurang sesuai. Rentang setiap gain dicari lewat simulasi: ' +
  'batas atas Kp, Ki, dan Kd diuji pada sebuah grid nilai (sembilan kategori dan rule base tidak diubah). ' +
  'Kriteria J = rata-rata IAE pada skenario S1 dan S3, ditambah penalti 100 bila robot tidak tenang. Semakin kecil J, semakin baik.'));
const baseF = tunF.find((r) => r.Kp_maks === '20' && r.Ki_maks === '20' && r.Kd_maks === '20');
isi.push(capT('Hasil pencarian rentang output fuzzy, grid awal (lima terbaik dan rentang asli)'));
const rowsF = tunF.slice(0, 5).map((r) => [`0–${r.Kp_maks}`, `0–${r.Ki_maks}`, `0–${r.Kd_maks}`, fmt(r.J, 1),
  fmt(r.ts_S1, 2), fmt(r.ts_S3, 2), fmt(r.os_S1, 2)]);
if (baseF) rowsF.push([`0–20 (asli)`, '0–20', '0–20', fmt(baseF.J, 1), fmt(baseF.ts_S1, 2), fmt(baseF.ts_S3, 2), fmt(baseF.os_S1, 2)]);
isi.push(tabel(['Kp', 'Ki', 'Kd', 'J', 'Settling S1 (s)', 'Settling S3 (s)', 'Overshoot S1 (cm)'],
  rowsF, [1250, 1100, 1100, 1100, 1500, 1500, 1522], { sorot: [rowsF.length - 1] }));
isi.push(P('Kd ternyata parameter yang menentukan. Δe dihitung per sampel (± 0,5 cm per siklus), sehingga dengan ' +
  'Kd ≤ 20 suku D terlalu kecil untuk meredam robot dan robot berayun terus. Karena hasil terbaik grid awal ' +
  'berada di batas grid (Kd 0–150), grid diperluas ke Kd 200–400 dengan Ki 0–20 (Tabel 5).'));
isi.push(capT('Grid tambahan rentang Kd (Ki 0–20)'));
const gridTambah = [  // keluaran run grid tambahan (IAE S1/S3, settling S1/S3, overshoot S1)
  ['0–10', '0–200', 87.5, 73.4, 3.6, 5.5, 3.5], ['0–10', '0–300', 82.0, 69.0, 4.2, 0.0, 2.5],
  ['0–10', '0–400', 84.6, 65.7, 4.9, 0.0, 2.3], ['0–20', '0–200', 92.0, 73.1, 2.6, 0.0, 5.0],
  ['0–20', '0–300', 82.7, 68.3, 2.7, 0.0, 3.8], ['0–20', '0–400', 75.8, 64.0, 3.0, 0.0, 2.6]];
isi.push(tabel(['Kp', 'Kd', 'J', 'IAE S1', 'IAE S3', 'Settling S1 (s)', 'Settling S3 (s)', 'Overshoot S1 (cm)'],
  gridTambah.map((r) => [r[0], r[1], fmt((r[2] + r[3]) / 2, 1), fmt(r[2], 1), fmt(r[3], 1), fmt(r[4], 1), fmt(r[5], 1), fmt(r[6], 1)]),
  [900, 1000, 900, 1000, 1000, 1400, 1400, 1472], { sorot: [4] }));
isi.push(P('Rentang yang dipilih adalah Kp 0–20 dan Ki 0–20 (tetap seperti rancangan dosen) serta Kd 0–300 ' +
  '(titik pusat setiap 37,5). Dibanding rentang asli, J turun dari ' + fmt(baseF ? baseF.J : NaN, 1) + ' menjadi 75,5 dan ' +
  'settling time S1 dari ' + fmt(baseF ? baseF.ts_S1 : NaN, 1) + ' s menjadi 2,7 s. Kd 0–400 masih sedikit lebih baik ' +
  '(J 69,9), tetapi Kd yang makin besar makin memperkuat derau sensor pada suku D. Karena itu Kd 0–300 dipilih sebagai ' +
  'kompromi. Pengaruh Kd di atas 300 terhadap derau belum diuji lebih jauh. Rentang ini juga diterapkan pada firmware v13.'));
isi.push(...gambar(path.join(HASIL, 'surface_Kd.png'), 11, 'Permukaan fuzzy Kd terhadap e dan Δe dengan rentang Kd 0–300'));

// 4. Skenario
isi.push(H1('4. Skenario Pengujian'));
isi.push(H2('4.1 Metode yang dibandingkan'));
const pTun = tunP[0];
isi.push(capT('Empat konfigurasi kendali yang dibandingkan'));
isi.push(tabel(['Metode', 'Gain / rentang', 'Keterangan'], [
  [LBL['PID-Fuzzy'], 'Kp 0–20, Ki 0–20, Kd 0–300', 'Usulan (dipakai pada firmware v13)'],
  [LBL['PID-Fuzzy 0-20'], 'Kp, Ki, Kd 0–20', 'Rentang asli rancangan dosen'],
  [LBL['PID tetap 9,25'], 'Kp = Ki = Kd = 9,25', 'Gain tuning manual pada robot asli'],
  [LBL['PID tetap (15/0/300)'], `Kp = ${pTun.Kp}, Ki = ${pTun.Ki}, Kd = ${pTun.Kd}`,
    'Gain terbaik dari grid 72 kombinasi, prosedur dan kriteria sama dengan metode A'],
], [3300, 2600, 3172], { kiri: [1, 2] }));
isi.push(P('Metode A dan D disetel dengan prosedur dan kriteria yang sama. Karena itu, perbandingan A dengan D ' +
  'adalah perbandingan yang adil antara PID-Fuzzy dan PID tetap. Metode B dan C menunjukkan kondisi sebelum penyetelan.'));

isi.push(H2('4.2 Skenario'));
isi.push(P('Setiap skenario dijalankan di lorong lurus 12 m selebar 97,6 cm. S1–S5 adalah kondisi nominal (sama ' +
  'dengan kondisi saat penyetelan). S6–S7 adalah uji ketahanan: kondisi diubah tanpa menyetel ulang gain atau rentang.'));
isi.push(capT('Skenario pengujian'));
isi.push(tabel(['Kode', 'Kondisi', 'Tujuan'], SK.map((s) => [s[1], s[2], s[3]]), [900, 4772, 3400], { kiri: [1, 2] }));

isi.push(H2('4.3 Indikator kinerja'));
isi.push(P('Simpangan y adalah jarak titik sensor samping robot dari garis tengah lorong (cm), yang diukur dari posisi ' +
  'sebenarnya robot di simulasi. Indikator dihitung hanya saat robot berjalan lurus (status JELAJAH atau MELAMBAT).'));
isi.push(capT('Indikator kinerja'));
isi.push(tabel(['Indikator', 'Definisi'], [
  ['Overshoot (cm)', 'Simpangan terjauh di seberang garis tengah setelah robot pertama kali melewatinya'],
  ['Settling time (s)', 'Waktu sampai |y| masuk dan tetap di dalam pita ±5 cm (± 5 % lebar lorong)'],
  ['Steady-state error (cm)', 'Rata-rata |y| pada 5 detik terakhir'],
  ['IAE (cm·s)', 'Integral |y| terhadap waktu, yaitu besar simpangan total'],
  ['ISE (cm²·s)', 'Integral y² terhadap waktu, memberi bobot besar pada simpangan besar'],
  ['RMSE (cm)', 'Akar rata-rata kuadrat simpangan'],
  ['Run tak tenang', 'Jumlah run yang tidak pernah tetap di pita ±5 cm sampai akhir lorong'],
], [2400, 6672], { kiri: [1] }));

isi.push(H2('4.4 Prosedur'));
isi.push(num('Setiap kombinasi metode × skenario dijalankan dengan 5 benih derau sensor yang berbeda (total 140 run).', 'prosedur'));
isi.push(num('Robot mulai dari kondisi awal skenario dan berjalan sampai ujung lorong uji (± 40–60 s).', 'prosedur'));
isi.push(num('Indikator dihitung tiap run, lalu dilaporkan sebagai rata-rata dan simpangan baku dari 5 run.', 'prosedur'));
isi.push(num('Pengujian fungsional juga dijalankan pada lintasan lain (lurus, belokan, rintangan) dan lintasan asli jalur.dxf.', 'prosedur'));

// 5. Hasil
isi.push(H1('5. Hasil Pengujian'));
isi.push(H2('5.1 Hasil per skenario'));
isi.push(P('Angka pada tabel berikut adalah rata-rata 5 run. Angka dalam kurung adalah simpangan baku.'));
for (const s of SK) {
  isi.push(capT(`Hasil skenario ${s[1]}: ${s[2]}`));
  isi.push(tabel(['Metode', 'Overshoot (cm)', 'Settling (s)', 'Ess (cm)', 'IAE (cm·s)', 'RMSE (cm)'],
    MET.map((m) => { const r = getR(s[0], m);
      const ts = r.tidak_tenang !== '0' ? `${fmt(r.settling_s)} [${r.tidak_tenang}/5 tak tenang]` : `${fmt(r.settling_s)} (${fmt(r.settling_s_sd)})`;
      return [LBL[m], `${fmt(r.overshoot_cm)} (${fmt(r.overshoot_cm_sd)})`, ts,
        `${fmt(r.ess_cm)} (${fmt(r.ess_cm_sd)})`, `${fmt(r.IAE, 1)} (${fmt(r.IAE_sd, 1)})`, `${fmt(r.RMSE_cm)} (${fmt(r.RMSE_cm_sd)})`]; }),
    [2872, 1250, 1500, 1150, 1200, 1100], { size: 18 }));
}
isi.push(P([I('Catatan: "tak tenang" berarti pada sebagian run robot tidak pernah tetap di pita ±5 cm. Rata-rata ' +
  'settling time hanya dihitung dari run yang tenang.')]));

isi.push(H2('5.2 Rata-rata seluruh skenario'));
const g = (k, m) => rata.find((r) => r.kelompok.startsWith(k) && r.metode === m);
isi.push(capT('Rata-rata indikator per kelompok skenario'));
const rowsR = [];
for (const k of ['nominal', 'ketahanan']) for (const m of MET) { const r = g(k, m);
  rowsR.push([k === 'nominal' ? 'Nominal (S1–S5)' : 'Ketahanan (S6–S7)', LBL[m], fmt(r.overshoot_cm), fmt(r.settling_s),
    `${r.run_tak_tenang}/${k === 'nominal' ? 25 : 10}`, fmt(r.ess_cm), fmt(r.IAE, 1), fmt(r.RMSE_cm)]); }
isi.push(tabel(['Kelompok', 'Metode', 'Overshoot (cm)', 'Settling (s)', 'Tak tenang', 'Ess (cm)', 'IAE', 'RMSE (cm)'],
  rowsR, [1450, 2422, 1000, 900, 900, 850, 750, 800], { size: 18, kiri: [1], sorot: [0, 4] }));

const A = g('nominal', 'PID-Fuzzy'), C = g('nominal', 'PID tetap 9,25'), D = g('nominal', 'PID tetap (15/0/300)');
const Ak = g('ketahanan', 'PID-Fuzzy'), Dk = g('ketahanan', 'PID tetap (15/0/300)');
isi.push(capT('Selisih metode A (PID-Fuzzy) terhadap pembanding, kondisi nominal'));
isi.push(tabel(['Indikator', 'A terhadap C (PID tetap 9,25)', 'A terhadap D (PID tetap tuning)'], [
  ['IAE', fpct(pct(n(A.IAE), n(C.IAE))), fpct(pct(n(A.IAE), n(D.IAE)))],
  ['RMSE', fpct(pct(n(A.RMSE_cm), n(C.RMSE_cm))), fpct(pct(n(A.RMSE_cm), n(D.RMSE_cm)))],
  ['Overshoot', fpct(pct(n(A.overshoot_cm), n(C.overshoot_cm))), fpct(pct(n(A.overshoot_cm), n(D.overshoot_cm)))],
  ['Steady-state error', fpct(pct(n(A.ess_cm), n(C.ess_cm))), fpct(pct(n(A.ess_cm), n(D.ess_cm)))],
  ['Run tak tenang', `${A.run_tak_tenang} vs ${C.run_tak_tenang} dari 25`, `${A.run_tak_tenang} vs ${D.run_tak_tenang} dari 25`],
], [2600, 3236, 3236]));
isi.push(P([I('Nilai negatif berarti PID-Fuzzy lebih kecil (lebih baik) daripada pembanding.')]));

isi.push(H2('5.3 Grafik hasil'));
isi.push(...gambar(path.join(PB, 'ringkasan_batang.png'), 16, 'Perbandingan IAE, RMSE, settling time, dan overshoot pada tujuh skenario (rata-rata 5 run, garis = simpangan baku)'));
isi.push(...gambar(path.join(PB, 'respons4_S1_geser_kiri.png'), 16, 'Posisi lateral keempat metode pada skenario S1 (benih 1)'));
isi.push(...gambar(path.join(PB, 'respons4_S3_miring_kiri.png'), 16, 'Posisi lateral keempat metode pada skenario S3 (benih 1)'));
isi.push(...gambar(path.join(PB, 'respons4_S6_derau_dm556.png'), 16, 'Posisi lateral keempat metode pada skenario S6, dengan gangguan sensor (benih 1)'));
isi.push(...gambar(path.join(PB, 'perbandingan_S1_geser_kiri.png'), 16, 'Detail metode A dan D pada S1: posisi lateral, error, sudut kemudi, dan keluaran PID'));

isi.push(H2('5.4 Pengujian fungsional dan lintasan asli'));
isi.push(P('Selain perbandingan metode, simulasi dijalankan pada beberapa bentuk lintasan untuk memeriksa mesin keadaan, ' +
  'penyiraman, dan manuver belok. Metode PID-Fuzzy memakai rentang Kd 0–300, dan PID tetap memakai gain robot (9,25).'));
const NAMA = { lurus: 'Lorong lurus, mulai di tengah', tidak_di_tengah: 'Lorong lurus, mulai bergeser 15 cm',
  miring: 'Lorong lurus, mulai miring 8°', belokan: 'Belok kanan 90° (lorong 150 cm)',
  belokan_sempit: 'Belok kanan 90° (lorong 100 cm)', rintangan: 'Lorong buntu (rintangan di depan)',
  jalur: 'Lintasan asli jalur.dxf (2 U-turn)' };
isi.push(capT('Hasil pengujian fungsional'));
isi.push(tabel(['Lintasan', 'Metode', 'Durasi (s)', 'Ts rata-rata (s)', 'Tanaman disiram', 'Tertahan (s)', 'Status dilalui'],
  fungsi.map((r) => [NAMA[r.skenario] ?? r.skenario, r.metode, fmt(r.durasi_s, 1), fmt(r.Ts_rata_s, 3),
    `${r.disiram}/${r.jml_tanaman}`, fmt(n(r.ms_tertahan) / 1000, 1), r.status_dilalui]),
  [2672, 1100, 900, 1000, 1100, 1000, 1300], { size: 18, kiri: [0] }));
isi.push(P('Pada lintasan asli, kedua metode menyelesaikan tiga lorong dan dua U-turn dalam ± 49 s tanpa tabrakan, dan ' +
  '16 dari 20 tanaman tersiram. Pada belokan 90° di lorong 100 cm dan pada lorong buntu, robot sempat tertahan ' +
  'dinding (kolom "Tertahan"). Kedua kasus ini adalah batas geometri: robot sepanjang 70 cm dengan radius putar ' +
  '± 50 cm tidak muat berbelok di ruang tersebut. Saat U-turn, metode kendali tidak berpengaruh karena kemudi ' +
  'diatur mesin keadaan, bukan PID.'));
isi.push(...gambar(path.join(HASIL, 'lintasan_jalur_fuzzy.png'), 15, 'Jejak robot PID-Fuzzy pada lintasan asli: tiga lorong, dua U-turn, tanaman biru = sudah disiram'));
isi.push(...gambar(path.join(HASIL, 'respons_jalur_fuzzy.png'), 16, 'Respons PID-Fuzzy selama satu putaran lintasan asli'));

// 6. Pembahasan
isi.push(H1('6. Pembahasan'));
isi.push(num([B('Rentang Kd menentukan redaman. '), `Dengan rentang asli 0–20, PID-Fuzzy (B) berayun hampir sebesar ` +
  `simpangan awalnya dan rata-rata settling time-nya ${fmt(g('nominal', 'PID-Fuzzy 0-20').settling_s, 1)} s. Dengan Kd 0–300 (A), ` +
  `settling time turun menjadi ${fmt(A.settling_s, 1)} s. Penyebabnya, Δe dihitung per sampel sehingga nilainya kecil; ` +
  'gain D harus jauh lebih besar dari Kp agar sebanding.'], 'bahas'));
isi.push(num([B('PID-Fuzzy jauh lebih baik daripada PID tetap 9,25 yang dipakai robot sekarang. '),
  `Pada kondisi nominal, IAE turun ${Math.abs(pct(n(A.IAE), n(C.IAE))).toFixed(0)} % dan RMSE turun ` +
  `${Math.abs(pct(n(A.RMSE_cm), n(C.RMSE_cm))).toFixed(0)} %. PID tetap 9,25 tidak tenang pada ${C.run_tak_tenang} dari 25 run, ` +
  `sedangkan PID-Fuzzy selalu tenang (${A.run_tak_tenang} dari 25).`], 'bahas'));
isi.push(num([B('Terhadap PID tetap yang disetel dengan cara yang sama, PID-Fuzzy belum lebih unggul. '),
  `Pada kondisi nominal, IAE PID-Fuzzy ${fpct(pct(n(A.IAE), n(D.IAE)))} dan overshoot ${fpct(pct(n(A.overshoot_cm), n(D.overshoot_cm)))} ` +
  `dibanding PID tetap hasil tuning (Kp ${pTun.Kp}, Ki ${pTun.Ki}, Kd ${pTun.Kd}). Pada uji ketahanan selisihnya serupa ` +
  `(IAE ${fpct(pct(n(Ak.IAE), n(Dk.IAE)))}). Settling time keduanya setara (${fmt(A.settling_s, 1)} s dan ${fmt(D.settling_s, 1)} s). ` +
  'Artinya, keunggulan PID-Fuzzy pada simulasi ini berasal dari penyetelan rentangnya, bukan dari penalaan adaptifnya.'], 'bahas'));
isi.push(num([B('Gain Ki kecil pengaruhnya. '), 'Integral dibatasi ±0,5 dan di-reset setiap error berganti tanda, ' +
  'sehingga suku I hampir tidak bekerja. PID tetap terbaik justru memakai Ki = 0.'], 'bahas'));
isi.push(num([B('Ayunan kecil ±2–3 cm tetap ada pada semua metode. '), 'Ayunan ini disebabkan zona mati 2 cm, sudut kemudi ' +
  'yang dibulatkan ke bawah ke derajat bulat, dan pembatas laju 4° per siklus pada firmware. Hal ini membatasi ' +
  'steady-state error kedua metode di ± 1,3 cm.'], 'bahas'));
isi.push(num([B('Letak sensor samping memengaruhi redaman. '), 'Sensor samping hanya 11 cm di depan poros belakang, ' +
  'sehingga hampir tidak "melihat" arah hadap robot. Peredaman harus datang dari suku D.'], 'bahas'));

// 7. Kesimpulan
isi.push(H1('7. Kesimpulan'));
isi.push(num('Simulasi berhasil meniru logika firmware dan geometri robot serta lintasan dari SolidWorks. Dengan firmware v13, robot menyelesaikan lintasan asli dengan dua U-turn tanpa tabrakan.', 'simpul'));
isi.push(num('Rentang output fuzzy yang sesuai adalah Kp 0–20, Ki 0–20, dan Kd 0–300. Rentang Kd asli (0–20) membuat robot berayun terus di lorong.', 'simpul'));
isi.push(num(`Dibanding PID tetap 9,25 yang dipakai robot sekarang, PID-Fuzzy menurunkan IAE ± ${Math.abs(pct(n(A.IAE), n(C.IAE))).toFixed(0)} % dan membuat robot selalu kembali ke tengah lorong.`, 'simpul'));
isi.push(num(`Bila PID tetap disetel dengan prosedur yang sama, kinerjanya setara atau sedikit lebih baik dari PID-Fuzzy (IAE ${fpct(pct(n(A.IAE), n(D.IAE)))} untuk PID-Fuzzy). Hasil ini perlu disebut jujur dalam laporan.`, 'simpul'));

// 8. Keterbatasan & saran
isi.push(H1('8. Keterbatasan dan Saran'));
isi.push(bul('Seluruh hasil berasal dari simulasi. Validasi perlu dilakukan dengan log robot asli (/log.csv) memakai skrip bandingkan_log.m.'));
isi.push(bul('Kecepatan robot pada tiap PWM, letak poros belakang, serta ukuran dan letak tanaman masih berupa asumsi.'));
isi.push(bul('Model belum memuat slip roda, kelembaman kemudi, pantulan ultrasonik yang kompleks, dan variasi permukaan tanah.'));
isi.push(bul('Rule base fuzzy belum disetel; hanya rentang outputnya. Penyetelan rule atau penambahan umpan maju arah (heading) dapat diuji pada simulasi yang sama.'));
isi.push(bul('Zona mati 2 cm dan pembulatan sudut kemudi dapat diperhalus agar ayunan kecil ±2–3 cm berkurang.'));

// Lampiran
isi.push(H1('Lampiran: Lokasi File'));
isi.push(P('Kode dan data tersedia di repositori github.com/asadbahauddin/robot-penyiram-tanaman-fuzzy-pid.'));
isi.push(tabel(['File', 'Isi'], [
  ['simulasi_matlab/main.m', 'Menjalankan semua skenario, grafik, video, dan perbandingan'],
  ['simulasi_matlab/perbandingan_metode.m', 'Pengujian 4 metode × 7 skenario × 5 benih'],
  ['simulasi_matlab/tuning_rentang.m, tuning_pid_tetap.m', 'Penentuan rentang fuzzy dan gain PID tetap'],
  ['simulasi_matlab/hasil/perbandingan/', 'Tabel metrik (CSV) dan grafik perbandingan'],
  ['simulasi_matlab/hasil/video_jalur_fuzzy.mp4', 'Video robot dari start sampai finish'],
  ['firmware/program_robot_final_v13/', 'Firmware ESP32 dengan logika dan rentang hasil simulasi'],
], [4000, 5072], { kiri: [1], size: 18 }));

// ---------- dokumen ----------
const doc = new Document({
  creator: 'Laporan simulasi', title: 'Laporan Simulasi dan Pengujian Robot Penyiram Fuzzy-PID',
  features: { updateFields: true },
  styles: {
    default: { document: { run: { font: FONT, size: 24 } } },
    paragraphStyles: [
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { font: FONT, size: 28, bold: true, color: '1F4D2B' },
        paragraph: { spacing: { before: 360, after: 160 }, keepNext: true, outlineLevel: 0 } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { font: FONT, size: 24, bold: true, color: '1F4D2B' },
        paragraph: { spacing: { before: 240, after: 120 }, keepNext: true, outlineLevel: 1 } },
    ],
  },
  numbering: { config: [
    { reference: 'bullet', levels: [{ level: 0, format: LevelFormat.BULLET, text: '•', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 540, hanging: 300 } } } }] },
    ...['angka', 'prosedur', 'bahas', 'simpul'].map((ref) => ({ reference: ref, levels: [{ level: 0,
      format: LevelFormat.DECIMAL, text: '%1.', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 540, hanging: 360 } } } }] })),
  ] },
  sections: [{
    properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1417, bottom: 1417, left: 1417, right: 1417 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER,
      children: [new TextRun({ children: [PageNumber.CURRENT], size: 20 })] })] }) },
    children: isi,
  }],
});
fs.mkdirSync(path.dirname(OUT), { recursive: true });
Packer.toBuffer(doc).then((b) => { fs.writeFileSync(OUT, b); console.log('ditulis:', OUT, b.length, 'byte'); });
