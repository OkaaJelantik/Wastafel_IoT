# Wastafel_RRQSahabatan: Orkestra Sistem

## Ringkasan Eksekutif & Filosofi Sistem

"Wastafel_RRQSahabatan" bukan sekadar implementasi sensorik, melainkan perwujudan keandalan sistem tertanam (embedded systems) pada skala IoT. Filosofi kami berpijak pada integritas data di atas segalanya:

*   **Stabilitas Data**: Mengubah data sensor yang fluktuatif menjadi insight yang stabil dan presisi.
*   **Isolasi Thread**: Menjamin keamanan eksekusi sistem dengan memisahkan domain akuisisi data, pengolahan, dan antarmuka.
*   **Keandalan IoT**: Membangun konektivitas tangguh yang mampu bertahan dalam lingkungan dinamis dengan latensi minimal.

Seluruh komponen sistem diorkestrasi untuk memastikan setiap milidetik waktu eksekusi berkontribusi pada stabilitas dan akurasi fungsional.

---

## Indeks Utama

Berikut adalah direktori pusat untuk mendalami arsitektur dan spesifikasi sistem:

- [Arsitektur Alur Utama](./MAIN_FLOW.md): Inti operasional, state machine, dan manajemen siklus hidup sistem.
- [Hardware Abstraction Layer](./HARDWARE.md): Spesifikasi integrasi perangkat keras dan pinout.
- [Spesifikasi Konektivitas IoT](./IOT_SPEC.md): Protokol telemetri dan integrasi cloud.
- [Spesifikasi Layanan Sistem](./SERVICES_SPEC.md): Algoritma filtrasi data, kalkulasi, dan antarmuka feedback visual.

---

## Perjalanan Pengguna: Panduan Navigasi

Tergantung pada peran dan fokus Anda dalam proyek ini, berikut adalah titik masuk yang direkomendasikan:

### 1. Arsitek / Insinyur Perangkat Keras
Fokus pada integritas sinyal dan konektivitas perangkat.
*   **Prioritas**: [Hardware Abstraction Layer](./HARDWARE.md) -> [Arsitektur Alur Utama](./MAIN_FLOW.md)

### 2. Insinyur IoT / Jaringan
Fokus pada aliran data dari perangkat ke platform cloud.
*   **Prioritas**: [Spesifikasi Konektivitas IoT](./IOT_SPEC.md) -> [Logika Pemrosesan Data](./DATA_PROCESSING.md)

### 3. Pengembang Aplikasi / Sistem
Fokus pada alur kontrol, logika aplikasi, dan keandalan sistem secara keseluruhan.
*   **Prioritas**: [Arsitektur Alur Utama](./MAIN_FLOW.md) -> [Logika Pemrosesan Data](./DATA_PROCESSING.md)
