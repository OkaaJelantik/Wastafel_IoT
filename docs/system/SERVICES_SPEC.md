# SERVICES_SPEC.md: Spesifikasi Layanan Sistem

Dokumen ini menggabungkan layanan pemrosesan data dan feedback antarmuka pengguna, memastikan sistem beroperasi dengan stabilitas tinggi dan menyediakan umpan balik yang relevan kepada pengguna.

---

## 1. Pemrosesan Data (Data Processing)

### 1.1 Filosofi Stabilitas Data
Sistem menggunakan `MovingAverage` untuk melakukan pemfilteran *low-pass* pada data sensor ultrasonik. Filosofi utama di sini adalah memisahkan data "eksak" (untuk perhitungan volume dan catatan basis data) dari data "mentah" (untuk *trigger* sistem real-time).

### 1.2 Kalkulasi Volume
*   **Mitigasi Derau Sensor**: Pembacaan HC-SR04 melalui pemfilteran untuk menghapus pencilan (*outliers*) yang disebabkan oleh riak permukaan dalam `STATE_STABILIZING`.
*   **Kalkulasi Volume**: Volume delta dihitung dengan membandingkan status stabil sebelum dan sesudah pemompaan, memastikan pengukuran yang akurat melalui perbandingan data yang terisolasi dari *noise* real-time.

---

## 2. Strategi Feedback Pengguna (Display Service)

### 2.1 Filosofi Antarmuka
Menyediakan feedback yang jelas dan informatif terkait perubahan status operasional tanpa membebani bus I2C secara berlebihan (strategi *flicker-prevention*).

### 2.2 Spesifikasi Antarmuka
*   **Perangkat Keras**: I2C LCD 1602.
*   **Indikator Status**: Menginformasikan status sistem secara real-time (misalnya: Memompa, Menstabilkan, Menghitung, Menampilkan Hasil).
*   **Strategi Redraw**: Layar hanya akan diperbarui jika data status atau informasi yang akan ditampilkan telah berubah, sehingga mencegah kedipan layar (*flicker*) yang mengganggu.
