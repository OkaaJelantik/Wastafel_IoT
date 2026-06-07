# MAIN_FLOW.md: Logika & Threading

## 1. Seksi 1: Boot Sistem & Orkestrasi

### Filosofi Inisialisasi Sistem
Dalam arsitektur RTOS seperti pada Wastafel IoT, inisialisasi yang teratur dan deterministik adalah tulang punggung stabilitas. Kita menggunakan paradigma **NVS -> HW -> Services**.

1.  **NVS (Non-Volatile Storage)**: Harus diinisialisasi pertama kali. Sistem memerlukan akses ke konfigurasi persisten, parameter kalibrasi sensor, dan status terakhir (state recovery) sebelum hardware atau jaringan aktif. Kegagalan di sini berarti sistem tidak bisa menjamin integritas konfigurasi.
2.  **Hardware (HAL)**: Setelah parameter dimuat, hardware (HC-SR04, pompa, touch sensor) harus dipetakan dan dikonfigurasi. Ini adalah lapisan terendah yang berinteraksi langsung dengan dunia fisik.
3.  **Services**: Lapisan tertinggi, termasuk *DisplayService* dan *IoTService*. Layanan ini bergantung pada hardware untuk membaca data dan pada NVS untuk kredensial jaringan.

Urutan ini mencegah *race condition* di mana layanan mencoba mengakses periferal yang belum siap. Dalam RTOS, kegagalan inisialisasi pada salah satu komponen harus menyebabkan sistem berhenti (*fail-fast*) untuk menghindari operasi yang berbahaya (misal: pompa aktif tanpa kontrol sensor).

### Analisis Blok Kode: app_main & Inisialisasi State

Proses boot dimulai dari `app_main` [APP_MAIN]:

```cpp
// [APP_MAIN]
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);
```
Blok ini memastikan NVS dalam kondisi sehat. Jika terdapat partisi yang rusak, kita melakukan *erase* dan inisialisasi ulang. Penggunaan `ESP_ERROR_CHECK` memastikan jika inisialisasi NVS gagal total, CPU akan melakukan *panic* dan *reboot*, mencegah eksekusi kode dengan konfigurasi korup.

Setelah NVS siap:
```cpp
ESP_ERROR_CHECK(hw.init()); // Hardware Manager Inisialisasi
ESP_ERROR_CHECK(DisplayService::init());
```
Hardware diinisialisasi melalui `hardware_manager`. Jika sensor HC-SR04 tidak merespons atau pin mapping salah, `hw.init()` akan mengembalikan error, dan eksekusi berhenti di sini.

Inisialisasi layanan IoT [IOT_INIT]:
```cpp
// [IOT_INIT]
ESP_ERROR_CHECK(IoTService::init());
```
Inisialisasi `IoTService` menyiapkan stack TCP/IP dan koneksi MQTT/Blynk. Layanan ini diinisialisasi terakhir karena bergantung pada ketersediaan hardware (untuk data) dan konfigurasi yang mungkin dimuat dari NVS.

### Dampak Operasional Kegagalan Hardware
Kegagalan pada salah satu perangkat keras (misal: sensor jarak HC-SR04 rusak) akan memicu `ESP_ERROR_CHECK` pada `hw.init()`. Secara operasional:
*   Sistem tidak akan menjalankan task `sensor_task`, `control_task`, atau `iot_task`.
*   Sistem akan berhenti dalam kondisi *boot loop* atau berhenti total, mencegah perilaku sistem yang tidak terprediksi (misalnya: pompa yang tidak bisa dimatikan karena sensor tidak terbaca).
*   *Debug* menjadi lebih mudah karena *fail-fast* ini memberikan pesan error spesifik pada serial monitor, mengindikasikan komponen mana yang gagal diinisialisasi.

---

## 2. Seksi 2: Logika Sensor & Kontrol (Loop)

### Filosofi Sinkronisasi
Sistem Wastafel IoT mengandalkan loop kontrol yang responsif dan deterministik. Sinkronisasi pembacaan sensor pada frekuensi 10Hz (diimplementasikan via `vTaskDelayUntil` dalam `sensor_task`) adalah kunci stabilitas karena:

1.  **Konsistensi Data**: Menjamin *State Machine* dalam `control_task` menerima pembaruan nilai `s_water_distance_cm` yang teratur (setiap 100ms), mencegah *aliasing* atau *jitter* yang disebabkan oleh variasi timing pembacaan.
2.  **Responsivitas**: 10Hz adalah titik keseimbangan optimal antara latensi deteksi tangan dan beban CPU. Frekuensi yang lebih rendah membuat sistem terasa "lambat", sedangkan frekuensi yang lebih tinggi (tanpa perubahan logika) tidak memberikan manfaat tambahan karena sifat fisik sensor HC-SR04 yang memiliki keterbatasan kecepatan propagasi bunyi.

### Analisis Blok Kode: sensor_task & control_task

`sensor_task` bertindak sebagai *produsen data*:
```cpp
// [sensor_task]
float water_raw = hw.get_water_distance_cm();
if (water_raw > 0) {
    s_water_distance_cm = s_water_filter.add(water_raw);
    s_current_vol_ml = volume_from_distance(s_water_distance_cm);
}
```
Ia melakukan akuisisi data mentah dan menggunakan `s_water_filter` untuk memitigasi *noise* sensor sebelum data dikonsumsi oleh `control_task`.

`control_task` bertindak sebagai *pengambil keputusan* berbasis state:
*   **Transisi `[STATE_PUMPING]` ke `[STATE_STABILIZING]`**: Terjadi di `logic_handle_pumping()` ketika tangan tidak terdeteksi melebihi `HAND_OFF_TIMEOUT_MS`. Transisi ini menandakan akhir dari interaksi pengguna dan awal dari fase pembersihan data.
*   **Transisi `[STATE_STABILIZING]` ke `[STATE_CALCULATING]`**: Terjadi di `logic_handle_stabilizing()`. Sistem menunggu permukaan air stabil (`s_water_filter.is_stable()`) atau mencapai timeout (`STABILITY_MAX_MS`) sebelum melakukan kalkulasi volume final untuk menghindari pembacaan yang tidak akurat.

### Mitigasi Ripple Noise
Ripple noise (pantulan gelombang air) adalah tantangan utama pasca-pemompaan. Sebelum transisi ke `[STATE_CALCULATING]`, noise ini *wajib* dimitigasi.
*   **Dampak Transisi Terlalu Cepat**: Jika transisi terjadi saat air masih bergejolak, sensor akan membaca jarak yang berubah-ubah dengan cepat. Kalkulasi volume berbasis jarak tersebut akan menghasilkan data yang salah (misalnya: volume dispensi lebih besar dari yang sebenarnya karena pembacaan jarak sesaat yang lebih dekat akibat puncak gelombang).
*   **Strategi**: `is_stable(STABILITY_THRESHOLD_CM)` memastikan bahwa fluktuasi air berada di bawah ambang batas yang dapat diterima, sehingga kalkulasi di `[STATE_CALCULATING]` menggunakan data yang merepresentasikan volume air yang sebenarnya tenang.

---

## 3. Definisi State Machine

*   **[LABEL: STATE_IDLE]**: Operasi sistem normal, menunggu interaksi.
*   **[LABEL: STATE_PUMPING]**: Air sedang dikeluarkan. Snapshot volume tangki diambil pada awal status ini untuk memfasilitasi kalkulasi delta.
*   **[LABEL: STATE_STABILIZING]**: Fase pasca-pemompaan. Sistem menunggu riak permukaan air mereda sebelum melakukan kalkulasi volume akhir.
*   **[LABEL: STATE_CALCULATING]**: Fase komputasi. Diisolasi dari loop utama untuk mencegah *noise* mempengaruhi keputusan kritis.
*   **[LABEL: STATE_SHOW_RESULT]**: Fase pembaruan UI menampilkan volume yang dikeluarkan.
*   **[LABEL: STATE_REFILL]**: Menangani aliran air masuk, berbeda dari logika aliran keluar.

## 3. Seksi 3: Isolasi Kalkulasi & Threading

### Filosofi: Isolasi Kalkulasi
Kalkulasi volume (konversi sensor mentah ke ml) sensitif terhadap *noise* yang dihasilkan saat air bergerak atau saat pompa bekerja. Jika kalkulasi dilakukan dalam loop kontrol utama yang berjalan cepat (10Hz), nilai yang dihasilkan akan berfluktuasi tajam (*jitter*), menyebabkan pembacaan volume yang tidak akurat bagi pengguna dan sistem logging.

Memisahkan data ke dalam:
*   **Data Real-time (Loop Utama)**: Berfungsi sebagai *trigger* (deteksi tangan, status pompa). Membutuhkan responsivitas tinggi.
*   **Data Eksak (State [STATE_CALCULATING])**: Berfungsi untuk penagihan, telemetri, dan feedback pengguna. Membutuhkan akurasi tinggi dan bebas *noise*.

Dengan threading, kita memastikan kalkulasi berat atau *filtering* tambahan tidak memblokir loop utama yang mengelola logika real-time.

### Analisis Blok Kode: MovingAverage & Barrier
Kelas `MovingAverage` dalam `components/data_processing/include/data_processing.h` mengimplementasikan *low-pass filter* menggunakan *circular buffer* dengan ukuran `DATA_MA_WINDOW` (default: 10 sampel).

```cpp
// [MovingAverage::add]
_sum -= _buffer[_index]; // Hapus nilai terlama
_buffer[_index] = value; // Tambah nilai baru
_sum += value;
// ... hitung rata-rata
```

`[STATE_CALCULATING]` bertindak sebagai *data barrier*:
1.  Setelah pompa berhenti, sistem masuk ke `[STATE_STABILIZING]`.
2.  `is_stable()` menggunakan `get_spread()` untuk memastikan fluktuasi air (delta max-min) dalam window `DATA_MA_WINDOW` di bawah ambang batas tertentu.
3.  Hanya setelah stabil, `[STATE_CALCULATING]` dieksekusi. Ini mencegah pembacaan noise (akibat riak) terkirim sebagai hasil akhir, karena *state machine* memblokir kalkulasi hingga data valid tersedia.

### Analisis Operasional: `DATA_MA_WINDOW`
Konstanta `DATA_MA_WINDOW` (saat ini 10) adalah *trade-off* fundamental:
*   **Nilai Kecil**: Waktu stabilisasi lebih cepat (responsif terhadap perubahan), namun rentan terhadap *noise* (akurasi rendah).
*   **Nilai Besar**: Waktu stabilisasi lebih lambat (terasa "laggy" dalam pembaruan data), namun memberikan keakuratan/penghalusan yang sangat baik (sangat efektif membuang noise sensor).

Untuk aplikasi Wastafel IoT, 10 adalah titik yang dipilih untuk menyeimbangkan latensi yang dirasakan pengguna dan ketepatan perhitungan volume setelah dispensasi air.

## 4. Batasan Kritis
*   **Aturan Keamanan A**: Pastikan pelacakan pompa tetap berlanjut meskipun gerakan terdeteksi oleh sensor.
*   **Aturan Keamanan B**: Terapkan batas dasar keras (*hard floor limits*) untuk ketinggian air guna mencegah kerusakan pompa.
