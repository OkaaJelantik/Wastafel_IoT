# IOT_SPEC.md: Telemetry & Komunikasi

Dokumen ini merinci strategi komunikasi IoT, struktur data, dan perilaku operasional untuk sistem Wastafel IoT.

## 1. Protokol Konektivitas & Telemetri
*   **Protokol**: MQTT melalui Wi-Fi.

## 2. Kerangka Teoretis: Level Air sebagai Single Source of Truth (SSOT)
Dalam arsitektur Wastafel IoT, sensor Level Air berfungsi sebagai referensi status yang definitif. Pilihan ini didasarkan pada stabilitas fisiknya dibandingkan dengan pengukuran aliran transien.

*   **Logika Validasi**: Meskipun aliran masuk dan keluar dihitung berdasarkan kejadian sensor (misalnya: pulsa flow meter), ini pada dasarnya rentan terhadap kesalahan akumulasi dari waktu ke waktu. Dengan menggunakan level air sebagai SSOT, sistem secara berkala merekonsiliasi volume yang dihitung:
    *   `V_dihitung = V_sebelumnya + (V_inflow - V_outflow)`
    *   `V_diamati = f(Sensor_level)`
    *   Jika `|V_dihitung - V_diamati| > Ambang_Batas`, sistem memicu kalibrasi ulang terhadap level yang diamati, secara efektif mengoreksi pergeseran (*drift*) dalam telemetri volume.

## 3. Struktur Data: Payload MQTT
Semua data telemetri dipublikasikan melalui MQTT dengan struktur JSON berikut:

```json
{
  "timestamp": "ISO8601",
  "level": float,    // cm
  "volume": float,   // Liter
  "delta": float     // Perubahan volume sejak pembacaan terakhir
}
```

*   **Pentingnya Konteks Level**: Setiap titik data, baik dipicu oleh kejadian aliran masuk atau aliran keluar, WAJIB menyertakan level saat ini. Ini memastikan bahwa setiap paket telemetri lengkap secara konteks, memungkinkan backend merekonstruksi status fisik basin yang tepat pada saat kejadian tanpa memerlukan ketergantungan sekuensial pada pesan sebelumnya.

## 4. Resiliensi Operasional: Buffering Offline & Sinkronisasi
Untuk memastikan integritas data selama pemadaman jaringan:

*   **Offline Buffer**: Sistem menggunakan *circular buffer* lokal untuk menyimpan paket telemetri saat koneksi Wi-Fi tidak tersedia.
*   **Strategi Sinkronisasi**: Setelah membangun kembali koneksi, sistem melakukan "sinkronisasi burst", mentransmisikan paket yang diantrekan secara kronologis. Backend mengidentifikasi pesan yang di-buffer berdasarkan `timestamp` mereka dan memprosesnya untuk mengisi celah dalam data historis, memastikan pemantauan yang konsisten bahkan setelah konektivitas terputus-putus.
