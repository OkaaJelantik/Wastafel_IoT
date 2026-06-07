# HARDWARE.md: Pinout & Integrasi

Dokumen ini mendefinisikan persyaratan antarmuka fisik dan pertimbangan desain untuk proyek `Wastafel_RRQSahabatan`.

## 1. Pemetaan Pin (ESP32 DevKit V1)

| Periferal | Komponen | GPIO | Alokasi Rail |
| :--- | :--- | :--- | :--- |
| **Sensor Tangan** | HC-SR04 | TRIG: 32, ECHO: 33 | Kiri |
| **Sensor Air** | HC-SR04 | TRIG: 5, ECHO: 18 | Kiri |
| **Touch Pad** | Pemicu Isi Ulang | 4 | Kiri |
| **I2C LCD** | PCF8574 | SDA: 21, SCL: 22 | Kanan |
| **Relay** | Pompa Air | 26 (Active High) | Kanan |

## 2. Kerangka Teoretis: Penginderaan Ultrasonik & Mitigasi EMI
Sensor HC-SR04 beroperasi berdasarkan prinsip pengukuran waktu tempuh (*Time-of-Flight* / ToF). Sensor ini memancarkan burst ultrasonik pada 40 kHz, yang dipicu oleh pulsa logika tinggi pada pin TRIG, dan mengukur durasi pulsa balik pada pin ECHO untuk menghitung jarak ($d = \frac{v \cdot t}{2}$).

Dalam sistem ini, EMI (Electromagnetic Interference) menimbulkan risiko signifikan terhadap akurasi pengukuran. Relay, sebagai beban induktif, menghasilkan *spike back-EMF* yang substansial saat pensaklaran, yang dicirikan oleh $dI/dt$ yang tinggi. *Spike* ini dapat berpasangan secara kapasitif atau induktif ke jalur sinyal sensor yang sensitif (TRIG/ECHO), menimbulkan *jitter* atau pemicu palsu. Mempertahankan jarak fisik antara jalur sinyal sensor dan jalur pensaklaran relay induktif sangat penting untuk menjaga integritas sinyal.

## 3. Mitigasi Perangkat Keras: Rasional Tata Letak Pin
Penetapan pin dalam `pins.h` mencerminkan pendekatan isolasi tingkat perangkat keras yang strategis:
*   **Jalur Sensor (Rail Kiri):** GPIO 32, 33 (Sensor Tangan), GPIO 5, 18 (Sensor Air), dan GPIO 4 (Touch Pad) dikelompokkan untuk memisahkan sinyal input sensitif berarus rendah dari jalur kontrol yang lebih berderau (*noisy*).
*   **Jalur Aktuator/Bus (Rail Kanan):** GPIO 26 (Relay) dan GPIO 21, 22 (I2C) ditempatkan untuk meminimalkan kedekatan fisik antara kejadian pensaklaran arus tinggi (relay) dan jalur sinyal ultrasonik yang sensitif, secara langsung memitigasi *crosstalk* dengan memaksimalkan jarak antara sumber derau potensial dan korban pada tata letak PCB/breadboard.

## 4. Prosedur Debugging Operasional
Ketika terjadi anomali (misalnya: pengukuran jarak yang tidak menentu atau pemicu bayangan), ikuti prosedur debugging fisik berikut:
1.  **Isolasi:** Verifikasi stabilitas daya (pastikan HC-SR04 disuplai dengan sumber 5V yang bersih dan terpisah dari beban induktif relay jika memungkinkan).
2.  **Validasi:** Gunakan antarmuka serial (`ESP_LOG` / UART) untuk memantau durasi pulsa `ECHO` mentah. Varians yang tinggi dalam timing, bahkan ketika tidak ada objek, mengindikasikan interferensi derau listrik atau kalibrasi sensor yang salah.
3.  **Integritas Fisik:** Verifikasi semua koneksi ground rapat dan berbagi (*Common Ground*) untuk mencegah *ground loop*, yang merupakan penyebab umum instabilitas akibat EMI di lingkungan pensaklaran.
4.  **Verifikasi:** Manfaatkan osiloskop untuk mengamati sinyal pemicu dan respons gema untuk degradasi bentuk gelombang, secara khusus mencari *ringing* induktif yang bertepatan dengan perubahan status relay.
