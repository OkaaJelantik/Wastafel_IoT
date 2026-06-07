# Wastafel_RRQSahabatan — Smart Sink IoT

Proyek ini adalah sistem Wastafel Pintar berbasis IoT yang dikembangkan menggunakan framework **ESP-IDF v5.x** untuk ESP32. Proyek ini bertujuan untuk mengimplementasikan sistem kontrol keran otomatis dengan pemantauan volume air dan integrasi cloud, sebagai bagian dari mata kuliah Sensor & IoT.

## 📖 Dokumentasi Modular

Untuk pemahaman mendalam mengenai arsitektur dan desain sistem, silakan merujuk pada dokumen berikut:

*   **[Arsitektur Utama (ORCHESTRA.md)](docs/system/ORCHESTRA.md)**: Visi proyek, filosofi stabilitas data, dan struktur sistem secara keseluruhan.
*   **[Alur Logika (MAIN_FLOW.md)](docs/system/MAIN_FLOW.md)**: Definisi State Machine dan strategi isolasi threading untuk kalkulasi volume.
*   **[Hardware & Pinout (HARDWARE.md)](docs/system/HARDWARE.md)**: Pemetaan pin GPIO untuk sensor HC-SR04, LCD I2C, Relay Pompa, dan Touch Pad.

---

## 🚀 Panduan Setup ESP-IDF v5.x

Pastikan Anda menggunakan ESP-IDF versi 5.x untuk kompatibilitas penuh.

### 🐧 Linux / 🍎 macOS

1.  **Instalasi Dependensi**: Ikuti panduan resmi untuk [Linux](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-setup.html) atau [macOS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/macos-setup.html).
2.  **Clone ESP-IDF**:
    ```bash
    git clone --recursive https://github.com/espressif/esp-idf.git
    cd esp-idf
    ./install.sh esp32
    ```
3.  **Export Path**:
    ```bash
    . ./export.sh
    ```

### 🪟 Windows

1.  Sangat direkomendasikan menggunakan **[ESP-IDF Windows Installer](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html)**.
2.  Pilih versi v5.x selama proses instalasi.
3.  Installer akan secara otomatis mengatur toolchain dan environment variables.

---

## 💻 Integrasi VS Code

1.  Buka VS Code dan instal ekstensi **Espressif IDF**.
2.  Tekan `F1` dan ketik `ESP-IDF: Configure ESP-IDF Extension`.
3.  Pilih metode instalasi (Express atau Manual) dan arahkan ke lokasi ESP-IDF yang telah diinstal.
4.  Gunakan ikon di status bar bawah untuk Build, Flash, dan Monitor.

---

## ⚙️ Konfigurasi

Sebelum melakukan build, Anda mungkin perlu menyesuaikan konfigurasi proyek (seperti kredensial WiFi atau parameter sensor).

*   **CLI**: Jalankan `idf.py menuconfig`.
*   **VS Code**: Klik ikon roda gigi di status bar bawah untuk membuka GUI konfigurasi.

---

## 🛠️ Build & Flash

### Menggunakan CLI

1.  **Build Proyek**:
    ```bash
    idf.py build
    ```
2.  **Flash ke ESP32**:
    ```bash
    idf.py -p [PORT] flash
    ```
    *Ganti `[PORT]` dengan port perangkat Anda (misal: `/dev/ttyUSB0` atau `COM3`).*
3.  **Monitor Output**:
    ```bash
    idf.py -p [PORT] monitor
    ```

---

## 📁 Struktur Folder

*   `main/`: Berisi kode aplikasi utama dan file `Kconfig.projbuild`.
*   `components/`: Modul-modul modular yang dapat digunakan kembali:
    *   `data_processing`: Algoritma filter dan kalkulasi volume.
    *   `display_service`: Logika antarmuka LCD.
    *   `hardware_manager`: Manajemen driver perangkat keras.
    *   `hcsr04`: Driver untuk sensor ultrasonik.
    *   `i2c_lcd`: Driver untuk LCD via I2C.
    *   `iot_service`: Integrasi telemetri IoT.
*   `docs/system/`: Spesifikasi teknis dan detail operasional sistem.
*   `managed_components/`: Komponen yang dikelola oleh ESP-IDF Component Manager.
