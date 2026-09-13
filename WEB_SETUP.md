# Web Setup Hydroflow

1. Tekan MENU pada LCD, pilih WEB SETUP dengan UP/DOWN, lalu OK.
2. Hubungkan HP ke hotspot Hydroflow-XXXXXX. Password acak muncul di LCD dan berubah setiap sesi.
3. Jika HP menampilkan "tidak ada internet", tetap gunakan jaringan tersebut. Buka http://192.168.4.1 secara manual (belum ada captive portal otomatis).
4. Isi endpoint HTTP lengkap, interval 15–3600 detik (default 30), dan Ethernet DHCP atau IP statis.
5. Simpan & Terapkan menyimpan satu record konfigurasi ke NVS. Hotspot ditutup setelah 2 detik dan jaringan diterapkan setelah request HTTP yang sedang berjalan selesai. Tidak perlu reboot.

MENU menutup hotspot tanpa menyimpan form. Hotspot juga berhenti setelah 5 menit tanpa permintaan halaman. Pengaturan menu LCD yang belum di-SAVE tetap perlu disimpan terpisah.

Token HTTP dan password MQTT yang kosong mempertahankan nilai lama; gunakan checkbox hapus untuk mengosongkannya. Nilai rahasia tidak dikirim kembali ke browser. HTTP memakai Authorization: Bearer jika token diisi.

Endpoint saat ini hanya mendukung http:// dengan hostname atau IPv4 dan port opsional. HTTPS ditolak karena transport telemetry saat ini EthernetClient tanpa TLS. Device ID masih mengikuti DEVICE_SN pada firmware. Topic yang disiapkan: devices/<DEVICE_SN>/telemetry.

Pengaturan MQTT tersimpan untuk implementasi berikutnya, belum menjalankan koneksi/publish/subscribe. TLS CA belum bisa dikonfigurasi dari halaman ini.

Halaman tersedia hanya melalui hotspot sementara. Semua aset tertanam dalam firmware sehingga tidak memerlukan internet atau upload filesystem terpisah. Wi-Fi bukan jalur telemetry.

Verifikasi di board: cek hotspot dan navigasi LCD, simpan lalu power-cycle untuk memastikan persistensi, uji URL/IP invalid (harus ditolak), uji POST menuju server baru, uji DHCP/IP statis dan cabut-pasang LAN, lalu cek timeout hotspot. Build saja belum membuktikan perilaku hardware ini.

Catatan waktu eksekusi: timer hotspot memakai millis tanpa delay. WebServer bawaan tetap sinkron saat membaca request; klien lambat dapat menahan loop sesuai timeout library. DNS/connect/DHCP Ethernet juga masih sinkron seperti sebelumnya. Respons Modbus selama akses web perlu diuji di board sebelum penggunaan produksi yang mensyaratkan batas latensi ketat.
