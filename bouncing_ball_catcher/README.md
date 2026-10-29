# Bouncing Ball Catcher (ROS2 + OpenCV)

Game: beberapa bola virtual memantul di frame kamera. Pegang objek berwarna di atas sebuah bola
selama **2 detik tanpa putus** -> skor +1 dan bola respawn di posisi acak.

## Cara Menjalankan
```bash
# 1. Taruh package di workspace
mkdir -p ~/ros2_ws/src && cp -r bouncing_ball_catcher ~/ros2_ws/src/
cd ~/ros2_ws
sudo apt install python3-opencv        # jika belum ada
colcon build --packages-select bouncing_ball_catcher
source install/setup.bash

# 2. Jalankan (webcam 0)
ros2 launch bouncing_ball_catcher game.launch.py
# atau pakai file video
ros2 launch bouncing_ball_catcher game.launch.py source:=/path/ke/video.mp4

# 3. Lihat hasil
ros2 run rqt_image_view rqt_image_view     # pilih /game/debug_image
ros2 topic echo /game/score
rqt_graph                                   # untuk screenshot
```
Kalibrasi HSV (bonus): `ros2 run bouncing_ball_catcher hsv_calibrator`
(geser trackbar, tekan `p` untuk cetak nilai, lalu salin ke `config/params.yaml`).

Ubah parameter saat jalan: `ros2 param set /game_node hold_duration 3.0`
Reset game: `ros2 service call /game/reset std_srvs/srv/Trigger`

## Arsitektur
```
camera_node --/camera/image_raw--> game_node --/game/score (Int32)
                                            \--/game/debug_image (Image)
```
| Node | Fungsi |
|---|---|
| camera_node | Baca webcam/file video (loop), publish `sensor_msgs/Image` (bgr8) |
| game_node | Deteksi HSV, simulasi bola, logika timer, publish skor & debug image |
| hsv_calibrator | (bonus) Tool trackbar penentu rentang HSV |

**Parameter camera_node:** `source`, `fps`, `width`, `height`, `mirror`
**Parameter game_node:** `hsv_lower`, `hsv_upper`, `num_balls`, `hold_duration`, `ball_radius`,
`ball_speed`, `min_area`, `smoothing_alpha`, `speed_increase_per_score`
**Service:** `/game/reset` (`std_srvs/Trigger`)

## Asumsi & Keputusan Desain
- Objek default berwarna **hijau** (H 40-80); ubah di `config/params.yaml`.
- Hanya kontur **terbesar** yang dianggap objek pemain.
- Waktu dihitung dari selisih `header.stamp` antar frame (bukan jumlah frame). Selisih negatif
  atau > 0.5 s dianggap 0 agar timer tidak melompat.
- Bila centroid berada di beberapa bola yang tumpang tindih, hanya bola dengan jarak
  terdekat yang timernya berjalan; bola lain di-reset.
- Konversi Image <-> numpy dilakukan manual (tanpa `cv_bridge`) agar tidak bermasalah versi NumPy.
- Skor dipublikasikan saat awal dan setiap berubah.

## Kendala & Solusi
- *Jitter posisi objek* -> smoothing eksponensial (`smoothing_alpha`).
- *Noise mask* -> Gaussian blur + erode/dilate.
- *`-p source:=0` terbaca int* -> parameter `source` memakai dynamic typing lalu dikonversi ke string.
- (Isi kendala nyata yang kamu alami saat mencoba.)

## Bonus yang Dikerjakan
- Parameter dinamis (`ros2 param set`, efek langsung)
- Kalibrasi HSV (`hsv_calibrator`)
- Tracking halus (smoothing EMA)
- Bola makin cepat seiring skor (`speed_increase_per_score`)
- Service reset (`/game/reset`)
