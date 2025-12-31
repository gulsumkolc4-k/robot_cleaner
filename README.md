## QR lı Otonom Temizlik Robotu

Bu proje, **Robotiğe Giriş** dersi final ödevi kapsamında geliştirilmiştir. Gazebo simülasyon ortamında **TurtleBot3 Waffle** robotunun; SLAM ile haritalama yapmasını, bu harita üzerinde otonom olarak odaları gezmesini, **QR kod ile oda doğrulaması** yapmasını ve belirlenen temizlik rotalarını izlemesini sağlar.


## Gereksinimler
- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11

## Özellikler
- SLAM + Harita Kaydı
- AMCL tabanlı navigasyon
- Çoklu oda ve waypoint desteği
- OpenCV + QR Code doğrulama
- Hata toleranslı görev yönetimi
- Detaylı görev raporu (txt + topic)

## Proje Dizin Yapısı

```
robot_cleaner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── gazebo_sim.launch
│   ├── slam_map.launch
│   ├── navigation.launch
│   └── task_manager.launch
├── maps/
│   ├── map.yaml
│   └── map.pgm
├── config/
│   └── mission.yaml
├── worlds/
│   └── empty.world
├── models/
│   ├── qr_salon
│   ├── qr_mutfak
│   ├── qr_yatak_odasi
│   ├── qr_koridor
│   └── qr_cocuk_odasi
├── reports/
│   └── clean_report.txt
└── src/
    ├── task_manager_node.py
    ├── qr_reader_node.py
    └── get_pose.py

```
## Kurulum
**Workspace Derleme**
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
**Gerekli paketlerin kurulumu**

```
sudo apt update
sudo apt install ros-noetic-turtlebot3 \
ros-noetic-turtlebot3-simulations \
ros-noetic-turtlebot3-navigation \
ros-noetic-turtlebot3-slam \
ros-noetic-cv-bridge \
ros-noetic-vision-opencv \
python3-opencv
```

## Çalıştırma sırası
**Ortam Değişkeni**
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle

#2.SLAM (Haritalama) ve Gazebo Ortamını açmak için
roslaunch robot_cleaner slam_map.launch

**Robotu klavye ile gezdirmek için**
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

**Kamerayı görmek için**
rqt_image_view

#3.Harita Kaydetme
rosrun map_server map_saver -f $(rospack find robot_cleaner)/maps/map

#5.Görev Yöneticisi (Temizlik Başlatma)
roslaunch robot_cleaner task_manager.launch \
mission:=$(rospack find robot_cleaner)/config/mission.yaml


## NOTLAR

**mission.yaml**
`config/mission.yaml` osyası şu bilgileri içerir:
Oda sırası
Oda giriş waypoint’i
Temizlik waypoint’leri
Beklenen QR içeriği

**RViz’de robot konumunu doğrudan x, y, yaw olarak almak için**
python3 ~/catkin_ws/src/robot_cleaner/src/get_pose.py

**Temizlik Raporu**
robot_cleaner/reports/clean_report.txt

**ayarlanabilir parametreler**
goal_timeout, room_timeout, qr_timeout, max_retries değerleri
task_manager.launch içinden ayarlanabilir.

**Rapor**
- `~/catkin_ws/robot_cleaner/reports/clean_report.txt` içine `room,status,detail,duration` formatında yazılır.


