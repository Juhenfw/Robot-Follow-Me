# ROS 2 Jazzy on Follow Me Robot

## Persiapan Lingkungan

1. Mengatur Direktori Workspace
   source /opt/ros/jazzy/setup.bash
   cd ~
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

2. Clone Repositori (Contoh menggunakan repo rplidar):
   git clone https://github.com/Slamtec/sllidar_ros2.git

3. Periksa Isi Direktori:
   ls -la

## Membangun ROS 2 Package

1. Melakukan Build:
   cd ~/ros2_ws/
   colcon build --symlink-install

2. Periksa Kembali:
   ls -la

3. Compile:
   source ~/ros2_ws/install/setup.bash

4. Menjalankan LIDAR:
   ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py

## Cek Port dan Izin Akses

1. Cek Port USB Terhubung:
   ls -la /dev | grep USB

2. Membuka Permission:
   - Untuk membuka izin akses ke port:
     sudo chmod 777 /dev/ttyUSB0  # Ubah sesuai port yang digunakan
   - Atau bisa menggunakan:
     sudo chmod 666 /dev/ttyUSB0  # Ubah sesuai port yang digunakan

## Konfigurasi Port USB

- /dev/ttyLIDAR -> ttyUSB0
- /dev/ttyRS485-2 -> ttyUSB1
- /dev/ttyRS485-1 -> ttyUSB2
- /dev/gamepad_logitech -> input/event6

## Fiksasi Port USB

1. Menambahkan Aturan USB:
   sudo nano /etc/udev/rules.d/99-custom-usb.rules

   Isi dengan aturan berikut:
   ```bash
   # CP210x (LIDAR FT32 Serial UART)
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{busnum}=="2", ATTRS{devpath}=="1", SYMLINK+="ttyLIDAR"

   # FTDI USB Serial Device - Perangkat 1
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{busnum}=="2", ATTRS{devpath}=="2.1", SYMLINK+="ttyRS485-1"

   # FTDI USB Serial Device - Perangkat 2
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{busnum}=="2", ATTRS{devpath}=="2.2", SYMLINK+="ttyRS485-2"

   # Logitech F710 Gamepad
   SUBSYSTEM=="input", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21f", SYMLINK+="gamepad_logitech"
   ```

2. Simpan dan Terapkan Aturan:
   - Tekan Ctrl+O, Enter, kemudian Ctrl+X untuk keluar dari editor.
   - Terapkan aturan:
     sudo udevadm control --reload-rules && sudo udevadm trigger

## Mencari Device USB

### Metode 1: Menggunakan udevadm

1. Hubungkan perangkat FTDI ke port USB.
2. Identifikasi nama perangkat yang terhubung:
   ls -l /dev/ttyUSB*

3. Dapatkan atribut detail untuk perangkat tertentu:
   Ganti X dengan nomor yang sesuai:
   udevadm info -a -n /dev/ttyUSBX

4. Periksa atribut spesifik dalam output:
   ATTRS{busnum}=="X"
   ATTRS{devpath}=="X.X"

## Cek Joystick

Untuk memeriksa input dari joystick:
jstest /dev/input/js0

## Izin Akses pada Script

1. Berikan izin eksekusi pada file controller:
   chmod +x ~/ros2_ws/src/robot_control_system/robot_control_system/keyboard_mode_controller.py
   chmod +x ~/ros2_ws/src/robot_control_system/robot_control_system/simple_keyboard_controller.py

## Membangun Ulang Package

1. Membangun ulang package:
   cd ~/ros2_ws
   colcon build --packages-select robot_control_system

2. Sumberkan setup:
   source install/setup.bash

## Kontrol Roda DDSM 115

- Maju: L+, R-
- Mundur: L-, R+
- Kiri: L-, R-
- Kanan: L+, R+

## Struktur Direktori

### Struktur direktori untuk robot control system:
```bash
~/ros2_ws/
├── src/
│   ├── lidar_sensor/           # Paket untuk LIDAR
│   │   ├── lidar_sensor/       # Folder kode LIDAR
│   │   │   ├── __init__.py     # Inisialisasi paket
│   │   │   └── lidar_node.py   # Kode untuk mengontrol LIDAR
│   │   ├── launch/             # Folder untuk file launch
│   │   │   └── lidar_launch.py # File launch untuk node LIDAR
│   │   ├── setup.py            # Setup untuk paket LIDAR
│   │   └── package.xml         # Definisi paket LIDAR
│   │
│   ├── wheel_control/          # Paket untuk kontrol roda DDSM115
│   │   ├── wheel_control/      # Folder kode kontrol motor
│   │   │   ├── __init__.py     # Inisialisasi paket
│   │   │   └── motor_control.py# Kode untuk mengontrol motor
│   │   ├── launch/             # Folder untuk file launch
│   │   │   └── wheel_launch.py # File launch untuk node motor
│   │   ├── setup.py            # Setup untuk paket motor
│   │   └── package.xml         # Definisi paket motor
│   │
│   ├── uwb_sensor/             # Paket untuk sensor UWB HR-RTLS1
│   │   ├── uwb_sensor/         # Folder kode sensor UWB
│   │   │   ├── __init__.py     # Inisialisasi paket
│   │   │   └── uwb_node.py     # Kode untuk membaca data dari UWB
│   │   ├── launch/             # Folder untuk file launch
│   │   │   └── uwb_launch.py   # File launch untuk node UWB
│   │   ├── setup.py            # Setup untuk paket UWB
│   │   └── package.xml         # Definisi paket UWB
│   │
│   ├── gamepad_control/        # Paket untuk kontrol gamepad
│   │   ├── gamepad_control/    # Folder kode gamepad
│   │   │   ├── __init__.py     # Inisialisasi paket
│   │   │   └── gamepad_control.py# Kode untuk membaca input gamepad
│   │   ├── launch/             # Folder untuk file launch
│   │   │   └── gamepad_launch.py# File launch untuk node gamepad
│   │   ├── setup.py            # Setup untuk paket gamepad
│   │   └── package.xml         # Definisi paket gamepad
│   │
│   ├── robot_description/      # Deskripsi robot (URDF/XACRO)
│   │   ├── urdf/               # Folder URDF/XACRO
│   │   │   └── robot.urdf      # Deskripsi robot dalam format URDF
│   │   ├── launch/             # Folder untuk file launch
│   │   │   └── robot_state_publisher_launch.py # File launch untuk robot_state_publisher
│   │   ├── setup.py            # Setup untuk deskripsi robot
│   │   └── package.xml         # Definisi paket deskripsi robot
│
└── launch/
    └── robot_launch.py         # File Launch untuk meluncurkan semua node
```
