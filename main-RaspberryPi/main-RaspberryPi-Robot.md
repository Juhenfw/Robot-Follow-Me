# ROS 2 Jazzy on Follow Me Robot

## Persiapan Lingkungan

1. Mengatur Direktori Workspace
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

3. Clone Repositori (Contoh menggunakan repo rplidar):
   ```bash
   git clone https://github.com/Slamtec/sllidar_ros2.git
   ```

5. Periksa Isi Direktori:
   ```bash
   ls -la
   ```

## Membangun ROS 2 Package

1. Melakukan Build:
   ```bash
   cd ~/ros2_ws/
   colcon build --symlink-install
   ```

3. Periksa Kembali:
   ```bash
   ls -la
   ```

5. Compile:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

7. Menjalankan LIDAR:
   ```bash
   ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py
   ```

## Cek Port dan Izin Akses

1. Cek Port USB Terhubung:
   ```bash
   ls -la /dev | grep USB
   ```

3. Membuka Permission:
   - Untuk membuka izin akses ke port:
     ```bash
     sudo chmod 777 /dev/ttyUSB0  # Ubah sesuai port yang digunakan
     ```
   - Atau bisa menggunakan:
     ```bash
     sudo chmod 666 /dev/ttyUSB0  # Ubah sesuai port yang digunakan
     ```

## Konfigurasi Port USB
```bash
/dev/ttyLIDAR -> ttyUSB0
/dev/ttyRS485-2 -> ttyUSB1
/dev/ttyRS485-1 -> ttyUSB2
/dev/gamepad_logitech -> input/event6
```

## Fiksasi Port USB

1. Menambahkan Aturan USB:
   ```bash
   sudo nano /etc/udev/rules.d/99-custom-usb.rules
   ```

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

3. Simpan dan Terapkan Aturan:
   - Tekan Ctrl+O, Enter, kemudian Ctrl+X untuk keluar dari editor.
   - Terapkan aturan:
     ```bash
     sudo udevadm control --reload-rules && sudo udevadm trigger
     ```

## Mencari Device USB

### Metode 1: Menggunakan udevadm

1. Hubungkan perangkat FTDI ke port USB.
2. Identifikasi nama perangkat yang terhubung:
   ```bash
   ls -l /dev/ttyUSB*
   ```

4. Dapatkan atribut detail untuk perangkat tertentu:
   Ganti X dengan nomor yang sesuai:
   ```bash
   udevadm info -a -n /dev/ttyUSBX
   ```

6. Periksa atribut spesifik dalam output:
   ```bash
   ATTRS{busnum}=="X"
   ATTRS{devpath}=="X.X"
   ```

## Cek Joystick

Untuk memeriksa input dari joystick:
```bash
jstest /dev/input/js0
```

## Izin Akses pada Script

1. Berikan izin eksekusi pada file controller:
   ```bash
   chmod +x ~/ros2_ws/src/robot_control_system/robot_control_system/keyboard_mode_controller.py
   chmod +x ~/ros2_ws/src/robot_control_system/robot_control_system/simple_keyboard_controller.py
   ```

## Membangun Ulang Package

1. Membangun ulang package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_control_system
   ```

3. Sumberkan setup:
   ```bash
   source install/setup.bash
   ```

## Kontrol Roda DDSM 115

| Arah   | Kiri (L) | Kanan (R) |
|--------|----------|-----------|
| Maju   | L+       | R-        |
| Mundur | L-       | R+        |
| Kiri   | L-       | R-        |
| Kanan  | L+       | R+        |

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
