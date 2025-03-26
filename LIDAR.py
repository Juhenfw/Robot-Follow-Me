import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rplidar import RPLidar
import tkinter as tk
from threading import Thread
import time

# Konfigurasi LiDAR dengan path tetap
PORT_NAME = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f235ae4105e1d247940e6441b646a0b3-if00-port0"
lidar = RPLidar(PORT_NAME, baudrate=256000)

# Inisialisasi Data
angles = []
distances = []
colors = []
running = False
thread = None

def update_lidar():
    global angles, distances, colors, running
    try:
        while running:
            for scan in lidar.iter_scans():
                if not running:
                    break  # Keluar dari loop jika LiDAR dihentikan
                
                angles.clear()
                distances.clear()
                colors.clear()
                
                for _, angle, distance in scan:
                    if distance <= 2000:  # Batasi hanya untuk jarak â‰¤ 5000 mm
                        angles.append(np.deg2rad(angle))  # Ubah ke radian
                        distances.append(distance)

                        # Tentukan warna berdasarkan jarak
                        if distance < 500:
                            colors.append('red')  # Objek dekat
                        elif distance < 1500:
                            colors.append('orange')  # Jarak menengah
                        else:
                            colors.append('purple')  # Objek jauh
        lidar.stop()
    except Exception as e:
        print(f"Error: {e}")


# Fungsi untuk menggambar data LiDAR
def animate(i):
    plt.cla()
    plt.title("LiDAR Real-Time Scan")
    for angle, distance, color in zip(angles, distances, colors):
        plt.polar(angle, distance, 'o', color=color, markersize=3)

# Fungsi untuk memulai LiDAR
def start_lidar():
    global running, thread
    if not running:
        running = True
        thread = Thread(target=update_lidar, daemon=True)
        thread.start()

# Fungsi untuk menghentikan LiDAR
def stop_lidar():
    global running
    if running:
        print("Menghentikan LiDAR...")
        running = False
        lidar.stop()

# Fungsi untuk menutup program dengan aman
def close_program():
    global running
    print("Menutup program...")
    running = False
    time.sleep(1)  # Tunggu sebentar agar thread berhenti
    lidar.stop()
    lidar.disconnect()
    plt.close('all')  # Pastikan Matplotlib tertutup
    root.quit()
    root.destroy()

# Buat GUI menggunakan Tkinter
root = tk.Tk()
root.title("LiDAR Scanner")

frame = tk.Frame(root)
frame.pack()

btn_start = tk.Button(frame, text="Start LiDAR", command=start_lidar)
btn_start.pack()

btn_stop = tk.Button(frame, text="Stop LiDAR", command=stop_lidar)
btn_stop.pack()

btn_quit = tk.Button(frame, text="Exit", command=close_program)
btn_quit.pack()

# Inisialisasi Matplotlib
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ani = animation.FuncAnimation(fig, animate, interval=500)

# Menampilkan GUI
plt.show(block=False)  # Agar Tkinter tidak terhambat oleh Matplotlib
root.mainloop()
