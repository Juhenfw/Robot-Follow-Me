import pygame
import time
import ddsm115 as motor
import glob
import os
import serial
from serial.tools import list_ports

class RobotController:
    def __init__(self):
        # Inisialisasi Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Kontrol Robot DDSM115 - Drift Mode")
        self.clock = pygame.time.Clock()
        
        # Kecepatan motor
        self.speed = 300  # Kecepatan Normal
        self.lambat = self.speed / 6  # Kecepatan Rotasi Pelan
        self.serong = self.speed / 4  # Kecepatan Belok Normal
        self.drift_boost = self.speed * 1.3  # Kecepatan Drift
        self.stop = 0
        
        # Status kontrol
        self.running = True
        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop
        
        # Inisialisasi motor
        self.motor1 = None
        self.motor2 = None
        self.connect_motors()
    
    def find_ddsm115_ports(self):
        """Menemukan port yang terhubung dengan motor DDSM115"""
        print("Mencari port DDSM115...")
        
        # Metode 1: Coba cari port berdasarkan ID (lebih disukai)
        ports = glob.glob("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*")
        if len(ports) >= 2:
            return ports[:2]  # Ambil 2 port pertama
            
        # Metode 2: Cari semua port serial yang tersedia
        available_ports = [port.device for port in list_ports.comports() 
                          if "USB" in port.description]
        
        if len(available_ports) >= 2:
            return available_ports[:2]
            
        # Metode 3: Cari port ACM atau USB generik
        tty_ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        
        if len(tty_ports) >= 2:
            return tty_ports[:2]
            
        print("PERINGATAN: Tidak dapat menemukan cukup port untuk motor!")
        return [], []
    
    def connect_motors(self):
        """Menghubungkan ke motor dan mencoba lagi jika gagal"""
        ports = self.find_ddsm115_ports()
        
        if len(ports) < 2:
            print("KESALAHAN: Tidak cukup port ditemukan untuk kedua motor.")
            return False
        
        try:
            print(f"Menghubungkan motor kanan ke {ports[0]}")
            self.motor1 = motor.MotorControl(device=ports[0])
            self.motor1.set_drive_mode(1, 2)
            
            print(f"Menghubungkan motor kiri ke {ports[1]}")
            self.motor2 = motor.MotorControl(device=ports[1])
            self.motor2.set_drive_mode(1, 2)
            
            # Tes motor untuk memastikan koneksi
            self.motor1.send_rpm(1, 0)
            self.motor2.send_rpm(1, 0)
            
            print("Kedua motor berhasil terhubung!")
            return True
        except Exception as e:
            print(f"KESALAHAN saat menghubungkan motor: {e}")
            # Coba tutup koneksi jika ada
            if self.motor1:
                try:
                    self.motor1.close()
                except:
                    pass
            if self.motor2:
                try:
                    self.motor2.close()
                except:
                    pass
            
            return False
    
    def safe_send_rpm(self, motor_obj, motor_id, speed):
        """Mengirim RPM dengan penanganan kesalahan"""
        try:
            motor_obj.send_rpm(motor_id, speed)
            return True
        except Exception as e:
            print(f"Kesalahan komunikasi motor: {e}")
            return False
    
    def reconnect_if_needed(self):
        """Memeriksa koneksi motor dan menghubungkan ulang jika perlu"""
        reconnect_needed = False
        
        # Periksa motor 1
        if self.motor1:
            try:
                # Coba kirim perintah kecepatan 0 sebagai tes koneksi
                self.motor1.send_rpm(1, 0)
            except:
                print("Motor kanan terputus, mencoba menghubungkan ulang...")
                reconnect_needed = True
        else:
            reconnect_needed = True
        
        # Periksa motor 2
        if self.motor2:
            try:
                # Coba kirim perintah kecepatan 0 sebagai tes koneksi
                self.motor2.send_rpm(1, 0)
            except:
                print("Motor kiri terputus, mencoba menghubungkan ulang...")
                reconnect_needed = True
        else:
            reconnect_needed = True
        
        # Reconnect jika diperlukan
        if reconnect_needed:
            # Coba tutup koneksi sebelumnya
            if self.motor1:
                try:
                    self.motor1.close()
                except:
                    pass
            if self.motor2:
                try:
                    self.motor2.close()
                except:
                    pass
            
            # Tunggu sebentar sebelum reconnect
            time.sleep(1)
            return self.connect_motors()
        
        return True
    
    def run(self):
        """Menjalankan loop utama kontrol robot"""
        retry_count = 0
        max_retries = 5
        
        while self.running:
            # Periksa koneksi motor secara berkala
            if retry_count >= 3:  # Setelah beberapa kesalahan berturut-turut
                if not self.reconnect_if_needed():
                    retry_count += 1
                    if retry_count > max_retries:
                        print("Terlalu banyak percobaan gagal, keluar...")
                        break
                    time.sleep(1)  # Tunggu sebelum mencoba lagi
                    continue
                else:
                    retry_count = 0
            
            # Proses event Pygame
            self.screen.fill((0, 0, 0))  # Bersihkan layar
            pygame.display.flip()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            keys = pygame.key.get_pressed()
            drifting = keys[pygame.K_LSHIFT]  # Drift mode aktif jika SHIFT ditekan
            
            # ======== Kombinasi Tombol untuk Belok & Drift ========
            if keys[pygame.K_w] and keys[pygame.K_a]:  # Maju + Belok Kiri
                if drifting:  # Mode Drift
                    self.current_speed_rwheel = -self.drift_boost
                    self.current_speed_lwheel = self.serong
                else:  # Mode Normal
                    self.current_speed_rwheel = -self.speed
                    self.current_speed_lwheel = self.serong
            elif keys[pygame.K_w] and keys[pygame.K_d]:  # Maju + Belok Kanan
                if drifting:
                    self.current_speed_rwheel = -self.serong
                    self.current_speed_lwheel = -self.drift_boost
                else:
                    self.current_speed_rwheel = -self.serong
                    self.current_speed_lwheel = self.speed
            elif keys[pygame.K_s] and keys[pygame.K_a]:  # Mundur + Belok Kiri
                if drifting:
                    self.current_speed_rwheel = self.speed
                    self.current_speed_lwheel = -self.drift_boost
                else:
                    self.current_speed_rwheel = self.speed
                    self.current_speed_lwheel = -self.serong
            elif keys[pygame.K_s] and keys[pygame.K_d]:  # Mundur + Belok Kanan
                if drifting:
                    self.current_speed_rwheel = self.drift_boost
                    self.current_speed_lwheel = -self.speed
                else:
                    self.current_speed_rwheel = self.serong
                    self.current_speed_lwheel = -self.speed
            
            # ======== Kontrol Gerakan Normal ========
            elif keys[pygame.K_w]:  # Maju
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = self.speed
            elif keys[pygame.K_s]:  # Mundur
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = -self.speed
            elif keys[pygame.K_a]:  # Rotasi kiri
                self.current_speed_rwheel = -self.lambat
                self.current_speed_lwheel = -self.lambat
            elif keys[pygame.K_d]:  # Rotasi kanan
                self.current_speed_rwheel = self.lambat
                self.current_speed_lwheel = self.lambat
            elif keys[pygame.K_SPACE]:  # ROTASI CEPAT
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = -self.speed
            elif keys[pygame.K_p]:  # Keluar
                self.running = False
            else:
                self.current_speed_rwheel = self.stop
                self.current_speed_lwheel = self.stop
            
            # Kirim perintah ke motor dengan penanganan kesalahan
            if self.motor1 and self.motor2:
                success1 = self.safe_send_rpm(self.motor1, 1, self.current_speed_rwheel)
                success2 = self.safe_send_rpm(self.motor2, 1, self.current_speed_lwheel)
                
                if not (success1 and success2):
                    retry_count += 1
                    print(f"Komunikasi motor gagal (upaya ke-{retry_count})")
                else:
                    retry_count = 0  # Reset counter jika berhasil
            else:
                print("Motor tidak terhubung!")
                retry_count += 1
            
            self.clock.tick(30)  # Batasi ke 30 FPS
    
    def cleanup(self):
        """Membersihkan dan menutup koneksi"""
        print("Menutup koneksi dan membersihkan...")
        if self.motor1:
            try:
                self.motor1.send_rpm(1, 0)  # Hentikan motor
                self.motor1.close()
            except:
                pass
        if self.motor2:
            try:
                self.motor2.send_rpm(1, 0)  # Hentikan motor
                self.motor2.close()
            except:
                pass
        pygame.quit()

if __name__ == "__main__":
    robot = RobotController()
    try:
        robot.run()
    finally:
        robot.cleanup()
