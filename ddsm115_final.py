import pygame
import time
import ddsm115 as motor
import os
import serial

class RobotController:
    def __init__(self):
        # Port tetap yang ditentukan
        self.r_wheel_port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
        self.l_wheel_port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"
        
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
        
        # Status error dan reconnect
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        self.last_error_time = 0
        self.error_timeout = 2  # Detik
    
    def verify_port_exists(self, port):
        """Memverifikasi apakah port tersedia"""
        return os.path.exists(port)
    
    def connect_motors(self):
        """Menghubungkan ke motor dengan port tetap"""
        # Cek ketersediaan port roda kanan
        if self.verify_port_exists(self.r_wheel_port):
            try:
                print(f"Menghubungkan motor kanan ke {self.r_wheel_port}")
                self.motor1 = motor.MotorControl(device=self.r_wheel_port)
                self.motor1.set_drive_mode(1, 2)
                print("Motor kanan berhasil terhubung")
            except Exception as e:
                print(f"KESALAHAN saat menghubungkan motor kanan: {e}")
                self.motor1 = None
        else:
            print(f"PERINGATAN: Port motor kanan tidak ditemukan: {self.r_wheel_port}")
            self.motor1 = None
        
        # Cek ketersediaan port roda kiri
        if self.verify_port_exists(self.l_wheel_port):
            try:
                print(f"Menghubungkan motor kiri ke {self.l_wheel_port}")
                self.motor2 = motor.MotorControl(device=self.l_wheel_port)
                self.motor2.set_drive_mode(1, 2)
                print("Motor kiri berhasil terhubung")
            except Exception as e:
                print(f"KESALAHAN saat menghubungkan motor kiri: {e}")
                self.motor2 = None
        else:
            print(f"PERINGATAN: Port motor kiri tidak ditemukan: {self.l_wheel_port}")
            self.motor2 = None
        
        # Uji koneksi untuk memastikan berfungsi
        if self.motor1:
            try:
                self.motor1.send_rpm(1, 0)
            except:
                print("Pengujian motor kanan gagal")
                self.motor1 = None
        
        if self.motor2:
            try:
                self.motor2.send_rpm(1, 0)
            except:
                print("Pengujian motor kiri gagal")
                self.motor2 = None
        
        # Periksa status koneksi
        if self.motor1 and self.motor2:
            print("Kedua motor berhasil terhubung!")
            return True
        elif self.motor1 or self.motor2:
            print("PERINGATAN: Hanya satu motor yang terhubung")
            return True
        else:
            print("KESALAHAN: Kedua motor gagal terhubung")
            return False
    
    def safe_send_rpm(self, motor_obj, motor_id, speed):
        """Mengirim RPM dengan penanganan kesalahan"""
        if not motor_obj:
            return False
            
        try:
            motor_obj.send_rpm(motor_id, speed)
            return True
        except Exception as e:
            current_time = time.time()
            # Catat error
            if current_time - self.last_error_time > self.error_timeout:
                print(f"Kesalahan komunikasi motor: {e}")
                self.last_error_time = current_time
            return False
    
    def reconnect_if_needed(self):
        """Mencoba menghubungkan ulang jika ada masalah"""
        # Cek apakah perlu reconnect
        need_reconnect = False
        
        # Tes motor kanan jika terhubung
        if self.motor1:
            try:
                self.motor1.send_rpm(1, 0)
            except:
                need_reconnect = True
                print("Motor kanan terputus, perlu menghubungkan ulang")
        else:
            need_reconnect = True
        
        # Tes motor kiri jika terhubung
        if self.motor2:
            try:
                self.motor2.send_rpm(1, 0)
            except:
                need_reconnect = True
                print("Motor kiri terputus, perlu menghubungkan ulang")
        else:
            need_reconnect = True
        
        # Lakukan reconnect jika diperlukan
        if need_reconnect:
            # Tutup koneksi yang ada
            if self.motor1:
                try:
                    self.motor1.close()
                except:
                    pass
                self.motor1 = None
                
            if self.motor2:
                try:
                    self.motor2.close()
                except:
                    pass
                self.motor2 = None
            
            # Tunggu sebentar sebelum reconnect
            time.sleep(1)
            
            # Coba menghubungkan ulang
            return self.connect_motors()
        
        return True
    
    def run(self):
        """Menjalankan loop utama kontrol robot"""
        self.consecutive_errors = 0
        
        while self.running:
            # Cek untuk reconnect jika terlalu banyak error berturut-turut
            if self.consecutive_errors >= 3:
                print(f"Terdeteksi {self.consecutive_errors} error berturut-turut, mencoba menghubungkan ulang...")
                if self.reconnect_if_needed():
                    self.consecutive_errors = 0
                else:
                    self.consecutive_errors += 1
                    if self.consecutive_errors > self.max_consecutive_errors:
                        print("Terlalu banyak kegagalan koneksi berturut-turut, keluar...")
                        break
                    time.sleep(1)  # Tunggu sebentar sebelum coba lagi
            
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
            success1 = self.safe_send_rpm(self.motor1, 1, self.current_speed_rwheel)
            success2 = self.safe_send_rpm(self.motor2, 1, self.current_speed_lwheel)
            
            # Periksa status error
            if not (success1 and success2):
                self.consecutive_errors += 1
                # Jangan tampilkan pesan jika sudah terlalu banyak
                if self.consecutive_errors <= 3:
                    print(f"Komunikasi motor gagal (upaya ke-{self.consecutive_errors})")
            else:
                self.consecutive_errors = 0  # Reset counter jika berhasil
            
            self.clock.tick(30)  # Batasi ke 30 FPS
            
            # Tambahkan penundaan kecil untuk mengurangi beban CPU
            time.sleep(0.01)
    
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
