import serial
import math

port = "/dev/ttyUSB0"
baudrate = 115200
timeout = 1

# Fungsi Koreksi Bias
def correct_bias(raw_distance, bias):
    return raw_distance - bias

# Fungsi untuk menghitung panjang d atau e menggunakan hukum kosinus
def sisi_segitiga(a, b, sudut):
    """
    Fungsi untuk menghitung panjang sisi menggunakan hukum kosinus
    a: panjang sisi pertama
    b: panjang sisi kedua
    sudut: sudut antara sisi a dan b dalam derajat
    """
    # Mengkonversi sudut dari derajat ke radian
    sudut_radian = math.radians(sudut)
    
    # Menghitung panjang sisi menggunakan hukum kosinus
    panjang_sisi = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(sudut_radian))
    
    return panjang_sisi

try:
    ser = serial.Serial(port, baudrate, timeout=timeout)
    print(f"Connected to {port} at {baudrate} baud.")

    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip()
            if data.startswith("$KT0"):
                try:
                    parts = data.split(",")
                    if len(parts) >= 4:
                        raw_values = parts[1:4]
                        processed_values = []
                        for i, value in enumerate(raw_values):
                            if value.lower() == "null":
                                processed_values.append(0.0)
                            else:
                                processed_values.append(float(value))
                        A0, A1, A2 = processed_values

                        cal_A0 = correct_bias(A0*100, 30)
                        cal_A1 = correct_bias(A1*100, 0)
                        cal_A2 = correct_bias(A2*100, 0)

                        A0_A1 = sisi_segitiga(cal_A0, cal_A1, 150)
                        A0_A2 = sisi_segitiga(cal_A0, cal_A2, 150)
                        
                        print(
                            f"\nA0 = {cal_A0:.2f} centimeter | A1 = {cal_A1:.2f} centimeter | A2 = {cal_A2:.2f} centimeter"
                        )
                        print(
                            f"T0 to A0 = {cal_A0:.2f} centimeter | A0 to A1 = {A0_A1:.2f} centimeter | A0 to A2 = {A0_A2:.2f} centimeter"
                        )
                    else:
                        print("Error: Data tidak lengkap.")
                except ValueError as e:
                    print(f"Error processing data: {e}")
except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")