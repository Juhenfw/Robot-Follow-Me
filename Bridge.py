import serial

import math



port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

baudrate = 115200

timeout = 1



L = 65  # Panjang robot

W = 35  # Lebar robot



anchors = {

    "A0": (-L / 2, W / 2),  # Depan kiri

    "A1": (-L / 2, -W / 2),  # Depan kanan

    "A2": (L / 2, 0),  # Belakang tengah

}



def calculate_tag_position(distances):

    weights = []

    x_tag, y_tag = 0, 0

    total_weight = 0  # Perlu variabel total_weight untuk normalisasi

    for i, (anchor, dist) in enumerate(zip(anchors.values(), distances)):

        if dist <= 0:  # Pastikan jarak valid (tidak negatif atau nol)

            continue

        x, y = anchor

        weight = 1 / dist  # Bobot invers jarak

        weights.append(weight)

        x_tag += x * weight

        y_tag += y * weight

        total_weight += weight  # Penambahan ke total_weight

    if total_weight > 0:

        x_tag /= total_weight

        y_tag /= total_weight

    return x_tag, y_tag





def calculate_distance_and_angle(x_tag, y_tag):

    distance = math.sqrt(x_tag**2 + y_tag**2)

    angle = math.degrees(math.atan2(y_tag, x_tag))  # Menghitung sudut

    return distance, angle



try:

    ser = serial.Serial(port, baudrate, timeout=timeout)

    while True:

        if ser.in_waiting > 0:

            data = ser.readline().decode("utf-8").strip()

            if data.startswith("$KT0"):

                try:

                    parts = data.split(",")

                    if len(parts) >= 4:

                        raw_values = parts[1:4]

                        processed_values = []

                        for value in raw_values:

                            if value.lower() == "null":

                                processed_values.append(0.0)

                            else:

                                processed_values.append(float(value))

                        

                        # Skala jarak sensor menjadi satuan sentimeter

                        A0, A1, A2 = [val * 100 for val in processed_values]

                        

                        # Debugging: Tampilkan jarak yang diterima

                        print(f"A0: {A0} cm | A1: {A1} cm | A2: {A2} cm")



                        # Hitung posisi tag berdasarkan jarak

                        x_tag, y_tag = calculate_tag_position([A0, A1, A2])

                        

                        # Hitung jarak dan sudut

                        distance, angle = calculate_distance_and_angle(x_tag, y_tag)



                        # Debugging: Tampilkan hasil perhitungan

                        print(f"Posisi Tag: ({x_tag:.2f}, {y_tag:.2f})")

                        print(

                            f"A0 = {A0:.2f} cm | A1 = {A1:.2f} cm | A2 = {A2:.2f} cm | T0 to Robot = {distance:.2f} cm | Sudut = {angle:.2f}ï¿½"

                        )

                    else:

                        print("Error: Data tidak lengkap.")

                except ValueError as e:

                    print(f"Error processing data: {e}")

except serial.SerialException as e:

    print(f"Error: {e}")

finally:

    if "ser" in locals() and ser.is_open:

        ser.close()

        print("Serial connection closed.")
