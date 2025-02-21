import serial
import math

port = "COM5"
baudrate = 115200
timeout = 1

L = 165  # Panjang robot
W = 50  # Lebar robot

anchors = {
    "A0": (-L / 2, W / 2),  # Depan kiri
    "A1": (-L / 2, -W / 2),  # Depan kanan
    "A2": (L / 2, 0),  # Belakang tengah
}


def calculate_tag_position(distances):
    weights = []
    x_tag, y_tag = 0, 0
    for i, (anchor, dist) in enumerate(zip(anchors.values(), distances)):
        x, y = anchor
        weight = 1 / dist if dist > 0 else 0
        weights.append(weight)
        x_tag += x * weight
        y_tag += y * weight
        total_weight = sum(weights)
        if total_weight > 0:
            x_tag /= total_weight
            y_tag /= total_weight
        return x_tag, y_tag


def calculate_distance_and_angle(x_tag, y_tag):
    distance = math.sqrt(x_tag**2 + y_tag**2)
    angle = math.degrees(math.atan2(y_tag, x_tag))
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
                        A0, A1, A2 = [val * 100 for val in processed_values]
                        x_tag, y_tag = calculate_tag_position([A0, A1, A2])
                        distance, angle = calculate_distance_and_angle(x_tag, y_tag)
                        print(
                            f"A0 = {A0:.2f} centimeter | A1 = {A1:.2f} centimeter | A2 = {A2:.2f} centimeter | T0 to Robot = {distance:.2f} cm | Sudut = {angle:.2f}°"
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
