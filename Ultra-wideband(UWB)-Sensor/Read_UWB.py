import serial

port = "COM3"
baudrate = 115200
timeout = 1

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
                        print(
                            f"A0 = {A0*100:.2f} centimeter | A1 = {A1*100:.2f} centimeter | A2 = {A2*100:.2f} centimeter"
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
