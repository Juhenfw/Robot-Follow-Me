import socket

# Konfigurasi Server
HOST = ''  # Biarkan kosong untuk menggunakan semua interface jaringan
PORT = 65432  # Port untuk menerima data

# Buat socket TCP/IP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Server siap, menunggu koneksi pada port {PORT}...")

conn, addr = server_socket.accept()  # Terima koneksi dari Raspberry Pi pengirim
print(f"Koneksi diterima dari {addr}")

try:
    while True:
        # Terima data dari sender
        data = conn.recv(1024).decode('utf-8')  # Terima hingga 1024 byte
        if not data:
            break
        print(f"Data diterima: {data}")

        # Proses data (contohnya mencetak data ke terminal)
        # Anda dapat mengganti bagian ini untuk mengontrol robot berdasarkan data
except KeyboardInterrupt:
    print("Server dihentikan.")
finally:
    conn.close()
    server_socket.close()
