import socket

def start_tcp_server(host='192.168.1.110', port=10001):
    # Socket oluştur
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Socketi bir IP adresi ve port numarasına bağla
    server_socket.bind((host, port))

    # Bağlantıları dinle
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")

    while True:
        # Bağlantı kabul et
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address}")

        while True:
            # Veriyi oku
            data = client_socket.recv(1024)
            if not data:
                break
            print(f"Received data: {data.decode()}")

        # Bağlantıyı kapat
        client_socket.close()

if __name__ == "__main__":
    start_tcp_server()