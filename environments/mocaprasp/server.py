import socket
import numpy as np
import time

# Try to create client socket
try:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
    print(f"[INFO] Socket created successfully")

except socket.error as err:
    print(f"[ERROR] Socket creation failed with error: {err}")

server_ip = "0.0.0.0"
server_port = 8888
server_address = (server_ip, server_port)
server_socket.bind(server_address)

last_time = 0.0

print(f"[INFO] Receiving messages...")
try:
    while True:
        message_bytes, address = server_socket.recvfrom(1024)

        message = np.frombuffer(message_bytes, dtype=np.float64)

        curr_time = message[-1]

        try:
            print(f"{1/(curr_time - last_time):.2f}")
        except:
            pass

        blob_data = message[:-1].reshape(-1, 3)

        last_time = curr_time

        # print(blob_data)


except KeyboardInterrupt:
    pass
