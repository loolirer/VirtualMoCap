import socket
import numpy as np
import time

# Try to create client socket
try:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
    server_ip = "0.0.0.0"
    server_port = 25565
    server_address = (server_ip, server_port)
    server_socket.bind(server_address)

    print(f"[INFO] Socket created successfully")

except socket.error as err:
    print(f"[ERROR] Socket creation failed with error: {err}")

client_addresses = []


for ID in range(4):
    try:
        client_ip = socket.gethostbyname(f"mocaprasp-client-{ID}.local")
        client_port = 25565
        client_address = (client_ip, client_port)
        print(f"[INFO] Client {ID} found at: {client_address}...")
        client_addresses.append(client_address)

    except Exception as e:
        print(e)


print("[INFO] Sending capture request...")
delay = 5
capture_time = 15
message = np.array([delay, capture_time]).astype(np.float32)
message_bytes = message.tobytes()

for client_address in client_addresses: 
    server_socket.sendto(message_bytes, client_address)

print(f"[INFO] Receiving messages...")
try:
    while True:
        message_bytes, address = server_socket.recvfrom(1024)

        print(address)

        message = np.frombuffer(message_bytes, dtype=np.float32)

        curr_time = message[-1]
        shot = message[-2]

        blob_data = message[:-2].reshape(-1, 3)


except KeyboardInterrupt:
    pass
