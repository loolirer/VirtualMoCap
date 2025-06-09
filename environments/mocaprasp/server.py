import socket
import numpy as np

# Try to create client socket
try: 
    server_socket = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP
    print(f"[INFO] Socket created successfully")
    
except socket.error as err: 
    print(f"[ERROR] Socket creation failed with error: {err}")

server_ip = "0.0.0.0" 
server_port = 8888 
server_address = (server_ip, server_port)
server_socket.bind(server_address)

print(f"[INFO] Receiving messages...")
try:
    while True:
        message_bytes, address = server_socket.recvfrom(1024)

        message = np.frombuffer(message_bytes, dtype=np.float32)

        blob_data = message[:-1].reshape(-1, 3) 

        print(blob_data)


except KeyboardInterrupt:
    pass