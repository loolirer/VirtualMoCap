import copy
import time

from virtualmocap.integration.server import *
from virtualmocap.vision.synchronizer import *
from virtualmocap.vision.triangulator import *


class MoCapRasp_Server(Server):
    def __init__(self, clients=[], server_address=("127.0.0.1", 25565)):

        Server.__init__(self, clients, server_address)
        self.buffer_size = 1024  # In bytes

    def register_clients(self):
        # Clearing the previous addresses (client addresses may change from capture to capture)
        self.client_addresses.clear()

        # Check client connection to network
        for ID in range(self.n_clients):
            try:
                IP = socket.gethostbyname(f"mocaprasp-client-{ID}.local")
                address = (IP, 25565)  # Pre-established standard client port
                self.client_addresses[address] = ID
                self.clients[ID].address = address  # Update the client address

                print(f"\tClient {ID} registered")

            except:
                print(f"[SERVER] Client {ID} not connected!")
                return False
            
        return True

    def request_async_capture(self, delay_time, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Generate message
        message = f"{delay_time + time.time()} {int(synchronizer.capture_time)}"  # FIX THIS !!
        message_bytes = message.encode()

        # Send trigger to each client
        for client in self.clients:
            self.udp_socket.sendto(message_bytes, client.address)

        return True

    def request_sync_capture(self, delay_time, capture_time):
        # Initialize message logs
        for client in self.clients:
            client.message_log = []

        # Build triangulator
        self.triangulator = Triangulator(self.multiple_view)

        # Generate message
        message = np.array([delay_time, capture_time]).astype(np.float32)
        message_bytes = message.tobytes()

        # Send trigger to each client
        for client in self.clients:
            self.udp_socket.sendto(message_bytes, client.address)

        return True
