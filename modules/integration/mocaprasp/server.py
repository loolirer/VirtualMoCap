import copy
import time
import sys

from modules.integration.server import *
from modules.vision.synchronizer import *

class MoCapRasp_Server(Server): 
    def __init__(self, 
                 clients,
                 server_address
                 ):
        
        Server.__init__(self, 
                        clients, 
                        server_address)

        self.buffer_size = 1024 # In bytes

    def register_clients(self):
        # Clearing the previous addresses (client addresses may change from capture to capture)
        self.client_addresses.clear()

        # IP lookup from hostname
        for ID, client in enumerate(self.clients):
            try:
                # Get client address
                address = socket.gethostbyname(f'cam{ID}.local')

                 # Register client address
                self.client_addresses[address] = ID 
                client.address = address # Update the client's address

            except:
                print('[SERVER] Client {ID} not found!')
                sys.exit()

        print('[SERVER] All clients registered!')

    def request_capture(self, delay_time, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Generate message
        message = f'{delay_time + time.time()} {synchronizer.capture_time}'
        message_bytes = message.encode()

        # Send trigger to each client
        for client in range(self.clients): 
            self.udp_socket.sendto(data=message_bytes,
                                   address=client.address)
            
        return True