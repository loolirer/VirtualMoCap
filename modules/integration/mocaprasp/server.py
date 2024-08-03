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
        self.client_ips = {} # FIX THIS !!

    def register_clients(self):
        # Clearing the previous addresses (client addresses may change from capture to capture)
        self.client_addresses.clear()
        self.client_ips.clear()

        # Check client connection to network
        for ID in range(self.n_clients):
            try:
                IP = socket.gethostbyname(f'cam{ID}.local')
                self.client_ips[IP] = ID

            except:
                print(f'[SERVER] Client {ID} not connected!')
                sys.exit()

        print('[SERVER] Waiting for clients...')

        # Address registration
        while len(self.client_addresses.keys()) < self.n_clients: # Until all clients are identified
            try:
                _, address = self.udp_socket.recvfrom(self.buffer_size)
                IP, _ = address
                ID = self.client_ips[IP]

            except: # Invalid message for decoding
                continue # Look for another message
            
            # Register client address
            self.client_addresses[address] = ID 
            self.clients[ID].address = address # Update the client's address

            print(f'\tClient {ID} registered')

        print('[SERVER] All clients registered!')

    def request_capture(self, delay_time, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Generate message 
        message = f'{delay_time + time.time()} {int(synchronizer.capture_time)}' # FIX THIS !!
        message_bytes = message.encode()

        # Send trigger to each client
        for client in self.clients: 
            self.udp_socket.sendto(message_bytes, client.address)
            
        return True