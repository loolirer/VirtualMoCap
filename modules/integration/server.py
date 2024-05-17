from modules.integration.client import *
from modules.integration.UDP import *
from modules.vision.multiple_view import *

class Server: 
    def __init__(self, 
                 clients,
                 address
                 ):
        
        # Associated clients
        self.clients = clients     
        self.n_clients = len(self.clients)
        self.client_addresses = {}
        
        # UDP socket for sending and receiving messages
        self.address = address
        self.udp_socket = UDP(self.address)

        # Creating multiple view object
        self.multiple_view = MultipleView([c.camera for c in self.clients])

    def register_clients(self):
        print('[SERVER] Waiting for clients...')

        # Address lookup 
        while len(self.client_addresses.keys()) < self.n_clients: # Until all clients are identified
            buffer, address = self.udp_socket.recvfrom(1024)

            try:
                ID = int(buffer.decode()) # Decode message

            except: # Invalid message for decoding
                continue # Look for another message

            self.client_addresses[address] = ID

            print(f'\tClient {ID} registered')

        print('[SERVER] All clients registered!')
