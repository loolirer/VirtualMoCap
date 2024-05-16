from modules.integration.client import *
from modules.vision.multiple_view import *

class Server: 
    def __init__(self, 
                 clients,
                 udp_socket
                 ):
        
        # Associated clients
        self.clients = clients     
        self.n_clients = len(self.clients)
        self.client_addresses = {}
        
        # UDP socket for sending and receiving messages
        self.udp_socket = udp_socket

        # Creating multiple view object
        self.multiple_view = MultipleView([c.camera for c in clients])

    def handshake_clients(self):
        print(f'[SERVER] Waiting for clients...')

        # Address lookup 
        while len(self.client_addresses.keys()) < self.n_clients: # Until all clients are identified
            buffer, address = self.udp_socket.recvfrom(self.udp_socket.buffer_size)

            try:
                ID = int(buffer.decode()) # Decode message

            except: # Invalid message for decoding
                continue # Look for another message

            self.client_addresses[address] = ID

            print(f'\tClient {ID} connected')

        print(f'[SERVER] All clients connected!')
