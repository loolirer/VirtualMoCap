import copy
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

    def request_capture(self, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Send capture request 
        request = 'Capture'
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(synchronizer.capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print('[SERVER] Capture info sent')

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == 'Success':
                print('[SERVER] Capture confirmed!')

                return True
            
            print('[SERVER] Capture start failed!')

            return False # Did not confirm
            
        except:
            print('[SERVER] Parsing failed!')

            return False # Confirmation parsing failed