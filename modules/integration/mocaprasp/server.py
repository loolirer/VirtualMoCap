import copy

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

        print('[SERVER] Waiting for clients...')

        # Address registration
        while len(self.client_addresses.keys()) < self.n_clients: # Until all clients are identified
            try:
                buffer, address = self.udp_socket.recvfrom(self.buffer_size)
                ID = int(buffer.decode()) # Decode message

            except: # Invalid message for decoding
                continue # Look for another message
            
            # Register client address
            self.client_addresses[address] = ID 
            self.clients[ID].address = address # Update the client's address

            print(f'\tClient {ID} registered')

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
        
    def request_calibration(self, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Send extrinsic calibration request 
        request = 'Calibration'
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(synchronizer.capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print('[SERVER] Extrinsic Calibration info sent')

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == 'Success':
                print('[SERVER] Extrinsic Calibration confirmed!')

                return True
            
            print('[SERVER] Extrinsic Calibration start failed!')

            return False # Did not confirm
            
        except:
            print('[SERVER] Parsing failed!')

            return False # Confirmation parsing failed
        
    def request_reference(self, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Send reference update request 
        request = 'Reference'
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(synchronizer.capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print('[SERVER] Reference Update info sent')

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == 'Success':
                print('[SERVER] Reference Update confirmed!')

                return True
            
            print('[SERVER] Reference Update start failed!')

            return False # Did not confirm
            
        except:
            print('[SERVER] Parsing failed!')

            return False # Confirmation parsing failed
