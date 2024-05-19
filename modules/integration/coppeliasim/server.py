import copy
from modules.integration.server import *

class CoppeliaSim_Server(Server): 
    def __init__(self, 
                 clients,
                 server_address,
                 controller_address
                 ):
        
        Server.__init__(self, 
                        clients, 
                        server_address)
        
        self.controller_address = controller_address
        self.buffer_size = 1024 # In bytes

    def request_scene(self):
        # Send scene request 
        request = 'Scene'
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Initializing buffer
        buffer_array = None

        print('[SERVER] Wrapping up CoppeliaSim scene info')

        for C in [c.camera for c in self.clients]:
            # Wrap vision sensor parameters
            intrinsic_array = np.array([# Options
                                        2+4, # Bit 1 set: Perspective Mode
                                             # Bit 2 set: Invisible Viewing Frustum 
                                                                            
                                        # Integer parameters
                                        C.resolution[0], 
                                        C.resolution[1],
                                        0, # Reserved
                                        0, # Reserved

                                        # Float parameters
                                        0.01, # Near clipping plane in meters
                                        10, # Far clipping plane in meters
                                        C.fov_radians, # FOV view angle in radians
                                        0.1, # Sensor X size
                                        0.0, # Reserved
                                        0.0, # Reserved
                                        0.0, # Null pixel red-value
                                        0.0, # Null pixel green-value
                                        0.0, # Null pixel blue-value
                                        0.0, # Reserved
                                        0.0  # Reserved
                                        ])

            # X and Y axis of Coppelia's Vision Sensor are inverted
            coppeliasim_object_matrix = np.copy(C.object_matrix)
            coppeliasim_object_matrix[:,0] *= -1 # Invert x vector column
            coppeliasim_object_matrix[:,1] *= -1 # Invert y vector column

            extrinsic_array = np.ravel(coppeliasim_object_matrix)

            if buffer_array is not None:
                buffer_array = np.concatenate((buffer_array, intrinsic_array, extrinsic_array))

            else:
                buffer_array = np.concatenate((intrinsic_array, extrinsic_array))

        # Send scene info
        buffer = buffer_array.astype(np.float32).tobytes()
        self.udp_socket.sendto(buffer, self.controller_address)

        print('[SERVER] Scene info sent')

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == 'Success':
                print('[SERVER] Scene set!')

                return True
            
            print('[SERVER] Scene setup failed!')

            return False # Did not confirm
            
        except:
            print('[SERVER] Parsing failed!')

            return False # Confirmation parsing failed

    def request_capture(self, synchronizer):
        # Initialize synchronizers
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)

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
        
    def register_clients(self):
        print('[SERVER] Waiting for clients...')

        # Address lookup 
        while len(self.client_addresses.keys()) < self.n_clients: # Until all clients are identified
            buffer, address = self.udp_socket.recvfrom(self.buffer_size)

            try:
                ID = int(buffer.decode()) # Decode message

            except: # Invalid message for decoding
                continue # Look for another message
            
            # Register client address
            self.client_addresses[address] = ID 
            self.clients[ID].address = address

            print(f'\tClient {ID} registered')

        print('[SERVER] All clients registered!')