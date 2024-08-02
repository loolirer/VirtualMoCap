import copy

from modules.vision.multiple_view import *
from modules.vision.synchronizer import *
from modules.integration.client import *
from modules.integration.UDP import *

class Server: 
    def __init__(self, 
                 clients,
                 address
                 ):
        
        # Associating clients
        self.update_clients(clients)

        # UDP socket for sending and receiving messages
        self.address = address
        self.udp_socket = UDP(self.address)

    def update_clients(self, clients):
        # Associated clients
        self.clients = copy.deepcopy(clients)     
        self.n_clients = len(self.clients)
        self.client_addresses = {}

        # Create Multiple View only if all clients have an associated camera model
        self.multiple_view = None
        camera_models = [c.camera for c in self.clients]
        self.multiple_view = MultipleView(camera_models)



