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
