import os
import copy
import pickle
from datetime import datetime

from modules.vision.multiple_view import *
from modules.vision.synchronizer import *
from modules.integration.client import *
from modules.integration.UDP import *

class Server: 
    def __init__(self, 
                 clients = [],
                 address = ('127.0.0.1', 8888)
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

        if clients:
            self.multiple_view = MultipleView(camera_models)

    def save_calibration(self):
        now = datetime.now()
        ymd, HMS = now.strftime('%y-%m-%d'), now.strftime('%H-%M-%S')

        directory = os.path.join(os.getcwd(), 'calibration', ymd, HMS)

        # Check whether directory already exists
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)

        # Save data to pickle file
        for C, camera_model in enumerate(self.multiple_view.camera_models):
            # Save the object to a file (Pickling)
            try:
                with open(os.path.join(directory, f'{C}.pkl'), 'wb') as file:
                    pickle.dump(camera_model, file)
            except:
                continue

    def load_calibration(self, path):
        clients = []
        for C in range(len(os.listdir(path))):
            # Load the object from the file (Unpickling)
            try:
                with open(os.path.join(path, f'{C}.pkl'), 'rb') as file:
                    camera = pickle.load(file)
                    clients.append(Client(camera=camera))
            
            except:
                continue

        self.update_clients(clients)




