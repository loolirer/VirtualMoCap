class Client: 
    def __init__(self, 
                 address = (),
                 camera = None,
                 synchronizer = None
                 ):
        
        self.address = address # Network address (IP, Port)
        self.camera = camera # Associated camera model
        self.synchronizer = synchronizer # Synchronizer structure