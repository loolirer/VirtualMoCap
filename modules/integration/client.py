class Client: 
    def __init__(self, 
                 address = (),
                 camera = None
                 ):
        
        self.address = address # Network address (IP, Port)
        self.camera = camera # Associated camera model
        self.synchronizer = None # Synchronizer structure changed through server requests
        self.message_log = [] # Message history