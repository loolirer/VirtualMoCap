class Client: 
    def __init__(self, 
                 address = (),
                 camera = None,
                 capture_data = None
                 ):
        
        self.address = address # Network address (IP, Port)
        self.camera = camera # Associated camera model
        self.capture_data = capture_data # Capture data structure