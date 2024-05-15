import socket

class UDP(socket.socket): 
    def __init__(self,  
                 address=('127.0.0.1', 1024) # First valid address for free use
                 ):
        
        # Socket info
        self.ip = address[0] 
        self.port = address[1] 
        self.address = address 

        # Create the socket
        try: 
            socket.socket.__init__(self,
                                   socket.AF_INET,    # Internet
                                   socket.SOCK_DGRAM) # UDP
            
        except socket.error as err: 
            print(err)
            exit()

        # Binding socket to the address
        try:
            self.bind(self.address)

        except socket.error as err:
            print(err)
            exit()