import socket

class UDP: 
    def __init__(self, 
                 alias='',    
                 address=('127.0.0.1', 0000),
                 buffer_size=1024
                 ):
        
        self.alias = alias

        print(f'[{self.alias}] Creating socket...')

        self.udp_socket = None

        # Try to create socket
        try: 
            self.udp_socket = socket.socket(socket.AF_INET,    # Internet
                                            socket.SOCK_DGRAM) # UDP
            print(f'[{self.alias}] UDP Socket successfully created')
            
        except socket.error as err: 
            print(f'[{self.alias}] Socket creation failed with error {err}\n')
            print(f'[{self.alias}] Quitting code...')
            exit()

        self.ip   = address[0] # Socket IP
        self.port = address[1] # Socket Port
        self.address = address

        # Binding socket to the address
        self.udp_socket.bind(address)
        print(f'[{self.alias}] Bound to port {self.address}')

        self.buffer_size = buffer_size # Size of the messages in bytes