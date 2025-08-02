class Client:
    def __init__(self, alias="", mac_address="", camera=None):

        self.active = False # Connected flag

        self.alias = alias # Client arbitrary name
        self.mac_address = mac_address # MAC address
        self.camera = camera  # Associated camera model

        self.address = ()  # Network address (IP, Port)
        self.synchronizer = (
            None  # Synchronizer structure changed through server requests
        )
        self.message_log = []  # Message history
