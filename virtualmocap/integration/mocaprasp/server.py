import copy
import time
import yaml

from virtualmocap.integration.arp import *
from virtualmocap.integration.server import *
from virtualmocap.vision.synchronizer import *
from virtualmocap.vision.triangulator import *


class MoCapRasp_Server(Server):
    def __init__(self, config_path="", server_address=("127.0.0.1", 25565)):

        clients = self.load_client_configs(config_path)
        Server.__init__(self, clients, server_address)
        self.buffer_size = 1024  # In bytes

    def load_client_configs(self, config_path):
        try:
            with open(config_path, "r") as file:
                client_configs = yaml.safe_load(file)

            clients = []
            for client in client_configs.get("clients", []):
                alias = client["alias"]
                mac_address = client["mac_address"].upper()
                resolution = tuple(client["resolution"])
                intrinsic_matrix = np.array(client["intrinsic_matrix"])
                distortion_model = client["distortion_model"]
                distortion_coefficients = np.array(client["distortion_coefficients"])

                camera = Camera(
                    resolution=resolution,
                    intrinsic_matrix=intrinsic_matrix,
                    distortion_model=distortion_model,
                    distortion_coefficients=distortion_coefficients,
                )

                clients.append(
                    Client(alias=alias, mac_address=mac_address, camera=camera)
                )

            return clients

        except FileNotFoundError:
            return []

    def register_clients(self):
        try:
            # Clearing the previous addresses (client addresses may change from capture to capture)
            self.client_addresses.clear()

            mac_list = [client.mac_address for client in self.clients]
            self.mac_to_client = {client.mac_address: client for client in self.clients}
            self.mac_to_ip, self.ip_to_mac = get_mac_mapping(mac_list)

            for mac_address, ip in self.mac_to_ip.items():
                if mac_address in mac_list:
                    self.mac_to_client[mac_address].active = True
                    self.mac_to_client[mac_address].address = (ip, 25565)

                else:
                    self.mac_to_client[mac_address].active = False
                    self.mac_to_client[mac_address].address = ()

            return True

        except:
            return False

    def save_calibration(self):
        now = datetime.now()
        ymd, HMS = now.strftime("%y-%m-%d"), now.strftime("%H-%M-%S")

        directory = os.path.join(os.getcwd(), "calibration", "-".join([ymd, HMS]))

        # Check whether directory already exists
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)

        # Save data to pickle file
        for client in self.clients:
            # Save the object to a file (Pickling)
            try:
                file_name = client.mac_address.replace(":", "")
                with open(os.path.join(directory, f"{file_name}.pkl"), "wb") as file:
                    pickle.dump(client.camera, file)

            except:
                continue

    def load_calibration(self, path):
        self.mac_to_client = {client.mac_address: client for client in self.clients}

        for pickled_camera_model in os.listdir(path):
            mac = pickled_camera_model.removesuffix(".pkl")
            mac_address = ":".join(mac[i : i + 2] for i in range(0, len(mac), 2))

            # Check if client is present
            try:
                client = self.mac_to_client[mac_address]

            except:
                continue

            # Load the object from the file (Unpickling)
            try:
                with open(os.path.join(path, pickled_camera_model), "rb") as file:
                    camera_model = pickle.load(file)

                    client.camera = camera_model

            except:
                continue

        self.update_clients(self.clients)

    # LEGACY
    def request_async_capture(self, delay_time, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Generate message
        message = f"{delay_time + time.time()} {int(synchronizer.capture_time)}"  # FIX THIS !!
        message_bytes = message.encode()

        # Send trigger to each client
        for client in self.clients:
            self.udp_socket.sendto(message_bytes, client.address)

        return True

    def request_sync_capture(self, delay_time, capture_time):
        # Initialize message logs
        for client in self.clients:
            client.message_log = []

        # Build triangulator
        self.triangulator = Triangulator(self.multiple_view)

        # Generate message
        message = np.array([delay_time, capture_time]).astype(int)
        message_bytes = message.tobytes()

        # Send trigger to each client
        for client in self.clients:
            self.udp_socket.sendto(message_bytes, client.address)

        return True

    def offline_capture(self, expected_markers=3, timeout=5, verbose=True):
        self.udp_socket.settimeout(timeout)  # Set server timeout
        print(f"[INFO] Timeout set to {timeout} seconds\n")

        # Receiving messages
        while True:
            # Wait for message - Event guided!
            try:
                message_bytes, address = self.udp_socket.recvfrom(self.buffer_size)
                ip, port = address

            except TimeoutError:
                print("\n[INFO] Timed Out!")
                break  # Close capture loop due to timeout

            except ConnectionResetError:
                print("\n[INFO] Connection Reset!")
                continue  # Jump to wait for the next message

            # Check if message comes from any of the clients
            try:
                client = self.mac_to_client[self.ip_to_mac[ip]]  # Get client

            except:
                if verbose:
                    print("\t[WARNING] Address not recognized")

                continue  # Jump to wait for the next message

            # Show sender
            if verbose:
                print(f"\t[INFO] Received message from {client.alias} @ {ip}:{port}")

            # Save message
            client.message_log.append(message_bytes)

        # Post-processing
        for ID, client in enumerate(self.clients):
            # Parse through client's message history
            for message_bytes in client.message_log:
                # Decode message
                try:
                    message = np.frombuffer(message_bytes, dtype=np.float32)

                except:
                    if verbose:
                        print("\t[ERROR] Couldn't decode message")

                    continue  # Jump to the next message

                # Empty message
                if not message.size:
                    if verbose:
                        print("\t[INFO] Empty message")

                    continue  # Jump to the next message

                # Extracting the message's frame index
                frame_idx = int(message[-2])

                # Valid message is [u, v, A] per blob, PTS and frame index
                if message.size != 3 * expected_markers + 2:

                    if message.size == 2:  # Only PTS
                        if verbose:
                            print(f"\t[INFO] No blobs were detected - {frame_idx}")

                    else:
                        if verbose:
                            print(f"\t[INFO] Wrong blob count or corrupted message")
                            print(f"\t{message}")

                    continue  # Jump to the next message

                # Extracting blob data (coordinates & area)
                blob_data = message[:-2].reshape(-1, 3)  # All but last two elements

                # Extracting centroids
                blob_centroids = blob_data[:, :2]  # Ignoring their area

                # Undistorting blobs centroids
                undistorted_blobs = client.camera.undistort_points(blob_centroids)

                # Print blobs
                if verbose:
                    print(f"\t[INFO] Detected Blobs - {frame_idx}")
                    print("\t" + str(blob_data).replace("\n", "\n\t"))

                # Save data
                self.triangulator.save(ID, frame_idx, undistorted_blobs)

    def online_capture(
        self,
        expected_markers=0,
        visualizer_address=("127.0.0.1", 6666),
        capture_path="",
        timeout=5,  # In seconds
        verbose=True,
    ):
        self.udp_socket.settimeout(timeout)  # Set server timeout
        print(f"[INFO] Timeout set to {timeout} seconds\n")

        all_triangulated_markers = []

        # Breaks in the timeout
        while True:
            # Wait for message - Event guided!
            try:
                message_bytes, address = self.udp_socket.recvfrom(self.buffer_size)
                ip, port = address

            except TimeoutError:
                print("\n[INFO] Timed Out!")
                break  # Close capture loop due to timeout

            except ConnectionResetError:
                print("\n[INFO] Connection Reset!")
                continue  # Jump to wait for the next message

            # Check if message comes from any of the clients
            try:
                client = self.mac_to_client[self.ip_to_mac[ip]]  # Get client
                ID = int(client.alias[-1])

            except:
                if verbose:
                    print("\t[WARNING] Address not recognized")

                continue  # Jump to wait for the next message

            # Show sender
            if verbose:
                print(f"\t[INFO] Received message from {client.alias} @ {ip}:{port}")

            # Decode message
            try:
                message = np.frombuffer(message_bytes, dtype=np.float32)

            except:
                if verbose:
                    print("\t[ERROR] Couldn't decode message")

                continue  # Jump to wait for the next message

            # Empty message
            if not message.size:
                if verbose:
                    print("\t[INFO] Empty message")

                continue  # Jump to wait for the next message

            # Extracting the message's frame index
            frame_idx = int(message[-2])

            # Valid message is [u, v, A] per blob, PTS and frame index
            if expected_markers and message.size != 3 * expected_markers + 2:

                if message.size == 2:
                    if verbose:
                        print(f"\t[INFO] No blobs were detected - {frame_idx}")

                else:
                    if verbose:
                        print(f"\t[INFO] Wrong blob count or corrupted message")
                        print(f"\t{message}")

                continue  # Jump to wait for the next message
            
            try:
                # Extracting blob data (coordinates & area)
                blob_data = message[:-2].reshape(-1, 3)  # All but last two elements

                # Extracting centroids
                blob_centroids = blob_data[:, :2]  # Ignoring their area

                # Undistorting blobs centroids
                undistorted_blobs = client.camera.undistort_points(blob_centroids)

            except:
                undistorted_blobs = np.array([])

            # Print blobs
            if verbose:
                print(f"\t[INFO] Detected Blobs - {frame_idx}")
                print("\t" + str(blob_data).replace("\n", "\n\t"))


            # If no marker count is expected, triangulate by multiview
            if not expected_markers:
                triangulated_markers = self.triangulator.triangulate_by_multiview(
                    reference=ID,
                    frame_idx=frame_idx,
                    blobs_reference=undistorted_blobs,
                    max_hold=2,
                )

            # If a marker count is expected, triangulate by pair
            else:
                triangulated_markers = self.triangulator.triangulate_by_pair(
                    ID, frame_idx, undistorted_blobs
                )

            if triangulated_markers is None:
                continue  # Jump to wait for the next message

            if verbose:
                print("[INFO] Triangulation Successful!")

            # Send data to CoppeliaSim
            buffer = triangulated_markers.astype(np.float32).ravel().tobytes()
            self.udp_socket.sendto(buffer, visualizer_address)

            if capture_path:
                # Save data for plotting
                try:
                    all_triangulated_markers.append(triangulated_markers)

                except:
                    pass  # Don't access array if index is out of bounds

        if capture_path:
            try:
                if not all_triangulated_markers:
                    all_triangulated_markers = np.full((3, 1), np.nan)

                else:
                    all_triangulated_markers = np.hstack(all_triangulated_markers)

                np.savetxt(
                    capture_path, all_triangulated_markers, delimiter=","
                )

            except:
                print("[ERROR] Could not save capture")
                pass
