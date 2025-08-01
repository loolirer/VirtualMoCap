import copy
import time

from virtualmocap.integration.server import *
from virtualmocap.vision.synchronizer import *
from virtualmocap.vision.triangulator import *


class MoCapRasp_Server(Server):
    def __init__(self, clients=[], server_address=("127.0.0.1", 25565)):

        Server.__init__(self, clients, server_address)
        self.buffer_size = 1024  # In bytes

    def register_clients(self):
        # Clearing the previous addresses (client addresses may change from capture to capture)
        self.client_addresses.clear()

        # Check client connection to network
        for ID in range(self.n_clients):
            try:
                IP = socket.gethostbyname(f"mocaprasp-client-{ID}.local")
                address = (IP, 25565)  # Pre-established standard client port
                self.client_addresses[address] = ID
                self.clients[ID].address = address  # Update the client address

                print(f"\tClient {ID} registered")

            except:
                print(f"[SERVER] Client {ID} not connected!")
                return False

        return True

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

    def offline_capture(self, expected_markers=1, timeout=5, verbose=True):
        timeout = 5  # In seconds
        self.udp_socket.settimeout(timeout)  # Set server timeout
        print(f"[SERVER] Timeout set to {timeout} seconds\n")

        # Receiving messages
        while True:
            # Wait for message - Event guided!
            try:
                message_bytes, address = self.udp_socket.recvfrom(self.buffer_size)

            except TimeoutError:
                print("\n[SERVER] Timed Out!")
                break  # Close capture loop due to timeout

            except ConnectionResetError:
                print("\n[SERVER] Connection Reset!")
                continue  # Jump to wait for the next message

            # Check if message comes from any of the clients
            try:
                ID = self.client_addresses[address]  # Client Identifier

            except:
                if verbose:
                    print("> Client not recognized")

                continue  # Jump to wait for the next message

            # Show sender
            if verbose:
                print(
                    f"> Received message from Client {ID} ({address[0]}, {address[1]})"
                )

            # Save message
            self.clients[ID].message_log.append(message_bytes)

        # Post-processing
        for ID, client in enumerate(self.clients):
            # Parse through client's message history
            for message_bytes in client.message_log:
                # Decode message
                try:
                    message = np.frombuffer(message_bytes, dtype=np.float32)

                except:
                    if verbose:
                        print("> Couldn't decode message")

                    continue  # Jump to the next message

                # Empty message
                if not message.size:
                    if verbose:
                        print("\tEmpty message")

                    continue  # Jump to the next message

                # Extracting the message's frame index
                frame_idx = int(message[-2])

                # Valid message is [u, v, A] per blob, PTS and frame index
                if message.size != 3 * expected_markers + 2:

                    if message.size == 2:  # Only PTS
                        if verbose:
                            print(f"\tNo blobs were detected - {frame_idx}")

                    else:
                        if verbose:
                            print(f"\tWrong blob count or corrupted message")
                            print(f"\tCorrupted Message: {message}")

                    continue  # Jump to the next message

                # Extracting blob data (coordinates & area)
                blob_data = message[:-2].reshape(-1, 3)  # All but last two elements

                # Extracting centroids
                blob_centroids = blob_data[:, :2]  # Ignoring their area

                # Undistorting blobs centroids
                undistorted_blobs = client.camera.undistort_points(blob_centroids)

                # Print blobs
                if verbose:
                    print(f"\tDetected Blobs - {frame_idx}")
                    print("\t" + str(blob_data).replace("\n", "\n\t"))

                # Save data
                self.triangulator.save(ID, frame_idx, undistorted_blobs)

    def online_capture(
        self, expected_markers=1, visualizer_address=("127.0.0.1", 6666), condensed_output=[], verbose=True
    ):
        timeout = 5  # In seconds
        self.udp_socket.settimeout(timeout)  # Set server timeout
        print(f"[SERVER] Timeout set to {timeout} seconds\n")

        condensed_output.clear()

        # Breaks in the timeout
        while True:
            # Wait for message - Event guided!
            try:
                message_bytes, address = self.udp_socket.recvfrom(
                    self.buffer_size
                )

            except TimeoutError:
                print("\n[SERVER] Timed Out!")
                break  # Close capture loop due to timeout

            except ConnectionResetError:
                print("\n[SERVER] Connection Reset!")
                continue  # Jump to wait for the next message

            # Check if message comes from any of the clients
            try:
                ID = self.client_addresses[address]  # Client Identifier

            except:
                if verbose:
                    print("> Address not recognized")

                continue  # Jump to wait for the next message

            # Show sender
            if verbose:
                print(
                    f"> Received message from Client {ID} ({address[0]}, {address[1]}):"
                )

            # Decode message
            try:
                message = np.frombuffer(message_bytes, dtype=np.float32)

            except:
                if verbose:
                    print("> Couldn't decode message")

                continue  # Jump to wait for the next message

            # Empty message
            if not message.size:
                if verbose:
                    print("\tEmpty message")

                continue  # Jump to wait for the next message

            # Extracting the message's frame index
            frame_idx = int(message[-2])

            # Valid message is [u, v, A] per blob, PTS and frame index
            if message.size != 3 * expected_markers + 2:

                if message.size == 2:
                    if verbose:
                        print(f"\tNo blobs were detected - {frame_idx}")

                else:
                    if verbose:
                        print(f"\tWrong blob count or corrupted message")
                        print(f"\tCorrupted Message: {message}")

                continue  # Jump to wait for the next message

            # Extracting blob data (coordinates & area)
            blob_data = message[:-2].reshape(-1, 3)  # All but last two elements

            # Extracting centroids
            blob_centroids = blob_data[:, :2]  # Ignoring their area

            # Undistorting blobs centroids
            undistorted_blobs = self.clients[ID].camera.undistort_points(
                blob_centroids
            )

            # Print blobs
            if verbose:
                print(f"\tDetected Blobs - {frame_idx}")
                print("\t" + str(blob_data).replace("\n", "\n\t"))

            triangulated_markers = self.triangulator.triangulate(
                ID, frame_idx, undistorted_blobs
            )

            if triangulated_markers is None:
                continue  # Jump to wait for the next message

            if verbose:
                print("Triangulated!")

            # Send data to CoppeliaSim
            buffer = triangulated_markers.astype(np.float32).ravel().tobytes()
            self.udp_socket.sendto(buffer, visualizer_address)

            # Save data for plotting
            try:
                condensed_output.append(triangulated_markers)

            except:
                pass  # Don't access array if index is out of bounds
