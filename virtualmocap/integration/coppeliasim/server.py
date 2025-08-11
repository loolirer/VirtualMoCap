import copy

from virtualmocap.integration.server import *
from virtualmocap.vision.synchronizer import *
from virtualmocap.vision.triangulator import *


class CoppeliaSim_Server(Server):
    def __init__(
        self,
        clients=[],
        server_address=("127.0.0.1", 8888),
        controller_address=("127.0.0.1", 7777),
    ):

        Server.__init__(self, clients, server_address)

        self.controller_address = controller_address
        self.buffer_size = 1024  # In bytes

    def register_clients(self):
        # Clearing the previous addresses (client addresses may change from capture to capture)
        self.client_addresses.clear()

        print("[INFO] Waiting for clients...")

        # Address registration
        while (
            len(self.client_addresses.keys()) < self.n_clients
        ):  # Until all clients are identified
            try:
                buffer, address = self.udp_socket.recvfrom(self.buffer_size)
                ID = int(buffer.decode())  # Decode message

            except:  # Invalid message for decoding
                continue  # Look for another message

            # Register client address
            self.client_addresses[address] = ID
            self.clients[ID].address = address  # Update the client's address

            print(f"\tClient {ID} registered")

        print("[INFO] All clients registered!")

    def request_scene(self):
        # Send scene request
        request = "Scene"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Initializing buffer
        buffer_array = None

        print("[INFO] Wrapping up CoppeliaSim scene info")

        for camera in [c.camera for c in self.clients]:
            # Wrap vision sensor parameters
            camera_array = np.array(
                [  # Options
                    2 + 4,  # Bit 1 set: Perspective Mode
                    # Bit 2 set: Invisible Viewing Frustum
                    # Integer parameters
                    camera.resolution[0],
                    camera.resolution[1],
                    0,  # Reserved
                    0,  # Reserved
                    # Float parameters
                    0.01,  # Near clipping plane in meters
                    10,  # Far clipping plane in meters
                    camera.fov_radians,  # FOV view angle in radians
                    0.1,  # Sensor X size
                    0.0,  # Reserved
                    0.0,  # Reserved
                    0.0,  # Null pixel red-value
                    0.0,  # Null pixel green-value
                    0.0,  # Null pixel blue-value
                    0.0,  # Reserved
                    0.0,  # Reserved
                ]
            )

            # X and Y axis of Coppelia's Vision Sensor are inverted
            coppeliasim_object_matrix = np.copy(camera.pose[:3, :4])
            coppeliasim_object_matrix[
                :, :2
            ] *= -1  # Multiplies by -1 the first two columns

            pose_array = np.ravel(coppeliasim_object_matrix)

            if buffer_array is not None:
                buffer_array = np.concatenate((buffer_array, camera_array, pose_array))

            else:
                buffer_array = np.concatenate((camera_array, pose_array))

        # Send scene info
        buffer = buffer_array.astype(np.float32).tobytes()
        self.udp_socket.sendto(buffer, self.controller_address)

        print("[INFO] Scene info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Scene set!")

                return True

            print("[ERROR] Scene setup failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def request_async_calibration(self, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Send extrinsic calibration request
        request = "Calibration"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(synchronizer.capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print("[INFO] Extrinsic Calibration info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Extrinsic Calibration confirmed!")

                return True

            print("[ERROR] Extrinsic Calibration start failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def request_async_reference(self, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Send reference update request
        request = "Reference"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(synchronizer.capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print("[INFO] Reference Update info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Reference Update confirmed!")

                return True

            print("[ERROR] Reference Update start failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def request_async_capture(self, synchronizer):
        # Initialize synchronizers and message logs
        for client in self.clients:
            client.synchronizer = copy.deepcopy(synchronizer)
            client.message_log = []

        # Send capture request
        request = "Capture"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(synchronizer.capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print("[INFO] Capture info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Capture confirmed!")

                return True

            print("[ERROR] Capture start failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def request_sync_calibration(self, capture_time):
        # Initialize message logs
        for client in self.clients:
            client.message_log = []

        # Build triangulator
        self.triangulator = Triangulator(self.multiple_view)

        # Send extrinsic calibration request
        request = "Calibration"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print("[INFO] Extrinsic Calibration info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Extrinsic Calibration confirmed!")

                return True

            print("[ERROR] Extrinsic Calibration start failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def request_sync_reference(self, capture_time):
        # Initialize message logs
        for client in self.clients:
            client.message_log = []

        # Build triangulator
        self.triangulator = Triangulator(self.multiple_view)

        # Send reference update request
        request = "Reference"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print("[INFO] Reference Update info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Reference Update confirmed!")

                return True

            print("[ERROR] Reference Update start failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def request_sync_capture(self, capture_time):
        # Initialize message logs
        for client in self.clients:
            client.message_log = []

        # Build triangulator
        self.triangulator = Triangulator(self.multiple_view)

        # Send capture request
        request = "Capture"
        request_bytes = request.encode()
        self.udp_socket.sendto(request_bytes, self.controller_address)

        # Send capture time
        message = str(capture_time)
        message_bytes = message.encode()
        self.udp_socket.sendto(message_bytes, self.controller_address)

        print("[INFO] Capture info sent")

        # Wait for controller setup confirmation
        try:
            confirmation_bytes, _ = self.udp_socket.recvfrom(self.buffer_size)
            confirmation = confirmation_bytes.decode()

            if confirmation == "Success":
                print("[INFO] Capture confirmed!")

                return True

            print("[ERROR] Capture start failed!")

            return False  # Did not confirm

        except:
            print("[ERROR] Parsing failed!")

            return False  # Confirmation parsing failed

    def offline_capture(self, expected_markers=3, timeout=5, verbose=True):
        # Wait for client identification
        self.register_clients()

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

            # Check if client exists
            try:
                ID = self.client_addresses[address]  # Client Identifier

            except:
                if verbose:
                    print("\t[WARNING] Address not recognized")

                continue  # Jump to wait for the next message

            # Show sender
            if verbose:
                print(f"\t[INFO] Received message from {ip}:{port}")

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
                        print("\t[ERROR] Couldn't decode message")

                    continue  # Jump to the next message

                # Empty message
                if not message.size:
                    if verbose:
                        print("\t[INFO] Empty message")

                    continue  # Jump to the next message

                # Extracting the message's frame index
                frame_idx = int(message[-1])

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
        max_head=2,
        max_hold=2,
        reprojection_tol=1,
        collinearity_tol=0.005,
        min_views=2,
        capture_path="",
        timeout=5,  # In seconds
        verbose=True,
    ):
        # Wait for client identification
        self.register_clients()

        timeout = 5  # In seconds
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
                ID = self.client_addresses[address]  # Client Identifier

            except:
                if verbose:
                    print("\t[WARNING] Address not recognized")

                continue  # Jump to wait for the next message

            # Show sender
            if verbose:
                print(f"\t[INFO] Received message from {ip}:{port}")

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
            frame_idx = int(message[-1])

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
                undistorted_blobs = self.clients[ID].camera.undistort_points(
                    blob_centroids
                )

            except:
                undistorted_blobs = np.array([])  # No blobs detected

            # Print blobs
            if verbose:
                print(f"\t[INFO] Detected Blobs - {frame_idx}")
                print("\t" + str(blob_data).replace("\n", "\n\t"))

            # If no marker count is expected, triangulate by multiview
            if not expected_markers:
                triangulated_markers = (
                    self.triangulator.multivision_triangulation_pipeline(
                        reference=ID,
                        frame_idx=frame_idx,
                        blobs_reference=undistorted_blobs,
                        max_head=max_head,
                        max_hold=max_hold,
                        reprojection_tol=reprojection_tol,
                        collinearity_tol=collinearity_tol,
                        min_views=min_views,
                    )
                )

            # If a marker count is expected, triangulate by pair
            else:
                triangulated_markers = self.triangulator.stereo_triangulation_pipeline(
                    ID, frame_idx, undistorted_blobs
                )

            if triangulated_markers is None:
                continue  # Jump to wait for the next message

            if verbose:
                print("[INFO] Triangulation Successful!")

            # Send data to CoppeliaSim
            buffer = triangulated_markers.astype(np.float32).ravel().tobytes()
            self.udp_socket.sendto(buffer, visualizer_address)

            # Save data for plotting
            try:
                all_triangulated_markers.append(triangulated_markers)

            except:
                pass  # Don't access array if index is out of bounds

        if not all_triangulated_markers:
            all_triangulated_markers = np.full((3, 1), np.nan)

        else:
            all_triangulated_markers = np.hstack(all_triangulated_markers)

        if capture_path:
            try:
                np.savetxt(capture_path, all_triangulated_markers, delimiter=",")

            except:
                print("[ERROR] Could not save capture")
                pass

        return all_triangulated_markers
