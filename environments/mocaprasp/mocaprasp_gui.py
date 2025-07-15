# Importing modules...
import os
import sys
import time
import numpy as np
import streamlit as st

from virtualmocap.plot.viewer3d import Viewer3D

from virtualmocap.vision.camera import Camera

from virtualmocap.integration.client import Client
from virtualmocap.integration.mocaprasp.server import MoCapRasp_Server
from virtualmocap.integration.mocaprasp.calib_data import (
    all_intrinsic_matrices,
    all_distortion_coefficients,
)

# Create server
if "server" not in st.session_state:
    # Create server
    st.session_state.server = MoCapRasp_Server(server_address=("0.0.0.0", 25565))

# Calibration wand distances
if "wand_distances_calibration" not in st.session_state:
    st.session_state.wand_distances_calibration = np.array(
        [5.35e-2, 10.30e-2, 15.70e-2]
    )  # In meters

# Measured distances between perpendicularly matched marker distances
# Distances: [D_x, D_y]
if "wand_distances_reference" not in st.session_state:
    st.session_state.wand_distances_reference = np.array(
        [10.10e-2, 15.05e-2]
    )  # In meters

# Collected calibration data
if "wand_blobs" not in st.session_state:
    st.session_state.wand_blobs = None

# Collected capture data
if "triangulated_markers" not in st.session_state:
    st.session_state.triangulated_markers = None

st.set_page_config(page_title="Motion Capture Arena", layout="centered")
st.image("mocaprasp.png")

st.subheader("⚙️ Arena Setup")
if st.button("Load intrinsics"):
    placeholder = st.empty()
    placeholder.info("Preparing clients...", icon="ℹ️")

    clients = []  # Clients list

    # Create clients
    for K, k_d in zip(all_intrinsic_matrices, all_distortion_coefficients):
        # Generate associated camera model
        camera = Camera(  # Intrinsic Parameters
            resolution=(960, 720),
            intrinsic_matrix=K.copy(),
            # Fisheye Lens Distortion Model
            distortion_model="fisheye",
            distortion_coefficients=k_d.copy(),
        )

        clients.append(Client(camera=camera))

    # Create server
    st.session_state.server.update_clients(clients=clients)

    placeholder.empty()  # Clear info message

    placeholder.success("Intrinsic parameters loaded successfully!", icon="✅")
    time.sleep(5)  # Wait 5 seconds before disappearing
    placeholder.empty()

if st.button("Register clients"):
    placeholder = st.empty()

    # Register clients
    if not st.session_state.server.register_clients():
        placeholder.error("Client register failed!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()

    else:
        placeholder.success(f"Client register successful!", icon="✅")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()

st.markdown("---")

st.subheader("⚖️ System Calibration")

calibration_path = "calibration/"
os.makedirs(calibration_path, exist_ok=True)  # Create the folder if it doesn't exist

calibrations = sorted(
    [
        name
        for name in os.listdir(calibration_path)
        if os.path.isdir(os.path.join(calibration_path, name))
    ]
)

selected_folder = st.selectbox("Select a Calibration", [""] + calibrations)

if st.button("Load Calibration") and selected_folder != "":
    placeholder = st.empty()

    full_path = os.path.join(calibration_path, selected_folder)

    st.session_state.server.load_calibration(full_path)

    # Create the Scene Viewer
    scene = Viewer3D(title="Loaded Calibration", size=10)

    # Add camera frames to the scene
    for ID, camera in enumerate(st.session_state.server.multiple_view.camera_models):
        scene.add_frame(camera.pose, f"Camera {ID}", axis_size=0.4)

    scene.add_frame(np.eye(4), f"Reference", axis_size=0.4)

    st.plotly_chart(scene.figure)

    placeholder.success("Calibration loaded!", icon="✅")
    time.sleep(5)  # Wait 5 seconds before disappearing
    placeholder.empty()

# User inputs
calibration_delay = st.number_input("Calibration delay (s)", min_value=0.0, step=1.0)
calibration_duration = st.number_input(
    "Calibration duration (s)", min_value=1.0, step=1.0
)

if st.button("Extrinsic Calibration"):
    placeholder = st.empty()
    placeholder.info("Extrinsic calibration requested", icon="ℹ️")

    # Capture specifications
    blob_count = 3  # Number of expected markers
    verbose = True

    # Request capture
    if not st.session_state.server.request_sync_capture(
        calibration_delay, calibration_duration
    ):
        placeholder.error("Calibration request failed!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()
        sys.exit()

    else:
        placeholder.success(
            f"Calibration request successful! Waiting {calibration_delay} s...",
            icon="✅",
        )
        time.sleep(calibration_delay)  # Wait before disappearing

    timeout = 5.0  # In seconds
    st.session_state.server.udp_socket.settimeout(timeout)  # Set server timeout

    placeholder.info(
        f"Running extrinsic calibration for {calibration_duration} s...", icon="ℹ️"
    )

    # Receiving messages
    while True:
        # Wait for message - Event guided!
        try:
            message_bytes, address = st.session_state.server.udp_socket.recvfrom(
                st.session_state.server.buffer_size
            )

        except TimeoutError:
            print("\n[SERVER] Timed Out!")
            break  # Close capture loop due to timeout

        except ConnectionResetError:
            print("\n[SERVER] Connection Reset!")
            continue  # Jump to wait for the next message

        # Check if message comes from any of the clients
        try:
            ID = st.session_state.server.client_addresses[address]  # Client Identifier

        except:
            if verbose:
                print("> Client not recognized")

            continue  # Jump to wait for the next message

        # Show sender
        if verbose:
            print(f"> Received message from Client {ID} ({address[0]}, {address[1]})")

        # Save message
        st.session_state.server.clients[ID].message_log.append(message_bytes)

    # Post-processing
    for ID, client in enumerate(st.session_state.server.clients):
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
            if message.size != 3 * blob_count + 2:

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
            st.session_state.server.triangulator.save(ID, frame_idx, undistorted_blobs)

    st.session_state.wand_blobs = st.session_state.server.triangulator.full_vision()

    if not st.session_state.server.multiple_view.calibrate(
        st.session_state.wand_blobs, st.session_state.wand_distances_calibration
    ):
        placeholder.error("Calibration failed!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()
        sys.exit()

    else:
        placeholder.success("Calibration successful!", icon="✅")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()

    # Create the Scene Viewer
    scene = Viewer3D(title="Calibrated Camera Poses", size=10)

    # Add camera frames to the scene
    for ID, camera in enumerate(st.session_state.server.multiple_view.camera_models):
        scene.add_frame(camera.pose, f"Camera {ID}", axis_size=0.4)

    st.plotly_chart(scene.figure)


if st.button("Bundle Adjustment"):
    placeholder = st.empty()

    if st.session_state.wand_blobs is not None:
        placeholder.info("Performing bundle adjustment...", icon="ℹ️")

        # Perform bundle adjustment
        n_observations = 72  # Choose a number multiple of the total number of unique pairs: n_cameras * (n_cameras - 1) / 2
        st.session_state.server.multiple_view.bundle_adjustment(
            st.session_state.wand_blobs,
            st.session_state.wand_distances_calibration,
            n_observations,
        )

        # Create the Scene Viewer
        scene = Viewer3D(title="Adjusted Camera Poses", size=10)

        # Add camera frames to the scene
        for ID, camera in enumerate(
            st.session_state.server.multiple_view.camera_models
        ):
            scene.add_frame(camera.pose, f"Camera {ID}", axis_size=0.4)

        st.plotly_chart(scene.figure)

        placeholder.success("Bundle adjustment successful!", icon="✅")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()

    else:
        placeholder.error("Cannot perform bundle adjustment!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()
        sys.exit()


if st.button("Reference Update"):
    placeholder = st.empty()
    placeholder.info("Reference update requested", icon="ℹ️")

    # Capture specifications
    blob_count = 3  # Number of expected markers
    verbose = True

    # Request capture
    if not st.session_state.server.request_sync_capture(
        calibration_delay, calibration_duration
    ):
        placeholder.error("Reference request failed!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()
        sys.exit()

    else:
        placeholder.success(
            f"Reference request successful! Waiting {calibration_delay} s...", icon="✅"
        )
        time.sleep(calibration_delay)  # Wait before disappearing

    timeout = 5.0  # In seconds
    st.session_state.server.udp_socket.settimeout(timeout)  # Set server timeout

    placeholder.info(
        f"Running reference update for {calibration_duration} s...", icon="ℹ️"
    )

    # Receiving messages
    while True:
        # Wait for message - Event guided!
        try:
            message_bytes, address = st.session_state.server.udp_socket.recvfrom(
                st.session_state.server.buffer_size
            )

        except TimeoutError:
            print("\n[SERVER] Timed Out!")
            break  # Close capture loop due to timeout

        except ConnectionResetError:
            print("\n[SERVER] Connection Reset!")
            continue  # Jump to wait for the next message

        # Check if client exists
        try:
            ID = st.session_state.server.client_addresses[address]  # Client Identifier

        except:
            if verbose:
                print("> Client not recognized")

            continue  # Jump to wait for the next message

        # Show sender
        if verbose:
            print(f"> Received message from Client {ID} ({address[0]}, {address[1]})")

        # Save message
        st.session_state.server.clients[ID].message_log.append(message_bytes)

    # Post-processing
    for ID, client in enumerate(st.session_state.server.clients):
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
            if message.size != 3 * blob_count + 2:

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
            st.session_state.server.triangulator.save(ID, frame_idx, undistorted_blobs)

    # Triangulation pair
    pair = (0, 2)  # Diagonal pairs seems to produce more stable results

    full_vision = st.session_state.server.triangulator.full_vision()
    wand_blobs_reference = [full_vision[ID] for ID in pair]

    # Update reference
    st.session_state.server.multiple_view.update_reference(
        wand_blobs_reference, st.session_state.wand_distances_reference, pair
    )

    # Create the Scene Viewer
    scene = Viewer3D(title="Updated Camera Poses", size=10)

    # Add camera frames to the scene
    for ID, camera in enumerate(st.session_state.server.multiple_view.camera_models):
        scene.add_frame(camera.pose, f"Camera {ID}", axis_size=0.4)

    # Add new reference
    scene.add_frame(np.eye(4), "Reference", axis_size=0.4)

    st.plotly_chart(scene.figure)

    placeholder.success("Reference update successful!", icon="✅")
    time.sleep(5)  # Wait 5 seconds before disappearing
    placeholder.empty()


if st.button("Save Calibration"):
    placeholder = st.empty()

    if st.session_state.server.multiple_view is None:
        placeholder.error("Cannot save calibration!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()
        sys.exit()

    else:
        # Save calibration in disk
        st.session_state.server.save_calibration()

        placeholder.success("Calibration saved!", icon="✅")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()

st.markdown("---")

st.subheader("📸 Capture Scene")

# User inputs
blob_count = st.number_input("Marker count", min_value=1, step=1)
capture_delay = st.number_input("Capture delay (s)", min_value=0.0, step=1.0)
capture_duration = st.number_input("Capture duration (s)", min_value=1.0, step=1.0)

if st.button("Start Capture"):
    placeholder = st.empty()
    placeholder.info("Standard capture requested", icon="ℹ️")

    verbose = True
    visualizer_address = ("127.0.0.1", 6666)
    all_triangulated_markers = []

    # Request capture
    if not st.session_state.server.request_sync_capture(
        capture_delay, capture_duration
    ):
        placeholder.error("Capture request failed!", icon="🚨")
        time.sleep(5)  # Wait 5 seconds before disappearing
        placeholder.empty()
        sys.exit()

    else:
        placeholder.success(
            f"Capture request successful! Waiting {capture_delay} s...", icon="✅"
        )
        time.sleep(capture_delay)  # Wait before disappearing

    timeout = 5.0  # In seconds
    st.session_state.server.udp_socket.settimeout(timeout)  # Set server timeout

    placeholder.info(f"Running standard capture for {capture_duration} s...", icon="ℹ️")

    # Breaks in the timeout
    while True:
        # Wait for message - Event guided!
        try:
            message_bytes, address = st.session_state.server.udp_socket.recvfrom(
                st.session_state.server.buffer_size
            )

        except TimeoutError:
            print("\n[SERVER] Timed Out!")
            break  # Close capture loop due to timeout

        except ConnectionResetError:
            print("\n[SERVER] Connection Reset!")
            continue  # Jump to wait for the next message

        # Check if message comes from any of the clients
        try:
            ID = st.session_state.server.client_addresses[address]  # Client Identifier

        except:
            if verbose:
                print("> Address not recognized")

            continue  # Jump to wait for the next message

        # Show sender
        if verbose:
            print(f"> Received message from Client {ID} ({address[0]}, {address[1]}):")

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
        if message.size != 3 * blob_count + 2:

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
        undistorted_blobs = st.session_state.server.clients[ID].camera.undistort_points(
            blob_centroids
        )

        # Print blobs
        if verbose:
            print(f"\tDetected Blobs - {frame_idx}")
            print("\t" + str(blob_data).replace("\n", "\n\t"))

        triangulated_markers = st.session_state.server.triangulator.triangulate(
            ID, frame_idx, undistorted_blobs
        )

        if triangulated_markers is None:
            continue  # Jump to wait for the next message

        if verbose:
            print("Triangulated!")

        # Send data to CoppeliaSim
        buffer = triangulated_markers.astype(np.float32).ravel().tobytes()
        st.session_state.server.udp_socket.sendto(buffer, visualizer_address)

        # Save data for plotting
        try:
            all_triangulated_markers.append(triangulated_markers)

        except:
            pass  # Don't access array if index is out of bounds

    # Join collected data
    all_triangulated_markers = np.hstack(all_triangulated_markers)

    # Create the Scene Viewer
    scene = Viewer3D(title="Capture Profile", size=10)

    # Add camera frames to the scene
    for ID, camera in enumerate(st.session_state.server.multiple_view.camera_models):
        scene.add_frame(camera.pose, f"Camera {ID}", axis_size=0.4)

    # Add reference
    scene.add_frame(np.eye(4), "Reference", axis_size=0.4)

    # Add triangulated markers to the scene
    scene.add_points(all_triangulated_markers, f"Triangulated positions")

    # Plot scene
    st.plotly_chart(scene.figure)

    placeholder.success("Standard capture successful!", icon="✅")
    time.sleep(5)  # Wait 5 seconds before disappearing
    placeholder.empty()


st.markdown("---")
st.markdown(
    "Developed by [@loolirer](https://github.com/loolirer) and [@debOliveira](https://github.com/debOliveira)."
)
