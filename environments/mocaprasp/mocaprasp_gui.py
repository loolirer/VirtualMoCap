# Importing modules...
import os
import time
import socket
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


def plot_calibration(server, title):
    # Create the Scene Viewer
    scene = Viewer3D(title=title, size=10)

    # Add camera frames to the scene
    if server.multiple_view is not None:
        for ID, camera in enumerate(server.multiple_view.camera_models):
            scene.add_frame(camera.pose, f"Camera {ID}", axis_size=0.4)

        scene.add_frame(np.eye(4), f"Reference", axis_size=0.4)

    return scene


# How much time to wait before disappearing
if "message_timeout" not in st.session_state:
    st.session_state.message_timeout = 3  # In seconds

# Create server
if "server" not in st.session_state:
    # Create server
    st.session_state.server = MoCapRasp_Server(server_address=("0.0.0.0", 25565))

# Collected calibration data
if "wand_blobs" not in st.session_state:
    st.session_state.wand_blobs = None

# Collected capture data
if "triangulated_markers" not in st.session_state:
    st.session_state.triangulated_markers = None

# Timed capture flag
if "timed_capture" not in st.session_state:
    st.session_state.timed_capture = False

st.set_page_config(page_title="Motion Capture Arena", layout="centered")
st.image("mocaprasp.png")

setup_tab, calibration_tab, capture_tab = st.tabs(["⚙️", "⚖️", "📸"])

with setup_tab:
    st.subheader("⚙️ Arena Setup")

    client_setup_columns = st.columns([1, 1])

    with client_setup_columns[0]:
        st.caption("\u200d")
        load_intrinsics_flag = st.button("Load Intrinsics", use_container_width=True)

    with client_setup_columns[1]:
        st.caption("\u200d")
        register_clients_flag = st.button("Register Clients", use_container_width=True)

    if load_intrinsics_flag:
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
        time.sleep(st.session_state.message_timeout)  # Wait before disappearing
        placeholder.empty()

    if register_clients_flag:
        placeholder = st.empty()

        # Register clients
        if not st.session_state.server.register_clients():
            placeholder.error("Client register failed!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            placeholder.success(f"Client register successful!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

with calibration_tab:
    st.subheader("⚖️ System Calibration")

    calibration_path = "calibration/"
    os.makedirs(
        calibration_path, exist_ok=True
    )  # Create the folder if it doesn't exist
    calibrations = sorted(
        [
            name
            for name in os.listdir(calibration_path)
            if os.path.isdir(os.path.join(calibration_path, name))
        ]
    )

    selected_folder = st.selectbox("Select a Calibration", [""] + calibrations)

    if (
        st.button("Load Calibration", use_container_width=True)
        and selected_folder != ""
    ):
        placeholder = st.empty()

        full_path = os.path.join(calibration_path, selected_folder)

        st.session_state.server.load_calibration(full_path)

        scene = plot_calibration(
            server=st.session_state.server, title="Loaded Calibration"
        )

        st.plotly_chart(scene.figure)

        placeholder.success("Calibration loaded!", icon="✅")
        time.sleep(st.session_state.message_timeout)  # Wait before disappearing
        placeholder.empty()

    extrinsic_calibration_columns = st.columns([1, 1])

    with extrinsic_calibration_columns[0]:
        st.caption("\u200d")
        extrinsic_calibration_flag = st.button(
            "Extrinsic Calibration", use_container_width=True
        )

        st.caption("AB Distance (cm)")
        AB = st.number_input(
            label="AB Distance (cm)",
            min_value=0.0,
            value=5.40,
            format="%0.3f",
            label_visibility="collapsed",
        )

    with extrinsic_calibration_columns[1]:
        st.caption("Capture Duration (s)")
        calibration_duration = st.number_input(
            label="Extrinsic Calibration Duration (s)",
            min_value=15.0,
            value=30.0,
            step=15.0,
            format="%0.1f",
            label_visibility="collapsed",
        )

        st.caption("BC Distance (cm)")
        BC = st.number_input(
            label="BC Distance (cm)",
            min_value=0.0,
            value=10.25,
            format="%0.3f",
            label_visibility="collapsed",
        )

    # Calibration wand distances
    if "wand_distances_calibration" not in st.session_state:
        st.session_state.wand_distances_calibration = (
            np.array([AB, BC, AB + BC]) * 1e-2
        )  # In meters

    if extrinsic_calibration_flag:
        placeholder = st.empty()
        placeholder.info("Extrinsic calibration requested", icon="ℹ️")

        # Request capture (start simulation)
        if not st.session_state.server.request_sync_capture(
            delay_time=0.0, capture_time=calibration_duration
        ):
            placeholder.error("Calibration request failed!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            placeholder.success("Calibration request successful!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing

            st.session_state.server.offline_capture(expected_markers=3, timeout=5)

            st.session_state.calibration_blobs = (
                st.session_state.server.triangulator.full_vision()
            )

            if not st.session_state.server.multiple_view.calibrate(
                st.session_state.calibration_blobs,
                st.session_state.wand_distances_calibration,
            ):
                placeholder.error("Calibration failed!", icon="🚨")
                time.sleep(st.session_state.message_timeout)  # Wait before disappearing
                placeholder.empty()

            else:
                placeholder.success("Calibration successful!", icon="✅")
                time.sleep(st.session_state.message_timeout)  # Wait before disappearing
                placeholder.empty()

                scene = plot_calibration(
                    server=st.session_state.server, title="Calibrated Camera Poses"
                )

                st.plotly_chart(scene.figure)

    bundle_adjusment_columns = st.columns([1, 1])

    with bundle_adjusment_columns[0]:
        st.caption("\u200d")
        bundle_adjustment_flag = st.button(
            "Bundle Adjustment", use_container_width=True
        )

    with bundle_adjusment_columns[1]:
        st.caption("Number of Observations")

        min_samples = 0
        if st.session_state.server.multiple_view is not None:
            n_cameras = st.session_state.server.multiple_view.n_cameras
            min_samples = int(n_cameras * (n_cameras - 1) / 2)

        # Choose a number multiple of the total number of unique pairs: n_cameras * (n_cameras - 1) / 2
        n_observations = st.number_input(
            "Choose a number of samples",
            min_value=min_samples,
            max_value=20 * min_samples,
            value=10 * min_samples,
            step=min_samples,
            label_visibility="collapsed",
            disabled=st.session_state.server.multiple_view is None,
        )

    if bundle_adjustment_flag:
        placeholder = st.empty()

        if st.session_state.calibration_blobs is not None:
            placeholder.info("Performing bundle adjustment...", icon="ℹ️")

            st.session_state.server.multiple_view.bundle_adjustment(
                st.session_state.calibration_blobs,
                st.session_state.wand_distances_calibration,
                n_observations,
            )

            scene = plot_calibration(
                server=st.session_state.server, title="Adjusted Camera Poses"
            )

            st.plotly_chart(scene.figure)

            placeholder.success("Bundle adjustment successful!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            placeholder.error("Cannot perform bundle adjustment!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

    reference_update_columns = st.columns([1, 1])

    with reference_update_columns[0]:
        st.caption("\u200d")
        reference_update_flag = st.button("Reference Update", use_container_width=True)

        st.caption("OX Distance (cm)")
        OX = st.number_input(
            label="OX Distance (cm)",
            min_value=0.0,
            value=10.25,
            format="%0.3f",
            label_visibility="collapsed",
        )

    with reference_update_columns[1]:
        st.caption("Capture Duration (s)")
        reference_duration = st.number_input(
            label="Reference Update Duration (s)",
            min_value=1.0,
            value=1.0,
            step=1.0,
            format="%0.1f",
            label_visibility="collapsed",
        )

        st.caption("OY Distance (cm)")
        OY = st.number_input(
            label="OY Distance (cm)",
            min_value=0.0,
            value=15.10,
            format="%0.3f",
            label_visibility="collapsed",
        )

    # Measured distances between perpendicularly matched marker distances
    if "wand_distances_reference" not in st.session_state:
        st.session_state.wand_distances_reference = (
            np.array([OX, OY]) * 1e-2
        )  # In meters

    if reference_update_flag:
        placeholder = st.empty()
        placeholder.info("Reference update requested", icon="ℹ️")

        # Request capture (start simulation)
        if not st.session_state.server.request_sync_capture(
            delay_time=0.0, capture_time=reference_duration
        ):
            placeholder.error("Reference request failed!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            placeholder.success("Reference request successful!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing

            st.session_state.server.offline_capture(expected_markers=3, timeout=5)

            reference_blobs = st.session_state.server.triangulator.full_vision()

            pair = (0, 2)  # Diagonal pairs seems to produce more stable results
            st.session_state.reference_blobs = [reference_blobs[ID] for ID in pair]

            # Update reference
            st.session_state.server.multiple_view.update_reference(
                st.session_state.reference_blobs,
                st.session_state.wand_distances_reference,
                pair,
            )

            scene = plot_calibration(
                server=st.session_state.server, title="Updated Camera Poses"
            )

            st.plotly_chart(scene.figure)

            placeholder.success("Reference update successful!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

    if st.button("Save Calibration", use_container_width=True):
        placeholder = st.empty()

        if st.session_state.server.multiple_view is None:
            placeholder.error("Cannot save calibration!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            # Save calibration in disk
            st.session_state.server.save_calibration()

            placeholder.success("Calibration saved!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

with capture_tab:
    st.subheader("📸 Capture Scene")

    capture_columns = st.columns([1, 1, 1])

    with capture_columns[0]:
        st.caption("Expected Markers")
        expected_markers = st.number_input(
            label="Marker count", min_value=1, step=1, label_visibility="collapsed"
        )

        st.caption("Publishing Hostname")
        publishing_hostname = st.text_input(
            label="Publishing Hostname",
            value=socket.gethostname(),
            label_visibility="collapsed",
        )

        # Check if publishing hostname is resolvable
        try:
            publishing_ip = socket.gethostbyname(publishing_hostname)

        except:
            publishing_ip = None

        start_capture_flag = st.button("Start Capture", use_container_width=True)

    with capture_columns[1]:
        st.caption("Capture Delay (s)")
        capture_delay = st.number_input(
            label="Capture Delay (s)",
            min_value=0,
            step=1,
            label_visibility="collapsed",
        )

        st.caption("Publishing Port")
        publishing_port = st.number_input(
            label="Publishing Port",
            min_value=1024,
            max_value=65535,
            value=6666,
            step=1,
            label_visibility="collapsed",
        )

        terminate_capture_flag = st.button(
            "Terminate Capture", use_container_width=True
        )

    with capture_columns[2]:
        st.caption("Capture Duration (s)")
        capture_duration = st.number_input(
            label="Capture Duration (s)",
            min_value=1,
            step=1,
            label_visibility="collapsed",
            disabled=st.session_state.timed_capture,
        )

        if not st.session_state.timed_capture:
            capture_duration = -1

        st.caption("Make capture timed")
        st.session_state.timed_capture = st.checkbox("Timed Capture")



    if start_capture_flag:
        placeholder = st.empty()
        placeholder.info("Standard capture requested", icon="ℹ️")

        # Request capture (start simulation)
        if publishing_ip is None:
            placeholder.error("Publishing hostname is not valid!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        elif not st.session_state.server.request_sync_capture(
            delay_time=capture_delay, capture_time=capture_duration
        ):
            placeholder.error("Capture request failed!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            placeholder.success("Capture request successful!", icon="✅")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing

            st.session_state.triangulated_markers = (
                st.session_state.server.online_capture(
                    expected_markers=expected_markers,
                    visualizer_address=(publishing_ip, publishing_port),
                )
            )

        if terminate_capture_flag:
            placeholder = st.empty()
            placeholder.info("Requested capture termination", icon="ℹ️")

            # Request capture (start simulation)
            if publishing_ip is None:
                placeholder.error("Publishing hostname is not valid!", icon="🚨")
                time.sleep(st.session_state.message_timeout)  # Wait before disappearing
                placeholder.empty()

            elif not st.session_state.server.request_sync_capture(
                delay_time=0, capture_time=0
            ):
                placeholder.error("Termination request failed!", icon="🚨")
                time.sleep(st.session_state.message_timeout)  # Wait before disappearing
                placeholder.empty()

            else:
                placeholder.success("Termination request successful!", icon="✅")
                time.sleep(st.session_state.message_timeout)  # Wait before disappearing

                st.session_state.triangulated_markers = (
                    st.session_state.server.online_capture(
                        expected_markers=expected_markers,
                        visualizer_address=(publishing_ip, publishing_port),
                    )
                )

    scene = plot_calibration(server=st.session_state.server, title="Capture Profile")

    # Add triangulated markers to the scene
    if st.session_state.triangulated_markers is not None:
        scene.add_points(st.session_state.triangulated_markers, f"Triangulated positions")

    # Plot scene
    st.plotly_chart(scene.figure)


st.markdown("---")
st.markdown(
    "Developed by [@loolirer](https://github.com/loolirer) and [@debOliveira](https://github.com/debOliveira)."
)
