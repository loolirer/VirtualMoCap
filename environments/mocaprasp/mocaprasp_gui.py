# Importing modules...
import os
import time
import socket
import numpy as np
import streamlit as st
from multiprocessing import Process

from virtualmocap.plot.viewer3d import Viewer3D
from virtualmocap.vision.camera import Camera
from virtualmocap.integration.client import Client
from virtualmocap.integration.mocaprasp.server import MoCapRasp_Server


def format_matrix_latex(matrix):
    rows = [" & ".join(map(str, row)) for row in matrix]
    return "\\begin{bmatrix}\n" + " \\\\ \n".join(rows) + "\n\\end{bmatrix}"


def plot_calibration(server, title):
    # Create the Scene Viewer
    scene = Viewer3D(title=title, size=10)

    # Add camera frames to the scene
    for client in server.clients:
        scene.add_frame(client.camera.pose, client.alias, axis_size=0.4)

    scene.add_frame(np.eye(4), f"Reference", axis_size=0.4)

    return scene


if "first_run" not in st.session_state:
    st.session_state.first_run = True
else:
    st.session_state.first_run = False

# How much time to wait before disappearing
if "message_timeout" not in st.session_state:
    st.session_state.message_timeout = 2  # In seconds

# Collected calibration data
if "wand_blobs" not in st.session_state:
    st.session_state.wand_blobs = None

# Collected capture data
if "triangulated_markers" not in st.session_state:
    st.session_state.condensed_output = []

# Timed capture flag
if "disable_timed_capture" not in st.session_state:
    st.session_state.disable_timed_capture = True

if "online_capture_process" not in st.session_state:
    st.session_state.online_capture_process = Process(daemon=True)

if "cache_directory" not in st.session_state:
    st.session_state.cache_directory = "cache/"
    os.makedirs(
        st.session_state.cache_directory, exist_ok=True
    )  # Create the folder if it doesn't exist

st.set_page_config(page_title="Motion Capture Arena", layout="centered")
st.image("assets/mocaprasp.png")

setup_tab, calibration_tab, capture_tab = st.tabs(["⚙️", "⚖️", "📸"])

with setup_tab:
    st.subheader("⚙️ Arena Setup")

    # Create server
    if "server" not in st.session_state:
        placeholder = st.empty()
        placeholder.info("Preparing clients...", icon="ℹ️")

        # Create server
        st.session_state.server = MoCapRasp_Server(
            config_path="config.yaml", server_address=("0.0.0.0", 25565)
        )

        placeholder.empty()  # Clear info message

    register_clients_flag = st.button("Register Clients", use_container_width=True)

    if register_clients_flag or st.session_state.first_run:
        st.session_state.server.register_clients()

    for client in st.session_state.server.clients:
        status_color = "green" if client.active else "red"
        status_icon = "🟢" if client.active else "🔴"
        status_text = "Online" if client.active else "Offline"

        with st.expander(label=rf"### {status_icon} :{status_color}[**{client.alias}**]"):
            st.markdown(f"**Status:** {status_text}")
            if client.address:
                st.markdown(f"**IP Address:** `{client.address[0]}`")
            st.markdown(f"**MAC Address:** `{client.mac_address}`")

            # Intrinsics matrix as LaTeX
            st.markdown("**Intrinsic Matrix:**")
            st.latex(r"K = " + format_matrix_latex(client.camera.intrinsic_matrix))

            # Distortion coefficients as LaTeX array
            distortion_str = " & ".join(map(str, client.camera.distortion_coefficients))
            st.markdown("**Distortion Coefficients:**")
            st.latex(r"k_d = " + r"\begin{bmatrix}" + distortion_str + r"\end{bmatrix}")


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

        # Register clients
        st.session_state.server.register_clients()

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

    capture_path = os.path.join(st.session_state.cache_directory, "capture.csv")

    capture_columns = st.columns([1, 1])

    with capture_columns[1]:
        st.caption("Time Limited Capture")
        st.session_state.disable_timed_capture = not st.checkbox(
            "Enable Timed Capture", value=False
        )
        st.write("")

        st.caption("Capture Delay (s)")
        capture_delay = st.number_input(
            label="Capture Delay (s)",
            min_value=0,
            step=1,
            label_visibility="collapsed",
            disabled=st.session_state.disable_timed_capture,
        )

        if st.session_state.disable_timed_capture:
            capture_delay = 0

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

    with capture_columns[0]:
        st.caption("Expected Markers")
        expected_markers = st.number_input(
            label="Marker count", min_value=0, step=1, label_visibility="collapsed"
        )

        st.caption("Capture Duration (s)")
        capture_duration = st.number_input(
            label="Capture Duration (s)",
            min_value=1,
            step=1,
            label_visibility="collapsed",
            disabled=st.session_state.disable_timed_capture,
        )

        if st.session_state.disable_timed_capture:
            capture_duration = -1

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

        start_capture_flag = False

        start_capture_flag = st.button(
            label="Start Capture",
            use_container_width=True,
            disabled=publishing_ip is None,
        )

    if start_capture_flag:
        placeholder = st.empty()

        # Request capture (start simulation)
        if publishing_ip is None:
            placeholder.error("Publishing hostname is not valid!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        elif st.session_state.online_capture_process.is_alive():
            placeholder.error(
                "Last capture is still alive! Finish it to begin new one.", icon="🚨"
            )
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        elif not st.session_state.server.request_sync_capture(
            delay_time=capture_delay, capture_time=capture_duration
        ):
            placeholder.error("Capture request failed!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

        else:
            # Call new thread
            st.session_state.online_capture_process = Process(
                target=st.session_state.server.online_capture,
                kwargs={
                    "expected_markers": expected_markers,
                    "visualizer_address": (publishing_ip, publishing_port),
                    "max_head": 4,
                    "max_hold": 4,
                    "reprojection_tol": 1,
                    "collinearity_tol": 0.01,
                    "min_views": 3, 
                    "capture_path": capture_path,
                    "verbose": False,
                },
                daemon=True,
            )

            st.session_state.online_capture_process.start()

    if terminate_capture_flag:
        placeholder = st.empty()

        if not st.session_state.server.request_sync_capture(
            delay_time=0, capture_time=0
        ):
            placeholder.error("Termination request failed!", icon="🚨")
            time.sleep(st.session_state.message_timeout)  # Wait before disappearing
            placeholder.empty()

    placeholder = st.empty()

    if st.session_state.online_capture_process.is_alive():
        placeholder.success("Running capture!", icon="✅")

    # Block plotting if the thread is running
    while st.session_state.online_capture_process.is_alive():
        continue

    placeholder.empty()

    scene = plot_calibration(server=st.session_state.server, title="Capture Profile")

    try:
        condensed_capture_output = np.loadtxt(capture_path, delimiter=",").reshape((3, -1))
        scene.add_points(condensed_capture_output, f"Triangulated markers")

    except FileNotFoundError:
        print("[ERROR] File not found")
        pass

    except:
        print("[ERROR] Corrupted file")
        pass

    # Plot scene
    st.plotly_chart(scene.figure)


st.markdown("---")
st.markdown(
    "Developed by [@loolirer](https://github.com/loolirer) and [@debOliveira](https://github.com/debOliveira)."
)
