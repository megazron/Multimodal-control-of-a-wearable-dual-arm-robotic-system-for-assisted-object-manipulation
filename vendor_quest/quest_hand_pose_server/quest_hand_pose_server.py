from __future__ import annotations

import argparse
import asyncio
import copy
import json
import math
import shutil
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from websockets.asyncio.server import serve


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
ADB_CHECK_INTERVAL_SECONDS = 2.0
GUI_REFRESH_MS = 100
TERMINAL_PRINT_EVERY_N_PACKETS = 3
GUI_POSITION_SCALE = 3.0
GUI_HAND_AXIS_LENGTH = 0.42


@dataclass
class RuntimeState:
    lock: threading.Lock = field(default_factory=threading.Lock)

    adb_status: str = "Checking..."
    adb_serial: str = "-"
    reverse_status: str = "Not configured"

    server_running: bool = False
    server_address: str = "-"
    client_connected: bool = False
    client_address: str = "-"

    packet_count: int = 0
    packet_rate_hz: float = 0.0
    last_sequence: int = -1
    skipped_sequence_count: int = 0
    reordered_sequence_count: int = 0
    latest_packet: dict[str, Any] = field(default_factory=dict)
    packet_times: deque[float] = field(
        default_factory=lambda: deque(maxlen=120)
    )

    last_error: str = ""

    def set_adb(
        self,
        status: str,
        serial: str = "-",
        reverse_status: str = "Not configured",
        error: str = "",
    ) -> None:
        with self.lock:
            self.adb_status = status
            self.adb_serial = serial
            self.reverse_status = reverse_status
            if error:
                self.last_error = error

    def set_server(self, running: bool, address: str) -> None:
        with self.lock:
            self.server_running = running
            self.server_address = address

    def set_client(
        self,
        connected: bool,
        address: str = "-",
    ) -> None:
        with self.lock:
            self.client_connected = connected
            self.client_address = address

            if connected:
                self.last_sequence = -1
                self.skipped_sequence_count = 0
                self.reordered_sequence_count = 0
                self.packet_count = 0
                self.packet_times.clear()

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message

    def update_packet(self, packet: dict[str, Any]) -> None:
        now = time.perf_counter()
        sequence = int(packet.get("sequence", -1))

        with self.lock:
            if self.last_sequence >= 0:
                difference = sequence - self.last_sequence

                if difference > 1:
                    self.skipped_sequence_count += difference - 1
                elif difference <= 0:
                    self.reordered_sequence_count += 1

            if self.last_sequence < 0 or sequence > self.last_sequence:
                self.last_sequence = sequence

            self.packet_count += 1
            self.latest_packet = packet
            self.packet_times.append(now)

            if len(self.packet_times) >= 2:
                duration = self.packet_times[-1] - self.packet_times[0]
                if duration > 0:
                    self.packet_rate_hz = (
                        len(self.packet_times) - 1
                    ) / duration

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "adb_status": self.adb_status,
                "adb_serial": self.adb_serial,
                "reverse_status": self.reverse_status,
                "server_running": self.server_running,
                "server_address": self.server_address,
                "client_connected": self.client_connected,
                "client_address": self.client_address,
                "packet_count": self.packet_count,
                "packet_rate_hz": self.packet_rate_hz,
                "last_sequence": self.last_sequence,
                "skipped_sequence_count": self.skipped_sequence_count,
                "reordered_sequence_count":
                    self.reordered_sequence_count,
                "latest_packet": copy.deepcopy(self.latest_packet),
                "last_error": self.last_error,
            }


def run_subprocess(
    command: list[str],
    timeout: float = 8.0,
) -> subprocess.CompletedProcess[str]:
    creationflags = 0
    if sys.platform == "win32":
        creationflags = getattr(
            subprocess,
            "CREATE_NO_WINDOW",
            0,
        )

    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
        creationflags=creationflags,
    )


def find_adb() -> str | None:
    return shutil.which("adb")


def read_adb_devices(adb_path: str) -> dict[str, str]:
    result = run_subprocess([adb_path, "devices"])

    if result.returncode != 0:
        raise RuntimeError(
            result.stderr.strip()
            or "adb devices failed."
        )

    devices: dict[str, str] = {}

    for raw_line in result.stdout.splitlines()[1:]:
        line = raw_line.strip()
        if not line or "\t" not in line:
            continue

        serial, status = line.split("\t", maxsplit=1)
        devices[serial.strip()] = status.strip()

    return devices


def ensure_adb_reverse(
    adb_path: str,
    serial: str,
    port: int,
) -> None:
    reverse_spec = f"tcp:{port}"

    result = run_subprocess(
        [
            adb_path,
            "-s",
            serial,
            "reverse",
            reverse_spec,
            reverse_spec,
        ]
    )

    if result.returncode != 0:
        raise RuntimeError(
            result.stderr.strip()
            or "adb reverse failed."
        )

    verify = run_subprocess(
        [adb_path, "-s", serial, "reverse", "--list"]
    )

    if verify.returncode != 0:
        raise RuntimeError(
            verify.stderr.strip()
            or "Could not verify adb reverse."
        )

    if reverse_spec not in verify.stdout:
        raise RuntimeError(
            f"Reverse mapping {reverse_spec} was not found."
        )


def adb_monitor_loop(
    state: RuntimeState,
    stop_event: threading.Event,
    port: int,
    preferred_serial: str | None,
) -> None:
    while not stop_event.is_set():
        adb_path = find_adb()

        if adb_path is None:
            state.set_adb(
                "ADB not found",
                error=(
                    "adb is not in PATH. Add Android Platform-Tools "
                    "to PATH, or launch from an environment where "
                    "adb is available."
                ),
            )
            stop_event.wait(ADB_CHECK_INTERVAL_SECONDS)
            continue

        try:
            devices = read_adb_devices(adb_path)

            if preferred_serial:
                status = devices.get(preferred_serial)
                if status is None:
                    state.set_adb(
                        "Preferred device not found",
                        serial=preferred_serial,
                    )
                    stop_event.wait(
                        ADB_CHECK_INTERVAL_SECONDS
                    )
                    continue
                serial = preferred_serial
            else:
                authorized = [
                    serial
                    for serial, status in devices.items()
                    if status == "device"
                ]
                unauthorized = [
                    serial
                    for serial, status in devices.items()
                    if status == "unauthorized"
                ]

                if not authorized:
                    if unauthorized:
                        state.set_adb(
                            "USB debugging not authorized",
                            serial=unauthorized[0],
                            error=(
                                "Put on the Quest headset and allow "
                                "USB debugging."
                            ),
                        )
                    else:
                        state.set_adb(
                            "No Quest/Android device detected"
                        )

                    stop_event.wait(
                        ADB_CHECK_INTERVAL_SECONDS
                    )
                    continue

                serial = authorized[0]

            if devices.get(serial) != "device":
                state.set_adb(
                    f"Device status: {devices.get(serial, 'unknown')}",
                    serial=serial,
                )
                stop_event.wait(
                    ADB_CHECK_INTERVAL_SECONDS
                )
                continue

            ensure_adb_reverse(adb_path, serial, port)

            state.set_adb(
                "Device connected",
                serial=serial,
                reverse_status=(
                    f"tcp:{port} → PC tcp:{port}"
                ),
            )

        except (
            RuntimeError,
            subprocess.SubprocessError,
            OSError,
        ) as error:
            state.set_adb(
                "ADB setup error",
                error=str(error),
            )

        stop_event.wait(ADB_CHECK_INTERVAL_SECONDS)


def format_vector3(value: dict[str, Any]) -> str:
    return (
        f"({float(value.get('x', 0.0)): .4f}, "
        f"{float(value.get('y', 0.0)): .4f}, "
        f"{float(value.get('z', 0.0)): .4f})"
    )


def format_quaternion(value: dict[str, Any]) -> str:
    return (
        f"({float(value.get('x', 0.0)): .4f}, "
        f"{float(value.get('y', 0.0)): .4f}, "
        f"{float(value.get('z', 0.0)): .4f}, "
        f"{float(value.get('w', 1.0)): .4f})"
    )


def print_packet_to_terminal(
    packet: dict[str, Any],
    snapshot: dict[str, Any],
) -> None:
    left = packet.get("left", {})
    right = packet.get("right", {})

    print(
        f"seq={snapshot['last_sequence']:7d} "
        f"rate={snapshot['packet_rate_hz']:5.1f} Hz "
        f"skipped={snapshot['skipped_sequence_count']:4d} "
        f"reordered={snapshot['reordered_sequence_count']:4d} "
        f"space={packet.get('space', 'unknown')}"
    )
    print(
        "  LEFT  "
        f"tracked={str(left.get('tracked', False)):<5} "
        f"confidence="
        f"{str(left.get('confidence', 'Unknown')):<7} "
        f"p={format_vector3(left.get('position', {}))} "
        f"q={format_quaternion(left.get('rotation', {}))}"
    )
    print(
        "  RIGHT "
        f"tracked={str(right.get('tracked', False)):<5} "
        f"confidence="
        f"{str(right.get('confidence', 'Unknown')):<7} "
        f"p={format_vector3(right.get('position', {}))} "
        f"q={format_quaternion(right.get('rotation', {}))}"
    )
    print()


async def handle_quest_connection(
    websocket: Any,
    state: RuntimeState,
    terminal_output: bool,
) -> None:
    remote = websocket.remote_address
    remote_text = str(remote) if remote else "USB tunnel"

    state.set_client(True, remote_text)
    print("Quest connected through USB ADB reverse.")

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                try:
                    message = message.decode("utf-8")
                except UnicodeDecodeError as error:
                    state.set_error(
                        f"Invalid UTF-8 packet: {error}"
                    )
                    continue

            try:
                packet = json.loads(message)
            except json.JSONDecodeError as error:
                state.set_error(
                    f"Invalid JSON packet: {error}"
                )
                continue

            if packet.get("protocol") != "wen.quest.handpose.v1":
                state.set_error(
                    "Unexpected protocol: "
                    f"{packet.get('protocol', 'unknown')}"
                )
                continue

            state.update_packet(packet)
            snapshot = state.snapshot()

            if (
                terminal_output
                and snapshot["packet_count"]
                % TERMINAL_PRINT_EVERY_N_PACKETS
                == 0
            ):
                print_packet_to_terminal(
                    packet,
                    snapshot,
                )

    except Exception as error:
        state.set_error(
            f"Quest connection closed: {error}"
        )

    finally:
        state.set_client(False)
        print("Quest disconnected.")


async def websocket_server_main(
    state: RuntimeState,
    stop_event: threading.Event,
    host: str,
    port: int,
    terminal_output: bool,
) -> None:
    address = f"ws://{host}:{port}"

    async def connection_handler(websocket: Any) -> None:
        await handle_quest_connection(
            websocket,
            state,
            terminal_output,
        )

    try:
        async with serve(
            connection_handler,
            host,
            port,
            ping_interval=None,
            max_size=1_000_000,
        ):
            state.set_server(True, address)
            print(f"WebSocket server listening on {address}")
            print("Waiting for Quest...")

            while not stop_event.is_set():
                await asyncio.sleep(0.1)

    except OSError as error:
        state.set_error(
            f"Could not start WebSocket server: {error}"
        )
        raise

    finally:
        state.set_server(False, address)


def quaternion_to_rotation_matrix(
    quaternion: dict[str, Any],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    x = float(quaternion.get("x", 0.0))
    y = float(quaternion.get("y", 0.0))
    z = float(quaternion.get("z", 0.0))
    w = float(quaternion.get("w", 1.0))

    norm = math.sqrt(x * x + y * y + z * z + w * w)

    if norm <= 1e-8:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


class PoseGui:
    def __init__(
        self,
        root: Any,
        state: RuntimeState,
        stop_event: threading.Event,
    ) -> None:
        import tkinter as tk
        from tkinter import ttk

        from matplotlib.backends.backend_tkagg import (
            FigureCanvasTkAgg,
        )
        from matplotlib.figure import Figure

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.state = state
        self.stop_event = stop_event

        root.title("Quest Hand Pose Server")
        root.geometry("1320x820")
        root.minsize(1050, 700)

        main = ttk.Frame(root, padding=8)
        main.pack(fill=tk.BOTH, expand=True)

        info_panel = ttk.Frame(main)
        info_panel.pack(
            side=tk.LEFT,
            fill=tk.Y,
            padx=(0, 8),
        )

        plot_panel = ttk.Frame(main)
        plot_panel.pack(
            side=tk.RIGHT,
            fill=tk.BOTH,
            expand=True,
        )

        self.status_variables = {
            name: tk.StringVar(value="-")
            for name in (
                "adb",
                "serial",
                "reverse",
                "server",
                "client",
                "rate",
                "sequence",
                "packet_quality",
                "active_hands",
                "left",
                "right",
                "error",
            )
        }

        self._create_status_panel(info_panel)

        figure = Figure(figsize=(8, 7), dpi=100)
        self.axes = figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(
            figure,
            master=plot_panel,
        )
        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True,
        )

        controls = ttk.Frame(plot_panel)
        controls.pack(fill=tk.X, pady=(6, 0))

        ttk.Label(
            controls,
            text=(
                "Visualization only: position ×3; "
                "translation vector = tracking origin → hand pose."
            ),
        ).pack(side=tk.LEFT)

        ttk.Button(
            controls,
            text="Close",
            command=self.close,
        ).pack(side=tk.RIGHT)

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()

    def _create_status_panel(self, parent: Any) -> None:
        ttk = self.ttk
        tk = self.tk

        connection = ttk.LabelFrame(
            parent,
            text="Connection",
            padding=8,
        )
        connection.pack(fill=tk.X, pady=(0, 8))

        self._add_status_row(
            connection,
            "ADB",
            "adb",
        )
        self._add_status_row(
            connection,
            "Device serial",
            "serial",
        )
        self._add_status_row(
            connection,
            "USB reverse",
            "reverse",
        )
        self._add_status_row(
            connection,
            "Server",
            "server",
        )
        self._add_status_row(
            connection,
            "Quest client",
            "client",
        )

        stream = ttk.LabelFrame(
            parent,
            text="Pose stream",
            padding=8,
        )
        stream.pack(fill=tk.X, pady=(0, 8))

        self._add_status_row(
            stream,
            "Receive rate",
            "rate",
        )
        self._add_status_row(
            stream,
            "Sequence",
            "sequence",
        )
        self._add_status_row(
            stream,
            "Sequence check",
            "packet_quality",
        )
        self._add_status_row(
            stream,
            "Active hands",
            "active_hands",
        )

        hands = ttk.LabelFrame(
            parent,
            text="Latest hand poses",
            padding=8,
        )
        hands.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(
            hands,
            text="LEFT",
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor=tk.W)

        ttk.Label(
            hands,
            textvariable=self.status_variables["left"],
            justify=tk.LEFT,
            wraplength=350,
        ).pack(anchor=tk.W, pady=(2, 10))

        ttk.Label(
            hands,
            text="RIGHT",
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor=tk.W)

        ttk.Label(
            hands,
            textvariable=self.status_variables["right"],
            justify=tk.LEFT,
            wraplength=350,
        ).pack(anchor=tk.W, pady=(2, 0))

        error_frame = ttk.LabelFrame(
            parent,
            text="Last message",
            padding=8,
        )
        error_frame.pack(fill=tk.X)

        ttk.Label(
            error_frame,
            textvariable=self.status_variables["error"],
            justify=tk.LEFT,
            wraplength=350,
        ).pack(anchor=tk.W)

    def _add_status_row(
        self,
        parent: Any,
        title: str,
        variable_name: str,
    ) -> None:
        ttk = self.ttk

        row = ttk.Frame(parent)
        row.pack(fill=self.tk.X, pady=2)

        ttk.Label(
            row,
            text=f"{title}:",
            width=15,
        ).pack(side=self.tk.LEFT)

        ttk.Label(
            row,
            textvariable=self.status_variables[variable_name],
            wraplength=245,
        ).pack(
            side=self.tk.LEFT,
            fill=self.tk.X,
            expand=True,
        )

    def close(self) -> None:
        self.stop_event.set()
        self.root.destroy()

    def refresh(self) -> None:
        if self.stop_event.is_set():
            return

        snapshot = self.state.snapshot()
        packet = snapshot["latest_packet"]

        self.status_variables["adb"].set(
            snapshot["adb_status"]
        )
        self.status_variables["serial"].set(
            snapshot["adb_serial"]
        )
        self.status_variables["reverse"].set(
            snapshot["reverse_status"]
        )
        self.status_variables["server"].set(
            snapshot["server_address"]
            if snapshot["server_running"]
            else "Stopped"
        )
        self.status_variables["client"].set(
            "Connected"
            if snapshot["client_connected"]
            else "Waiting"
        )
        self.status_variables["rate"].set(
            f"{snapshot['packet_rate_hz']:.1f} Hz"
        )
        self.status_variables["sequence"].set(
            str(snapshot["last_sequence"])
        )
        self.status_variables["packet_quality"].set(
            f"skipped={snapshot['skipped_sequence_count']}, "
            f"reordered={snapshot['reordered_sequence_count']}"
        )

        left = packet.get("left", {})
        right = packet.get("right", {})

        active_names: list[str] = []
        if left.get("tracked", False):
            active_names.append("Left")
        if right.get("tracked", False):
            active_names.append("Right")

        self.status_variables["active_hands"].set(
            " + ".join(active_names)
            if active_names
            else "None"
        )

        self.status_variables["left"].set(
            self._format_hand_for_gui(left)
        )
        self.status_variables["right"].set(
            self._format_hand_for_gui(right)
        )
        self.status_variables["error"].set(
            snapshot["last_error"] or "No error"
        )

        self._draw_pose(packet)
        self.root.after(GUI_REFRESH_MS, self.refresh)

    @staticmethod
    def _format_hand_for_gui(
        hand: dict[str, Any],
    ) -> str:
        return (
            f"tracked: {hand.get('tracked', False)}\n"
            f"confidence: "
            f"{hand.get('confidence', 'Unknown')}\n"
            f"position: "
            f"{format_vector3(hand.get('position', {}))}\n"
            f"rotation: "
            f"{format_quaternion(hand.get('rotation', {}))}"
        )

    def _draw_pose(
        self,
        packet: dict[str, Any],
    ) -> None:
        axes = self.axes
        axes.clear()

        axes.set_title(
            "Quest hand roots in Tracking Space — 3× display scale"
        )
        axes.set_xlabel("Quest X (m)")
        axes.set_ylabel("Quest Z (m)")
        axes.set_zlabel("Quest Y (m)")

        # Display Unity (x, y, z) as plot (x, z, y) so that
        # vertical Unity Y appears vertical on screen.
        axes.set_xlim(-1.5, 1.5)
        axes.set_ylim(-1.0, 3.0)
        axes.set_zlim(0.0, 5.0)
        axes.set_box_aspect((3.0, 4.0, 5.0))
        axes.grid(True)

        # Tracking-space origin.
        axes.scatter(
            [0.0],
            [0.0],
            [0.0],
            marker="+",
            s=80,
            label="Tracking origin",
        )

        left = packet.get("left", {})
        right = packet.get("right", {})

        self._draw_hand(
            axes,
            left,
            "Left",
            marker="o",
            translation_color="tab:blue",
        )
        self._draw_hand(
            axes,
            right,
            "Right",
            marker="^",
            translation_color="tab:orange",
        )

        axes.legend(loc="upper left")
        self.canvas.draw_idle()

    def _draw_hand(
        self,
        axes: Any,
        hand: dict[str, Any],
        name: str,
        marker: str,
        translation_color: str,
    ) -> None:
        if not hand.get("tracked", False):
            return

        position = hand.get("position", {})
        quaternion = hand.get("rotation", {})

        x = float(position.get("x", 0.0))
        y = float(position.get("y", 0.0))
        z = float(position.get("z", 0.0))

        # Visualization only:
        # enlarge the displayed translation by three times.
        # The raw values shown in the status panel are unchanged.
        px = GUI_POSITION_SCALE * x
        py = GUI_POSITION_SCALE * z
        pz = GUI_POSITION_SCALE * y

        # Translation vector from Tracking-Space origin
        # to the displayed hand-pose origin.
        axes.quiver(
            0.0,
            0.0,
            0.0,
            px,
            py,
            pz,
            length=1.0,
            normalize=False,
            color=translation_color,
            linewidth=2.2,
            arrow_length_ratio=0.06,
            label=f"{name} translation",
        )

        axes.scatter(
            [px],
            [py],
            [pz],
            marker=marker,
            s=170,
            color=translation_color,
            label=f"{name} hand pose",
        )
        axes.text(
            px,
            py,
            pz,
            f"  {name[0]}",
            fontsize=14,
            fontweight="bold",
        )

        rotation = quaternion_to_rotation_matrix(
            quaternion
        )

        # Three-times larger than the previous 0.14 m display axes.
        axis_length = GUI_HAND_AXIS_LENGTH

        # Columns of the rotation matrix are local X/Y/Z axes.
        local_axes = (
            (
                rotation[0][0],
                rotation[1][0],
                rotation[2][0],
                "r",
            ),
            (
                rotation[0][1],
                rotation[1][1],
                rotation[2][1],
                "g",
            ),
            (
                rotation[0][2],
                rotation[1][2],
                rotation[2][2],
                "b",
            ),
        )

        for ux, uy, uz, color in local_axes:
            # Convert Unity vector (ux, uy, uz) to plot
            # vector (ux, uz, uy).
            axes.quiver(
                px,
                py,
                pz,
                ux,
                uz,
                uy,
                length=axis_length,
                normalize=True,
                color=color,
                linewidth=2.0,
                arrow_length_ratio=0.25,
            )

def start_background_thread(
    target: Any,
    name: str,
) -> threading.Thread:
    thread = threading.Thread(
        target=target,
        name=name,
        daemon=True,
    )
    thread.start()
    return thread


def run_gui_mode(
    state: RuntimeState,
    stop_event: threading.Event,
    host: str,
    port: int,
) -> None:
    try:
        import tkinter as tk

        import matplotlib  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            "GUI dependencies are missing. Install matplotlib "
            "and ensure tkinter is available."
        ) from error

    server_thread = start_background_thread(
        lambda: asyncio.run(
            websocket_server_main(
                state,
                stop_event,
                host,
                port,
                terminal_output=False,
            )
        ),
        "WebSocketServer",
    )

    root = tk.Tk()
    PoseGui(root, state, stop_event)
    root.mainloop()

    stop_event.set()
    server_thread.join(timeout=3.0)


def run_terminal_mode(
    state: RuntimeState,
    stop_event: threading.Event,
    host: str,
    port: int,
) -> None:
    try:
        asyncio.run(
            websocket_server_main(
                state,
                stop_event,
                host,
                port,
                terminal_output=True,
            )
        )
    except KeyboardInterrupt:
        print("\nStopping...")
        stop_event.set()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Quest hand pose server with automatic "
            "ADB reverse setup and optional GUI."
        )
    )

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--gui",
        action="store_true",
        help="Open the Tkinter + Matplotlib GUI.",
    )
    mode.add_argument(
        "--no-gui",
        action="store_true",
        help="Use terminal output only.",
    )

    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
    )
    parser.add_argument(
        "--serial",
        default=None,
        help=(
            "Optional ADB device serial when several devices "
            "are connected."
        ),
    )
    parser.add_argument(
        "--skip-adb",
        action="store_true",
        help="Start the server without configuring adb reverse.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    use_gui = args.gui or not args.no_gui

    state = RuntimeState()
    stop_event = threading.Event()

    adb_thread: threading.Thread | None = None

    if not args.skip_adb:
        adb_thread = start_background_thread(
            lambda: adb_monitor_loop(
                state,
                stop_event,
                args.port,
                args.serial,
            ),
            "AdbMonitor",
        )
    else:
        state.set_adb(
            "ADB setup skipped",
            reverse_status="Not managed by this program",
        )

    try:
        if use_gui:
            run_gui_mode(
                state,
                stop_event,
                args.host,
                args.port,
            )
        else:
            run_terminal_mode(
                state,
                stop_event,
                args.host,
                args.port,
            )

    except RuntimeError as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)

    finally:
        stop_event.set()

        if adb_thread is not None:
            adb_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
