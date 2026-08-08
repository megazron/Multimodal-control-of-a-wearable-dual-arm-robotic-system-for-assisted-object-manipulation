from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import tkinter as tk
from tkinter import ttk

import websockets
from websockets.exceptions import ConnectionClosed
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


HOST = "127.0.0.1"
PORT = 8766

GUI_REFRESH_MS = 100
LOCAL_AXIS_LENGTH_M = 0.25

# Plot bounds in Unity coordinates.
X_LIMIT = (-1.5, 1.5)
Y_LIMIT = (-0.2, 2.2)
Z_LIMIT = (-1.5, 1.5)


def empty_controller(side: str) -> dict[str, Any]:
    buttons = {"x": False, "y": False} if side == "left" else {
        "a": False,
        "b": False,
    }

    return {
        "connected": False,
        "positionTracked": False,
        "rotationTracked": False,
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "trigger": 0.0,
        "grip": 0.0,
        "thumbstick": {
            "x": 0.0,
            "y": 0.0,
            "pressed": False,
            "touched": False,
        },
        "buttons": buttons,
    }


def empty_frame() -> dict[str, Any]:
    return {
        "schemaVersion": 1,
        "sequence": 0,
        "timestamp": 0.0,
        "left": empty_controller("left"),
        "right": empty_controller("right"),
    }


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    frame: dict[str, Any] = field(default_factory=empty_frame)

    client_connected: bool = False
    client_address: str = "-"
    last_receive_monotonic: float = 0.0

    receive_count_window: int = 0
    receive_hz: float = 0.0
    rate_window_start: float = field(default_factory=time.monotonic)

    parse_error: str = ""

    def set_connection(self, connected: bool, address: str = "-") -> None:
        with self.lock:
            self.client_connected = connected
            self.client_address = address

            if not connected:
                self.receive_hz = 0.0
                self.receive_count_window = 0
                self.rate_window_start = time.monotonic()

    def update_message(self, message: str) -> None:
        try:
            parsed = json.loads(message)

            if not isinstance(parsed, dict):
                raise ValueError("JSON root is not an object.")

            if "left" not in parsed or "right" not in parsed:
                raise ValueError("JSON must contain left and right controllers.")

        except Exception as exc:
            with self.lock:
                self.parse_error = str(exc)
            return

        now = time.monotonic()

        with self.lock:
            self.frame = parsed
            self.last_receive_monotonic = now
            self.parse_error = ""

            self.receive_count_window += 1
            elapsed = now - self.rate_window_start

            if elapsed >= 1.0:
                self.receive_hz = self.receive_count_window / elapsed
                self.receive_count_window = 0
                self.rate_window_start = now

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            age = (
                time.monotonic() - self.last_receive_monotonic
                if self.last_receive_monotonic > 0.0
                else math.inf
            )

            return {
                "frame": deepcopy(self.frame),
                "client_connected": self.client_connected,
                "client_address": self.client_address,
                "receive_hz": self.receive_hz,
                "age": age,
                "parse_error": self.parse_error,
            }


class ControllerWebSocketServer:
    def __init__(self, state: SharedState) -> None:
        self.state = state
        self.loop: asyncio.AbstractEventLoop | None = None
        self.thread: threading.Thread | None = None
        self.server: Any = None
        self.ready_event = threading.Event()
        self.startup_error: Exception | None = None

    def start(self) -> None:
        self.thread = threading.Thread(
            target=self._thread_main,
            name="ControllerWebSocketServer",
            daemon=True,
        )
        self.thread.start()

        if not self.ready_event.wait(timeout=5.0):
            raise RuntimeError(
                "Timed out while starting the WebSocket server."
            )

        if self.startup_error is not None:
            raise self.startup_error

    def _thread_main(self) -> None:
        if sys.platform.startswith("win"):
            try:
                asyncio.set_event_loop_policy(
                    asyncio.WindowsSelectorEventLoopPolicy()
                )
            except AttributeError:
                pass

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        try:
            self.loop.run_until_complete(self._start_server())
            self.ready_event.set()
            self.loop.run_forever()
        except Exception as exc:
            self.startup_error = exc
            self.ready_event.set()
        finally:
            if self.server is not None:
                self.server.close()
                self.loop.run_until_complete(self.server.wait_closed())

            pending = asyncio.all_tasks(self.loop)
            for task in pending:
                task.cancel()

            if pending:
                self.loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )

            self.loop.close()

    async def _start_server(self) -> None:
        self.server = await websockets.serve(
            self._handle_client,
            HOST,
            PORT,
            ping_interval=20,
            ping_timeout=20,
            max_size=1_000_000,
        )

    async def _handle_client(
        self,
        websocket: Any,
        path: Any = None,
    ) -> None:
        del path

        address = str(getattr(websocket, "remote_address", "-"))
        self.state.set_connection(True, address)

        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    message = message.decode("utf-8")

                self.state.update_message(message)

        except ConnectionClosed:
            pass
        except Exception as exc:
            with self.state.lock:
                self.state.parse_error = f"Server error: {exc}"
        finally:
            self.state.set_connection(False)

    def stop(self) -> None:
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.loop.stop)


class IndicatorButton(tk.Label):
    def __init__(
        self,
        parent: tk.Misc,
        text: str,
        width: int = 10,
    ) -> None:
        super().__init__(
            parent,
            text=text,
            width=width,
            height=2,
            relief=tk.RAISED,
            borderwidth=2,
            font=("Segoe UI", 10, "bold"),
            bg="#d9d9d9",
            fg="#202020",
        )

    def set_active(self, active: bool) -> None:
        if active:
            self.configure(
                relief=tk.SUNKEN,
                bg="#73d673",
                fg="#101010",
            )
        else:
            self.configure(
                relief=tk.RAISED,
                bg="#d9d9d9",
                fg="#202020",
            )


class AnalogBar(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        label: str,
    ) -> None:
        super().__init__(parent)

        ttk.Label(
            self,
            text=label,
            width=10,
            anchor="w",
        ).grid(row=0, column=0, padx=(0, 5), sticky="w")

        self.progress = ttk.Progressbar(
            self,
            orient="horizontal",
            mode="determinate",
            maximum=1.0,
            length=180,
        )
        self.progress.grid(row=0, column=1, sticky="ew")

        self.value_label = ttk.Label(
            self,
            text="0.000",
            width=7,
            anchor="e",
        )
        self.value_label.grid(row=0, column=2, padx=(6, 0))

        self.columnconfigure(1, weight=1)

    def set_value(self, value: float) -> None:
        value = float(np.clip(value, 0.0, 1.0))
        self.progress["value"] = value
        self.value_label.configure(text=f"{value:.3f}")


class JoystickView(ttk.Frame):
    SIZE = 150
    MARGIN = 16

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent)

        self.canvas = tk.Canvas(
            self,
            width=self.SIZE,
            height=self.SIZE,
            bg="white",
            highlightthickness=1,
            highlightbackground="#999999",
        )
        self.canvas.pack()

        c = self.SIZE / 2
        r = self.SIZE / 2 - self.MARGIN

        self.canvas.create_oval(
            c - r,
            c - r,
            c + r,
            c + r,
            outline="#505050",
            width=2,
        )
        self.canvas.create_line(
            c,
            self.MARGIN,
            c,
            self.SIZE - self.MARGIN,
            fill="#b0b0b0",
        )
        self.canvas.create_line(
            self.MARGIN,
            c,
            self.SIZE - self.MARGIN,
            c,
            fill="#b0b0b0",
        )

        self.dot = self.canvas.create_oval(
            c - 6,
            c - 6,
            c + 6,
            c + 6,
            fill="#2563eb",
            outline="",
        )

        self.value_label = ttk.Label(
            self,
            text="X=+0.000   Y=+0.000",
            anchor="center",
        )
        self.value_label.pack(pady=(4, 0))

    def set_value(self, x: float, y: float) -> None:
        x = float(np.clip(x, -1.0, 1.0))
        y = float(np.clip(y, -1.0, 1.0))

        c = self.SIZE / 2
        r = self.SIZE / 2 - self.MARGIN

        px = c + x * r
        py = c - y * r

        self.canvas.coords(
            self.dot,
            px - 6,
            py - 6,
            px + 6,
            py + 6,
        )

        self.value_label.configure(
            text=f"X={x:+.3f}   Y={y:+.3f}"
        )


def quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(quaternion)

    if norm < 1e-9:
        return np.eye(3)

    x, y, z, w = quaternion / norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def unity_to_plot(vector_xyz: np.ndarray) -> np.ndarray:
    """
    Matplotlib uses its third coordinate as the visually vertical axis.
    Display Unity (X, Y, Z) as plot (X, Z, Y), so Unity Y is vertical.

    Numerical labels elsewhere remain the unmodified Unity X/Y/Z values.
    """
    return np.array(
        [vector_xyz[0], vector_xyz[2], vector_xyz[1]],
        dtype=float,
    )


class ControllerPanel(ttk.LabelFrame):
    def __init__(
        self,
        parent: tk.Misc,
        side: str,
    ) -> None:
        title = "Left Controller" if side == "left" else "Right Controller"
        super().__init__(parent, text=title, padding=8)

        self.side = side

        top = ttk.Frame(self)
        top.pack(fill=tk.BOTH, expand=True)

        plot_frame = ttk.Frame(top)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        controls_frame = ttk.Frame(top)
        controls_frame.pack(
            side=tk.RIGHT,
            fill=tk.Y,
            padx=(10, 0),
        )

        self.figure = Figure(figsize=(6.2, 5.6), dpi=100)
        self.axis = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(
            self.figure,
            master=plot_frame,
        )
        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True,
        )

        status_frame = ttk.LabelFrame(
            controls_frame,
            text="Tracking",
            padding=6,
        )
        status_frame.pack(fill=tk.X, pady=(0, 8))

        self.connection_label = ttk.Label(
            status_frame,
            text="Disconnected",
            width=24,
        )
        self.connection_label.pack(anchor="w")

        self.tracking_label = ttk.Label(
            status_frame,
            text="Position: No | Rotation: No",
            width=30,
        )
        self.tracking_label.pack(anchor="w")

        analog_frame = ttk.LabelFrame(
            controls_frame,
            text="Analog Input",
            padding=6,
        )
        analog_frame.pack(fill=tk.X, pady=(0, 8))

        self.trigger_bar = AnalogBar(analog_frame, "Trigger")
        self.trigger_bar.pack(fill=tk.X, pady=2)

        self.grip_bar = AnalogBar(analog_frame, "Grip")
        self.grip_bar.pack(fill=tk.X, pady=2)

        joystick_frame = ttk.LabelFrame(
            controls_frame,
            text="Joystick",
            padding=6,
        )
        joystick_frame.pack(fill=tk.X, pady=(0, 8))

        self.joystick_view = JoystickView(joystick_frame)
        self.joystick_view.pack()

        button_frame = ttk.LabelFrame(
            controls_frame,
            text="Buttons",
            padding=6,
        )
        button_frame.pack(fill=tk.X, pady=(0, 8))

        self.buttons: dict[str, IndicatorButton] = {}

        if side == "left":
            button_names = [
                ("x", "X"),
                ("y", "Y"),
                ("stick_pressed", "Stick Click"),
                ("stick_touched", "Stick Touch"),
            ]
        else:
            button_names = [
                ("a", "A"),
                ("b", "B"),
                ("stick_pressed", "Stick Click"),
                ("stick_touched", "Stick Touch"),
            ]

        for index, (key, text) in enumerate(button_names):
            button = IndicatorButton(
                button_frame,
                text=text,
                width=11,
            )
            button.grid(
                row=index // 2,
                column=index % 2,
                padx=4,
                pady=4,
                sticky="nsew",
            )
            self.buttons[key] = button

        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        pose_frame = ttk.LabelFrame(
            controls_frame,
            text="Raw Unity Pose",
            padding=6,
        )
        pose_frame.pack(fill=tk.X)

        self.pose_text = tk.Text(
            pose_frame,
            width=39,
            height=7,
            font=("Consolas", 9),
            state=tk.DISABLED,
        )
        self.pose_text.pack(fill=tk.X)

        self.update_data(empty_controller(side))

    def update_data(self, data: dict[str, Any]) -> None:
        connected = bool(data.get("connected", False))
        position_tracked = bool(data.get("positionTracked", False))
        rotation_tracked = bool(data.get("rotationTracked", False))

        self.connection_label.configure(
            text="Connected" if connected else "Disconnected",
            foreground="#147d18" if connected else "#a00000",
        )

        self.tracking_label.configure(
            text=(
                f"Position: {'Yes' if position_tracked else 'No'} | "
                f"Rotation: {'Yes' if rotation_tracked else 'No'}"
            )
        )

        trigger = float(data.get("trigger", 0.0))
        grip = float(data.get("grip", 0.0))

        self.trigger_bar.set_value(trigger)
        self.grip_bar.set_value(grip)

        thumbstick = data.get("thumbstick", {})
        stick_x = float(thumbstick.get("x", 0.0))
        stick_y = float(thumbstick.get("y", 0.0))

        self.joystick_view.set_value(stick_x, stick_y)

        buttons = data.get("buttons", {})

        if self.side == "left":
            self.buttons["x"].set_active(bool(buttons.get("x", False)))
            self.buttons["y"].set_active(bool(buttons.get("y", False)))
        else:
            self.buttons["a"].set_active(bool(buttons.get("a", False)))
            self.buttons["b"].set_active(bool(buttons.get("b", False)))

        self.buttons["stick_pressed"].set_active(
            bool(thumbstick.get("pressed", False))
        )
        self.buttons["stick_touched"].set_active(
            bool(thumbstick.get("touched", False))
        )

        position = data.get("position", {})
        orientation = data.get("orientation", {})

        px = float(position.get("x", 0.0))
        py = float(position.get("y", 0.0))
        pz = float(position.get("z", 0.0))

        qx = float(orientation.get("x", 0.0))
        qy = float(orientation.get("y", 0.0))
        qz = float(orientation.get("z", 0.0))
        qw = float(orientation.get("w", 1.0))

        self._update_pose_text(
            px,
            py,
            pz,
            qx,
            qy,
            qz,
            qw,
        )

        self._draw_pose(
            connected=connected,
            position_tracked=position_tracked,
            rotation_tracked=rotation_tracked,
            position=np.array([px, py, pz], dtype=float),
            quaternion=np.array([qx, qy, qz, qw], dtype=float),
        )

    def _update_pose_text(
        self,
        px: float,
        py: float,
        pz: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        text = (
            f"Position [m]\n"
            f"  X = {px:+.4f}\n"
            f"  Y = {py:+.4f}\n"
            f"  Z = {pz:+.4f}\n"
            f"Quaternion\n"
            f"  ({qx:+.4f}, {qy:+.4f},\n"
            f"   {qz:+.4f}, {qw:+.4f})"
        )

        self.pose_text.configure(state=tk.NORMAL)
        self.pose_text.delete("1.0", tk.END)
        self.pose_text.insert("1.0", text)
        self.pose_text.configure(state=tk.DISABLED)

    def _draw_pose(
        self,
        connected: bool,
        position_tracked: bool,
        rotation_tracked: bool,
        position: np.ndarray,
        quaternion: np.ndarray,
    ) -> None:
        ax = self.axis
        ax.clear()

        ax.set_title(
            f"{'Left' if self.side == 'left' else 'Right'} Controller Pose\n"
            "Black: origin-to-controller vector | "
            "RGB: local X/Y/Z axes",
            fontsize=10,
        )

        # The display order is (Unity X, Unity Z, Unity Y).
        ax.set_xlabel("Unity X [m]")
        ax.set_ylabel("Unity Z [m]")
        ax.set_zlabel("Unity Y [m]")

        ax.set_xlim(X_LIMIT)
        ax.set_ylim(Z_LIMIT)
        ax.set_zlim(Y_LIMIT)

        ax.set_box_aspect(
            (
                X_LIMIT[1] - X_LIMIT[0],
                Z_LIMIT[1] - Z_LIMIT[0],
                Y_LIMIT[1] - Y_LIMIT[0],
            )
        )

        ax.view_init(elev=22, azim=-58)
        ax.grid(True, alpha=0.3)

        origin = np.zeros(3, dtype=float)

        # Global Tracking Space axes at the origin.
        global_length = 0.35
        global_axes_unity = np.eye(3) * global_length
        axis_colors = ("red", "green", "blue")
        axis_names = ("X", "Y", "Z")

        origin_plot = unity_to_plot(origin)

        for index in range(3):
            direction_plot = unity_to_plot(global_axes_unity[:, index])
            ax.quiver(
                *origin_plot,
                *direction_plot,
                color=axis_colors[index],
                alpha=0.25,
                linewidth=1.2,
                arrow_length_ratio=0.14,
            )

        ax.scatter(
            [0.0],
            [0.0],
            [0.0],
            color="black",
            s=28,
        )
        ax.text(0.02, 0.02, 0.02, "Origin", fontsize=8)

        pose_valid = (
            connected and position_tracked and rotation_tracked
        )

        if pose_valid:
            position_plot = unity_to_plot(position)

            # Origin-to-controller position vector.
            ax.quiver(
                *origin_plot,
                *position_plot,
                color="black",
                linewidth=2.3,
                arrow_length_ratio=0.08,
            )

            ax.scatter(
                [position_plot[0]],
                [position_plot[1]],
                [position_plot[2]],
                color="black",
                s=38,
            )

            rotation = quaternion_to_rotation_matrix(*quaternion)

            # Columns of R are the local X/Y/Z directions in Tracking Space.
            for index in range(3):
                direction_unity = (
                    rotation[:, index] * LOCAL_AXIS_LENGTH_M
                )
                direction_plot = unity_to_plot(direction_unity)

                ax.quiver(
                    *position_plot,
                    *direction_plot,
                    color=axis_colors[index],
                    linewidth=3.0,
                    arrow_length_ratio=0.20,
                )

                endpoint = position_plot + direction_plot
                ax.text(
                    endpoint[0],
                    endpoint[1],
                    endpoint[2],
                    axis_names[index],
                    color=axis_colors[index],
                    fontsize=10,
                    fontweight="bold",
                )
        else:
            ax.text2D(
                0.5,
                0.5,
                "Controller pose unavailable",
                transform=ax.transAxes,
                ha="center",
                va="center",
                fontsize=13,
                color="#9a0000",
            )

        self.canvas.draw_idle()


class ControllerSignalGUI:
    def __init__(
        self,
        root: tk.Tk,
        state: SharedState,
        server: ControllerWebSocketServer,
    ) -> None:
        self.root = root
        self.state = state
        self.server = server

        self.root.title(
            "Quest 3 Controller Signal Server"
        )
        self.root.geometry("1660x930")
        self.root.minsize(1300, 760)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_gui()

    def _build_ui(self) -> None:
        status = ttk.LabelFrame(
            self.root,
            text="Server Status",
            padding=8,
        )
        status.pack(
            fill=tk.X,
            padx=10,
            pady=(10, 5),
        )

        self.server_label = ttk.Label(
            status,
            text=f"Listening: ws://{HOST}:{PORT}",
            width=34,
        )
        self.server_label.grid(
            row=0,
            column=0,
            padx=(0, 12),
            sticky="w",
        )

        self.connection_label = ttk.Label(
            status,
            text="Quest: Disconnected",
            width=30,
        )
        self.connection_label.grid(
            row=0,
            column=1,
            padx=(0, 12),
            sticky="w",
        )

        self.rate_label = ttk.Label(
            status,
            text="Receive: 0.0 Hz",
            width=20,
        )
        self.rate_label.grid(
            row=0,
            column=2,
            padx=(0, 12),
            sticky="w",
        )

        self.sequence_label = ttk.Label(
            status,
            text="Sequence: 0",
            width=18,
        )
        self.sequence_label.grid(
            row=0,
            column=3,
            padx=(0, 12),
            sticky="w",
        )

        self.age_label = ttk.Label(
            status,
            text="Packet age: --",
            width=20,
        )
        self.age_label.grid(
            row=0,
            column=4,
            sticky="w",
        )

        self.error_label = ttk.Label(
            status,
            text="",
            foreground="#a00000",
        )
        self.error_label.grid(
            row=1,
            column=0,
            columnspan=5,
            pady=(6, 0),
            sticky="w",
        )

        content = ttk.Frame(self.root)
        content.pack(
            fill=tk.BOTH,
            expand=True,
            padx=10,
            pady=(5, 10),
        )

        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(0, weight=1)

        self.left_panel = ControllerPanel(content, "left")
        self.left_panel.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 5),
        )

        self.right_panel = ControllerPanel(content, "right")
        self.right_panel.grid(
            row=0,
            column=1,
            sticky="nsew",
            padx=(5, 0),
        )

    def _update_gui(self) -> None:
        snapshot = self.state.snapshot()
        frame = snapshot["frame"]

        connected = snapshot["client_connected"]
        age = snapshot["age"]

        self.connection_label.configure(
            text=(
                "Quest: Connected"
                if connected
                else "Quest: Disconnected"
            ),
            foreground="#147d18" if connected else "#a00000",
        )

        self.rate_label.configure(
            text=f"Receive: {snapshot['receive_hz']:.1f} Hz"
        )

        self.sequence_label.configure(
            text=f"Sequence: {frame.get('sequence', 0)}"
        )

        if math.isfinite(age):
            self.age_label.configure(
                text=f"Packet age: {age * 1000.0:.0f} ms"
            )
        else:
            self.age_label.configure(text="Packet age: --")

        error = snapshot["parse_error"]
        self.error_label.configure(
            text=f"JSON error: {error}" if error else ""
        )

        self.left_panel.update_data(
            frame.get("left", empty_controller("left"))
        )
        self.right_panel.update_data(
            frame.get("right", empty_controller("right"))
        )

        self.root.after(GUI_REFRESH_MS, self._update_gui)

    def _on_close(self) -> None:
        self.server.stop()
        self.root.destroy()


def run_terminal_mode() -> None:
    """
    Start the same WebSocket server without the graphical interface.
    The latest controller state is printed at a readable rate.
    """
    state = SharedState()
    server = ControllerWebSocketServer(state)
    server.start()

    print("=" * 72)
    print(" Quest 3 Controller Signal Server - Terminal Mode")
    print("=" * 72)
    print(f" Listening: ws://{HOST}:{PORT}")
    print(" Press Ctrl+C to stop.")
    print()

    last_print_time = 0.0
    last_sequence = None

    try:
        while True:
            snapshot = state.snapshot()
            frame = snapshot["frame"]
            now = time.monotonic()

            if (
                snapshot["client_connected"]
                and now - last_print_time >= 0.5
                and frame.get("sequence") != last_sequence
            ):
                left = frame.get("left", empty_controller("left"))
                right = frame.get("right", empty_controller("right"))

                lp = left.get("position", {})
                rp = right.get("position", {})
                lt = left.get("thumbstick", {})
                rt = right.get("thumbstick", {})
                lb = left.get("buttons", {})
                rb = right.get("buttons", {})

                age_ms = (
                    snapshot["age"] * 1000.0
                    if math.isfinite(snapshot["age"])
                    else float("nan")
                )

                print(
                    f"[Seq {frame.get('sequence', 0):>7}] "
                    f"RX={snapshot['receive_hz']:5.1f} Hz  "
                    f"Age={age_ms:6.1f} ms"
                )
                print(
                    "  Left : "
                    f"Pos=({float(lp.get('x', 0.0)):+.3f}, "
                    f"{float(lp.get('y', 0.0)):+.3f}, "
                    f"{float(lp.get('z', 0.0)):+.3f})  "
                    f"Trig={float(left.get('trigger', 0.0)):.3f}  "
                    f"Grip={float(left.get('grip', 0.0)):.3f}  "
                    f"Stick=({float(lt.get('x', 0.0)):+.3f}, "
                    f"{float(lt.get('y', 0.0)):+.3f})  "
                    f"X={int(bool(lb.get('x', False)))} "
                    f"Y={int(bool(lb.get('y', False)))} "
                    f"Click={int(bool(lt.get('pressed', False)))}"
                )
                print(
                    "  Right: "
                    f"Pos=({float(rp.get('x', 0.0)):+.3f}, "
                    f"{float(rp.get('y', 0.0)):+.3f}, "
                    f"{float(rp.get('z', 0.0)):+.3f})  "
                    f"Trig={float(right.get('trigger', 0.0)):.3f}  "
                    f"Grip={float(right.get('grip', 0.0)):.3f}  "
                    f"Stick=({float(rt.get('x', 0.0)):+.3f}, "
                    f"{float(rt.get('y', 0.0)):+.3f})  "
                    f"A={int(bool(rb.get('a', False)))} "
                    f"B={int(bool(rb.get('b', False)))} "
                    f"Click={int(bool(rt.get('pressed', False)))}"
                )
                print("-" * 72)

                last_print_time = now
                last_sequence = frame.get("sequence")

            elif (
                not snapshot["client_connected"]
                and now - last_print_time >= 2.0
            ):
                print(
                    f"[Waiting] No Quest client connected on "
                    f"ws://{HOST}:{PORT}"
                )
                last_print_time = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping terminal server...")
    finally:
        server.stop()


def run_gui_mode() -> None:
    state = SharedState()
    server = ControllerWebSocketServer(state)
    server.start()

    root = tk.Tk()
    app = ControllerSignalGUI(root, state, server)
    del app
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Quest 3 Controller Signal WebSocket Server"
    )
    parser.add_argument(
        "--mode",
        choices=("terminal", "gui"),
        default="gui",
        help="Server display mode.",
    )
    args = parser.parse_args()

    try:
        if args.mode == "terminal":
            run_terminal_mode()
        else:
            run_gui_mode()

    except OSError as exc:
        print(
            f"\n[ERROR] Cannot start WebSocket server on "
            f"{HOST}:{PORT}\n{exc}\n"
        )
        print(
            "Close any previous server instance that is using port 8766."
        )
        raise

    except Exception as exc:
        print(f"\n[ERROR] Server startup failed:\n{exc}\n")
        raise


if __name__ == "__main__":
    main()
