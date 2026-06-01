#!/usr/bin/env python3
import time
import threading
import sys
import os
import json
import asyncio
import socket
import numpy as np
from collections import deque
from pathlib import Path as PathlibPath
from pydantic import ConfigDict

# Automatically resolve paths
current_dir = os.path.dirname(os.path.abspath(__file__))
dimos_root = os.path.abspath(os.path.join(current_dir, "..", "dimos"))
if dimos_root not in sys.path:
    sys.path.insert(0, dimos_root)

import reactivex.operators as ops
from reactivex.disposable import Disposable
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# MJPEG Server - 独立的 MJPEG 流服务器，避免 RobotWebInterface 的订阅问题
class MJPEGServer:
    """Simple MJPEG streaming server that maintains a stable connection."""

    def __init__(self, port: int = 7782):
        self._port = port
        self._latest_frame: bytes | None = None
        self._lock = threading.Lock()
        self._running = False
        self._server = None
        self._server_thread: threading.Thread | None = None

    def update_frame(self, frame: np.ndarray):
        """Update the latest frame. Called by LCM subscription."""
        try:
            import cv2
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._lock:
                self._latest_frame = buffer.tobytes()
        except Exception as e:
            logger.debug(f"Frame encode error: {e}")

    def run(self):
        """Run the HTTP server."""
        from http.server import HTTPServer, BaseHTTPRequestHandler
        from socketserver import ThreadingMixIn

        server = self

        class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
            """Handle requests in separate threads."""
            daemon_threads = True
            allow_reuse_address = True

        class MJPEGHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith('/video_feed'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.end_headers()

                    frame_count = 0
                    while server._running:
                        with server._lock:
                            frame = server._latest_frame
                        if frame:
                            try:
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                                self.wfile.write(frame)
                                self.wfile.write(b'\r\n')
                                frame_count += 1
                            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                                break
                        time.sleep(0.03)  # ~30 FPS max
                    logger.debug(f"MJPEG client disconnected after {frame_count} frames")
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, format, *args):
                pass  # Suppress default logging

        try:
            self._server = ThreadedHTTPServer(('0.0.0.0', self._port), MJPEGHandler)
            logger.info(f"✅ MJPEG server listening on port {self._port}")
            while self._running:
                self._server.handle_request()
        except Exception as e:
            if self._running:
                logger.error(f"MJPEG server error: {e}")

    def start(self):
        """Start the server in a background thread."""
        self._running = True
        self._server_thread = threading.Thread(target=self.run, daemon=True)
        self._server_thread.start()

    def stop(self):
        """Stop the server."""
        self._running = False
        # 发送一个请求来解除 handle_request 的阻塞
        try:
            import urllib.request
            urllib.request.urlopen(f'http://127.0.0.1:{self._port}/shutdown', timeout=1)
        except Exception:
            pass

# Dimos log directories
DIMOS_LOGS_DIR = PathlibPath.home() / ".local/state/dimos/logs"
DIMOS_RUNS_DIR = PathlibPath.home() / ".local/state/dimos/runs"


def is_pid_alive(pid: int) -> bool:
    """Check if a process with the given PID is still running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        # Process exists but we can't signal it — still alive.
        return True



def check_port_in_use(port: int) -> bool:
    """Check if a port is already in use by another process.

    Uses ss to check if there's an actual LISTENING socket on the port.
    This is more reliable than socket.bind() which can fail due to TIME_WAIT.
    """
    import subprocess
    try:
        result = subprocess.run(
            ['ss', '-tlnp'],
            capture_output=True,
            text=True,
            timeout=2
        )
        # Check if port is in LISTEN state
        for line in result.stdout.split('\n'):
            if f':{port}' in line and 'LISTEN' in line:
                return True
        return False
    except Exception:
        # Fallback: try socket bind with SO_REUSEADDR
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(('127.0.0.1', port))
                return False
            except OSError:
                return True


def check_port_connectable(port: int, timeout: float = 1.0) -> bool:
    """Check if a TCP connection can be established to localhost:port."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect(('127.0.0.1', port))
            return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False


def check_socketio_handshake(port: int, timeout: float = 2.0) -> bool:
    """Check if Socket.IO handshake succeeds via polling endpoint."""
    try:
        import urllib.request
        # Disable proxy for localhost requests to avoid proxy-related misdetection
        no_proxy_handler = urllib.request.ProxyHandler({})
        opener = urllib.request.build_opener(no_proxy_handler)
        url = f"http://127.0.0.1:{port}/socket.io/?EIO=4&transport=polling"
        req = urllib.request.Request(url)
        with opener.open(req, timeout=timeout) as response:
            if response.status == 200:
                content = response.read().decode('utf-8', errors='ignore')
                # Valid Socket.IO polling response starts with "0{" for open packet
                if content.startswith('0{') or content.startswith('0'):
                    return True
    except Exception:
        pass
    return False


def get_port_owner(port: int) -> str | None:
    """Get the process name owning a port (Linux only)."""
    try:
        result = os.popen(f"ss -ltnp | grep ':{port}' | head -1").read()
        if 'pid=' in result:
            import re
            m = re.search(r'pid=(\d+)', result)
            if m:
                pid = int(m.group(1))
                try:
                    with open(f'/proc/{pid}/comm', 'r') as f:
                        return f.read().strip()
                except:
                    return f"pid={pid}"
    except:
        pass
    return None


class UIBridgeConfig(ModuleConfig):
    model_config = ConfigDict(extra='allow')
    web_port: int = 7782   # MJPEG video feed (avoids conflict with DimOS 5555)
    vis_port: int = 7781   # Socket.IO data bridge (avoids conflict with DimOS 7779)
    pc_max_points: int = 1500
    pc_throttle_sec: float = 0.2


class DimosLogPoller:
    """Polls Dimos JSONL log files and emits to Socket.IO."""

    # Re-emit "no active run" status every 30 seconds so new clients receive it
    NO_RUN_EMIT_INTERVAL_SEC = 30.0
    RUN_GRACE_PERIOD_SEC = 15.0  # Grace period before marking run as inactive

    def __init__(self, vis_module, on_new_run=None, get_last_data_at=None):
        self._vis_module = vis_module
        self._on_new_run = on_new_run  # Callback when new run_id detected
        self._get_last_data_at = get_last_data_at  # Callback to get last LCM data timestamp
        self._running = True
        self._last_pos = 0
        self._current_log: PathlibPath | None = None
        self._current_run_id: str | None = None
        self._previous_run_id: str | None = None  # Track previous run for change detection
        self._thread: threading.Thread | None = None
        self._last_no_run_emit_time: float = 0.0  # Track last no-run status emit
        self._last_active_time: float = 0.0  # Last time we saw active evidence

    def check_active_dimos_run(self) -> tuple[str | None, PathlibPath | None]:
        """Check if Dimos is actually running. Active if ANY of:
        - PID is alive
        - log file mtime < 60s
        - LCM data received < 10s ago
        """
        try:
            if not DIMOS_RUNS_DIR.exists():
                return None, None

            # Get all run files and check each one
            run_files = sorted(
                DIMOS_RUNS_DIR.glob("*.json"),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )

            now = time.time()
            best_run_id = None
            best_log_path = None
            best_score = 0  # Higher score = more active

            for run_file in run_files:
                try:
                    with open(run_file, 'r') as f:
                        entry = json.load(f)

                    pid = entry.get('pid')
                    run_id = entry.get('run_id')
                    log_dir = entry.get('log_dir')

                    # Calculate activity score
                    score = 0

                    # Check if PID is alive (strongest signal)
                    pid_alive = pid and is_pid_alive(pid)
                    if pid_alive:
                        score += 100

                    # Check log file mtime
                    log_path = None
                    if log_dir:
                        log_path = PathlibPath(log_dir) / "main.jsonl"
                    if not log_path or not log_path.exists():
                        log_path = DIMOS_LOGS_DIR / run_id / "main.jsonl" if run_id else None

                    if log_path and log_path.exists():
                        mtime_age = now - log_path.stat().st_mtime
                        if mtime_age < 60:
                            score += 50
                        elif mtime_age < 120:
                            score += 20

                    # Check LCM data freshness (via callback)
                    if self._get_last_data_at:
                        last_data_age = now - self._get_last_data_at()
                        if last_data_age < 10:
                            score += 30
                        elif last_data_age < 30:
                            score += 10

                    # Keep best candidate
                    if score > best_score:
                        best_score = score
                        best_run_id = run_id
                        best_log_path = log_path if log_path and log_path.exists() else None

                except (json.JSONDecodeError, KeyError, OSError) as e:
                    logger.debug(f"Error reading run entry {run_file}: {e}")
                    continue

            # Consider active if score > 0 (any evidence of activity)
            if best_score > 0 and best_run_id:
                self._last_active_time = now
                return best_run_id, best_log_path

            # Grace period: don't immediately mark as inactive
            if self._current_run_id and (now - self._last_active_time) < self.RUN_GRACE_PERIOD_SEC:
                logger.debug(f"Run {self._current_run_id} in grace period")
                return self._current_run_id, self._current_log

            # No active runs - clean up VERY stale entries (> 5 minutes old)
            for run_file in run_files:
                try:
                    mtime_age = now - run_file.stat().st_mtime
                    if mtime_age > 300:  # 5 minutes
                        with open(run_file, 'r') as f:
                            entry = json.load(f)
                        pid = entry.get('pid')
                        if pid and not is_pid_alive(pid):
                            logger.debug(f"Cleaning very stale run entry: {run_file.stem}")
                            run_file.unlink(missing_ok=True)
                except:
                    continue

        except Exception as e:
            logger.debug(f"Error checking active run: {e}")

        return None, None

    def find_active_run(self) -> tuple[str | None, PathlibPath | None]:
        """Find the most recent active Dimos run log. Returns (run_id, log_path)."""
        run_id, log_path = self.check_active_dimos_run()
        return run_id, log_path

    def tail_jsonl(self, log_path: PathlibPath) -> tuple[list[str], int]:
        """Read new lines from JSONL file since last position."""
        try:
            with open(log_path, 'r') as f:
                f.seek(self._last_pos)
                new_lines = f.readlines()
                new_pos = f.tell()
            return [l.strip() for l in new_lines if l.strip()], new_pos
        except Exception:
            return [], self._last_pos

    def poll_loop(self):
        """Main polling loop."""
        while self._running:
            try:
                # Check for active Dimos run - get run_id BEFORE updating state
                run_id, log_path = self.find_active_run()

                if run_id and log_path:
                    # We have an active run - reset no-run emit time
                    self._last_no_run_emit_time = 0.0

                    # Check for run_id change BEFORE updating state
                    if run_id != self._current_run_id:
                        # Save previous run_id for comparison
                        self._previous_run_id = self._current_run_id
                        self._current_run_id = run_id
                        self._current_log = log_path
                        self._last_pos = 0
                        logger.info(f"📁 Tracking active Dimos run: {run_id} (previous: {self._previous_run_id})")
                        self._emit_status("dimos_active", f"Dimos 运行中: {run_id}", "success")

                        # Notify bridge module to rebind LCM transports
                        if self._on_new_run:
                            self._on_new_run(run_id)

                    if self._current_log and self._current_log.exists():
                        lines, self._last_pos = self.tail_jsonl(self._current_log)
                        for line in lines:
                            try:
                                entry = json.loads(line)
                                self._emit_log_entry(entry)
                            except json.JSONDecodeError:
                                continue
                else:
                    # No active EmergeOS run
                    self._current_log = None
                    self._last_pos = 0
                    self._previous_run_id = self._current_run_id
                    self._current_run_id = None

                    # Emit "no active run" status periodically
                    now = time.time()
                    if now - self._last_no_run_emit_time >= self.NO_RUN_EMIT_INTERVAL_SEC:
                        if self._emit_status("emergeos_status", "No active EmergeOS run", "warning"):
                            self._last_no_run_emit_time = now
                            logger.info("⚠️ No active EmergeOS run detected")

            except Exception as e:
                logger.debug(f"Log poll error: {e}")

            time.sleep(0.5)

    def _emit_status(self, topic: str, message: str, status: str) -> bool:
        """Emit a status message to Socket.IO. Returns True if emit succeeded."""
        try:
            self._vis_module._emit("app_log", {
                "message": message,
                "status": status,
                "topic": topic
            })
            return True
        except Exception as e:
            logger.debug(f"Emit status error: {e}")
            return False

    def _emit_log_entry(self, entry: dict):
        """Emit a log entry to Socket.IO."""
        try:
            # Extract message from various log formats
            msg = entry.get('message') or entry.get('msg') or entry.get('text', str(entry))
            level = entry.get('level', entry.get('levelname', 'info')).lower()
            module = entry.get('name', entry.get('module', 'dimos'))

            status = 'error' if level == 'error' else 'warning' if level in ('warn', 'warning') else 'success'

            # 将 dimos 替换为 emergeos 以匹配 UI 显示名称
            module_display = module.split('.')[-1] if '.' in module else module
            if module_display.startswith('dimos') or module_display == 'dimos':
                module_display = module_display.replace('dimos', 'emergeos')

            self._vis_module._emit("app_log", {
                "message": str(msg),
                "status": status,
                "topic": module_display
            })
        except Exception as e:
            logger.debug(f"Emit log entry error: {e}")

    def start(self):
        """Start the polling thread."""
        self._thread = threading.Thread(target=self.poll_loop, daemon=True)
        self._thread.start()
        logger.info("✅ Dimos log poller started")

    def stop(self):
        """Stop the polling thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)


class UIBridgeModule(Module[UIBridgeConfig]):
    default_config = UIBridgeConfig
    color_image: In[Image]
    global_map: In[PointCloud2]
    odom: In[PoseStamped]
    path: In[Path]
    global_costmap: In[OccupancyGrid]
    battery_status: In[dict]

    BATTERY_STALE_THRESHOLD_SEC = 10.0  # Battery data older than this is stale
    DATA_STALE_THRESHOLD_SEC = 5.0      # DimOS data older than this triggers rebind

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._last_pc_time = 0
        self._last_pose = None
        self._last_pose_time = 0

        self._fps_buffer = deque(maxlen=30)
        self._current_fps = 0.0
        self._img_width = 0
        self._img_height = 0

        # Battery state
        self._battery_percent: int | None = None
        self._battery_voltage: float | None = None
        self._battery_updated_at: float = 0.0
        self._battery_source: str | None = None

        # Data freshness tracking for rebind
        self._last_data_at: float = 0.0
        self._dimos_run_id: str | None = None
        self._rebind_lock = threading.Lock()
        self._last_rebind_at: float = 0.0  # Throttle rebind
        self._last_video_status_ts: int = 0
        self._running = True
        self._subscriptions: dict[str, object] = {}  # Track active subscriptions for disposal
        self._vis_rebuild_lock = threading.Lock()  # Lock for vis module rebuild

        # Diagnostic counters
        self._recv_counts = {"image": 0, "odom": 0, "global_map": 0, "path": 0, "costmap": 0, "battery": 0}
        self._last_log_time = 0.0
        self._first_recv = {"image": False, "odom": False, "global_map": False, "path": False, "costmap": False, "battery": False}

        # Check port availability before starting servers
        port_error = False
        for port_name, port in [("Socket.IO (vis)", self.config.vis_port), ("MJPEG (web)", self.config.web_port)]:
            if check_port_in_use(port):
                owner = get_port_owner(port) or "unknown"
                logger.error(f"❌ Port {port} ({port_name}) is already in use by: {owner}")
                logger.error(f"   EmergeUI bridge cannot start.")
                logger.error(f"   Run: pkill -f bridge_dimos_ui.py  # to kill old bridge process")
                port_error = True
            else:
                logger.info(f"✅ Port {port} ({port_name}) is available")

        if port_error:
            raise RuntimeError("Required ports are in use. Cannot start EmergeUI bridge.")

        # Helper to convert DimOS Image to BGR frame for OpenCV encoding
        def _image_to_bgr_frame(msg: Image):
            return np.ascontiguousarray(msg.to_bgr().data)

        # 使用自定义的 MJPEGServer 替代 RobotWebInterface，解决视频流卡住问题
        self._mjpeg_server = MJPEGServer(port=self.config.web_port)

        self._vis_module = WebsocketVisModule(port=self.config.vis_port)
        self._mjpeg_thread = None
        self._log_poller = DimosLogPoller(
            self._vis_module,
            on_new_run=self._handle_new_dimos_run,
            get_last_data_at=lambda: self._last_data_at
        )

    def _log_recv(self, name: str):
        """Log first receive and periodic counts for diagnostics."""
        self._recv_counts[name] += 1
        now = time.time()

        # First receive log
        if not self._first_recv[name]:
            self._first_recv[name] = True
            logger.info(f"📥 First {name} received")

        # Periodic summary every 5 seconds
        if now - self._last_log_time >= 5.0:
            counts = ", ".join(f"{k}:{v}" for k, v in self._recv_counts.items())
            logger.info(f"📊 Receive counts: {counts}")
            self._last_log_time = now

    def _on_image(self, msg: Image):
        self._log_recv("image")
        now = time.time()
        self._last_data_at = now
        self._fps_buffer.append(now)
        if len(self._fps_buffer) > 1:
            duration = self._fps_buffer[-1] - self._fps_buffer[0]
            if duration > 0:
                self._current_fps = (len(self._fps_buffer) - 1) / duration
        self._img_width = msg.width
        self._img_height = msg.height

        # 发送帧到 MJPEG 服务器
        try:
            frame = np.ascontiguousarray(msg.to_bgr().data)
            self._mjpeg_server.update_frame(frame)
        except Exception as e:
            logger.debug(f"MJPEG frame update error: {e}")

        # Emit video_status ~1Hz for frontend video state tracking
        current_ts = int(now)
        if current_ts != self._last_video_status_ts:
            self._last_video_status_ts = current_ts
            self._vis_module._emit("video_status", {
                "fps": round(self._current_fps, 1),
                "res": f"{self._img_width}x{self._img_height}",
                "ts": now
            })

    def _on_odom(self, msg: PoseStamped):
        self._log_recv("odom")
        self._last_data_at = time.time()
        self._vis_module._on_robot_pose(msg)
        now = time.time()
        speed = 0.0
        if self._last_pose and now > self._last_pose_time:
            dt = now - self._last_pose_time
            dx = msg.position.x - self._last_pose.position.x
            dy = msg.position.y - self._last_pose.position.y
            speed = np.sqrt(dx*dx + dy*dy) / dt
        self._last_pose = msg
        self._last_pose_time = now

        # Convert quaternion to euler angles (roll, pitch, yaw)
        roll, pitch, yaw = 0.0, 0.0, 0.0
        try:
            q = msg.orientation
            # Quaternion to euler conversion
            sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (q.w * q.y - q.z * q.x)
            pitch = np.arcsin(np.clip(sinp, -1, 1))

            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            # Convert to degrees
            roll = np.degrees(roll)
            pitch = np.degrees(pitch)
            yaw = np.degrees(yaw)
        except Exception:
            pass

        try:
            battery_data = self._get_battery_telemetry()
            self._vis_module._emit("telemetry", {
                "ts": time.time(),
                "speed": round(float(speed), 2),
                "fps": round(float(self._current_fps), 1),
                "res": f"{self._img_width}x{self._img_height}" if self._img_width > 0 else "N/A",
                "roll": round(float(roll), 1),
                "pitch": round(float(pitch), 1),
                "yaw": round(float(yaw), 1),
                **battery_data,
            })
        except Exception as e:
            logger.debug(f"Telemetry emit error: {e}")

    def _on_global_map(self, msg: PointCloud2):
        self._log_recv("global_map")
        self._last_data_at = time.time()
        now = time.time()
        if now - self._last_pc_time < self.config.pc_throttle_sec:
            return
        try:
            points, _ = msg.as_numpy()
            if len(points) > self.config.pc_max_points:
                step = max(1, len(points) // self.config.pc_max_points)
                points = points[::step][:self.config.pc_max_points]
            self._vis_module._emit("global_map", {"points": points.tolist()})
            self._last_pc_time = now
        except Exception as e:
            logger.error(f"Global Map Bridge Error: {e}")

    def _on_path(self, msg: Path):
        self._log_recv("path")
        self._last_data_at = time.time()
        self._vis_module._on_path(msg)

    def _on_costmap(self, msg: OccupancyGrid):
        self._log_recv("costmap")
        self._last_data_at = time.time()
        self._vis_module._on_global_costmap(msg)

    def _on_battery_status(self, msg: dict):
        """Handle battery status from DimOS."""
        self._log_recv("battery")
        self._last_data_at = time.time()
        try:
            percent = msg.get("percent")
            if percent is not None:
                self._battery_percent = max(0, min(100, int(percent)))
                self._battery_voltage = msg.get("voltage")
                self._battery_updated_at = msg.get("ts", time.time())
                self._battery_source = msg.get("source")
                logger.debug(f"Battery: {self._battery_percent}% from {self._battery_source}")
        except Exception as e:
            logger.debug(f"Battery status parse error: {e}")

    def _get_battery_telemetry(self) -> dict:
        """Get battery data for telemetry with stale check."""
        # No battery data received yet
        if self._battery_percent is None:
            return {
                "battery": None,
                "batteryStale": True,
            }

        # Check if battery data is stale
        now = time.time()
        stale = (now - self._battery_updated_at) > self.BATTERY_STALE_THRESHOLD_SEC

        if stale:
            return {
                "battery": None,
                "batteryStale": True,
            }

        return {
            "battery": self._battery_percent,
            "batteryVoltage": self._battery_voltage,
            "batterySource": self._battery_source,
            "batteryStale": False,
        }

    def _handle_new_dimos_run(self, run_id: str):
        """Handle new DimOS run detected by log poller."""
        if run_id != self._dimos_run_id:
            logger.info(f"🆕 New DimOS run detected: {run_id} (previous: {self._dimos_run_id})")
            self._dimos_run_id = run_id
            self._rebind_dimos_transports()

    def _rebind_dimos_transports(self):
        """Recreate LCM transports and subscriptions for new DimOS run."""
        with self._rebind_lock:
            # Throttle: don't rebind more than once every 3 seconds
            now = time.time()
            if self._last_rebind_at > 0 and (now - self._last_rebind_at) < 3.0:
                logger.debug(f"Skipping rebind (throttled, last was {now - self._last_rebind_at:.1f}s ago)")
                return

            self._last_rebind_at = now
            logger.info("🔄 Rebinding LCM transports...")

            # 1. Dispose old subscriptions (support dispose(), unsubscribe(), or callable)
            for name, sub in self._subscriptions.items():
                try:
                    if hasattr(sub, 'dispose'):
                        sub.dispose()
                    elif hasattr(sub, 'unsubscribe'):
                        sub.unsubscribe()
                    elif callable(sub):
                        sub()  # Call as unsubscribe function
                    logger.debug(f"Disposed subscription: {name}")
                except Exception as e:
                    logger.debug(f"Error disposing {name}: {e}")
            self._subscriptions.clear()

            # 2. Stop old transports (they may have internal LCM state)
            for stream_name in ["color_image", "odom", "global_map", "path", "global_costmap", "battery_status"]:
                stream = getattr(self, stream_name, None)
                if stream and hasattr(stream, 'transport'):
                    old_transport = stream.transport
                    if old_transport and hasattr(old_transport, 'stop'):
                        try:
                            old_transport.stop()
                        except Exception as e:
                            logger.debug(f"Error stopping transport {stream_name}: {e}")

            # 3. Recreate transports
            self.color_image.transport = LCMTransport("/color_image", Image)
            self.global_map.transport = LCMTransport("/global_map", PointCloud2)
            self.odom.transport = LCMTransport("/odom", PoseStamped)
            self.path.transport = LCMTransport("/path", Path)
            self.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
            self.battery_status.transport = pLCMTransport("/battery_status")

            # 4. Re-subscribe all streams
            self._subscriptions["color_image"] = self.color_image.subscribe(self._on_image)
            self._subscriptions["odom"] = self.odom.subscribe(self._on_odom)
            self._subscriptions["path"] = self.path.subscribe(self._on_path)
            self._subscriptions["global_costmap"] = self.global_costmap.subscribe(self._on_costmap)
            self._subscriptions["global_map"] = self.global_map.subscribe(self._on_global_map)
            self._subscriptions["battery_status"] = self.battery_status.subscribe(self._on_battery_status)

            # 5. Reset counters and flags
            self._recv_counts = {k: 0 for k in self._recv_counts}
            self._first_recv = {k: False for k in self._first_recv}
            self._last_data_at = time.time()

            # Notify frontend
            self._vis_module._emit("dimos_rebind", {"run_id": self._dimos_run_id, "ts": time.time()})
            logger.info(f"✅ LCM transports rebound for run: {self._dimos_run_id}")

    def _stale_check_loop(self):
        """Background thread to check data freshness and trigger rebind if stale."""
        REBIND_COOLDOWN_SEC = 10.0  # Minimum interval between stale-triggered rebinds
        last_stale_rebind_at: float = 0.0

        while self._running:
            time.sleep(2.0)
            if not self._running:
                break

            # Only rebind if: DimOS is running AND data is stale AND cooldown passed
            if self._dimos_run_id and self._last_data_at > 0:
                elapsed = time.time() - self._last_data_at
                cooldown_elapsed = time.time() - last_stale_rebind_at

                if elapsed > self.DATA_STALE_THRESHOLD_SEC and cooldown_elapsed >= REBIND_COOLDOWN_SEC:
                    logger.warning(f"⏰ DimOS data stale for {elapsed:.1f}s, rebinding...")
                    last_stale_rebind_at = time.time()
                    self._rebind_dimos_transports()

    def _is_vis_emit_ready(self, vis) -> bool:
        """Check if WebsocketVisModule can emit events (comprehensive health check)."""
        if vis is None:
            return False
        loop = getattr(vis, "_server_loop", None)
        thread = getattr(vis, "_uvicorn_server_thread", None)
        return (
            loop is not None
            and not loop.is_closed()
            and loop.is_running()
            and thread is not None
            and thread.is_alive()
            and check_socketio_handshake(self.config.vis_port, timeout=1.0)
        )

    def _vis_watchdog_loop(self):
        """Background thread to check Socket.IO (7781) health and rebuild if dead."""
        REBUILD_COOLDOWN_SEC = 15.0  # Longer cooldown to avoid thrashing
        last_rebuild_at: float = 0.0

        while self._running:
            time.sleep(3.0)
            if not self._running:
                break

            try:
                vis = self._vis_module

                # Comprehensive health check: loop + thread + handshake must ALL be healthy
                is_healthy = self._is_vis_emit_ready(vis)

                # Check if _server_loop is dead (internal broken state)
                server_loop_live = False
                if vis:
                    loop = getattr(vis, '_server_loop', None)
                    server_loop_live = loop is not None and not loop.is_closed() and loop.is_running()

                # If _server_loop is dead, this is internal broken state - MUST rebuild
                if not server_loop_live and vis:
                    logger.error("❌ _server_loop is dead (closed or not running) - internal broken state")
                    cooldown_elapsed = time.time() - last_rebuild_at
                    if cooldown_elapsed >= REBUILD_COOLDOWN_SEC:
                        logger.error("🔄 Force rebuild to fix broken _server_loop state")
                        last_rebuild_at = time.time()
                        # Force rebuild - don't skip even if handshake succeeds
                        self._rebuild_vis_module(force=True, reason="dead_server_loop")
                    continue

                if is_healthy:
                    logger.debug("✅ Socket.IO health check passed")
                else:
                    # Health check failed - check what's wrong
                    handshake_ok = check_socketio_handshake(self.config.vis_port, timeout=2.0)
                    thread_alive = False
                    if vis:
                        uvicorn_thread = getattr(vis, "_uvicorn_server_thread", None)
                        thread_alive = uvicorn_thread is not None and uvicorn_thread.is_alive()

                    # Check for external port conflict
                    port_owner = get_port_owner(self.config.vis_port) if check_port_in_use(self.config.vis_port) else None
                    current_pid = os.getpid()
                    is_external_owner = False
                    if port_owner and not port_owner.startswith(f"pid={current_pid}"):
                        is_external_owner = True
                        logger.error(f"⚠️ Port {self.config.vis_port} owned by external process: {port_owner}")
                        if vis:
                            vis._emit("bridge_status", {
                                "status": "port_conflict",
                                "port": self.config.vis_port,
                                "owner": port_owner,
                                "message": f"端口 {self.config.vis_port} 被外部进程占用: {port_owner}"
                            })
                        continue  # Can't rebuild if external process owns port

                    logger.warning(
                        f"⚠️ Socket.IO unhealthy (handshake={handshake_ok}, "
                        f"thread={thread_alive}, loop_live={server_loop_live})"
                    )

                    # Rebuild if cooldown passed
                    cooldown_elapsed = time.time() - last_rebuild_at
                    if cooldown_elapsed >= REBUILD_COOLDOWN_SEC:
                        logger.error("❌ Socket.IO unhealthy, triggering rebuild")
                        last_rebuild_at = time.time()
                        self._rebuild_vis_module(force=True, reason="health_check_failed")

            except Exception as e:
                logger.error(f"Vis watchdog error: {e}")

    def _rebuild_vis_module(self, force: bool = False, reason: str = ""):
        """Cleanly stop and recreate the WebsocketVisModule.

        Args:
            force: If True, rebuild even if handshake succeeds (for dead _server_loop)
            reason: Reason for rebuild (for logging)
        """
        with self._vis_rebuild_lock:
            # Pre-check: skip rebuild if vis is healthy and not forced
            if not force:
                if self._is_vis_emit_ready(self._vis_module):
                    logger.info("Socket.IO healthy, skipping rebuild")
                    return
                # If port is in use by external process, can't rebuild
                if check_port_in_use(self.config.vis_port):
                    port_owner = get_port_owner(self.config.vis_port)
                    current_pid = os.getpid()
                    if port_owner and not port_owner.startswith(f"pid={current_pid}"):
                        logger.error(f"❌ Cannot rebuild: port {self.config.vis_port} owned by external process: {port_owner}")
                        return

            logger.info(f"🔄 Rebuilding WebsocketVisModule (force={force}, reason={reason})...")

            old_vis = self._vis_module

            # CRITICAL: For same-port rebuild, we MUST stop old instance FIRST
            # Cannot have two instances binding to the same port
            if old_vis:
                logger.info("Stopping old WebsocketVisModule to release port...")
                try:
                    server = getattr(old_vis, "_uvicorn_server", None)
                    if server:
                        server.should_exit = True
                    old_vis.stop()
                    # Wait for uvicorn thread to exit and release port
                    uvicorn_thread = getattr(old_vis, "_uvicorn_server_thread", None)
                    if uvicorn_thread:
                        uvicorn_thread.join(timeout=5.0)
                    broadcast_thread = getattr(old_vis, "_broadcast_thread", None)
                    if broadcast_thread:
                        broadcast_thread.join(timeout=3.0)
                    logger.info("✅ Old vis module stopped")
                except Exception as e:
                    logger.warning(f"Error stopping old vis module: {e}")

            # Wait for port to be released
            port_released = False
            for i in range(10):  # Wait up to 5 seconds
                if not check_port_in_use(self.config.vis_port):
                    port_released = True
                    break
                time.sleep(0.5)
                logger.debug(f"Waiting for port {self.config.vis_port} to be released...")

            if not port_released:
                logger.error(f"❌ Port {self.config.vis_port} not released after 5s")
                # Try to continue anyway - might still work

            # Now create and start new vis module
            new_vis = None
            try:
                new_vis = WebsocketVisModule(port=self.config.vis_port)
                new_vis.start()

                # Wait for _server_loop to be LIVE (running, not closed)
                server_loop_live = False
                for i in range(30):  # Wait up to 3 seconds
                    time.sleep(0.1)
                    loop = getattr(new_vis, '_server_loop', None)
                    if loop is not None and not loop.is_closed() and loop.is_running():
                        server_loop_live = True
                        logger.info("New WebsocketVisModule _server_loop live")
                        break

                if not server_loop_live:
                    logger.error("❌ New WebsocketVisModule _server_loop not live after 3s")
                    new_vis.stop()
                    logger.warning("⚠️ Rebuild failed: _server_loop not live")
                    return

                # Wait for uvicorn thread to be alive
                uvicorn_thread = getattr(new_vis, "_uvicorn_server_thread", None)
                if uvicorn_thread is None or not uvicorn_thread.is_alive():
                    logger.error("❌ New WebsocketVisModule uvicorn thread not alive")
                    new_vis.stop()
                    logger.warning("⚠️ Rebuild failed: uvicorn thread dead")
                    return

                # Wait for Socket.IO handshake to succeed
                handshake_ok = False
                for i in range(6):
                    time.sleep(0.5)
                    if check_socketio_handshake(self.config.vis_port, timeout=1.0):
                        handshake_ok = True
                        break

                if not handshake_ok:
                    logger.error(f"❌ New WebsocketVisModule handshake failed on port {self.config.vis_port}")
                    new_vis.stop()
                    logger.warning("⚠️ Rebuild failed: handshake failed")
                    return

                # All checks passed - rebuild successful
                logger.info("✅ New WebsocketVisModule fully healthy (loop live, thread alive, handshake OK)")

                # Replace the reference
                self._vis_module = new_vis

                # Update log poller's vis reference
                if self._log_poller:
                    self._log_poller._vis_module = self._vis_module

                # Notify frontend of bridge recovery
                self._vis_module._emit("bridge_status", {
                    "status": "recovered",
                    "ts": time.time(),
                    "message": "Socket.IO 通道已恢复"
                })

                logger.info("✅ WebsocketVisModule rebuild complete")

            except Exception as e:
                logger.error(f"Failed to rebuild vis module: {e}")
                # Clean up new instance if created
                if new_vis:
                    try:
                        new_vis.stop()
                    except Exception:
                        pass

    def start(self):
        super().start()
        self._running = True
        # 启动自定义 MJPEG 服务器
        self._mjpeg_server.start()

        # Start websocket vis module FIRST so broadcast loop is ready before we receive any data
        self._vis_module.start()

        # Wait for _server_loop to be LIVE (not just not None, but running and not closed)
        server_loop_ready = False
        for i in range(30):  # Max 3 seconds
            time.sleep(0.1)
            loop = getattr(self._vis_module, '_server_loop', None)
            if loop is not None and not loop.is_closed() and loop.is_running():
                server_loop_ready = True
                logger.info("WebsocketVisModule _server_loop live (running, not closed)")
                break

        if not server_loop_ready:
            logger.error("❌ WebsocketVisModule _server_loop not live after 3s")

        # Wait for _broadcast_loop to be ready (it's created in a background thread)
        broadcast_ready = False
        for i in range(20):  # Max 2 seconds
            time.sleep(0.1)
            if hasattr(self._vis_module, '_broadcast_loop') and self._vis_module._broadcast_loop is not None:
                broadcast_ready = True
                logger.debug("WebsocketVisModule _broadcast_loop ready")
                break

        if not broadcast_ready:
            logger.warning("⚠️ WebsocketVisModule _broadcast_loop not ready after 2s")

        # Wait for Socket.IO to be actually ready (critical for startup)
        vis_ready = False
        for i in range(10):  # Max 5 seconds
            time.sleep(0.5)
            if check_socketio_handshake(self.config.vis_port, timeout=1.0):
                vis_ready = True
                break

        if not vis_ready:
            logger.error(f"❌ Socket.IO (7781) failed to start after 5 seconds")
            logger.error(f"   Check if another process is using port {self.config.vis_port}")
            raise RuntimeError(f"Socket.IO failed to bind on port {self.config.vis_port}")

        logger.info(f"✅ Socket.IO ready on port {self.config.vis_port}")

        # Register custom Socket.IO event handlers for DimOS control
        self._register_event_handlers()

        # NOW create LCM subscriptions - AFTER vis module is fully ready
        # This ensures _emit() will succeed when data arrives
        self._subscriptions["color_image"] = self.color_image.subscribe(self._on_image)
        self._subscriptions["odom"] = self.odom.subscribe(self._on_odom)
        self._subscriptions["path"] = self.path.subscribe(self._on_path)
        self._subscriptions["global_costmap"] = self.global_costmap.subscribe(self._on_costmap)
        self._subscriptions["global_map"] = self.global_map.subscribe(self._on_global_map)
        self._subscriptions["battery_status"] = self.battery_status.subscribe(self._on_battery_status)

        # Helper to dispose subscription
        def _make_dispose_fn(sub):
            def _dispose():
                if hasattr(sub, 'dispose'):
                    sub.dispose()
                elif hasattr(sub, 'unsubscribe'):
                    sub.unsubscribe()
                elif callable(sub):
                    sub()
            return _dispose

        # Add to disposables for cleanup on stop()
        for name, sub in self._subscriptions.items():
            self._disposables.add(Disposable(_make_dispose_fn(sub)))

        # Start Dimos log poller (reads from JSONL files, not LCM)
        self._log_poller.start()

        # Start stale check thread
        self._stale_thread = threading.Thread(target=self._stale_check_loop, daemon=True)
        self._stale_thread.start()

        # Start vis watchdog thread
        self._vis_watchdog_thread = threading.Thread(target=self._vis_watchdog_loop, daemon=True)
        self._vis_watchdog_thread.start()

        logger.info(f"✅ Dimos UI Bridge Active (Telemetry + Logs)")

    def _register_event_handlers(self) -> None:
        """Register custom Socket.IO event handlers for DimOS control."""
        sio = self._vis_module.sio
        if sio is None:
            logger.error("❌ Cannot register event handlers: sio is None")
            return

        @sio.event
        async def start_dimos(sid, data=None):
            """Start DimOS with specified blueprint."""
            blueprint = data.get("blueprint", "unitree-go2-agentic") if data else "unitree-go2-agentic"
            simulation = data.get("simulation", False) if data else False
            logger.info(f"🚀 Start DimOS request from {sid}: blueprint={blueprint}, sim={simulation}")

            import subprocess
            dimos_bin = f"{dimos_root}/.venv/bin/dimos"

            # 先检查是否已有 DimOS 在运行
            try:
                result = subprocess.run(
                    [dimos_bin, "status"],
                    cwd=dimos_root,
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                # 如果 status 返回成功且有运行中的进程，说明已运行
                if result.returncode == 0 and ("run_id" in result.stdout.lower() or "pid" in result.stdout.lower()):
                    logger.warning(f"⚠️ DimOS already running")
                    self._vis_module._emit("app_log", {
                        "message": "EmergeOS 已在运行中，无需重复启动",
                        "status": "warning",
                        "topic": "emergeos_control"
                    })
                    await sio.emit("dimos_start_ack", {"status": "already_running", "message": "EmergeOS 已在运行中"}, room=sid)
                    return
            except subprocess.TimeoutExpired:
                logger.warning("dimos status timeout, proceeding with start")
            except Exception as e:
                logger.debug(f"dimos status check error: {e}")

            # 检查是否有活跃的 run 文件
            try:
                import json
                from pathlib import Path
                runs_dir = Path.home() / ".local/state/dimos/runs"
                if runs_dir.exists():
                    for run_file in runs_dir.glob("*.json"):
                        try:
                            with open(run_file) as f:
                                entry = json.load(f)
                            pid = entry.get('pid')
                            if pid and is_pid_alive(pid):
                                logger.warning(f"⚠️ DimOS already running with PID {pid}")
                                self._vis_module._emit("app_log", {
                                    "message": "EmergeOS 已在运行中，无需重复启动",
                                    "status": "warning",
                                    "topic": "emergeos_control"
                                })
                                await sio.emit("dimos_start_ack", {"status": "already_running", "message": "EmergeOS 已在运行中"}, room=sid)
                                return
                        except Exception:
                            continue
            except Exception as e:
                logger.debug(f"Run file check error: {e}")

            # 没有 DimOS 运行，启动新的
            dimos_cmd = [dimos_bin, "--viewer", "none", "run", "--daemon", blueprint]
            if simulation:
                dimos_cmd.insert(1, "--simulation")

            try:
                # Run in background, no new terminal
                proc = subprocess.Popen(
                    dimos_cmd,
                    cwd=dimos_root,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    start_new_session=True
                )
                logger.info(f"✅ DimOS start command issued: {' '.join(dimos_cmd)}, PID={proc.pid}")
                # 发送启动日志到前端
                self._vis_module._emit("app_log", {
                    "message": f"正在启动 EmergeOS: {blueprint}",
                    "status": "success",
                    "topic": "emergeos_control"
                })
                await sio.emit("dimos_start_ack", {"status": "ok", "blueprint": blueprint, "pid": proc.pid}, room=sid)
            except Exception as e:
                logger.error(f"❌ Failed to start DimOS: {e}")
                self._vis_module._emit("app_log", {
                    "message": f"启动 EmergeOS 失败: {str(e)}",
                    "status": "error",
                    "topic": "emergeos_control"
                })
                await sio.emit("dimos_start_ack", {"status": "error", "message": str(e)}, room=sid)

        @sio.event
        async def stop_dimos(sid, data=None):
            """Stop DimOS using dimos stop command."""
            logger.info(f"🛑 Stop DimOS request from {sid}")
            import subprocess
            dimos_bin = f"{dimos_root}/.venv/bin/dimos"
            try:
                # 使用 dimos stop 命令
                result = subprocess.run(
                    [dimos_bin, "stop"],
                    cwd=dimos_root,
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                logger.info(f"dimos stop output: {result.stdout} {result.stderr}")
                await sio.emit("dimos_stop_ack", {"status": "ok"}, room=sid)
                logger.info("✅ DimOS stopped")
            except Exception as e:
                logger.error(f"❌ Failed to stop DimOS: {e}")
                await sio.emit("dimos_stop_ack", {"status": "error", "message": str(e)}, room=sid)

        @sio.event
        async def emergency_stop(sid, data=None):
            """Emergency stop: stop DimOS using dimos stop --force."""
            logger.info(f"🚨 Emergency stop request from {sid}")
            import subprocess

            # 只杀 DimOS，不杀 hermes（hermes 的打断由前端通过 hermes_bridge 处理）
            dimos_bin = f"{dimos_root}/.venv/bin/dimos"
            try:
                # 先尝试 dimos stop --force
                result = subprocess.run(
                    [dimos_bin, "stop", "--force"],
                    cwd=dimos_root,
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                logger.info(f"dimos stop --force output: {result.stdout} {result.stderr}")
                logger.info("✅ DimOS stopped")
                # 发送日志到前端
                self._vis_module._emit("app_log", {
                    "message": "紧急停止: Emerge OS已中止",
                    "status": "warning",
                    "topic": "emergeos_control"
                })
            except subprocess.TimeoutExpired:
                # 超时则强制杀进程
                logger.warning("⚠️ dimos stop timed out, force killing")
                subprocess.run(["pkill", "-9", "-f", f"python.*{dimos_root}"], timeout=10)
            except Exception as e:
                logger.warning(f"⚠️ Failed to stop DimOS: {e}")

            await sio.emit("emergency_stop_ack", {"status": "ok"}, room=sid)
            logger.info("🚨 Emergency stop completed")

        logger.info("✅ DimOS control event handlers registered")

    def _shutdown_vis_socketio_cleanly_safe(self):
        """Safely shutdown Socket.IO without setting sio=None (for rebuild scenarios)."""
        vis = self._vis_module
        loop = getattr(vis, "_broadcast_loop", None)
        sio = getattr(vis, "sio", None)

        if not sio or not loop or loop.is_closed():
            return

        async def _shutdown():
            try:
                await sio.shutdown()
            except Exception as e:
                logger.debug(f"Socket.IO shutdown error: {e}")

        try:
            future = asyncio.run_coroutine_threadsafe(_shutdown(), loop)
            future.result(timeout=2.0)
        except Exception as e:
            logger.debug(f"Socket.IO shutdown wait error: {e}")

        # NOTE: Do NOT set vis.sio = None here!
        # The uvicorn thread may still be handling requests and will crash with AttributeError.

    def _shutdown_vis_socketio_cleanly(self):
        """Properly shutdown Socket.IO to avoid pending coroutine warning."""
        vis = self._vis_module
        loop = getattr(vis, "_broadcast_loop", None)
        sio = getattr(vis, "sio", None)

        if not sio or not loop or loop.is_closed():
            return

        async def _shutdown():
            try:
                await sio.shutdown()
            except Exception as e:
                logger.debug(f"Socket.IO shutdown error: {e}")

        try:
            future = asyncio.run_coroutine_threadsafe(_shutdown(), loop)
            future.result(timeout=2.0)
        except Exception as e:
            logger.debug(f"Socket.IO shutdown wait error: {e}")

        # Only set sio = None after confirming uvicorn thread has exited
        # This prevents AttributeError in old handlers
        uvicorn_thread = getattr(vis, "_uvicorn_server_thread", None)
        if uvicorn_thread and not uvicorn_thread.is_alive():
            vis.sio = None

    def stop(self):
        self._running = False
        self._log_poller.stop()
        if self._mjpeg_server: self._mjpeg_server.stop()
        if self._vis_module:
            self._shutdown_vis_socketio_cleanly()
            self._vis_module.stop()
        super().stop()

if __name__ == "__main__":
    import signal

    module = UIBridgeModule()
    # Use typed LCMTransport for DimOS message types (not pickle)
    module.color_image.transport = LCMTransport("/color_image", Image)
    module.global_map.transport = LCMTransport("/global_map", PointCloud2)
    module.odom.transport = LCMTransport("/odom", PoseStamped)
    module.path.transport = LCMTransport("/path", Path)
    module.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    module.battery_status.transport = pLCMTransport("/battery_status")

    def handle_shutdown(signum=None, frame=None):
        logger.info("Shutting down bridge...")
        module.stop()
        import sys
        sys.exit(0)

    # Handle both SIGINT (Ctrl+C) and SIGTERM (kill)
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    module.start()
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        handle_shutdown()
