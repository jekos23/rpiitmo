import argparse
import json
import os
import threading
import time
from typing import Any, Dict, Optional

from flask import Flask, jsonify, request

import main_slam as robot


app = Flask(__name__)


DEFAULT_CONFIG = {
    "lidar_port": "/dev/ttyUSB0",
    "arduino_port": "/dev/ttyACM0",
    "camera_port": "/dev/video0",
    "camera_stream_host": "0.0.0.0",
    "camera_stream_port": 5000,
    "map_choice": "3",
    "yolo_choice": "1",
    "selected_model_name": "",
    "run_mode": "2",
    "auto_speed": 1500,
    "manual_speed": 1400,
    "route_source_mode": "none",
    "route_corridor_m": 3.0,
    "slam_route_record_step_m": 0.35,
}


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Control</title>
  <style>
    :root {
      --bg: #09111a;
      --panel: #132130;
      --panel-2: #1b2f42;
      --text: #eef5fb;
      --muted: #9eb4c6;
      --accent: #3ddc97;
      --warn: #ffb454;
      --danger: #ff6b6b;
      --line: rgba(255,255,255,0.08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", system-ui, sans-serif;
      background:
        radial-gradient(circle at top left, rgba(61,220,151,0.12), transparent 30%),
        linear-gradient(180deg, #071018, var(--bg));
      color: var(--text);
    }
    .wrap {
      max-width: 1320px;
      margin: 0 auto;
      padding: 20px;
      display: grid;
      gap: 18px;
    }
    .hero {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 20px;
      padding: 18px 20px;
      border: 1px solid var(--line);
      border-radius: 20px;
      background: rgba(19,33,48,0.82);
      backdrop-filter: blur(10px);
    }
    .hero h1 {
      margin: 0;
      font-size: 28px;
    }
    .hero p {
      margin: 6px 0 0;
      color: var(--muted);
    }
    .status-pill {
      padding: 10px 14px;
      border-radius: 999px;
      font-weight: 700;
      background: rgba(255,255,255,0.08);
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: 360px minmax(0, 1fr);
      gap: 18px;
    }
    .panel {
      background: rgba(19,33,48,0.88);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      box-shadow: 0 18px 42px rgba(0,0,0,0.20);
    }
    .panel h2, .panel h3 {
      margin-top: 0;
    }
    label {
      display: block;
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 6px;
    }
    input, select {
      width: 100%;
      padding: 11px 12px;
      border-radius: 12px;
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.05);
      color: var(--text);
      margin-bottom: 12px;
    }
    .button-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 10px;
    }
    button {
      border: 0;
      border-radius: 12px;
      padding: 12px 14px;
      font-weight: 700;
      cursor: pointer;
      color: #041118;
      background: var(--accent);
    }
    button.secondary {
      background: #d7e4ef;
    }
    button.warn {
      background: var(--warn);
    }
    button.danger {
      background: var(--danger);
      color: white;
    }
    button.dark {
      background: var(--panel-2);
      color: var(--text);
      border: 1px solid var(--line);
    }
    .video {
      width: 100%;
      min-height: 320px;
      object-fit: cover;
      border-radius: 18px;
      border: 1px solid var(--line);
      background: #02070b;
    }
    .split {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 320px;
      gap: 18px;
    }
    .manual-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      align-items: center;
      justify-items: stretch;
    }
    .manual-grid .blank {
      visibility: hidden;
    }
    .stat-list {
      display: grid;
      gap: 8px;
      margin-top: 12px;
    }
    .stat-item {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      padding: 10px 12px;
      border-radius: 12px;
      background: rgba(255,255,255,0.04);
      color: var(--muted);
    }
    .stat-item strong {
      color: var(--text);
      font-weight: 600;
      text-align: right;
    }
    .full {
      grid-column: 1 / -1;
    }
    .log {
      min-height: 70px;
      white-space: pre-wrap;
      color: var(--muted);
      font-size: 14px;
      background: rgba(255,255,255,0.04);
      border-radius: 12px;
      padding: 12px;
    }
    @media (max-width: 980px) {
      .grid, .split {
        grid-template-columns: 1fr;
      }
      .hero {
        flex-direction: column;
        align-items: flex-start;
      }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <div>
        <h1>Robot Control Station</h1>
        <p>Start the robot, watch the camera stream, and switch between autopilot and manual control from any device in your LAN.</p>
      </div>
      <div id="heroStatus" class="status-pill">Loading...</div>
    </section>

    <div class="grid">
      <section class="panel">
        <h2>Launch Settings</h2>
        <label for="lidar_port">LiDAR port</label>
        <input id="lidar_port" />

        <label for="arduino_port">Arduino port</label>
        <input id="arduino_port" />

        <label for="camera_port">Camera port</label>
        <input id="camera_port" />

        <label for="camera_stream_port">Camera stream port</label>
        <input id="camera_stream_port" type="number" />

        <label for="run_mode">Run mode</label>
        <select id="run_mode">
          <option value="2">Autopilot</option>
          <option value="1">Manual</option>
        </select>

        <label for="auto_speed">Autopilot speed</label>
        <input id="auto_speed" type="number" min="0" max="4095" />

        <label for="manual_speed">Manual speed</label>
        <input id="manual_speed" type="number" min="0" max="4095" />

        <label for="yolo_choice">YOLO source</label>
        <select id="yolo_choice">
          <option value="1">Phone / PC detector over Wi-Fi</option>
          <option value="2">Local model on Raspberry Pi</option>
          <option value="3">Disabled</option>
        </select>

        <label for="route_source_mode">Route source</label>
        <select id="route_source_mode">
          <option value="none">Disabled</option>
          <option value="slam">LiDAR SLAM route</option>
          <option value="gps">Phone GPS / indoor positioning</option>
        </select>

        <label for="route_corridor_m">Route corridor (meters)</label>
        <input id="route_corridor_m" type="number" min="0.5" max="10" step="0.1" />

        <label for="slam_route_record_step_m">SLAM point step (meters)</label>
        <input id="slam_route_record_step_m" type="number" min="0.1" max="2" step="0.05" />

        <label for="selected_model_name">Local model folder name</label>
        <input id="selected_model_name" placeholder="optional" />

        <div class="button-row">
          <button class="secondary" onclick="saveConfig()">Save config</button>
          <button onclick="startRobot()">Start robot</button>
        </div>
        <div class="button-row">
          <button class="warn" onclick="prepareBucket()">Prepare bucket</button>
          <button class="danger" onclick="stopRobot()">Stop robot</button>
        </div>

        <div class="stat-list">
          <div class="stat-item"><span>Robot state</span><strong id="statRunning">--</strong></div>
          <div class="stat-item"><span>Mode</span><strong id="statMode">--</strong></div>
          <div class="stat-item"><span>Camera stream</span><strong id="statVideo">--</strong></div>
          <div class="stat-item"><span>Trash detector</span><strong id="statTrash">--</strong></div>
          <div class="stat-item"><span>Bucket Arduino</span><strong id="statArduino">--</strong></div>
          <div class="stat-item"><span>Active model</span><strong id="statModel">--</strong></div>
          <div class="stat-item"><span>Route source</span><strong id="statRouteSource">--</strong></div>
          <div class="stat-item"><span>Route status</span><strong id="statRoute">--</strong></div>
        </div>
      </section>

      <section class="panel">
        <div class="split">
          <div>
            <h2>Live Video</h2>
            <img id="videoFrame" class="video" alt="video stream" />
          </div>
          <div>
            <h2>Manual Drive</h2>
            <div class="manual-grid">
              <button class="blank">.</button>
              <button class="dark" onclick="drive('forward')">Forward</button>
              <button class="blank">.</button>
              <button class="dark" onclick="drive('left')">Left</button>
              <button class="danger" onclick="drive('stop')">STOP</button>
              <button class="dark" onclick="drive('right')">Right</button>
              <button class="blank">.</button>
              <button class="dark" onclick="drive('backward')">Backward</button>
              <button class="blank">.</button>
            </div>

            <h3 style="margin-top:18px;">Bucket Control</h3>
            <div class="button-row">
              <button class="dark" onclick="bucket('wall_down')">Wall down</button>
              <button class="dark" onclick="bucket('wall_up')">Wall up</button>
            </div>
            <div class="button-row">
              <button class="dark" onclick="bucket('scoop_down')">Scoop 90°</button>
              <button class="dark" onclick="bucket('scoop_up')">Scoop 0°</button>
            </div>
            <div class="button-row">
              <button class="dark" onclick="bucket('bucket_test')">Timed test</button>
              <button onclick="bucket('collect')">Collect cycle</button>
            </div>

            <h3 style="margin-top:18px;">SLAM Route</h3>
            <div class="button-row">
              <button class="dark" onclick="slamRoute('start_record')">Record route</button>
              <button class="dark" onclick="slamRoute('stop_record')">Stop record</button>
            </div>
            <div class="button-row">
              <button class="warn" onclick="slamRoute('clear')">Clear route</button>
              <button class="secondary" onclick="refreshStatus()">Refresh</button>
            </div>
          </div>
        </div>

        <div class="full" style="margin-top:18px;">
          <h3>System Log</h3>
          <div id="logBox" class="log">Waiting for status...</div>
        </div>
      </section>
    </div>
  </div>

  <script>
    let lastStatus = null;

    function formPayload() {
      return {
        lidar_port: document.getElementById('lidar_port').value.trim(),
        arduino_port: document.getElementById('arduino_port').value.trim(),
        camera_port: document.getElementById('camera_port').value.trim(),
        camera_stream_port: parseInt(document.getElementById('camera_stream_port').value || '5000', 10),
        run_mode: document.getElementById('run_mode').value,
        auto_speed: parseInt(document.getElementById('auto_speed').value || '1500', 10),
        manual_speed: parseInt(document.getElementById('manual_speed').value || '1400', 10),
        yolo_choice: document.getElementById('yolo_choice').value,
        route_source_mode: document.getElementById('route_source_mode').value,
        route_corridor_m: parseFloat(document.getElementById('route_corridor_m').value || '3.0'),
        slam_route_record_step_m: parseFloat(document.getElementById('slam_route_record_step_m').value || '0.35'),
        selected_model_name: document.getElementById('selected_model_name').value.trim(),
      };
    }

    function applyConfig(config) {
      if (!config) return;
      document.getElementById('lidar_port').value = config.lidar_port || '';
      document.getElementById('arduino_port').value = config.arduino_port || '';
      document.getElementById('camera_port').value = config.camera_port || '';
      document.getElementById('camera_stream_port').value = config.camera_stream_port || 5000;
      document.getElementById('run_mode').value = String(config.run_mode || '2');
      document.getElementById('auto_speed').value = config.auto_speed || 1500;
      document.getElementById('manual_speed').value = config.manual_speed || 1400;
      document.getElementById('yolo_choice').value = String(config.yolo_choice || '1');
      document.getElementById('route_source_mode').value = String(config.route_source_mode || 'none');
      document.getElementById('route_corridor_m').value = config.route_corridor_m || 3.0;
      document.getElementById('slam_route_record_step_m').value = config.slam_route_record_step_m || 0.35;
      document.getElementById('selected_model_name').value = config.selected_model_name || '';
    }

    function setLog(text) {
      document.getElementById('logBox').textContent = text || 'No messages yet.';
    }

    function updateStatusUi(status) {
      lastStatus = status;
      applyConfig(status.config);
      const routeSourceLabels = {
        none: 'disabled',
        slam: 'LiDAR SLAM',
        gps: 'Phone GPS/UWB',
      };
      let routeSummary = 'disabled';
      if (status.route_source_mode === 'slam') {
        routeSummary = `${status.slam_route_points || 0} pts`;
        if (typeof status.route_distance_m === 'number') {
          routeSummary += ` | ${status.route_distance_m.toFixed(2)} m`;
        }
        if (status.slam_route_recording) {
          routeSummary += ' | recording';
        }
      } else if (status.route_source_mode === 'gps') {
        routeSummary = status.route_fresh ? 'phone route active' : 'waiting for phone route';
        if (typeof status.route_distance_m === 'number') {
          routeSummary += ` | ${status.route_distance_m.toFixed(2)} m`;
        }
        if (status.route_enabled) {
          routeSummary += status.route_within_corridor ? ' | inside corridor' : ' | outside corridor';
        }
      }

      document.getElementById('heroStatus').textContent = status.running
        ? `Running: ${status.mode_label}`
        : 'Stopped';
      document.getElementById('heroStatus').style.background = status.running
        ? 'rgba(61,220,151,0.18)'
        : 'rgba(255,107,107,0.15)';
      document.getElementById('statRunning').textContent = status.running ? 'online' : 'offline';
      document.getElementById('statMode').textContent = status.mode_label || '--';
      document.getElementById('statVideo').textContent = status.video_stream_active ? status.stream_url : 'stopped';
      document.getElementById('statTrash').textContent = status.trash_summary || 'disabled';
      document.getElementById('statArduino').textContent = status.bucket_arduino_connected ? 'connected' : 'disconnected';
      document.getElementById('statModel').textContent = status.selected_model_name || 'phone / none';
      document.getElementById('statRouteSource').textContent = routeSourceLabels[status.route_source_mode] || status.route_source_mode || 'disabled';
      document.getElementById('statRoute').textContent = routeSummary;

      const logLines = [
        status.message || '',
        status.error ? `Error: ${status.error}` : '',
        status.stream_url ? `Stream: ${status.stream_url}` : '',
        status.detector_debug ? `Detector: ${status.detector_debug}` : '',
        status.route_debug ? `Route: ${status.route_debug}` : '',
      ].filter(Boolean);
      setLog(logLines.join('\\n'));

      const video = document.getElementById('videoFrame');
      if (status.stream_url && video.dataset.currentSrc !== status.stream_url) {
        video.src = status.stream_url + '?t=' + Date.now();
        video.dataset.currentSrc = status.stream_url;
      }
      if (!status.stream_url) {
        video.removeAttribute('src');
        video.dataset.currentSrc = '';
      }
    }

    async function api(path, payload) {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload || {}),
      });
      const data = await response.json();
      updateStatusUi(data);
      return data;
    }

    async function saveConfig() {
      await api('/api/config', formPayload());
    }

    async function startRobot() {
      await api('/api/start', formPayload());
    }

    async function stopRobot() {
      await api('/api/stop', {});
    }

    async function prepareBucket() {
      await api('/api/bucket', { action: 'prepare' });
    }

    async function drive(action) {
      const payload = { action, speed: parseInt(document.getElementById('manual_speed').value || '1400', 10) };
      await api('/api/drive', payload);
    }

    async function bucket(action) {
      await api('/api/bucket', { action });
    }

    async function slamRoute(action) {
      await api('/api/slam_route', { action });
    }

    async function refreshStatus() {
      const response = await fetch('/api/status');
      const data = await response.json();
      updateStatusUi(data);
    }

    refreshStatus();
    setInterval(refreshStatus, 1200);
  </script>
</body>
</html>
"""


class RobotManager:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.driver = None
        self.detector = None
        self.slam_thread = None
        self.worker_thread = None
        self.running = False
        self.mode = "idle"
        self.message = "Robot is stopped."
        self.error = ""
        self.started_at = None
        self.selected_model_name = ""

    def _defaults(self) -> Dict[str, Any]:
        config = dict(DEFAULT_CONFIG)
        config.update(robot.load_config())
        return config

    def _normalize_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        normalized = self._defaults()
        normalized.update({k: v for k, v in config.items() if v is not None})
        normalized["lidar_port"] = str(normalized.get("lidar_port", "")).strip()
        normalized["arduino_port"] = str(normalized.get("arduino_port", "")).strip()
        normalized["camera_port"] = str(normalized.get("camera_port", "")).strip()
        normalized["camera_stream_host"] = str(
            normalized.get("camera_stream_host", "0.0.0.0")
        ).strip() or "0.0.0.0"
        normalized["camera_stream_port"] = int(
            normalized.get("camera_stream_port", 5000)
        )
        normalized["map_choice"] = str(normalized.get("map_choice", "3"))
        normalized["yolo_choice"] = str(normalized.get("yolo_choice", "1"))
        normalized["run_mode"] = str(normalized.get("run_mode", "2"))
        normalized["auto_speed"] = int(normalized.get("auto_speed", 1500))
        normalized["manual_speed"] = int(normalized.get("manual_speed", 1400))
        normalized["route_source_mode"] = str(
            normalized.get("route_source_mode", "none")
        ).strip().lower()
        if normalized["route_source_mode"] not in {"none", "slam", "gps"}:
            normalized["route_source_mode"] = "none"
        try:
            normalized["route_corridor_m"] = max(
                0.1, float(normalized.get("route_corridor_m", 3.0))
            )
        except (TypeError, ValueError):
            normalized["route_corridor_m"] = 3.0
        try:
            normalized["slam_route_record_step_m"] = max(
                0.05, float(normalized.get("slam_route_record_step_m", 0.35))
            )
        except (TypeError, ValueError):
            normalized["slam_route_record_step_m"] = 0.35
        normalized["selected_model_name"] = str(
            normalized.get("selected_model_name", "")
        ).strip()
        return normalized

    def current_config(self) -> Dict[str, Any]:
        return self._normalize_config(robot.load_config())

    def save_config(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            config = self._normalize_config(payload)
            robot.global_config = dict(config)
            robot.save_config(config)
            self.message = "Configuration saved."
            self.error = ""
            return self.status()

    def _available_models(self):
        models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        if not os.path.exists(models_dir):
            return []
        return sorted(
            name
            for name in os.listdir(models_dir)
            if os.path.isdir(os.path.join(models_dir, name))
        )

    def _build_detector(self, config: Dict[str, Any]):
        choice = str(config.get("yolo_choice", "1"))
        self.selected_model_name = ""

        if choice == "1" and robot.RemoteTrashListener:
            detector = robot.RemoteTrashListener(
                on_servo_command=robot.handle_remote_servo_command,
                on_motor_command=robot.handle_remote_bucket_motor_command,
                allow_text_commands=(str(config.get("run_mode", "2")) == "1"),
            )
            detector.start()
            return detector

        if choice == "2" and robot.TrashDetector:
            available_models = self._available_models()
            if not available_models:
                self.message = "No local YOLO models were found in the models folder."
                return None

            requested_name = str(config.get("selected_model_name", "")).strip()
            if requested_name and requested_name in available_models:
                model_name = requested_name
            else:
                model_name = available_models[0]
                config["selected_model_name"] = model_name
                robot.global_config = dict(config)
                robot.save_config(config)

            model_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "models",
                model_name,
            )
            self.selected_model_name = model_name
            detector = robot.TrashDetector(model_path=model_path)
            detector.start()
            return detector

        return None

    def _shutdown_runtime(self):
        robot.stop_all()

        detector = self.detector
        self.detector = None
        if detector and hasattr(detector, "stop"):
            try:
                detector.stop()
            except Exception:
                pass

        driver = self.driver
        self.driver = None
        if driver:
            try:
                driver.stop()
            except Exception:
                pass

        try:
            robot.close_bucket_arduino()
        except Exception:
            pass

        try:
            robot.stop_video_streamer()
        except Exception:
            pass

        self.slam_thread = None
        self.worker_thread = None
        self.running = False
        self.mode = "idle"
        self.started_at = None

    def _run_autopilot(self, auto_speed):
        try:
            robot.autonomous_loop(self.driver, auto_speed, self.detector)
        except Exception as exc:
            with self.lock:
                self.error = f"Autopilot crashed: {exc}"
                self.message = "Autopilot stopped with an error."
        finally:
            with self.lock:
                self._shutdown_runtime()

    def start(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            if self.running:
                self.message = "Robot is already running."
                return self.status()

            config = self._normalize_config(payload)
            self.error = ""
            self.message = "Starting robot..."

            if not config["lidar_port"]:
                self.error = "LiDAR port is required."
                self.message = "Start failed."
                return self.status()

            if not config["arduino_port"]:
                self.error = "Arduino port is required."
                self.message = "Start failed."
                return self.status()

            if not config["camera_port"]:
                self.error = "Camera port is required."
                self.message = "Start failed."
                return self.status()

            if (
                config["route_source_mode"] == "gps"
                and str(config.get("yolo_choice", "1")) != "1"
            ):
                self.error = (
                    "Phone GPS route mode requires YOLO source 1 "
                    "(phone / PC detector over Wi-Fi)."
                )
                self.message = "Start failed."
                return self.status()

            robot.global_config = dict(config)
            robot.save_config(config)

            try:
                robot.start_video_streamer(config)

                if not robot.init_bucket_arduino(config):
                    raise RuntimeError("Failed to connect to bucket Arduino.")

                robot.move_bucket_wall_to_search_position()
                robot.set_servo_bucket(down=True)

                detector = self._build_detector(config)
                driver = robot.LD06Driver(port=config["lidar_port"])
                driver.start()
                if not driver.running:
                    raise RuntimeError("LiDAR did not start.")

                if config["map_choice"] == "2":
                    os.environ["DISPLAY"] = ":0"
                    show_map = True
                else:
                    show_map = config["map_choice"] == "1"

                slam_thread = threading.Thread(
                    target=robot.slam_thread_function,
                    args=(driver, show_map),
                    daemon=True,
                )
                slam_thread.start()

                self.driver = driver
                self.detector = detector
                self.slam_thread = slam_thread
                self.running = True
                self.started_at = time.time()

                if config["run_mode"] == "2":
                    auto_speed = int(config.get("auto_speed", 1500))
                    self.mode = "autopilot"
                    self.worker_thread = threading.Thread(
                        target=self._run_autopilot,
                        args=(auto_speed,),
                        daemon=True,
                    )
                    self.worker_thread.start()
                    self.message = "Robot started in autopilot mode."
                else:
                    self.mode = "manual"
                    self.message = "Robot started in manual web-control mode."

            except Exception as exc:
                self.error = str(exc)
                self.message = "Robot start failed."
                self._shutdown_runtime()

            return self.status()

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            self.message = "Stopping robot..."
            self.error = ""
            self._shutdown_runtime()
            self.message = "Robot stopped."
            return self.status()

    def prepare_bucket(self) -> Dict[str, Any]:
        with self.lock:
            try:
                robot.move_bucket_wall_to_search_position()
                robot.set_servo_bucket(down=True)
                self.message = "Bucket moved to search position."
                self.error = ""
            except Exception as exc:
                self.error = str(exc)
                self.message = "Failed to prepare bucket."
            return self.status()

    def manual_drive(self, action: str, speed: int) -> Dict[str, Any]:
        with self.lock:
            if not self.running:
                self.message = "Robot is not running."
                return self.status()

            speed = max(0, min(4095, int(speed)))

            if action == "stop":
                robot.stop_all()
                self.message = "Emergency stop."
                return self.status()

            if self.mode != "manual":
                self.message = "Switch run mode to manual to use drive buttons."
                return self.status()

            with robot.movement_lock:
                robot.current_speed = speed

            if action == "forward":
                with robot.movement_lock:
                    robot.current_mode = 1
                robot.set_motors(speed, 0, speed, 0)
                self.message = f"Driving forward at {speed}."
            elif action == "backward":
                with robot.movement_lock:
                    robot.current_mode = 2
                robot.set_motors(0, speed, 0, speed)
                self.message = f"Driving backward at {speed}."
            elif action == "left":
                with robot.movement_lock:
                    robot.current_mode = 3
                robot.set_motors(speed, 0, 0, speed)
                self.message = f"Turning left at {speed}."
            elif action == "right":
                with robot.movement_lock:
                    robot.current_mode = 4
                robot.set_motors(0, speed, speed, 0)
                self.message = f"Turning right at {speed}."
            else:
                self.message = f"Unknown drive action: {action}"

            return self.status()

    def bucket_action(self, action: str) -> Dict[str, Any]:
        with self.lock:
            try:
                if action == "wall_up":
                    robot.move_bucket_wall_to_search_position()
                    self.message = "Wall raised."
                elif action == "wall_down":
                    robot.move_bucket_wall_to_lower_position()
                    self.message = "Wall lowered."
                elif action == "scoop_up":
                    robot.set_servo_bucket(down=False, wait=True)
                    self.message = "Scoop moved to 0 degrees."
                elif action == "scoop_down":
                    robot.set_servo_bucket(down=True, wait=True)
                    self.message = "Scoop moved to 90 degrees."
                elif action == "bucket_test":
                    robot.run_bucket_wall_timed_test()
                    self.message = "Bucket timed test finished."
                elif action == "collect":
                    robot.run_bucket_collect_cycle()
                    self.message = "Collect cycle finished."
                elif action == "prepare":
                    robot.move_bucket_wall_to_search_position()
                    robot.set_servo_bucket(down=True)
                    self.message = "Bucket prepared."
                else:
                    self.message = f"Unknown bucket action: {action}"
            except Exception as exc:
                self.error = str(exc)
                self.message = "Bucket action failed."

            return self.status()

    def slam_route_action(self, action: str) -> Dict[str, Any]:
        with self.lock:
            try:
                if action == "start_record":
                    if not self.running or not self.driver or not getattr(self.driver, "running", False):
                        self.message = "Start the robot before recording a SLAM route."
                        return self.status()
                    robot.start_slam_route_recording(clear_existing=True)
                    self.message = "SLAM route recording started. Drive the robot along the desired path."
                elif action == "stop_record":
                    robot.stop_slam_route_recording()
                    self.message = "SLAM route recording stopped."
                elif action == "clear":
                    robot.stop_slam_route_recording()
                    robot.clear_slam_route()
                    self.message = "SLAM route cleared."
                else:
                    self.message = f"Unknown SLAM route action: {action}"
                self.error = ""
            except Exception as exc:
                self.error = str(exc)
                self.message = "SLAM route action failed."

            return self.status()

    def status(self) -> Dict[str, Any]:
        config = self.current_config()
        detector = self.detector
        route_state = robot.get_route_state(detector)
        slam_route_status = robot.get_slam_route_status()
        trash_summary = "disabled"
        detector_debug = ""
        if detector:
            trash_detected = bool(getattr(detector, "trash_detected", False))
            trash_angle = float(getattr(detector, "trash_angle", 0.0))
            if trash_detected:
                trash_summary = f"trash at {trash_angle:.1f} deg"
            else:
                trash_summary = "active, waiting"
            detector_debug = (
                f"detected={trash_detected}, angle={trash_angle:.1f}, "
                f"allow_text={getattr(detector, 'allow_text_commands', False)}"
            )
            if route_state["source_mode"] == "gps":
                detector_debug += (
                    f", route_enabled={route_state['route_enabled']}, "
                    f"route_fresh={route_state['route_fresh']}"
                )

        stream_port = int(config.get("camera_stream_port", 5000))
        stream_url = f"http://{request.host.split(':')[0]}:{stream_port}/video_feed"
        video_stream_active = bool(
            robot.video_streamer_process and robot.video_streamer_process.poll() is None
        )
        if not video_stream_active:
            stream_url = ""

        uptime_sec = None
        if self.started_at:
            uptime_sec = max(0, int(time.time() - self.started_at))

        route_debug_parts = [
            f"source={route_state['source_mode']}",
            f"enabled={route_state['route_enabled']}",
            f"within={route_state['within_corridor']}",
        ]
        if route_state["distance_m"] is not None:
            route_debug_parts.append(f"distance={float(route_state['distance_m']):.2f}m")
        if slam_route_status["points_count"]:
            route_debug_parts.append(f"slam_points={slam_route_status['points_count']}")
        if slam_route_status["recording"]:
            route_debug_parts.append("recording=yes")
        route_debug = ", ".join(route_debug_parts)

        return {
            "running": self.running,
            "mode": self.mode,
            "mode_label": {
                "idle": "Stopped",
                "manual": "Manual web control",
                "autopilot": "Autopilot",
            }.get(self.mode, self.mode),
            "message": self.message,
            "error": self.error,
            "config": config,
            "uptime_sec": uptime_sec,
            "video_stream_active": video_stream_active,
            "stream_url": stream_url,
            "bucket_arduino_connected": robot.bucket_arduino is not None,
            "trash_summary": trash_summary,
            "detector_debug": detector_debug,
            "selected_model_name": self.selected_model_name or config.get("selected_model_name", ""),
            "current_mode": robot.current_mode,
            "current_speed": robot.current_speed,
            "lidar_running": bool(self.driver and getattr(self.driver, "running", False)),
            "route_source_mode": route_state["source_mode"],
            "route_enabled": route_state["route_enabled"],
            "route_within_corridor": route_state["within_corridor"],
            "route_distance_m": route_state["distance_m"],
            "route_fresh": route_state["route_fresh"],
            "route_debug": route_debug,
            "slam_route_recording": slam_route_status["recording"],
            "slam_route_points": slam_route_status["points_count"],
            "slam_pose": slam_route_status["pose"],
        }


manager = RobotManager()


@app.get("/")
def index():
    return HTML_PAGE


@app.get("/api/status")
def api_status():
    return jsonify(manager.status())


@app.post("/api/config")
def api_config():
    payload = request.get_json(silent=True) or {}
    return jsonify(manager.save_config(payload))


@app.post("/api/start")
def api_start():
    payload = request.get_json(silent=True) or {}
    return jsonify(manager.start(payload))


@app.post("/api/stop")
def api_stop():
    return jsonify(manager.stop())


@app.post("/api/drive")
def api_drive():
    payload = request.get_json(silent=True) or {}
    return jsonify(
        manager.manual_drive(
            action=str(payload.get("action", "stop")),
            speed=int(payload.get("speed", 1400)),
        )
    )


@app.post("/api/bucket")
def api_bucket():
    payload = request.get_json(silent=True) or {}
    return jsonify(manager.bucket_action(str(payload.get("action", ""))))


@app.post("/api/slam_route")
def api_slam_route():
    payload = request.get_json(silent=True) or {}
    return jsonify(manager.slam_route_action(str(payload.get("action", ""))))


def main():
    parser = argparse.ArgumentParser(description="Robot web control panel")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP host")
    parser.add_argument("--port", type=int, default=8088, help="HTTP port")
    args = parser.parse_args()

    config = manager.current_config()
    print("[WEB] Robot control UI is starting.")
    print(
        f"[WEB] Open http://<RASPBERRY_PI_IP>:{args.port} from a device in your local network."
    )
    print(
        f"[WEB] Camera stream will be available on http://<RASPBERRY_PI_IP>:{config.get('camera_stream_port', 5000)}/video_feed"
    )
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
