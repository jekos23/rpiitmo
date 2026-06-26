import argparse
import os
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

from flask import Flask, Response, jsonify, request

import main_slam as robot


app = Flask(__name__)


@app.after_request
def add_no_cache_headers(response):
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response


@app.get("/favicon.ico")
def favicon():
    return Response(status=204)


DEFAULT_CONFIG = {
    "lidar_port": "/dev/ttyUSB0",
    "camera_port": "/dev/video0",
    "camera_stream_host": "0.0.0.0",
    "camera_stream_port": 5000,
    "drive_pca_address": "0x40",
    "servo_pca_address": "0x42",
    "bucket_servo_channel": 0,
    "servo_down_angle": 93,
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


def _safe_json_value(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _safe_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_safe_json_value(item) for item in value]
    try:
        return float(value)
    except Exception:
        pass
    return str(value)

def _repair_mojibake_text(text: str) -> str:
    if not text:
        return text
    try:
        return text.encode("cp1251").decode("utf-8")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return text


def _repair_mojibake_block(text: str) -> str:
    return "".join(_repair_mojibake_text(line) for line in text.splitlines(keepends=True))


HTML_PAGE = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Web UI</title>
  <style>
    body {
      margin: 0;
      padding: 16px;
      font-family: Arial, sans-serif;
      background: #0f1720;
      color: #f2f5f7;
    }
    h1, h2, h3 { margin: 0 0 12px; }
    .wrap { max-width: 1200px; margin: 0 auto; }
    .panel {
      background: #172330;
      border: 1px solid #294055;
      border-radius: 8px;
      padding: 14px;
      margin-bottom: 16px;
    }
    .grid {
      display: grid;
      grid-template-columns: 360px 1fr;
      gap: 16px;
    }
    .row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .field { margin-bottom: 10px; }
    label {
      display: block;
      font-size: 12px;
      color: #b8c7d4;
      margin-bottom: 4px;
    }
    input, select, button {
      width: 100%;
      padding: 10px;
      border-radius: 6px;
      border: 1px solid #37506a;
      font-size: 14px;
      box-sizing: border-box;
    }
    input, select {
      background: #0f1720;
      color: #f2f5f7;
    }
    button {
      cursor: pointer;
      background: #2f8f5b;
      color: #ffffff;
      font-weight: bold;
    }
    button.secondary { background: #33506b; }
    button.warn { background: #a3741d; }
    button.danger { background: #a33c3c; }
    .status {
      padding: 10px 12px;
      border-radius: 6px;
      background: #203244;
      margin-bottom: 12px;
      font-weight: bold;
    }
    .hint, .log, .stats {
      font-size: 13px;
      line-height: 1.45;
      color: #d7e2ea;
    }
    .hint { color: #aebdca; margin: 4px 0 10px; }
    .video {
      width: 100%;
      min-height: 320px;
      background: #06090c;
      border: 1px solid #37506a;
      border-radius: 8px;
      object-fit: cover;
    }
    .log {
      white-space: pre-wrap;
      background: #101922;
      border: 1px solid #2b3d4f;
      border-radius: 8px;
      padding: 10px;
      min-height: 90px;
    }
    .drive-grid {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 8px;
      margin-top: 8px;
    }
    .drive-grid .blank {
      visibility: hidden;
    }
    @media (max-width: 900px) {
      .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="panel">
      <h1>Robot Web UI</h1>
      <div id="statusBox" class="status">Loading...</div>
      <div id="quickStats" class="stats"></div>
    </div>

    <div class="grid">
      <div class="panel">
        <h2>Config</h2>
        <div class="field"><label for="lidar_port">LiDAR port</label><input id="lidar_port"><div id="lidar_hint" class="hint"></div></div>
        <div class="field"><label for="camera_port">Camera port</label><input id="camera_port"><div id="camera_hint" class="hint"></div></div>
        <div class="field"><label for="drive_pca_address">Drive PCA address</label><input id="drive_pca_address"><div id="i2c_hint" class="hint"></div></div>
        <div class="field"><label for="servo_pca_address">Servo PCA address</label><input id="servo_pca_address"></div>
        <div class="row"><div class="field"><label for="bucket_servo_channel">Bucket servo channel</label><input id="bucket_servo_channel" type="number"></div><div class="field"><label for="servo_down_angle">Bucket down angle</label><input id="servo_down_angle" type="number" min="0" max="180"></div></div>
        <div class="row"><div class="field"><label for="camera_stream_port">Camera stream port</label><input id="camera_stream_port" type="number"></div><div class="field"></div></div>
        <div class="row"><div class="field"><label for="run_mode">Run mode</label><select id="run_mode"><option value="2">Autopilot</option><option value="1">Manual</option></select></div><div class="field"><label for="yolo_choice">YOLO</label><select id="yolo_choice"><option value="1">Phone / PC</option><option value="2">Local model</option><option value="3">Disabled</option></select></div></div>
        <div class="row"><div class="field"><label for="auto_speed">Autopilot speed</label><input id="auto_speed" type="number"></div><div class="field"><label for="manual_speed">Manual speed</label><input id="manual_speed" type="number"></div></div>
        <div class="row"><div class="field"><label for="route_source_mode">Route source</label><select id="route_source_mode"><option value="none">Disabled</option><option value="slam">SLAM</option><option value="gps">GPS</option></select></div><div class="field"><label for="selected_model_name">Model</label><select id="selected_model_name"></select></div></div>
        <div class="row"><div class="field"><label for="route_corridor_m">Route corridor</label><input id="route_corridor_m" type="number" step="0.1"></div><div class="field"><label for="slam_route_record_step_m">SLAM step</label><input id="slam_route_record_step_m" type="number" step="0.05"></div></div>
        <div class="row"><button class="secondary" type="button" onclick="saveConfig()">Save config</button><button type="button" onclick="startRobot()">Start robot</button></div>
        <div class="row" style="margin-top:10px;"><button class="warn" type="button" onclick="prepareBucket()">Prepare bucket</button><button class="danger" type="button" onclick="stopRobot()">Stop robot</button></div>
      </div>

      <div>
        <div class="panel"><h2>Video</h2><img id="videoFrame" class="video" alt="video"></div>
        <div class="panel"><h2>Manual drive</h2><div class="drive-grid"><button class="blank" type="button">.</button><button type="button" onclick="drive('forward')">Forward</button><button class="blank" type="button">.</button><button type="button" onclick="drive('left')">Left</button><button class="danger" type="button" onclick="drive('stop')">Stop</button><button type="button" onclick="drive('right')">Right</button><button class="blank" type="button">.</button><button type="button" onclick="drive('backward')">Backward</button><button class="blank" type="button">.</button></div></div>
        <div class="panel"><h3>Bucket</h3><div class="row"><button class="secondary" type="button" onclick="bucket('wall_down')">Wall down</button><button class="secondary" type="button" onclick="bucket('wall_up')">Wall up</button></div><div class="row" style="margin-top:10px;"><button class="secondary" type="button" onclick="bucket('scoop_down')">Scoop down</button><button class="secondary" type="button" onclick="bucket('scoop_up')">Scoop up</button></div><div class="row" style="margin-top:10px;"><button class="secondary" type="button" onclick="bucket('bucket_test')">Timed test</button><button type="button" onclick="bucket('collect')">Collect cycle</button></div></div>
        <div class="panel"><h3>SLAM route</h3><div class="row"><button class="secondary" type="button" onclick="slamRoute('start_record')">Record route</button><button class="secondary" type="button" onclick="slamRoute('stop_record')">Stop record</button></div><div class="row" style="margin-top:10px;"><button class="warn" type="button" onclick="slamRoute('clear')">Clear route</button><button class="secondary" type="button" onclick="refreshStatus()">Refresh</button></div></div>
        <div class="panel"><h2>Log</h2><div id="logBox" class="log">Waiting for status...</div></div>
      </div>
    </div>
  </div>

  <script>
    (function () {
      var markReady = function () {
        var box = document.getElementById('statusBox');
        var log = document.getElementById('logBox');
        if (box) box.innerHTML = 'Boot script OK';
        if (log) log.innerHTML = 'Boot script OK';
      };
      if (document.readyState === 'loading' && document.addEventListener) {
        document.addEventListener('DOMContentLoaded', markReady, false);
      } else {
        markReady();
      }
    }());
  </script>

  <script>
    function byId(id) {
      return document.getElementById(id);
    }

    function setText(id, text) {
      var el = byId(id);
      if (!el) return;
      if (typeof el.textContent !== 'undefined') el.textContent = text;
      else el.innerText = text;
    }

    function setValue(id, value) {
      var el = byId(id);
      if (!el) return;
      el.value = value === null || typeof value === 'undefined' ? '' : value;
    }

    function setSelectOptions(id, values, selectedValue) {
      var el = byId(id);
      var i;
      var option;
      var safeValues = values || [];
      if (!el) return;
      el.innerHTML = '';
      for (i = 0; i < safeValues.length; i += 1) {
        option = document.createElement('option');
        option.value = safeValues[i];
        option.textContent = safeValues[i];
        if (safeValues[i] === selectedValue) option.selected = true;
        el.appendChild(option);
      }
      if (!safeValues.length) {
        option = document.createElement('option');
        option.value = '';
        option.textContent = 'No models found';
        el.appendChild(option);
      }
    }

    function setLog(text) {
      setText('logBox', text || 'No messages');
    }

    function getConfigPayload() {
      return {
        lidar_port: byId('lidar_port').value,
        camera_port: byId('camera_port').value,
        drive_pca_address: byId('drive_pca_address').value,
        servo_pca_address: byId('servo_pca_address').value,
        bucket_servo_channel: byId('bucket_servo_channel').value,
        servo_down_angle: byId('servo_down_angle').value,
        camera_stream_port: byId('camera_stream_port').value,
        run_mode: byId('run_mode').value,
        yolo_choice: byId('yolo_choice').value,
        auto_speed: byId('auto_speed').value,
        manual_speed: byId('manual_speed').value,
        route_source_mode: byId('route_source_mode').value,
        selected_model_name: byId('selected_model_name').value,
        route_corridor_m: byId('route_corridor_m').value,
        slam_route_record_step_m: byId('slam_route_record_step_m').value
      };
    }

    function applyConfig(config) {
      config = config || {};
      setValue('lidar_port', config.lidar_port || '');
      setValue('camera_port', config.camera_port || '');
      setValue('camera_stream_port', config.camera_stream_port || 5000);
      setValue('drive_pca_address', config.drive_pca_address || '');
      setValue('servo_pca_address', config.servo_pca_address || '');
      setValue('bucket_servo_channel', config.bucket_servo_channel || 0);
      setValue('servo_down_angle', typeof config.servo_down_angle !== 'undefined' ? config.servo_down_angle : 93);
      setValue('run_mode', config.run_mode || '2');
      setValue('auto_speed', config.auto_speed || 1500);
      setValue('manual_speed', config.manual_speed || 1400);
      setValue('yolo_choice', config.yolo_choice || '1');
      setValue('route_source_mode', config.route_source_mode || 'none');
      setValue('route_corridor_m', config.route_corridor_m || 3.0);
      setValue('slam_route_record_step_m', config.slam_route_record_step_m || 0.35);
    }

    function requestJson(method, url, payload, onSuccess, onError) {
      var xhr = new XMLHttpRequest();
      xhr.open(method, url, true);
      xhr.setRequestHeader('Content-Type', 'application/json');
      xhr.onreadystatechange = function () {
        var data;
        if (xhr.readyState !== 4) return;
        if (xhr.status < 200 || xhr.status >= 300) {
          if (onError) onError('HTTP ' + xhr.status);
          return;
        }
        try {
          data = xhr.responseText ? JSON.parse(xhr.responseText) : {};
        } catch (e) {
          if (onError) onError('JSON parse error');
          return;
        }
        if (onSuccess) onSuccess(data);
      };
      xhr.onerror = function () {
        if (onError) onError('Network error');
      };
      xhr.send(payload ? JSON.stringify(payload) : null);
    }

    function updateStatus(status) {
      var lines = [];
      var video = byId('videoFrame');
      status = status || {};
      setText('statusBox', status.running ? ('Running / ' + status.mode) : 'Stopped');
      setSelectOptions('selected_model_name', status.available_models || [], (status.selected_model_name || (status.config || {}).selected_model_name || ''));
      applyConfig(status.config || {});
      setText('lidar_hint', status.available_serial_ports ? 'Serial: ' + status.available_serial_ports.join(', ') : 'No serial ports detected');
      setText('camera_hint', status.available_camera_ports ? 'Video: ' + status.available_camera_ports.join(', ') : 'No camera devices detected');
      setText('i2c_hint', status.available_i2c_addresses ? 'I2C: ' + status.available_i2c_addresses.join(', ') : 'No I2C addresses detected');
      setText('quickStats', 'LiDAR: ' + (status.lidar_running ? 'on' : 'off') + ' | Video: ' + (status.video_stream_active ? 'on' : 'off'));
      if (status.message) lines.push(status.message);
      if (status.error) lines.push('Error: ' + status.error);
      if (status.config_error) lines.push('Config: ' + status.config_error);
      if (status.i2c_error) lines.push('I2C: ' + status.i2c_error);
      if (status.video_error) lines.push('Video: ' + status.video_error);
      if (status.stream_url) lines.push('Stream: ' + status.stream_url);
      if (status.config_path) lines.push('Config file: ' + status.config_path);
      setLog(lines.join('\\n'));
      if (video) {
        if (status.stream_url) video.src = status.stream_url + '?t=' + (new Date().getTime());
        else video.removeAttribute('src');
      }
    }

    function onError(message) {
      setText('statusBox', 'UI error: ' + message);
      setLog('UI error: ' + message);
    }

    function refreshStatus() {
      setText('statusBox', 'Requesting /api/status ...');
      requestJson('GET', '/api/status', null, updateStatus, onError);
    }

    function postAction(url, payload) {
      requestJson('POST', url, payload, updateStatus, onError);
    }

    function saveConfig() { postAction('/api/config', getConfigPayload()); }
    function startRobot() { postAction('/api/start', getConfigPayload()); }
    function stopRobot() { postAction('/api/stop', {}); }
    function prepareBucket() { postAction('/api/bucket', { action: 'prepare' }); }
    function drive(action) { postAction('/api/drive', { action: action, speed: byId('manual_speed').value || 1400 }); }
    function bucket(action) { postAction('/api/bucket', { action: action }); }
    function slamRoute(action) { postAction('/api/slam_route', { action: action }); }

    function initUi() {
      setText('statusBox', 'Main script OK');
      refreshStatus();
    }

    if (document.readyState === 'loading' && document.addEventListener) {
      document.addEventListener('DOMContentLoaded', initUi, false);
    } else if (window.attachEvent) {
      window.attachEvent('onload', initUi);
    } else {
      initUi();
    }
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

    def _notify_status(self, text: str, dedupe_key: Optional[str] = None, dedupe_window_sec: float = 15.0) -> None:
        return None

    def _defaults(self) -> Dict[str, Any]:
        config = dict(DEFAULT_CONFIG)
        config.update(robot.load_config())
        return config

    def _normalize_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        normalized = self._defaults()
        normalized.update({k: v for k, v in config.items() if v is not None})
        normalized.pop("arduino_port", None)
        normalized.pop("arduino_baudrate", None)
        normalized.pop("arduino_timeout_sec", None)
        normalized.pop("arduino_boot_wait_sec", None)
        normalized["lidar_port"] = str(normalized.get("lidar_port", "")).strip()
        normalized["camera_port"] = str(normalized.get("camera_port", "")).strip()
        if normalized["camera_port"]:
            try:
                normalized["camera_port"] = robot.resolve_camera_port(
                    normalized["camera_port"]
                )
            except Exception:
                pass
        normalized["drive_pca_address"] = str(
            normalized.get("drive_pca_address", "0x40")
        ).strip() or "0x40"
        normalized["servo_pca_address"] = str(
            normalized.get("servo_pca_address", "0x42")
        ).strip() or "0x42"
        try:
            normalized["drive_pca_address"] = hex(
                robot._parse_i2c_address(
                    normalized.get("drive_pca_address", "0x40"),
                    robot.MOTOR_PCA_DEFAULT_ADDRESS,
                )
            )
        except Exception:
            normalized["drive_pca_address"] = "0x40"
        try:
            normalized["servo_pca_address"] = hex(
                robot._parse_i2c_address(
                    normalized.get("servo_pca_address", "0x42"),
                    robot.SERVO_PCA_DEFAULT_ADDRESS,
                )
            )
        except Exception:
            normalized["servo_pca_address"] = "0x42"
        try:
            normalized["bucket_servo_channel"] = max(
                0, min(15, int(normalized.get("bucket_servo_channel", 0)))
            )
        except (TypeError, ValueError):
            normalized["bucket_servo_channel"] = 0
        try:
            normalized["servo_down_angle"] = max(
                0, min(180, int(normalized.get("servo_down_angle", 93)))
            )
        except (TypeError, ValueError):
            normalized["servo_down_angle"] = 93
        normalized["camera_stream_host"] = str(
            normalized.get("camera_stream_host", "0.0.0.0")
        ).strip() or "0.0.0.0"
        try:
            normalized["camera_stream_port"] = int(
                normalized.get("camera_stream_port", 5000)
            )
        except (TypeError, ValueError):
            normalized["camera_stream_port"] = 5000
        normalized["camera_stream_port"] = max(
            1,
            min(65535, normalized["camera_stream_port"]),
        )
        normalized["map_choice"] = str(normalized.get("map_choice", "3"))
        normalized["yolo_choice"] = str(normalized.get("yolo_choice", "1"))
        normalized["run_mode"] = str(normalized.get("run_mode", "2"))
        try:
            normalized["auto_speed"] = int(normalized.get("auto_speed", 1500))
        except (TypeError, ValueError):
            normalized["auto_speed"] = 1500
        normalized["auto_speed"] = max(0, min(4095, normalized["auto_speed"]))
        try:
            normalized["manual_speed"] = int(normalized.get("manual_speed", 1400))
        except (TypeError, ValueError):
            normalized["manual_speed"] = 1400
        normalized["manual_speed"] = max(0, min(4095, normalized["manual_speed"]))
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
            config_status = (
                robot.get_config_status()
                if hasattr(robot, "get_config_status")
                else {"error": ""}
            )
            if config_status.get("error"):
                self.message = "Configuration save failed."
                self.error = str(config_status.get("error"))
            else:
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
            detector_camera_source = (
                f"http://127.0.0.1:{int(config.get('camera_stream_port', 5000))}/video_feed"
            )
            self.selected_model_name = model_name
            detector = robot.TrashDetector(
                model_path=model_path,
                camera_index=detector_camera_source,
            )
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
                self._notify_status(
                    f"РћС€РёР±РєР° Р°РІС‚РѕРїРёР»РѕС‚Р°: {exc}",
                    dedupe_key="autopilot_error",
                    dedupe_window_sec=30.0,
                )
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
                bucket_controller_ready = robot.init_bucket_servo_controller(config)
                if bucket_controller_ready:
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
                    self._notify_status(
                        f"Р РѕР±РѕС‚ РїРѕРµС…Р°Р».\nР РµР¶РёРј: Р°РІС‚РѕРїРёР»РѕС‚.\nРЎРєРѕСЂРѕСЃС‚СЊ: {auto_speed}.",
                        dedupe_key="robot_started_autopilot",
                    )
                else:
                    self.mode = "manual"
                    self.message = "Robot started in manual web-control mode."
                    self._notify_status(
                        "Р РѕР±РѕС‚ РіРѕС‚РѕРІ.\nР РµР¶РёРј: СЂСѓС‡РЅРѕРµ СѓРїСЂР°РІР»РµРЅРёРµ С‡РµСЂРµР· РІРµР±-РёРЅС‚РµСЂС„РµР№СЃ.",
                        dedupe_key="robot_started_manual",
                    )

                if not bucket_controller_ready:
                    self.message += (
                        " Bucket servo PCA9685 is unavailable. "
                        "The web UI and camera can still work, but bucket controls are disabled."
                    )

            except Exception as exc:
                self.error = str(exc)
                self.message = "Robot start failed."
                self._notify_status(
                    f"РќРµ СѓРґР°Р»РѕСЃСЊ Р·Р°РїСѓСЃС‚РёС‚СЊ СЂРѕР±РѕС‚Р°.\nРћС€РёР±РєР°: {exc}",
                    dedupe_key="robot_start_failed",
                    dedupe_window_sec=30.0,
                )
                self._shutdown_runtime()

            return self.status()

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            self.message = "Stopping robot..."
            self.error = ""
            self._shutdown_runtime()
            self.message = "Robot stopped."
            self._notify_status("Р РѕР±РѕС‚ РѕСЃС‚Р°РЅРѕРІР»РµРЅ.", dedupe_key="robot_stopped")
            return self.status()

    def prepare_bucket(self) -> Dict[str, Any]:
        with self.lock:
            try:
                robot.move_bucket_wall_to_search_position()
                robot.set_servo_bucket(down=True)
                self.message = "Bucket moved to search position."
                self.error = ""
                self._notify_status(
                    "Р РѕР±РѕС‚ РіРѕС‚РѕРІ.\nРљРѕРІС€ РїРµСЂРµРІРµРґРµРЅ РІ РїРѕРёСЃРєРѕРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ.",
                    dedupe_key="robot_ready",
                )
            except Exception as exc:
                self.error = str(exc)
                self.message = "Failed to prepare bucket."
            return self.status()

    def manual_drive(self, action: str, speed: int) -> Dict[str, Any]:
        with self.lock:
            speed = max(0, min(4095, int(speed)))
            config = self.current_config()
            robot.global_config = dict(config)

            if action == "stop":
                robot.stop_all()
                self.message = "Emergency stop."
                self._notify_status("Р­РєСЃС‚СЂРµРЅРЅР°СЏ РѕСЃС‚Р°РЅРѕРІРєР° СЂРѕР±РѕС‚Р°.", dedupe_key="manual_stop")
                return self.status()

            if self.running and self.mode != "manual":
                self.message = "Switch run mode to manual to use drive buttons."
                return self.status()

            if not robot.init_pca_controllers(config):
                self.error = "Drive PCA9685 is not available."
                self.message = "Manual drive is unavailable."
                return self.status()

            with robot.movement_lock:
                robot.current_speed = speed

            if action == "forward":
                with robot.movement_lock:
                    robot.current_mode = 1
                robot.set_motors(speed, 0, speed, 0)
                self.message = f"Driving forward at {speed}."
                self._notify_status(
                    f"Р РѕР±РѕС‚ РїРѕРµС…Р°Р» РІРїРµСЂРµРґ.\nРЎРєРѕСЂРѕСЃС‚СЊ: {speed}.",
                    dedupe_key="manual_forward",
                )
            elif action == "backward":
                with robot.movement_lock:
                    robot.current_mode = 2
                robot.set_motors(0, speed, 0, speed)
                self.message = f"Driving backward at {speed}."
                self._notify_status(
                    f"Р РѕР±РѕС‚ РїРѕРµС…Р°Р» РЅР°Р·Р°Рґ.\nРЎРєРѕСЂРѕСЃС‚СЊ: {speed}.",
                    dedupe_key="manual_backward",
                )
            elif action == "left":
                with robot.movement_lock:
                    robot.current_mode = 3
                robot.set_motors(speed, 0, 0, speed)
                self.message = f"Turning left at {speed}."
                self._notify_status(
                    f"Р РѕР±РѕС‚ РїРѕРІРµСЂРЅСѓР» РІР»РµРІРѕ.\nРЎРєРѕСЂРѕСЃС‚СЊ: {speed}.",
                    dedupe_key="manual_left",
                )
            elif action == "right":
                with robot.movement_lock:
                    robot.current_mode = 4
                robot.set_motors(0, speed, speed, 0)
                self.message = f"Turning right at {speed}."
                self._notify_status(
                    f"Р РѕР±РѕС‚ РїРѕРІРµСЂРЅСѓР» РІРїСЂР°РІРѕ.\nРЎРєРѕСЂРѕСЃС‚СЊ: {speed}.",
                    dedupe_key="manual_right",
                )
            else:
                self.message = f"Unknown drive action: {action}"

            self.error = ""
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
        config_error_details = ""
        try:
            config = self.current_config()
        except Exception as exc:
            config = self._defaults()
            config_error_details = f"Status config normalization failed: {exc}"
        detector = self.detector
        try:
            config_status = (
                robot.get_config_status()
                if hasattr(robot, "get_config_status")
                else {"path": "", "error": ""}
            )
        except Exception as exc:
            config_status = {"path": "", "error": f"Config status failed: {exc}"}
        try:
            video_status = (
                robot.get_video_streamer_status()
                if hasattr(robot, "get_video_streamer_status")
                else {"active": False, "source": config.get("camera_port", ""), "error": ""}
            )
        except Exception as exc:
            video_status = {
                "active": False,
                "source": config.get("camera_port", ""),
                "error": f"Video status failed: {exc}",
            }
        try:
            i2c_status = (
                robot.get_i2c_scan_status()
                if hasattr(robot, "get_i2c_scan_status")
                else {"error": ""}
            )
        except Exception as exc:
            i2c_status = {"error": f"I2C status failed: {exc}"}
        try:
            route_state = robot.get_route_state(detector)
        except Exception as exc:
            route_state = {
                "source_mode": "none",
                "route_enabled": False,
                "within_corridor": True,
                "distance_m": None,
                "route_fresh": False,
            }
            config_error_details = (
                f"{config_error_details} | Route status failed: {exc}".strip(" |")
                if config_error_details
                else f"Route status failed: {exc}"
            )
        try:
            slam_route_status = robot.get_slam_route_status()
        except Exception as exc:
            slam_route_status = {"recording": False, "points_count": 0, "pose": None}
            config_error_details = (
                f"{config_error_details} | SLAM route status failed: {exc}".strip(" |")
                if config_error_details
                else f"SLAM route status failed: {exc}"
            )
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

        stream_url = "/video_feed_proxy"
        video_stream_active = bool(video_status.get("active"))
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

        try:
            available_i2c_addresses = (
                robot.get_available_i2c_addresses()
                if hasattr(robot, "get_available_i2c_addresses")
                else []
            )
        except Exception as exc:
            available_i2c_addresses = []
            if not i2c_status.get("error"):
                i2c_status["error"] = f"I2C detection failed: {exc}"

        try:
            bucket_servo_controller_connected = robot.is_bucket_servo_controller_ready()
        except Exception as exc:
            bucket_servo_controller_connected = False
            if not i2c_status.get("error"):
                i2c_status["error"] = f"Servo controller status failed: {exc}"

        try:
            available_models = self._available_models()
        except Exception:
            available_models = []

        try:
            available_serial_ports = robot.find_serial_candidates()
        except Exception:
            available_serial_ports = []

        try:
            available_camera_ports = robot.find_camera_candidates()
        except Exception:
            available_camera_ports = []

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
            "bucket_arduino_connected": False,
            "bucket_servo_controller_connected": bucket_servo_controller_connected,
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
            "available_models": available_models,
            "available_serial_ports": available_serial_ports,
            "available_camera_ports": available_camera_ports,
            "available_i2c_addresses": available_i2c_addresses,
            "config_path": config_status.get("path", ""),
            "config_error": " | ".join(
                part
                for part in [
                    config_status.get("error", ""),
                    config_error_details,
                ]
                if part
            ),
            "video_source": video_status.get("source", ""),
            "video_error": video_status.get("error", ""),
            "i2c_error": i2c_status.get("error", ""),
        }


manager = RobotManager()


@app.get("/")
def index():
    return HTML_PAGE


@app.get("/video_feed_proxy")
def video_feed_proxy():
    config = manager.current_config()
    stream_port = int(config.get("camera_stream_port", 5000))
    upstream_url = f"http://127.0.0.1:{stream_port}/video_feed"

    try:
        upstream = urllib.request.urlopen(upstream_url, timeout=5)
    except urllib.error.URLError as exc:
        try:
            if hasattr(robot, "start_video_streamer"):
                robot.start_video_streamer(config)
                time.sleep(1.2)
                upstream = urllib.request.urlopen(upstream_url, timeout=5)
            else:
                raise exc
        except Exception as retry_exc:
            return Response(
                f"Video proxy error: {retry_exc}",
                status=502,
                mimetype="text/plain",
            )

    def generate():
        try:
            while True:
                chunk = upstream.read(8192)
                if not chunk:
                    break
                yield chunk
        finally:
            try:
                upstream.close()
            except Exception:
                pass

    content_type = upstream.headers.get(
        "Content-Type",
        "multipart/x-mixed-replace; boundary=frame",
    )
    return Response(generate(), mimetype=content_type)


@app.get("/api/status")
def api_status():
    try:
        return jsonify(_safe_json_value(manager.status()))
    except Exception as exc:
        fallback = {
            "running": False,
            "mode": "idle",
            "mode_label": "Stopped",
            "message": "Status fallback is active.",
            "error": "",
            "config": dict(DEFAULT_CONFIG),
            "uptime_sec": None,
            "video_stream_active": False,
            "stream_url": "",
            "bucket_arduino_connected": False,
            "bucket_servo_controller_connected": False,
            "trash_summary": "disabled",
            "detector_debug": "",
            "selected_model_name": "",
            "current_mode": 0,
            "current_speed": 0,
            "lidar_running": False,
            "route_source_mode": "none",
            "route_enabled": False,
            "route_within_corridor": True,
            "route_distance_m": None,
            "route_fresh": False,
            "route_debug": "",
            "slam_route_recording": False,
            "slam_route_points": 0,
            "slam_pose": None,
            "available_models": [],
            "available_serial_ports": [],
            "available_camera_ports": [],
            "available_i2c_addresses": [],
            "config_path": "",
            "config_error": f"api_status fallback: {exc}",
            "video_source": "",
            "video_error": "",
            "i2c_error": "",
        }
        return jsonify(_safe_json_value(fallback))


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


