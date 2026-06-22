import argparse
import os
import threading
import time
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


HTML_PAGE = _repair_mojibake_block("""<!doctype html>
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
    .hero-side {
      display: flex;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    .lang-switch {
      min-width: 140px;
      margin: 0;
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
    input:focus, select:focus {
      outline: none;
      border-color: rgba(61,220,151,0.55);
      box-shadow: 0 0 0 3px rgba(61,220,151,0.14);
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
    button:disabled {
      cursor: not-allowed;
      opacity: 0.52;
      filter: saturate(0.7);
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
    .video-shell {
      position: relative;
    }
    .video-empty {
      position: absolute;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 20px;
      text-align: center;
      color: var(--muted);
      font-size: 14px;
      pointer-events: none;
      background: linear-gradient(180deg, rgba(2,7,11,0.1), rgba(2,7,11,0.38));
      border-radius: 18px;
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
    .field-hint,
    .mode-note,
    .manual-hint {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
      margin: -6px 0 12px;
    }
    .panel-header {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 14px;
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 18px;
    }
    .mini-stat {
      padding: 12px;
      border-radius: 14px;
      background: rgba(255,255,255,0.04);
      border: 1px solid rgba(255,255,255,0.04);
    }
    .mini-stat span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 4px;
    }
    .mini-stat strong {
      display: block;
      font-size: 14px;
      line-height: 1.35;
    }
    [data-drive] {
      touch-action: manipulation;
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
      .status-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
    @media (max-width: 620px) {
      .status-grid {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <div>
        <h1 id="heroTitle">Robot Control Station</h1>
        <p id="heroSubtitle">Start the robot, watch the camera stream, and switch between autopilot and manual control from any device in your LAN.</p>
      </div>
      <div class="hero-side">
        <select id="languageSelect" class="lang-switch" onchange="setLanguage(this.value)">
          <option value="ru">Р СѓСЃСЃРєРёР№</option>
          <option value="en">English</option>
        </select>
        <div id="heroStatus" class="status-pill">Loading...</div>
      </div>
    </section>

    <div class="grid">
      <section class="panel">
        <h2 id="launchSettingsTitle">Launch Settings</h2>
        <label for="lidar_port" id="labelLidarPort">LiDAR port</label>
        <input id="lidar_port" />
        <div id="lidarHint" class="field-hint">Detected serial ports will appear here.</div>

        <label for="drive_pca_address" id="labelDrivePcaAddress">Drive PCA I2C address</label>
        <input id="drive_pca_address" />
        <div id="drivePcaHint" class="field-hint">Detected I2C addresses will appear here.</div>

        <label for="servo_pca_address" id="labelServoPcaAddress">Servo PCA I2C address</label>
        <input id="servo_pca_address" />
        <div id="servoPcaHint" class="field-hint">Do not use 0x70 for a PCA9685 board address.</div>

        <label for="bucket_servo_channel" id="labelBucketServoChannel">Bucket servo channel</label>
        <input id="bucket_servo_channel" type="number" min="0" max="15" />

        <label for="camera_port" id="labelCameraPort">Camera port</label>
        <input id="camera_port" />
        <div id="cameraHint" class="field-hint">Detected video devices will appear here.</div>

        <label for="camera_stream_port" id="labelCameraStreamPort">Camera stream port</label>
        <input id="camera_stream_port" type="number" />

        <label for="run_mode" id="labelRunMode">Run mode</label>
        <select id="run_mode">
          <option value="2" id="optionAutopilot">Autopilot</option>
          <option value="1" id="optionManual">Manual</option>
        </select>
        <div id="modeNote" class="mode-note">Autopilot starts the full runtime, manual mode keeps the robot ready for web controls.</div>

        <label for="auto_speed" id="labelAutoSpeed">Autopilot speed</label>
        <input id="auto_speed" type="number" min="0" max="4095" />

        <label for="manual_speed" id="labelManualSpeed">Manual speed</label>
        <input id="manual_speed" type="number" min="0" max="4095" />

        <label for="yolo_choice" id="labelYoloChoice">YOLO source</label>
        <select id="yolo_choice">
          <option value="1" id="optionYoloPhone">Phone / PC detector over Wi-Fi</option>
          <option value="2" id="optionYoloLocal">Local model on Raspberry Pi</option>
          <option value="3" id="optionDisabledA">Disabled</option>
        </select>

        <label for="route_source_mode" id="labelRouteSource">Route source</label>
        <select id="route_source_mode">
          <option value="none" id="optionDisabledB">Disabled</option>
          <option value="slam" id="optionRouteSlam">LiDAR SLAM route</option>
          <option value="gps" id="optionRouteGps">Phone GPS / indoor positioning</option>
        </select>

        <label for="route_corridor_m" id="labelRouteCorridor">Route corridor (meters)</label>
        <input id="route_corridor_m" type="number" min="0.5" max="10" step="0.1" />

        <label for="slam_route_record_step_m" id="labelSlamStep">SLAM point step (meters)</label>
        <input id="slam_route_record_step_m" type="number" min="0.1" max="2" step="0.05" />

        <label for="selected_model_name" id="labelModelName">Local model folder name</label>
        <select id="selected_model_name"></select>
        <div id="modelHint" class="field-hint">The list is loaded from the Raspberry Pi automatically.</div>

        <div class="button-row">
          <button class="secondary" onclick="saveConfig()" id="buttonSaveConfig">Save config</button>
          <button onclick="startRobot()" id="buttonStartRobot">Start robot</button>
        </div>
        <div class="button-row">
          <button class="warn" onclick="prepareBucket()" id="buttonPrepareBucket">Prepare bucket</button>
          <button class="danger" onclick="stopRobot()" id="buttonStopRobot">Stop robot</button>
        </div>

        <div class="stat-list">
          <div class="stat-item"><span id="labelStatRunning">Robot state</span><strong id="statRunning">--</strong></div>
          <div class="stat-item"><span id="labelStatMode">Mode</span><strong id="statMode">--</strong></div>
          <div class="stat-item"><span id="labelStatVideo">Camera stream</span><strong id="statVideo">--</strong></div>
          <div class="stat-item"><span id="labelStatTrash">Trash detector</span><strong id="statTrash">--</strong></div>
          <div class="stat-item"><span id="labelStatServoController">Bucket servo controller</span><strong id="statServoController">--</strong></div>
          <div class="stat-item"><span id="labelStatModel">Active model</span><strong id="statModel">--</strong></div>
          <div class="stat-item"><span id="labelStatRouteSource">Route source</span><strong id="statRouteSource">--</strong></div>
          <div class="stat-item"><span id="labelStatRoute">Route status</span><strong id="statRoute">--</strong></div>
        </div>
      </section>

      <section class="panel">
        <div class="status-grid">
          <div class="mini-stat">
            <span id="labelStatLidar">LiDAR</span>
            <strong id="statLidar">--</strong>
          </div>
          <div class="mini-stat">
            <span id="labelStatUptime">Uptime</span>
            <strong id="statUptime">--</strong>
          </div>
          <div class="mini-stat">
            <span id="labelStatSpeed">Speed</span>
            <strong id="statSpeed">--</strong>
          </div>
          <div class="mini-stat">
            <span id="labelStatPose">SLAM pose</span>
            <strong id="statPose">--</strong>
          </div>
        </div>
        <div class="split">
          <div>
            <div class="panel-header">
              <div>
                <h2 id="liveVideoTitle">Live Video</h2>
              </div>
            </div>
            <div class="video-shell">
              <img id="videoFrame" class="video" alt="video stream" />
              <div id="videoOverlay" class="video-empty">Waiting for live stream...</div>
            </div>
          </div>
          <div>
            <h2 id="manualDriveTitle">Manual Drive</h2>
            <div id="manualHint" class="manual-hint">Hold a direction button or use WASD / arrow keys. Release to stop.</div>
            <div class="manual-grid">
              <button class="blank">.</button>
              <button class="dark" type="button" data-drive="forward" id="buttonForward">Forward</button>
              <button class="blank">.</button>
              <button class="dark" type="button" data-drive="left" id="buttonLeft">Left</button>
              <button class="danger" type="button" data-drive="stop" id="buttonStop">STOP</button>
              <button class="dark" type="button" data-drive="right" id="buttonRight">Right</button>
              <button class="blank">.</button>
              <button class="dark" type="button" data-drive="backward" id="buttonBackward">Backward</button>
              <button class="blank">.</button>
            </div>

            <h3 style="margin-top:18px;" id="bucketControlTitle">Bucket Control</h3>
            <div class="button-row">
              <button class="dark" onclick="bucket('wall_down')" id="buttonWallDown">Wall down</button>
              <button class="dark" onclick="bucket('wall_up')" id="buttonWallUp">Wall up</button>
            </div>
            <div class="button-row">
              <button class="dark" onclick="bucket('scoop_down')">Scoop 90В°</button>
              <button class="dark" onclick="bucket('scoop_up')">Scoop 0В°</button>
            </div>
            <div class="button-row">
              <button class="dark" onclick="bucket('bucket_test')" id="buttonTimedTest">Timed test</button>
              <button onclick="bucket('collect')" id="buttonCollect">Collect cycle</button>
            </div>

            <h3 style="margin-top:18px;" id="slamRouteTitle">SLAM Route</h3>
            <div class="button-row">
              <button class="dark" onclick="slamRoute('start_record')" id="buttonRecordRoute">Record route</button>
              <button class="dark" onclick="slamRoute('stop_record')" id="buttonStopRecord">Stop record</button>
            </div>
            <div class="button-row">
              <button class="warn" onclick="slamRoute('clear')" id="buttonClearRoute">Clear route</button>
              <button class="secondary" onclick="refreshStatus()" id="buttonRefresh">Refresh</button>
            </div>
          </div>
        </div>

        <div class="full" style="margin-top:18px;">
          <h3 id="systemLogTitle">System Log</h3>
          <div id="logBox" class="log">Waiting for status...</div>
        </div>
      </section>
    </div>
  </div>

  <script>
    var lastStatus = null;
    var currentLanguage = 'ru';
    var manualDriveEnabled = false;
    var activeDriveAction = null;
    var activeDriveKey = null;
    var touchedFields = {};
    var configFieldIds = [
      'lidar_port',
      'drive_pca_address',
      'servo_pca_address',
      'bucket_servo_channel',
      'camera_port',
      'camera_stream_port',
      'run_mode',
      'auto_speed',
      'manual_speed',
      'yolo_choice',
      'route_source_mode',
      'route_corridor_m',
      'slam_route_record_step_m',
      'selected_model_name'
    ];
    var defaultConfig = {
      lidar_port: '/dev/ttyUSB0',
      drive_pca_address: '0x40',
      servo_pca_address: '0x42',
      bucket_servo_channel: 0,
      camera_port: '/dev/video0',
      camera_stream_port: 5000,
      run_mode: '2',
      auto_speed: 1500,
      manual_speed: 1400,
      yolo_choice: '1',
      route_source_mode: 'none',
      route_corridor_m: 3.0,
      slam_route_record_step_m: 0.35,
      selected_model_name: ''
    };
    var translations = {
      ru: {
        heroTitle: 'Станция управления роботом',
        heroSubtitle: 'Запуск, видеопоток, ручное управление и статусы в одном окне.',
        launchSettingsTitle: 'Параметры запуска',
        labelLidarPort: 'Порт лидара',
        labelDrivePcaAddress: 'I2C адрес PCA моторов',
        labelServoPcaAddress: 'I2C адрес PCA сервы',
        labelBucketServoChannel: 'Канал сервы ковша',
        labelCameraPort: 'Порт камеры',
        labelCameraStreamPort: 'Порт видеопотока',
        labelRunMode: 'Режим',
        optionAutopilot: 'Автопилот',
        optionManual: 'Ручной',
        labelAutoSpeed: 'Скорость автопилота',
        labelManualSpeed: 'Скорость ручного режима',
        labelYoloChoice: 'Источник YOLO',
        optionYoloPhone: 'Телефон / ПК по Wi-Fi',
        optionYoloLocal: 'Локальная модель на Raspberry Pi',
        optionDisabledA: 'Отключено',
        labelRouteSource: 'Источник маршрута',
        optionDisabledB: 'Отключено',
        optionRouteSlam: 'Маршрут LiDAR SLAM',
        optionRouteGps: 'Маршрут с телефона / indoor positioning',
        labelRouteCorridor: 'Коридор маршрута (м)',
        labelSlamStep: 'Шаг точек SLAM (м)',
        labelModelName: 'Локальная модель',
        buttonSaveConfig: 'Сохранить конфиг',
        buttonStartRobot: 'Запустить',
        buttonPrepareBucket: 'Подготовить ковш',
        buttonStopRobot: 'Остановить',
        labelStatRunning: 'Состояние',
        labelStatMode: 'Режим',
        labelStatVideo: 'Видеопоток',
        labelStatTrash: 'Детектор',
        labelStatServoController: 'PCA сервы ковша',
        labelStatModel: 'Модель',
        labelStatRouteSource: 'Источник маршрута',
        labelStatRoute: 'Маршрут',
        labelStatLidar: 'LiDAR',
        labelStatUptime: 'Время работы',
        labelStatSpeed: 'Скорость',
        labelStatPose: 'Поза SLAM',
        liveVideoTitle: 'Живое видео',
        manualDriveTitle: 'Ручное движение',
        buttonForward: 'Вперед',
        buttonLeft: 'Влево',
        buttonStop: 'СТОП',
        buttonRight: 'Вправо',
        buttonBackward: 'Назад',
        bucketControlTitle: 'Ковш',
        buttonWallDown: 'Стенка вниз',
        buttonWallUp: 'Стенка вверх',
        scoopDown: 'Совок 90°',
        scoopUp: 'Совок 0°',
        buttonTimedTest: 'Тест по времени',
        buttonCollect: 'Цикл сбора',
        slamRouteTitle: 'SLAM маршрут',
        buttonRecordRoute: 'Запись маршрута',
        buttonStopRecord: 'Стоп записи',
        buttonClearRoute: 'Очистить маршрут',
        buttonRefresh: 'Обновить',
        systemLogTitle: 'Журнал',
        heroStopped: 'Остановлен',
        heroRunning: 'Работает',
        online: 'онлайн',
        offline: 'офлайн',
        connected: 'подключено',
        disconnected: 'отключено',
        stopped: 'остановлен',
        loading: 'Загрузка...',
        noMessages: 'Сообщений пока нет.',
        cameraWaiting: 'Ожидание видеопотока...',
        manualLocked: 'Ручное управление станет доступно после запуска в manual mode.',
        manualReady: 'Удерживай кнопку или используй WASD / стрелки.',
        modeAutopilotNote: 'Автопилот запускает LiDAR, камеру и детектор.',
        modeManualNote: 'Ручной режим включает веб-кнопки и WASD / стрелки.',
        detectedSerial: 'Найденные serial:',
        detectedVideo: 'Найденные video:',
        detectedI2C: 'Найденные I2C:',
        drivePcaHint: 'Найденные I2C адреса появятся здесь.',
        servoPcaHint: 'Не используй 0x70 как адрес отдельной платы PCA9685.',
        lidarHint: 'Найденные serial-порты появятся здесь.',
        cameraHint: 'Найденные video-устройства появятся здесь.',
        modelHint: 'Список моделей читается из папки models автоматически.',
        autoModel: 'Автовыбор модели',
        noModels: 'Локальные модели не найдены',
        routeDisabled: 'отключен',
        phoneRouteActive: 'маршрут с телефона активен',
        waitingPhoneRoute: 'ожидание маршрута с телефона',
        routePoints: 'точек',
        errorPrefix: 'Ошибка',
        streamPrefix: 'Поток',
        cameraPrefix: 'Камера',
        videoPrefix: 'Видео',
        i2cPrefix: 'I2C',
        configPrefix: 'Конфиг',
        routePrefix: 'Маршрут'
      },
      en: {
        heroTitle: 'Robot Control Station',
        heroSubtitle: 'Start, stream, drive, and watch status in one place.',
        launchSettingsTitle: 'Launch Settings',
        labelLidarPort: 'LiDAR port',
        labelDrivePcaAddress: 'Drive PCA I2C address',
        labelServoPcaAddress: 'Servo PCA I2C address',
        labelBucketServoChannel: 'Bucket servo channel',
        labelCameraPort: 'Camera port',
        labelCameraStreamPort: 'Camera stream port',
        labelRunMode: 'Run mode',
        optionAutopilot: 'Autopilot',
        optionManual: 'Manual',
        labelAutoSpeed: 'Autopilot speed',
        labelManualSpeed: 'Manual speed',
        labelYoloChoice: 'YOLO source',
        optionYoloPhone: 'Phone / PC over Wi-Fi',
        optionYoloLocal: 'Local model on Raspberry Pi',
        optionDisabledA: 'Disabled',
        labelRouteSource: 'Route source',
        optionDisabledB: 'Disabled',
        optionRouteSlam: 'LiDAR SLAM route',
        optionRouteGps: 'Phone / indoor positioning',
        labelRouteCorridor: 'Route corridor (m)',
        labelSlamStep: 'SLAM point step (m)',
        labelModelName: 'Local model',
        buttonSaveConfig: 'Save config',
        buttonStartRobot: 'Start',
        buttonPrepareBucket: 'Prepare bucket',
        buttonStopRobot: 'Stop',
        labelStatRunning: 'State',
        labelStatMode: 'Mode',
        labelStatVideo: 'Camera stream',
        labelStatTrash: 'Detector',
        labelStatServoController: 'Bucket servo PCA',
        labelStatModel: 'Model',
        labelStatRouteSource: 'Route source',
        labelStatRoute: 'Route',
        labelStatLidar: 'LiDAR',
        labelStatUptime: 'Uptime',
        labelStatSpeed: 'Speed',
        labelStatPose: 'SLAM pose',
        liveVideoTitle: 'Live Video',
        manualDriveTitle: 'Manual Drive',
        buttonForward: 'Forward',
        buttonLeft: 'Left',
        buttonStop: 'STOP',
        buttonRight: 'Right',
        buttonBackward: 'Backward',
        bucketControlTitle: 'Bucket',
        buttonWallDown: 'Wall down',
        buttonWallUp: 'Wall up',
        scoopDown: 'Scoop 90°',
        scoopUp: 'Scoop 0°',
        buttonTimedTest: 'Timed test',
        buttonCollect: 'Collect cycle',
        slamRouteTitle: 'SLAM Route',
        buttonRecordRoute: 'Record route',
        buttonStopRecord: 'Stop record',
        buttonClearRoute: 'Clear route',
        buttonRefresh: 'Refresh',
        systemLogTitle: 'System Log',
        heroStopped: 'Stopped',
        heroRunning: 'Running',
        online: 'online',
        offline: 'offline',
        connected: 'connected',
        disconnected: 'disconnected',
        stopped: 'stopped',
        loading: 'Loading...',
        noMessages: 'No messages yet.',
        cameraWaiting: 'Waiting for live stream...',
        manualLocked: 'Manual drive becomes available after starting manual mode.',
        manualReady: 'Hold a button or use WASD / arrow keys.',
        modeAutopilotNote: 'Autopilot starts LiDAR, camera, and detector.',
        modeManualNote: 'Manual mode enables web buttons and WASD / arrows.',
        detectedSerial: 'Detected serial:',
        detectedVideo: 'Detected video:',
        detectedI2C: 'Detected I2C:',
        drivePcaHint: 'Detected I2C addresses will appear here.',
        servoPcaHint: 'Do not use 0x70 as a standalone PCA9685 board address.',
        lidarHint: 'Detected serial ports will appear here.',
        cameraHint: 'Detected video devices will appear here.',
        modelHint: 'Model list is read from the models folder automatically.',
        autoModel: 'Auto-pick model',
        noModels: 'No local models found',
        routeDisabled: 'disabled',
        phoneRouteActive: 'phone route active',
        waitingPhoneRoute: 'waiting for phone route',
        routePoints: 'pts',
        errorPrefix: 'Error',
        streamPrefix: 'Stream',
        cameraPrefix: 'Camera',
        videoPrefix: 'Video',
        i2cPrefix: 'I2C',
        configPrefix: 'Config',
        routePrefix: 'Route'
      }
    };

    function byId(id) { return document.getElementById(id); }
    function t(key) {
      var table = translations[currentLanguage] || translations.ru;
      if (table[key]) return table[key];
      return key;
    }

    function setLog(text) {
      var logBox = byId('logBox');
      if (logBox) logBox.textContent = text || t('noMessages');
    }

    function setFetchError(error) {
      var details = 'request failed';
      var hero = byId('heroStatus');
      if (error) {
        if (typeof error === 'string') details = error;
        else if (error.message) details = error.message;
      }
      if (hero) {
        hero.textContent = t('errorPrefix') + ': ' + details;
        hero.style.background = 'rgba(255,107,107,0.15)';
      }
      setLog(t('errorPrefix') + ': ' + details);
    }

    function setUiBootError(error) {
      var hero = byId('heroStatus');
      var details = error || 'unknown ui error';
      if (hero) {
        hero.textContent = 'JS error: ' + details;
        hero.style.background = 'rgba(255,107,107,0.15)';
      }
      setLog('JS error: ' + details);
    }

    function setLanguage(lang) {
      var select = byId('languageSelect');
      currentLanguage = translations[lang] ? lang : 'ru';
      try { localStorage.setItem('robot_ui_language', currentLanguage); } catch (e) {}
      document.documentElement.lang = currentLanguage;
      if (select) select.value = currentLanguage;
      applyTranslations();
      if (lastStatus) updateStatusUi(lastStatus, false);
      else {
        var hero = byId('heroStatus');
        if (hero) hero.textContent = t('loading');
        setLog('');
      }
    }

    function restoreLanguage() {
      var saved = 'ru';
      try { saved = localStorage.getItem('robot_ui_language') || 'ru'; } catch (e) {}
      setLanguage(saved);
    }

    function applyTranslations() {
      var textMap = {
        heroTitle: 'heroTitle',
        heroSubtitle: 'heroSubtitle',
        launchSettingsTitle: 'launchSettingsTitle',
        labelLidarPort: 'labelLidarPort',
        labelDrivePcaAddress: 'labelDrivePcaAddress',
        labelServoPcaAddress: 'labelServoPcaAddress',
        labelBucketServoChannel: 'labelBucketServoChannel',
        labelCameraPort: 'labelCameraPort',
        labelCameraStreamPort: 'labelCameraStreamPort',
        labelRunMode: 'labelRunMode',
        optionAutopilot: 'optionAutopilot',
        optionManual: 'optionManual',
        labelAutoSpeed: 'labelAutoSpeed',
        labelManualSpeed: 'labelManualSpeed',
        labelYoloChoice: 'labelYoloChoice',
        optionYoloPhone: 'optionYoloPhone',
        optionYoloLocal: 'optionYoloLocal',
        optionDisabledA: 'optionDisabledA',
        labelRouteSource: 'labelRouteSource',
        optionDisabledB: 'optionDisabledB',
        optionRouteSlam: 'optionRouteSlam',
        optionRouteGps: 'optionRouteGps',
        labelRouteCorridor: 'labelRouteCorridor',
        labelSlamStep: 'labelSlamStep',
        labelModelName: 'labelModelName',
        buttonSaveConfig: 'buttonSaveConfig',
        buttonStartRobot: 'buttonStartRobot',
        buttonPrepareBucket: 'buttonPrepareBucket',
        buttonStopRobot: 'buttonStopRobot',
        labelStatRunning: 'labelStatRunning',
        labelStatMode: 'labelStatMode',
        labelStatVideo: 'labelStatVideo',
        labelStatTrash: 'labelStatTrash',
        labelStatServoController: 'labelStatServoController',
        labelStatModel: 'labelStatModel',
        labelStatRouteSource: 'labelStatRouteSource',
        labelStatRoute: 'labelStatRoute',
        labelStatLidar: 'labelStatLidar',
        labelStatUptime: 'labelStatUptime',
        labelStatSpeed: 'labelStatSpeed',
        labelStatPose: 'labelStatPose',
        liveVideoTitle: 'liveVideoTitle',
        manualDriveTitle: 'manualDriveTitle',
        buttonForward: 'buttonForward',
        buttonLeft: 'buttonLeft',
        buttonStop: 'buttonStop',
        buttonRight: 'buttonRight',
        buttonBackward: 'buttonBackward',
        bucketControlTitle: 'bucketControlTitle',
        buttonWallDown: 'buttonWallDown',
        buttonWallUp: 'buttonWallUp',
        buttonTimedTest: 'buttonTimedTest',
        buttonCollect: 'buttonCollect',
        slamRouteTitle: 'slamRouteTitle',
        buttonRecordRoute: 'buttonRecordRoute',
        buttonStopRecord: 'buttonStopRecord',
        buttonClearRoute: 'buttonClearRoute',
        buttonRefresh: 'buttonRefresh',
        systemLogTitle: 'systemLogTitle'
      };
      var id;
      for (id in textMap) {
        if (Object.prototype.hasOwnProperty.call(textMap, id) && byId(id)) {
          byId(id).textContent = t(textMap[id]);
        }
      }
      if (byId('lidarHint')) byId('lidarHint').textContent = t('lidarHint');
      if (byId('cameraHint')) byId('cameraHint').textContent = t('cameraHint');
      if (byId('drivePcaHint')) byId('drivePcaHint').textContent = t('drivePcaHint');
      if (byId('servoPcaHint')) byId('servoPcaHint').textContent = t('servoPcaHint');
      if (byId('modelHint')) byId('modelHint').textContent = t('modelHint');
      if (byId('videoOverlay')) byId('videoOverlay').textContent = t('cameraWaiting');
      updateModeNote();
      updateModelSelectEnabled();
      updateBucketLabels();
    }

    function updateBucketLabels() {
      var buttons = document.querySelectorAll('button');
      var i;
      for (i = 0; i < buttons.length; i += 1) {
        if (buttons[i].getAttribute('onclick') === "bucket('scoop_down')") buttons[i].textContent = t('scoopDown');
        if (buttons[i].getAttribute('onclick') === "bucket('scoop_up')") buttons[i].textContent = t('scoopUp');
      }
    }

    function updateModeNote() {
      var modeNote = byId('modeNote');
      var runMode = byId('run_mode');
      if (!modeNote || !runMode) return;
      modeNote.textContent = runMode.value === '1' ? t('modeManualNote') : t('modeAutopilotNote');
    }

    function updateModelSelectEnabled() {
      var source = byId('yolo_choice');
      var modelSelect = byId('selected_model_name');
      if (source && modelSelect) modelSelect.disabled = source.value !== '2';
    }

    function markFieldTouched(event) {
      if (event && event.target && event.target.id) touchedFields[event.target.id] = true;
      updateModeNote();
      updateModelSelectEnabled();
    }

    function clearTouchedFields() { touchedFields = {}; }

    function bindConfigInputs() {
      var i;
      for (i = 0; i < configFieldIds.length; i += 1) {
        var element = byId(configFieldIds[i]);
        if (!element) continue;
        element.addEventListener('input', markFieldTouched);
        element.addEventListener('change', markFieldTouched);
      }
      updateModeNote();
      updateModelSelectEnabled();
    }

    function setFieldValue(id, value, force) {
      var element = byId(id);
      if (!element) return;
      if (!force && touchedFields[id]) return;
      if (value === null || typeof value === 'undefined') value = '';
      element.value = value;
    }

    function updateModelOptions(models, selectedValue, force) {
      var select = byId('selected_model_name');
      var i;
      if (!select) return;
      if (!force && touchedFields.selected_model_name) return;
      select.innerHTML = '';
      var autoOption = document.createElement('option');
      autoOption.value = '';
      autoOption.textContent = models && models.length ? t('autoModel') : t('noModels');
      select.appendChild(autoOption);
      if (models && models.length) {
        for (i = 0; i < models.length; i += 1) {
          var option = document.createElement('option');
          option.value = String(models[i]);
          option.textContent = String(models[i]);
          select.appendChild(option);
        }
      }
      select.value = selectedValue || '';
      updateModelSelectEnabled();
    }

    function formPayload() {
      return {
        lidar_port: byId('lidar_port').value.trim(),
        drive_pca_address: byId('drive_pca_address').value.trim(),
        servo_pca_address: byId('servo_pca_address').value.trim(),
        bucket_servo_channel: parseInt(byId('bucket_servo_channel').value || '0', 10),
        camera_port: byId('camera_port').value.trim(),
        camera_stream_port: parseInt(byId('camera_stream_port').value || '5000', 10),
        run_mode: byId('run_mode').value,
        auto_speed: parseInt(byId('auto_speed').value || '1500', 10),
        manual_speed: parseInt(byId('manual_speed').value || '1400', 10),
        yolo_choice: byId('yolo_choice').value,
        route_source_mode: byId('route_source_mode').value,
        route_corridor_m: parseFloat(byId('route_corridor_m').value || '3.0'),
        slam_route_record_step_m: parseFloat(byId('slam_route_record_step_m').value || '0.35'),
        selected_model_name: byId('selected_model_name').value.trim()
      };
    }

    function applyConfig(config, force) {
      config = config || {};
      setFieldValue('lidar_port', config.lidar_port || defaultConfig.lidar_port, force);
      setFieldValue('drive_pca_address', config.drive_pca_address || defaultConfig.drive_pca_address, force);
      setFieldValue('servo_pca_address', config.servo_pca_address || defaultConfig.servo_pca_address, force);
      setFieldValue('bucket_servo_channel', typeof config.bucket_servo_channel !== 'undefined' ? config.bucket_servo_channel : defaultConfig.bucket_servo_channel, force);
      setFieldValue('camera_port', config.camera_port || defaultConfig.camera_port, force);
      setFieldValue('camera_stream_port', typeof config.camera_stream_port !== 'undefined' ? config.camera_stream_port : defaultConfig.camera_stream_port, force);
      setFieldValue('run_mode', config.run_mode || defaultConfig.run_mode, force);
      setFieldValue('auto_speed', typeof config.auto_speed !== 'undefined' ? config.auto_speed : defaultConfig.auto_speed, force);
      setFieldValue('manual_speed', typeof config.manual_speed !== 'undefined' ? config.manual_speed : defaultConfig.manual_speed, force);
      setFieldValue('yolo_choice', config.yolo_choice || defaultConfig.yolo_choice, force);
      setFieldValue('route_source_mode', config.route_source_mode || defaultConfig.route_source_mode, force);
      setFieldValue('route_corridor_m', typeof config.route_corridor_m !== 'undefined' ? config.route_corridor_m : defaultConfig.route_corridor_m, force);
      setFieldValue('slam_route_record_step_m', typeof config.slam_route_record_step_m !== 'undefined' ? config.slam_route_record_step_m : defaultConfig.slam_route_record_step_m, force);
      updateModelOptions(config.available_models || [], config.selected_model_name || '', force);
      updateModeNote();
      updateModelSelectEnabled();
    }

    function formatUptime(seconds) {
      if (typeof seconds !== 'number') return '--';
      var hrs = Math.floor(seconds / 3600);
      var mins = Math.floor((seconds % 3600) / 60);
      var secs = Math.floor(seconds % 60);
      if (hrs > 0) return hrs + 'h ' + mins + 'm ' + secs + 's';
      if (mins > 0) return mins + 'm ' + secs + 's';
      return secs + 's';
    }

    function formatPose(pose) {
      if (!pose || typeof pose !== 'object') return '--';
      var x = typeof pose.x === 'number' ? pose.x.toFixed(2) : '--';
      var y = typeof pose.y === 'number' ? pose.y.toFixed(2) : '--';
      var theta = typeof pose.theta === 'number' ? pose.theta.toFixed(2) : '--';
      return x + ', ' + y + ', ' + theta;
    }

    function updateDeviceHints(status) {
      var serialText = t('lidarHint');
      var cameraText = t('cameraHint');
      var i2cText = t('drivePcaHint');
      if (status.available_serial_ports && status.available_serial_ports.length) serialText = t('detectedSerial') + ' ' + status.available_serial_ports.join(', ');
      if (status.available_camera_ports && status.available_camera_ports.length) cameraText = t('detectedVideo') + ' ' + status.available_camera_ports.join(', ');
      if (status.available_i2c_addresses && status.available_i2c_addresses.length) i2cText = t('detectedI2C') + ' ' + status.available_i2c_addresses.join(', ');
      if (byId('lidarHint')) byId('lidarHint').textContent = serialText;
      if (byId('cameraHint')) byId('cameraHint').textContent = cameraText;
      if (byId('drivePcaHint')) byId('drivePcaHint').textContent = i2cText;
      if (byId('servoPcaHint')) byId('servoPcaHint').textContent = i2cText + '. ' + t('servoPcaHint');
    }

    function updateDriveControls(status) {
      var buttons = document.querySelectorAll('[data-drive]');
      var i;
      manualDriveEnabled = !!(status.running && status.mode === 'manual');
      for (i = 0; i < buttons.length; i += 1) {
        var action = buttons[i].getAttribute('data-drive');
        buttons[i].disabled = !manualDriveEnabled && action !== 'stop';
      }
      if (byId('manualHint')) byId('manualHint').textContent = manualDriveEnabled ? t('manualReady') : t('manualLocked');
    }

    function updateStatusUi(status, forceConfig) {
      var configForUi = {};
      var key;
      var routeSummary = t('routeDisabled');
      var modelSummary = t('routeDisabled');
      var logLines = [];
      var hero = byId('heroStatus');
      var video = byId('videoFrame');
      var overlay = byId('videoOverlay');
      status = status || {};
      lastStatus = status;
      for (key in defaultConfig) {
        if (Object.prototype.hasOwnProperty.call(defaultConfig, key)) configForUi[key] = defaultConfig[key];
      }
      if (status.config) {
        for (key in status.config) {
          if (Object.prototype.hasOwnProperty.call(status.config, key)) configForUi[key] = status.config[key];
        }
      }
      configForUi.available_models = status.available_models || [];
      configForUi.selected_model_name = status.selected_model_name || configForUi.selected_model_name || '';
      applyConfig(configForUi, !!forceConfig);

      if (status.route_source_mode === 'slam') routeSummary = String(status.slam_route_points || 0) + ' ' + t('routePoints');
      else if (status.route_source_mode === 'gps') routeSummary = status.route_fresh ? t('phoneRouteActive') : t('waitingPhoneRoute');

      if (status.selected_model_name) modelSummary = status.selected_model_name;
      else if (configForUi.yolo_choice === '2') modelSummary = (status.available_models && status.available_models.length) ? t('autoModel') : t('noModels');
      else if (configForUi.yolo_choice === '1') modelSummary = 'phone / none';

      if (hero) {
        hero.textContent = status.running ? (t('heroRunning') + ': ' + (status.mode || '--')) : t('heroStopped');
        hero.style.background = status.running ? 'rgba(61,220,151,0.18)' : 'rgba(255,107,107,0.15)';
      }
      if (byId('statRunning')) byId('statRunning').textContent = status.running ? t('online') : t('offline');
      if (byId('statMode')) byId('statMode').textContent = status.mode || '--';
      if (byId('statVideo')) byId('statVideo').textContent = status.video_stream_active ? status.stream_url : t('stopped');
      if (byId('statTrash')) byId('statTrash').textContent = status.trash_summary || t('routeDisabled');
      if (byId('statServoController')) byId('statServoController').textContent = status.bucket_servo_controller_connected ? t('connected') : t('disconnected');
      if (byId('statModel')) byId('statModel').textContent = modelSummary;
      if (byId('statRouteSource')) byId('statRouteSource').textContent = status.route_source_mode || t('routeDisabled');
      if (byId('statRoute')) byId('statRoute').textContent = routeSummary;
      if (byId('statLidar')) byId('statLidar').textContent = status.lidar_running ? t('connected') : t('disconnected');
      if (byId('statUptime')) byId('statUptime').textContent = formatUptime(status.uptime_sec);
      if (byId('statSpeed')) byId('statSpeed').textContent = typeof status.current_speed === 'number' ? String(status.current_speed) : '--';
      if (byId('statPose')) byId('statPose').textContent = formatPose(status.slam_pose);

      if (status.message) logLines.push(status.message);
      if (status.error) logLines.push(t('errorPrefix') + ': ' + status.error);
      if (status.stream_url) logLines.push(t('streamPrefix') + ': ' + status.stream_url);
      if (status.video_source) logLines.push(t('cameraPrefix') + ': ' + status.video_source);
      if (status.video_error) logLines.push(t('videoPrefix') + ': ' + status.video_error);
      if (status.i2c_error) logLines.push(t('i2cPrefix') + ': ' + status.i2c_error);
      if (status.config_error) logLines.push(t('configPrefix') + ': ' + status.config_error);
      if (status.route_debug) logLines.push(t('routePrefix') + ': ' + status.route_debug);
      setLog(logLines.join('\n'));

      if (video && overlay) {
        if (status.stream_url) {
          if (video.getAttribute('data-current-src') !== status.stream_url) {
            video.src = status.stream_url + '?t=' + Date.now();
            video.setAttribute('data-current-src', status.stream_url);
          }
          overlay.style.display = 'none';
        } else {
          video.removeAttribute('src');
          video.setAttribute('data-current-src', '');
          overlay.style.display = 'flex';
          overlay.textContent = t('cameraWaiting');
        }
      }

      updateDeviceHints(status);
      updateDriveControls(status);
    }

    function requestJson(method, url, payload, onSuccess, onError) {
      var xhr = new XMLHttpRequest();
      xhr.open(method, url, true);
      xhr.setRequestHeader('Content-Type', 'application/json');
      xhr.timeout = 3000;
      xhr.onreadystatechange = function () {
        var data;
        if (xhr.readyState !== 4) return;
        if (xhr.status < 200 || xhr.status >= 300) {
          if (onError) onError(new Error('HTTP ' + xhr.status));
          return;
        }
        try { data = xhr.responseText ? JSON.parse(xhr.responseText) : {}; }
        catch (e) {
          if (onError) onError(e);
          return;
        }
        if (onSuccess) onSuccess(data);
      };
      xhr.ontimeout = function () { if (onError) onError(new Error('request timeout')); };
      xhr.onerror = function () { if (onError) onError(new Error('network error')); };
      xhr.send(payload ? JSON.stringify(payload) : null);
    }

    function refreshStatus() {
      requestJson('GET', '/api/status', null, function (data) { updateStatusUi(data, false); }, setFetchError);
    }

    function api(path, payload) {
      requestJson('POST', path, payload, function (data) {
        clearTouchedFields();
        updateStatusUi(data, true);
      }, setFetchError);
    }

    function saveConfig() { api('/api/config', formPayload()); }
    function startRobot() { api('/api/start', formPayload()); }
    function stopRobot() { api('/api/stop', {}); }
    function prepareBucket() { api('/api/bucket', { action: 'prepare' }); }
    function drive(action) { api('/api/drive', { action: action, speed: parseInt(byId('manual_speed').value || '1400', 10) }); }
    function bucket(action) { api('/api/bucket', { action: action }); }
    function slamRoute(action) { api('/api/slam_route', { action: action }); }

    function engageDrive(action, keyName) {
      if (!manualDriveEnabled && action !== 'stop') return;
      if (activeDriveAction === action && activeDriveKey === keyName) return;
      activeDriveAction = action;
      activeDriveKey = keyName || '';
      drive(action);
    }

    function releaseDrive(keyName) {
      if (keyName && activeDriveKey && keyName !== activeDriveKey) return;
      if (!activeDriveAction) return;
      activeDriveAction = null;
      activeDriveKey = '';
      drive('stop');
    }

    function keyToAction(keyCode) {
      if (keyCode === 38 || keyCode === 87) return 'forward';
      if (keyCode === 40 || keyCode === 83) return 'backward';
      if (keyCode === 37 || keyCode === 65) return 'left';
      if (keyCode === 39 || keyCode === 68) return 'right';
      if (keyCode === 32) return 'stop';
      return '';
    }

    function bindDriveControls() {
      var buttons = document.querySelectorAll('[data-drive]');
      var i;
      for (i = 0; i < buttons.length; i += 1) {
        (function (button) {
          var action = button.getAttribute('data-drive');
          if (action === 'stop') {
            button.addEventListener('click', function () {
              activeDriveAction = null;
              activeDriveKey = '';
              drive('stop');
            });
            return;
          }
          button.addEventListener('mousedown', function (event) { event.preventDefault(); engageDrive(action, ''); });
          button.addEventListener('mouseup', function () { releaseDrive(''); });
          button.addEventListener('mouseleave', function () { releaseDrive(''); });
          button.addEventListener('touchstart', function (event) { event.preventDefault(); engageDrive(action, ''); });
          button.addEventListener('touchend', function () { releaseDrive(''); });
          button.addEventListener('touchcancel', function () { releaseDrive(''); });
        }(buttons[i]));
      }
      document.addEventListener('keydown', function (event) {
        var tag;
        var action = keyToAction(event.keyCode);
        if (!action || event.repeat) return;
        tag = document.activeElement ? document.activeElement.tagName : '';
        if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
        event.preventDefault();
        if (action === 'stop') { releaseDrive('key-' + event.keyCode); return; }
        engageDrive(action, 'key-' + event.keyCode);
      });
      document.addEventListener('keyup', function (event) {
        if (!keyToAction(event.keyCode)) return;
        event.preventDefault();
        releaseDrive('key-' + event.keyCode);
      });
      window.addEventListener('blur', function () { releaseDrive(''); });
      document.addEventListener('visibilitychange', function () { if (document.hidden) releaseDrive(''); });
    }

    window.onerror = function (message, source, lineno, colno) {
      setUiBootError(String(message || 'unknown') + ' @' + String(lineno || 0) + ':' + String(colno || 0));
      return false;
    };

    try {
      restoreLanguage();
      bindConfigInputs();
      bindDriveControls();
      applyConfig(defaultConfig, true);
      applyTranslations();
      refreshStatus();
      setInterval(refreshStatus, 1200);
    } catch (error) {
      setUiBootError(error && error.message ? error.message : String(error));
    }
  </script>
</body>
</html>
""")


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
            if not self.running:
                self.message = "Robot is not running."
                return self.status()

            speed = max(0, min(4095, int(speed)))

            if action == "stop":
                robot.stop_all()
                self.message = "Emergency stop."
                self._notify_status("Р­РєСЃС‚СЂРµРЅРЅР°СЏ РѕСЃС‚Р°РЅРѕРІРєР° СЂРѕР±РѕС‚Р°.", dedupe_key="manual_stop")
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

        stream_port = int(config.get("camera_stream_port", 5000))
        stream_url = f"http://{request.host.split(':')[0]}:{stream_port}/video_feed"
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


