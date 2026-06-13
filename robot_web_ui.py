import argparse
import os
import threading
import time
from typing import Any, Dict, Optional

from flask import Flask, jsonify, request

import main_slam as robot


app = Flask(__name__)


DEFAULT_CONFIG = {
    "lidar_port": "/dev/ttyUSB0",
    "camera_port": "/dev/video0",
    "camera_stream_host": "0.0.0.0",
    "camera_stream_port": 5000,
    "servo_pca_address": "0x41",
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
        <h1 id="heroTitle">Robot Control Station</h1>
        <p id="heroSubtitle">Start the robot, watch the camera stream, and switch between autopilot and manual control from any device in your LAN.</p>
      </div>
      <div class="hero-side">
        <select id="languageSelect" class="lang-switch" onchange="setLanguage(this.value)">
          <option value="ru">Русский</option>
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

        <label for="servo_pca_address" id="labelServoPcaAddress">Servo PCA I2C address</label>
        <input id="servo_pca_address" />

        <label for="bucket_servo_channel" id="labelBucketServoChannel">Bucket servo channel</label>
        <input id="bucket_servo_channel" type="number" min="0" max="15" />

        <label for="camera_port" id="labelCameraPort">Camera port</label>
        <input id="camera_port" />

        <label for="camera_stream_port" id="labelCameraStreamPort">Camera stream port</label>
        <input id="camera_stream_port" type="number" />

        <label for="run_mode" id="labelRunMode">Run mode</label>
        <select id="run_mode">
          <option value="2" id="optionAutopilot">Autopilot</option>
          <option value="1" id="optionManual">Manual</option>
        </select>

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
        <input id="selected_model_name" placeholder="optional" />

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
        <div class="split">
          <div>
            <h2 id="liveVideoTitle">Live Video</h2>
            <img id="videoFrame" class="video" alt="video stream" />
          </div>
          <div>
            <h2 id="manualDriveTitle">Manual Drive</h2>
            <div class="manual-grid">
              <button class="blank">.</button>
              <button class="dark" onclick="drive('forward')" id="buttonForward">Forward</button>
              <button class="blank">.</button>
              <button class="dark" onclick="drive('left')" id="buttonLeft">Left</button>
              <button class="danger" onclick="drive('stop')" id="buttonStop">STOP</button>
              <button class="dark" onclick="drive('right')" id="buttonRight">Right</button>
              <button class="blank">.</button>
              <button class="dark" onclick="drive('backward')" id="buttonBackward">Backward</button>
              <button class="blank">.</button>
            </div>

            <h3 style="margin-top:18px;" id="bucketControlTitle">Bucket Control</h3>
            <div class="button-row">
              <button class="dark" onclick="bucket('wall_down')" id="buttonWallDown">Wall down</button>
              <button class="dark" onclick="bucket('wall_up')" id="buttonWallUp">Wall up</button>
            </div>
            <div class="button-row">
              <button class="dark" onclick="bucket('scoop_down')">Scoop 90°</button>
              <button class="dark" onclick="bucket('scoop_up')">Scoop 0°</button>
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
    let lastStatus = null;
    const translations = {
      ru: {
        servo_pca_address: 'I2C адрес PCA сервы',
        bucket_servo_channel: 'Канал сервопривода ковша',
        bucket_servo_controller: 'Контроллер сервы ковша',
        hero_title: 'Станция управления роботом',
        hero_subtitle: 'Запускай робота, смотри видеопоток и переключайся между автопилотом и ручным управлением из локальной сети.',
        launch_settings: 'Параметры запуска',
        lidar_port: 'Порт LiDAR',
        arduino_port: 'Порт Arduino',
        camera_port: 'Порт камеры',
        camera_stream_port: 'Порт видеопотока',
        run_mode: 'Режим работы',
        autopilot: 'Автопилот',
        manual: 'Ручной',
        autopilot_speed: 'Скорость автопилота',
        manual_speed: 'Скорость ручного режима',
        yolo_source: 'Источник YOLO',
        yolo_phone: 'Телефон / ПК по Wi-Fi',
        yolo_local: 'Локальная модель на Raspberry Pi',
        disabled: 'Отключено',
        route_source: 'Источник маршрута',
        route_slam: 'Маршрут LiDAR SLAM',
        route_gps: 'Телефонный GPS / indoor positioning',
        route_corridor: 'Коридор маршрута (метры)',
        slam_step: 'Шаг точек SLAM (метры)',
        model_folder: 'Папка локальной модели',
        optional: 'необязательно',
        save_config: 'Сохранить конфиг',
        start_robot: 'Запустить робота',
        prepare_bucket: 'Подготовить ковш',
        stop_robot: 'Остановить робота',
        robot_state: 'Состояние робота',
        mode: 'Режим',
        camera_stream: 'Видеопоток',
        trash_detector: 'Детектор мусора',
        bucket_arduino: 'Arduino ковша',
        active_model: 'Активная модель',
        route_status: 'Статус маршрута',
        live_video: 'Живое видео',
        manual_drive: 'Ручное движение',
        forward: 'Вперёд',
        left: 'Влево',
        stop: 'СТОП',
        right: 'Вправо',
        backward: 'Назад',
        bucket_control: 'Управление ковшом',
        wall_down: 'Стенка вниз',
        wall_up: 'Стенка вверх',
        scoop_down: 'Совок 90°',
        scoop_up: 'Совок 0°',
        timed_test: 'Тест по времени',
        collect_cycle: 'Цикл сбора',
        slam_route: 'SLAM-маршрут',
        record_route: 'Записать маршрут',
        stop_record: 'Остановить запись',
        clear_route: 'Очистить маршрут',
        refresh: 'Обновить',
        system_log: 'Системный журнал',
        loading: 'Загрузка...',
        no_messages: 'Пока нет сообщений.',
        running: 'Работает',
        stopped: 'Остановлен',
        online: 'онлайн',
        offline: 'офлайн',
        connected: 'подключено',
        disconnected: 'отключено',
        mode_idle: 'Остановлен',
        mode_manual: 'Ручное управление',
        mode_autopilot: 'Автопилот',
        phone_none: 'телефон / нет',
        points_short: 'точек',
        meters_short: 'м',
        recording: 'запись',
        phone_route_active: 'маршрут с телефона активен',
        waiting_phone_route: 'ожидание маршрута с телефона',
        inside_corridor: 'внутри коридора',
        outside_corridor: 'вне коридора',
        error_prefix: 'Ошибка',
        stream_prefix: 'Поток',
        detector_prefix: 'Детектор',
        route_prefix: 'Маршрут',
        trash_waiting: 'активен, ожидание',
        trash_disabled: 'отключен',
        trash_at: 'мусор под углом',
      },
      en: {
        hero_title: 'Robot Control Station',
        hero_subtitle: 'Start the robot, watch the camera stream, and switch between autopilot and manual control from any device in your LAN.',
        launch_settings: 'Launch Settings',
        lidar_port: 'LiDAR port',
        arduino_port: 'Arduino port',
        servo_pca_address: 'Servo PCA I2C address',
        bucket_servo_channel: 'Bucket servo channel',
        camera_port: 'Camera port',
        camera_stream_port: 'Camera stream port',
        run_mode: 'Run mode',
        autopilot: 'Autopilot',
        manual: 'Manual',
        autopilot_speed: 'Autopilot speed',
        manual_speed: 'Manual speed',
        yolo_source: 'YOLO source',
        yolo_phone: 'Phone / PC detector over Wi-Fi',
        yolo_local: 'Local model on Raspberry Pi',
        disabled: 'Disabled',
        route_source: 'Route source',
        route_slam: 'LiDAR SLAM route',
        route_gps: 'Phone GPS / indoor positioning',
        route_corridor: 'Route corridor (meters)',
        slam_step: 'SLAM point step (meters)',
        model_folder: 'Local model folder name',
        optional: 'optional',
        save_config: 'Save config',
        start_robot: 'Start robot',
        prepare_bucket: 'Prepare bucket',
        stop_robot: 'Stop robot',
        robot_state: 'Robot state',
        mode: 'Mode',
        camera_stream: 'Camera stream',
        trash_detector: 'Trash detector',
        bucket_arduino: 'Bucket Arduino',
        bucket_servo_controller: 'Bucket servo controller',
        active_model: 'Active model',
        route_status: 'Route status',
        live_video: 'Live Video',
        manual_drive: 'Manual Drive',
        forward: 'Forward',
        left: 'Left',
        stop: 'STOP',
        right: 'Right',
        backward: 'Backward',
        bucket_control: 'Bucket Control',
        wall_down: 'Wall down',
        wall_up: 'Wall up',
        scoop_down: 'Scoop 90°',
        scoop_up: 'Scoop 0°',
        timed_test: 'Timed test',
        collect_cycle: 'Collect cycle',
        slam_route: 'SLAM Route',
        record_route: 'Record route',
        stop_record: 'Stop record',
        clear_route: 'Clear route',
        refresh: 'Refresh',
        system_log: 'System Log',
        loading: 'Loading...',
        no_messages: 'No messages yet.',
        running: 'Running',
        stopped: 'Stopped',
        online: 'online',
        offline: 'offline',
        connected: 'connected',
        disconnected: 'disconnected',
        mode_idle: 'Stopped',
        mode_manual: 'Manual web control',
        mode_autopilot: 'Autopilot',
        phone_none: 'phone / none',
        points_short: 'pts',
        meters_short: 'm',
        recording: 'recording',
        phone_route_active: 'phone route active',
        waiting_phone_route: 'waiting for phone route',
        inside_corridor: 'inside corridor',
        outside_corridor: 'outside corridor',
        error_prefix: 'Error',
        stream_prefix: 'Stream',
        detector_prefix: 'Detector',
        route_prefix: 'Route',
        trash_waiting: 'active, waiting',
        trash_disabled: 'disabled',
        trash_at: 'trash at',
      },
    };
    let currentLanguage = localStorage.getItem('robot_ui_language') || 'ru';

    function t(key) {
      return (translations[currentLanguage] && translations[currentLanguage][key])
        || translations.en[key]
        || key;
    }

    function translateMode(mode) {
      return {
        idle: t('mode_idle'),
        manual: t('mode_manual'),
        autopilot: t('mode_autopilot'),
      }[mode] || mode || '--';
    }

    function translateTrashSummary(summary) {
      if (!summary || summary === 'disabled') return t('trash_disabled');
      if (summary === 'active, waiting') return t('trash_waiting');
      if (summary.startsWith('trash at ')) {
        return summary.replace('trash at ', `${t('trash_at')} `);
      }
      return summary;
    }

    function updateStaticTexts() {
      const textMap = {
        heroTitle: 'hero_title',
        heroSubtitle: 'hero_subtitle',
        launchSettingsTitle: 'launch_settings',
        labelLidarPort: 'lidar_port',
        labelServoPcaAddress: 'servo_pca_address',
        labelBucketServoChannel: 'bucket_servo_channel',
        labelCameraPort: 'camera_port',
        labelCameraStreamPort: 'camera_stream_port',
        labelRunMode: 'run_mode',
        optionAutopilot: 'autopilot',
        optionManual: 'manual',
        labelAutoSpeed: 'autopilot_speed',
        labelManualSpeed: 'manual_speed',
        labelYoloChoice: 'yolo_source',
        optionYoloPhone: 'yolo_phone',
        optionYoloLocal: 'yolo_local',
        optionDisabledA: 'disabled',
        labelRouteSource: 'route_source',
        optionDisabledB: 'disabled',
        optionRouteSlam: 'route_slam',
        optionRouteGps: 'route_gps',
        labelRouteCorridor: 'route_corridor',
        labelSlamStep: 'slam_step',
        labelModelName: 'model_folder',
        buttonSaveConfig: 'save_config',
        buttonStartRobot: 'start_robot',
        buttonPrepareBucket: 'prepare_bucket',
        buttonStopRobot: 'stop_robot',
        labelStatRunning: 'robot_state',
        labelStatMode: 'mode',
        labelStatVideo: 'camera_stream',
        labelStatTrash: 'trash_detector',
        labelStatServoController: 'bucket_servo_controller',
        labelStatModel: 'active_model',
        labelStatRouteSource: 'route_source',
        labelStatRoute: 'route_status',
        liveVideoTitle: 'live_video',
        manualDriveTitle: 'manual_drive',
        buttonForward: 'forward',
        buttonLeft: 'left',
        buttonStop: 'stop',
        buttonRight: 'right',
        buttonBackward: 'backward',
        bucketControlTitle: 'bucket_control',
        buttonWallDown: 'wall_down',
        buttonWallUp: 'wall_up',
        buttonTimedTest: 'timed_test',
        buttonCollect: 'collect_cycle',
        slamRouteTitle: 'slam_route',
        buttonRecordRoute: 'record_route',
        buttonStopRecord: 'stop_record',
        buttonClearRoute: 'clear_route',
        buttonRefresh: 'refresh',
        systemLogTitle: 'system_log',
      };

      Object.entries(textMap).forEach(([id, key]) => {
        const element = document.getElementById(id);
        if (element) {
          element.textContent = t(key);
        }
      });

      document.getElementById('selected_model_name').placeholder = t('optional');
      const scoopDownButton = document.querySelector('[onclick="bucket(\'scoop_down\')"]');
      const scoopUpButton = document.querySelector('[onclick="bucket(\'scoop_up\')"]');
      if (scoopDownButton) scoopDownButton.textContent = t('scoop_down');
      if (scoopUpButton) scoopUpButton.textContent = t('scoop_up');
    }

    function setLanguage(lang) {
      currentLanguage = translations[lang] ? lang : 'ru';
      localStorage.setItem('robot_ui_language', currentLanguage);
      document.documentElement.lang = currentLanguage;
      const select = document.getElementById('languageSelect');
      if (select) {
        select.value = currentLanguage;
      }
      updateStaticTexts();
      if (lastStatus) {
        updateStatusUi(lastStatus);
      } else {
        document.getElementById('heroStatus').textContent = t('loading');
        setLog('');
      }
    }

    function formPayload() {
      return {
        lidar_port: document.getElementById('lidar_port').value.trim(),
        servo_pca_address: document.getElementById('servo_pca_address').value.trim(),
        bucket_servo_channel: parseInt(document.getElementById('bucket_servo_channel').value || '0', 10),
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
      document.getElementById('servo_pca_address').value = config.servo_pca_address || '0x41';
      document.getElementById('bucket_servo_channel').value = config.bucket_servo_channel ?? 0;
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
      document.getElementById('logBox').textContent = text || t('no_messages');
    }

    function setFetchError(error) {
      const details = (error && error.message) ? error.message : 'request failed';
      document.getElementById('heroStatus').textContent = `${t('error_prefix')}: ${details}`;
      document.getElementById('heroStatus').style.background = 'rgba(255,107,107,0.15)';
      setLog(`${t('error_prefix')}: ${details}`);
    }

    function updateStatusUi(status) {
      lastStatus = status;
      applyConfig(status.config);
      const routeSourceLabels = {
        none: t('disabled'),
        slam: t('route_slam'),
        gps: t('route_gps'),
      };
      let routeSummary = t('disabled');
      if (status.route_source_mode === 'slam') {
        routeSummary = `${status.slam_route_points || 0} ${t('points_short')}`;
        if (typeof status.route_distance_m === 'number') {
          routeSummary += ` | ${status.route_distance_m.toFixed(2)} ${t('meters_short')}`;
        }
        if (status.slam_route_recording) {
          routeSummary += ` | ${t('recording')}`;
        }
      } else if (status.route_source_mode === 'gps') {
        routeSummary = status.route_fresh ? t('phone_route_active') : t('waiting_phone_route');
        if (typeof status.route_distance_m === 'number') {
          routeSummary += ` | ${status.route_distance_m.toFixed(2)} ${t('meters_short')}`;
        }
        if (status.route_enabled) {
          routeSummary += status.route_within_corridor ? ` | ${t('inside_corridor')}` : ` | ${t('outside_corridor')}`;
        }
      }

      document.getElementById('heroStatus').textContent = status.running
        ? `${t('running')}: ${translateMode(status.mode)}`
        : t('stopped');
      document.getElementById('heroStatus').style.background = status.running
        ? 'rgba(61,220,151,0.18)'
        : 'rgba(255,107,107,0.15)';
      document.getElementById('statRunning').textContent = status.running ? t('online') : t('offline');
      document.getElementById('statMode').textContent = translateMode(status.mode);
      document.getElementById('statVideo').textContent = status.video_stream_active ? status.stream_url : t('stopped');
      document.getElementById('statTrash').textContent = translateTrashSummary(status.trash_summary);
      document.getElementById('statServoController').textContent = status.bucket_servo_controller_connected ? t('connected') : t('disconnected');
      document.getElementById('statModel').textContent = status.selected_model_name || t('phone_none');
      document.getElementById('statRouteSource').textContent = routeSourceLabels[status.route_source_mode] || status.route_source_mode || 'disabled';
      document.getElementById('statRoute').textContent = routeSummary;

      const logLines = [
        status.message || '',
        status.error ? `${t('error_prefix')}: ${status.error}` : '',
        status.stream_url ? `${t('stream_prefix')}: ${status.stream_url}` : '',
        status.detector_debug ? `${t('detector_prefix')}: ${status.detector_debug}` : '',
        status.route_debug ? `${t('route_prefix')}: ${status.route_debug}` : '',
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
      try {
        const response = await fetch(path, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload || {}),
        });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        const data = await response.json();
        updateStatusUi(data);
        return data;
      } catch (error) {
        setFetchError(error);
        throw error;
      }
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
      try {
        const response = await fetch('/api/status');
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        const data = await response.json();
        updateStatusUi(data);
      } catch (error) {
        setFetchError(error);
      }
    }

    setLanguage(currentLanguage);
    refreshStatus();
    setInterval(refreshStatus, 1200);
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
        normalized["servo_pca_address"] = str(
            normalized.get("servo_pca_address", "0x41")
        ).strip() or "0x41"
        try:
            normalized["bucket_servo_channel"] = max(
                0, min(15, int(normalized.get("bucket_servo_channel", 0)))
            )
        except (TypeError, ValueError):
            normalized["bucket_servo_channel"] = 0
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
                self._notify_status(
                    f"Ошибка автопилота: {exc}",
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

                if not robot.init_bucket_servo_controller(config):
                    raise RuntimeError("Failed to connect to bucket servo PCA9685 controller.")

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
                        f"Робот поехал.\nРежим: автопилот.\nСкорость: {auto_speed}.",
                        dedupe_key="robot_started_autopilot",
                    )
                else:
                    self.mode = "manual"
                    self.message = "Robot started in manual web-control mode."
                    self._notify_status(
                        "Робот готов.\nРежим: ручное управление через веб-интерфейс.",
                        dedupe_key="robot_started_manual",
                    )

            except Exception as exc:
                self.error = str(exc)
                self.message = "Robot start failed."
                self._notify_status(
                    f"Не удалось запустить робота.\nОшибка: {exc}",
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
            self._notify_status("Робот остановлен.", dedupe_key="robot_stopped")
            return self.status()

    def prepare_bucket(self) -> Dict[str, Any]:
        with self.lock:
            try:
                robot.move_bucket_wall_to_search_position()
                robot.set_servo_bucket(down=True)
                self.message = "Bucket moved to search position."
                self.error = ""
                self._notify_status(
                    "Робот готов.\nКовш переведен в поисковое положение.",
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
                self._notify_status("Экстренная остановка робота.", dedupe_key="manual_stop")
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
                    f"Робот поехал вперед.\nСкорость: {speed}.",
                    dedupe_key="manual_forward",
                )
            elif action == "backward":
                with robot.movement_lock:
                    robot.current_mode = 2
                robot.set_motors(0, speed, 0, speed)
                self.message = f"Driving backward at {speed}."
                self._notify_status(
                    f"Робот поехал назад.\nСкорость: {speed}.",
                    dedupe_key="manual_backward",
                )
            elif action == "left":
                with robot.movement_lock:
                    robot.current_mode = 3
                robot.set_motors(speed, 0, 0, speed)
                self.message = f"Turning left at {speed}."
                self._notify_status(
                    f"Робот повернул влево.\nСкорость: {speed}.",
                    dedupe_key="manual_left",
                )
            elif action == "right":
                with robot.movement_lock:
                    robot.current_mode = 4
                robot.set_motors(0, speed, speed, 0)
                self.message = f"Turning right at {speed}."
                self._notify_status(
                    f"Робот повернул вправо.\nСкорость: {speed}.",
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
            "bucket_arduino_connected": False,
            "bucket_servo_controller_connected": robot.is_bucket_servo_controller_ready(),
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
