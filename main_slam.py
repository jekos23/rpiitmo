import time
import board
import busio
from adafruit_pca9685 import PCA9685
try:
    import serial
except ImportError:
    serial = None
import threading
import sys
import math
import json
import os
import glob
import subprocess

from ld06_driver import LD06Driver
from Algorithm.OnlineFastSlam import OnlineFastSlam
try:
    from trash_detector import TrashDetector
except ImportError:
    TrashDetector = None

try:
    from remote_trash_listener import RemoteTrashListener
except ImportError:
    RemoteTrashListener = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "config.json")
def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except: pass
    return {}

def save_config(cfg):
    global global_config
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=4)
    except: pass
    global_config = dict(cfg)

def find_serial_candidates():
    candidates = []
    patterns = [
        "/dev/serial/by-id/*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    for pattern in patterns:
        candidates.extend(glob.glob(pattern))
    return sorted(dict.fromkeys(candidates))

def prompt_for_serial_port(label, saved_port=None, forbidden_ports=None):
    forbidden_ports = {port for port in (forbidden_ports or []) if port}
    candidates = [port for port in find_serial_candidates() if port not in forbidden_ports]

    if candidates:
        print(f"\nДоступные порты для {label}:")
        for port in candidates:
            print(f" - {_format_camera_label(port)}")

    while True:
        prompt = f"Введите порт {label}"
        if saved_port:
            prompt += f" (Enter для {saved_port})"
        prompt += ": "

        port = input(prompt).strip()
        if not port and saved_port:
            port = saved_port

        if not port:
            print(f"Порт для {label} обязателен.")
            continue

        if port in forbidden_ports:
            print(f"Порт {port} уже занят другим устройством. Для {label} нужен отдельный порт.")
            continue

        return port

def find_camera_candidates():
    return sorted(dict.fromkeys(glob.glob("/dev/video*")))

def _read_text_if_exists(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as handle:
            return handle.read().strip()
    except OSError:
        return None

def _normalize_camera_address(value):
    if not value:
        return None

    cleaned = str(value).strip()
    for separator in ("Bus", "bus", "Device", "device", ",", ";", "usb"):
        cleaned = cleaned.replace(separator, " ")

    cleaned = cleaned.replace("/", ":").replace("-", ":")
    if ":" in cleaned:
        bus_part, dev_part = cleaned.split(":", 1)
        if bus_part.strip().isdigit() and dev_part.strip().isdigit():
            return f"{int(bus_part):03d}:{int(dev_part):03d}"

    digits = [part for part in cleaned.split() if part.isdigit()]
    if len(digits) >= 2:
        return f"{int(digits[0]):03d}:{int(digits[1]):03d}"

    return None

def _get_camera_usb_identity(port):
    video_name = os.path.basename(port)
    sysfs_dir = os.path.join("/sys/class/video4linux", video_name)
    device_dir = os.path.realpath(os.path.join(sysfs_dir, "device"))
    if not os.path.exists(device_dir):
        return None

    current_dir = device_dir
    while True:
        bus_num = _read_text_if_exists(os.path.join(current_dir, "busnum"))
        dev_num = _read_text_if_exists(os.path.join(current_dir, "devnum"))
        if bus_num and dev_num and bus_num.isdigit() and dev_num.isdigit():
            product = _read_text_if_exists(os.path.join(current_dir, "product"))
            manufacturer = _read_text_if_exists(os.path.join(current_dir, "manufacturer"))
            return {
                "address": f"{int(bus_num):03d}:{int(dev_num):03d}",
                "product": product,
                "manufacturer": manufacturer,
            }

        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            break
        current_dir = parent_dir

    return None

def _format_camera_label(port):
    identity = _get_camera_usb_identity(port)
    if not identity:
        return port

    details = []
    if identity.get("manufacturer"):
        details.append(identity["manufacturer"])
    if identity.get("product"):
        details.append(identity["product"])

    if details:
        return f"{port} (USB {identity['address']}, {' '.join(details)})"

    return f"{port} (USB {identity['address']})"

def resolve_camera_port(camera_input):
    if not camera_input:
        return None

    camera_input = camera_input.strip()
    if camera_input.startswith("/dev/video"):
        return camera_input

    normalized_address = _normalize_camera_address(camera_input)
    if not normalized_address:
        return camera_input

    for port in find_camera_candidates():
        identity = _get_camera_usb_identity(port)
        if identity and identity.get("address") == normalized_address:
            return port

    return camera_input

def prompt_for_camera_port(saved_port=None):
    candidates = find_camera_candidates()
    if candidates:
        print("\nДоступные видеоустройства:")
        for port in candidates:
            print(f" - {port}")

    while True:
        prompt = "Введите порт камеры"
        if saved_port:
            prompt += f" (Enter для {saved_port})"
        prompt += ": "

        camera_port = input(prompt).strip()
        if not camera_port and saved_port:
            camera_port = saved_port

        if camera_port:
            return camera_port

        print("Порт камеры обязателен.")

# Загружаем глобальный конфиг, чтобы к нему был доступ из любых функций
# Override the prompt so camera address can be changed every start and can
# be entered either as /dev/videoX or as a USB address like 001:010.
def prompt_for_camera_port(saved_port=None):
    candidates = find_camera_candidates()
    if candidates:
        print("\nAvailable camera devices:")
        for port in candidates:
            print(f" - {_format_camera_label(port)}")

    while True:
        prompt = "Enter camera port or USB address (example 001:010)"
        if saved_port:
            prompt += f" [Enter for {saved_port}]"
        prompt += ": "

        camera_input = input(prompt).strip()
        if not camera_input and saved_port:
            camera_input = saved_port

        if not camera_input:
            print("Camera port is required.")
            continue

        resolved_port = resolve_camera_port(camera_input)
        if resolved_port != camera_input:
            print(f"Resolved {camera_input} -> {resolved_port}")
        return resolved_port

global_config = load_config()

# Инициализация I2C шины и PCA9685
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 1000
except Exception as e:
    print(f"I2C/PCA9685 инициализация не удалась (если вы на ПК, это нормально): {e}")
    pca = None

LEFT_FWD_CHANNELS = [0, 3, 5]
LEFT_REV_CHANNELS = [1, 2, 4]
RIGHT_FWD_CHANNELS = [7, 9, 11]
RIGHT_REV_CHANNELS = [6, 8, 10]

# Глобальные переменные состояния движения для псевдо-одометрии
DRIVE_CHANNELS = set(LEFT_FWD_CHANNELS + LEFT_REV_CHANNELS + RIGHT_FWD_CHANNELS + RIGHT_REV_CHANNELS)
BUCKET_MOTOR_DEFAULT_FWD_CHANNEL = 12
BUCKET_MOTOR_DEFAULT_REV_CHANNEL = 13

bucket_arduino = None
bucket_arduino_lock = threading.Lock()
bucket_arduino_running = False
bucket_arduino_reader_thread = None
last_bucket_pot_value = None
bucket_channel_warning_shown = False
video_streamer_process = None
bucket_wall_increase_direction_sign = None

current_mode = 0
current_speed = 0
movement_lock = threading.Lock()

# Параметры робота (обновлены по вашим размерам)
WHEEL_CIRCUMFERENCE = 0.39 # Длина окружности колеса в метрах (39 см)
MAX_WHEEL_RPM = 120.0      # Значение по умолчанию, будет перезаписано при калибровке
ROBOT_TRACK_WIDTH = 0.55   # Ширина колесной базы (55 см)
# Центр робота = (0, 0). Задний правый угол = X: -0.31м, Y: -0.275м. 
# Лидар на балке 14см от заднего угла -> X: -0.31 + 0.14 = -0.17м, Y: -0.275м
LIDAR_OFFSET_X = -0.17     # Смещение лидара по оси X (назад от центра)
LIDAR_OFFSET_Y = -0.275    # Смещение лидара по оси Y (вправо от центра)

def set_motors(left_fwd, left_rev, right_fwd, right_rev):
    if not pca: return
    def scale_speed(speed):
        speed = max(0, min(4095, speed))
        return int((speed / 4095.0) * 65535)

    l_fwd_pwm = scale_speed(left_fwd)
    l_rev_pwm = scale_speed(left_rev)
    r_fwd_pwm = scale_speed(right_fwd)
    r_rev_pwm = scale_speed(right_rev)

    for ch in LEFT_FWD_CHANNELS: pca.channels[ch].duty_cycle = l_fwd_pwm
    for ch in LEFT_REV_CHANNELS: pca.channels[ch].duty_cycle = l_rev_pwm
    for ch in RIGHT_FWD_CHANNELS: pca.channels[ch].duty_cycle = r_fwd_pwm
    for ch in RIGHT_REV_CHANNELS: pca.channels[ch].duty_cycle = r_rev_pwm

def _legacy_set_servo_bucket(down=True):
    if not pca or not servo: 
        print("[СЕРВОПРИВОД] Ошибка: PCA9685 или библиотека adafruit_motor не найдены!")
        return
        
    # Временно переключаем всю плату на 50 Гц для управления сервоприводом.
    # Так как моторы сейчас остановлены (PWM=0), это безопасно.
    pca.frequency = 50
    
    CHANNEL = 14
    servo_motor = servo.Servo(pca.channels[CHANNEL], min_pulse=500, max_pulse=2500)
    
    state = "ОПУСКАЮ" if down else "ПОДНИМАЮ"
    print(f"[СЕРВОПРИВОД - Канал {CHANNEL}] {state} ковш! (Частота переключена на 50 Гц)")
    
    # Читаем откалиброванные углы из конфигурации
    up_angle = global_config.get("servo_up_angle", 0)
    down_angle = global_config.get("servo_down_angle", 90)
    
    # Значения углов берем из файла конфигурации!
    if down:
        servo_motor.angle = down_angle  # Ковш опущен (положение для сбора)
    else:
        servo_motor.angle = up_angle    # Ковш поднят (нулевое/транспортное положение)
        
    # Ждем пока ковш физически опустится/поднимется
    time.sleep(1.0)
    
    # Отключаем ШИМ сигнал на сервопривод, чтобы он не дрожал и не тратил ток
    pca.channels[CHANNEL].duty_cycle = 0
    
    # Возвращаем частоту обратно на 1000 Гц для правильной работы моторов колес
    pca.frequency = 1000

def _scale_speed_to_duty_cycle(speed):
    speed = max(0, min(4095, int(speed)))
    return int((speed / 4095.0) * 65535)

def _get_bucket_motor_channels():
    global bucket_channel_warning_shown
    try:
        fwd = int(global_config.get("bucket_motor_forward_channel", BUCKET_MOTOR_DEFAULT_FWD_CHANNEL))
        rev = int(global_config.get("bucket_motor_reverse_channel", BUCKET_MOTOR_DEFAULT_REV_CHANNEL))
    except (TypeError, ValueError):
        fwd = BUCKET_MOTOR_DEFAULT_FWD_CHANNEL
        rev = BUCKET_MOTOR_DEFAULT_REV_CHANNEL

    if (
        fwd == rev
        or fwd in DRIVE_CHANNELS
        or rev in DRIVE_CHANNELS
        or not (0 <= fwd <= 15)
        or not (0 <= rev <= 15)
    ):
        if not bucket_channel_warning_shown:
            print(
                f"[КОВШ] Небезопасная конфигурация каналов bucket motor: {fwd}/{rev}. "
                "Они пересекаются с ходовыми моторами, поэтому мотор ковша отключен до исправления config.json."
            )
            bucket_channel_warning_shown = True
        return None, None
    return fwd, rev

def init_bucket_arduino(config):
    global bucket_arduino, bucket_arduino_running, bucket_arduino_reader_thread

    if bucket_arduino:
        return True

    if not serial:
        print("[ARDUINO] pyserial не установлен, управление сервой ковша через Arduino недоступно.")
        return False

    arduino_port = config.get("arduino_port", "/dev/ttyACM0")
    baudrate = int(config.get("arduino_baudrate", 9600))
    timeout_sec = float(config.get("arduino_timeout_sec", 0.1))
    boot_wait_sec = float(config.get("arduino_boot_wait_sec", 2.0))

    try:
        bucket_arduino = serial.Serial(
            arduino_port,
            baudrate,
            timeout=timeout_sec,
            write_timeout=timeout_sec,
        )
        time.sleep(boot_wait_sec)
        bucket_arduino.reset_input_buffer()
        bucket_arduino.reset_output_buffer()
        bucket_arduino_running = True
        bucket_arduino_reader_thread = threading.Thread(target=_bucket_arduino_reader_loop, daemon=True)
        bucket_arduino_reader_thread.start()
        print(f"[ARDUINO] Подключен контроллер ковша на {arduino_port} ({baudrate} бод).")
        return True
    except Exception as e:
        print(f"[ARDUINO] Не удалось подключиться к Arduino на {arduino_port}: {e}")
        bucket_arduino = None
        bucket_arduino_running = False
        return False

def _bucket_arduino_reader_loop():
    global last_bucket_pot_value, bucket_arduino_running

    while bucket_arduino_running and bucket_arduino:
        try:
            line = bucket_arduino.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            payload = line[4:] if line.startswith("POT:") else line
            if payload.isdigit():
                last_bucket_pot_value = int(payload)
            elif line.startswith("OK:") or line.startswith("ERR:"):
                print(f"[ARDUINO] {line}")
        except Exception as e:
            if bucket_arduino_running:
                print(f"[ARDUINO] Ошибка чтения Serial: {e}")
            time.sleep(0.2)

def close_bucket_arduino():
    global bucket_arduino, bucket_arduino_running

    bucket_arduino_running = False
    if bucket_arduino:
        try:
            bucket_arduino.close()
        except Exception:
            pass
    bucket_arduino = None

def send_bucket_arduino_command(command):
    if not bucket_arduino:
        print(f"[ARDUINO] Команда '{command}' пропущена: Arduino не подключен.")
        return False

    try:
        with bucket_arduino_lock:
            bucket_arduino.write(f"{command}\n".encode("utf-8"))
            bucket_arduino.flush()
        return True
    except Exception as e:
        print(f"[ARDUINO] Не удалось отправить команду '{command}': {e}")
        return False

def _move_bucket_servo_to_angle(target_angle, label, wait=True):
    target_angle = max(0, min(180, int(target_angle)))
    if not send_bucket_arduino_command(f"SERVO:{target_angle}"):
        return False

    print(f"[СЕРВОПРИВОД - ARDUINO] {label} ковш до {target_angle}°.")
    if wait:
        time.sleep(float(global_config.get("bucket_servo_move_sec", 0.8)))
    return True

def set_servo_bucket(down=True, wait=True):
    up_angle = global_config.get("servo_up_angle", 0)
    down_angle = global_config.get("servo_down_angle", 90)
    target_angle = down_angle if down else up_angle
    label = "ОПУСКАЮ" if down else "ПОДНИМАЮ"
    return _move_bucket_servo_to_angle(target_angle, label, wait=wait)

def set_bucket_motor(speed):
    if not pca:
        print("[КОВШ] PCA9685 недоступна, мотор ковша не управляется.")
        return False

    fwd_channel, rev_channel = _get_bucket_motor_channels()
    if fwd_channel is None or rev_channel is None:
        return False

    speed = max(-4095, min(4095, int(speed)))

    if speed > 0:
        pca.channels[fwd_channel].duty_cycle = _scale_speed_to_duty_cycle(speed)
        pca.channels[rev_channel].duty_cycle = 0
    elif speed < 0:
        pca.channels[fwd_channel].duty_cycle = 0
        pca.channels[rev_channel].duty_cycle = _scale_speed_to_duty_cycle(abs(speed))
    else:
        pca.channels[fwd_channel].duty_cycle = 0
        pca.channels[rev_channel].duty_cycle = 0
    return True

def legacy_pulse_bucket_motor_unused(speed, duration_sec=None):
    duration_sec = float(duration_sec if duration_sec is not None else global_config.get("bucket_motor_pulse_sec", 0.7))
    try:
        set_bucket_motor(speed)
        time.sleep(duration_sec)
    finally:
        set_bucket_motor(0)

def legacy_run_bucket_collect_cycle_unused():
    lower_before_collect = bool(global_config.get("bucket_lower_before_collect", True))
    lower_pause_sec = float(global_config.get("bucket_lower_pause_sec", 0.25))
    collect_speed = int(global_config.get("bucket_motor_collect_speed", 4095))
    collect_duration_sec = float(global_config.get("bucket_motor_collect_duration_sec", 1.1))
    settle_after_sec = float(global_config.get("bucket_collect_settle_sec", 0.2))

    try:
        if lower_before_collect:
            print("[КОВШ] Опускаю ковш перед захватом мусора.")
            set_servo_bucket(down=True, wait=True)
            time.sleep(lower_pause_sec)

        print("[КОВШ] Поднимаю ковш и запускаю стенку ковша одновременно.")
        set_bucket_motor(collect_speed)
        set_servo_bucket(down=False, wait=False)
        time.sleep(collect_duration_sec)
    finally:
        set_bucket_motor(0)
        time.sleep(settle_after_sec)

def handle_remote_servo_command(command):
    command = str(command).strip().upper()
    if command == "UP":
        set_servo_bucket(down=False, wait=False)
        return
    if command == "DOWN":
        set_servo_bucket(down=True, wait=False)
        return
    try:
        angle = float(command)
        _move_bucket_servo_to_angle(angle, "УСТАНАВЛИВАЮ", wait=False)
    except ValueError:
        print(f"[REMOTE] Неизвестная команда сервоприводу: {command}")

def legacy_handle_remote_bucket_motor_command_unused(command):
    command = str(command).strip().upper()
    collect_speed = int(global_config.get("bucket_motor_collect_speed", 2800))
    reverse_speed = int(global_config.get("bucket_motor_reverse_speed", -collect_speed))
    pulse_sec = float(global_config.get("bucket_motor_pulse_sec", 0.7))

    if command in {"DOWN", "FORWARD", "COLLECT"}:
        threading.Thread(target=pulse_bucket_motor, args=(collect_speed, pulse_sec), daemon=True).start()
    elif command in {"UP", "REVERSE", "BACK"}:
        threading.Thread(target=pulse_bucket_motor, args=(reverse_speed, pulse_sec), daemon=True).start()
    elif command in {"STOP", "0"}:
        set_bucket_motor(0)
    else:
        print(f"[REMOTE] Неизвестная команда мотору ковша: {command}")

def get_bucket_wall_position(wait_timeout_sec=1.0):
    deadline = time.time() + wait_timeout_sec
    while time.time() < deadline:
        if last_bucket_pot_value is not None:
            return int(last_bucket_pot_value)
        time.sleep(0.05)
    return int(last_bucket_pot_value) if last_bucket_pot_value is not None else None

def pulse_bucket_motor(speed, duration_sec=None):
    duration_sec = float(duration_sec if duration_sec is not None else global_config.get("bucket_motor_pulse_sec", 0.18))
    before = get_bucket_wall_position(wait_timeout_sec=0.2)
    try:
        set_bucket_motor(speed)
        time.sleep(duration_sec)
    finally:
        set_bucket_motor(0)
    time.sleep(float(global_config.get("bucket_motor_settle_after_pulse_sec", 0.12)))
    after = get_bucket_wall_position(wait_timeout_sec=0.5)
    return before, after

def _remember_bucket_wall_direction(speed, before, after):
    global bucket_wall_increase_direction_sign

    if before is None or after is None or before == after or speed == 0:
        return

    speed_sign = 1 if speed > 0 else -1
    if after > before:
        bucket_wall_increase_direction_sign = speed_sign
    else:
        bucket_wall_increase_direction_sign = -speed_sign

def ensure_bucket_wall_direction():
    global bucket_wall_increase_direction_sign

    if bucket_wall_increase_direction_sign in (-1, 1):
        return bucket_wall_increase_direction_sign

    test_speed = int(global_config.get("bucket_wall_detect_speed", 4095))
    for speed in (test_speed, -test_speed):
        before, after = pulse_bucket_motor(speed, global_config.get("bucket_wall_detect_pulse_sec", 0.12))
        _remember_bucket_wall_direction(speed, before, after)
        if bucket_wall_increase_direction_sign in (-1, 1):
            pulse_bucket_motor(-speed, global_config.get("bucket_wall_detect_pulse_sec", 0.12))
            return bucket_wall_increase_direction_sign

    print("[СТЕНКА] Не удалось определить направление изменения потенциометра. Проверьте подключение.")
    return None

def move_bucket_wall_to_position(target_value, label="Стенка ковша", timeout_sec=None):
    increase_direction = ensure_bucket_wall_direction()
    if increase_direction not in (-1, 1):
        return False

    if target_value is None:
        print(f"[СТЕНКА] Целевое положение '{label}' не задано.")
        return False

    tolerance = int(global_config.get("bucket_wall_tolerance", 25))
    min_speed = int(global_config.get("bucket_wall_min_speed", 1200))
    max_speed = int(global_config.get("bucket_wall_max_speed", 4095))
    kp = float(global_config.get("bucket_wall_kp", 10.0))
    timeout_sec = float(timeout_sec if timeout_sec is not None else global_config.get("bucket_wall_move_timeout_sec", 5.0))
    deadline = time.time() + timeout_sec

    while time.time() < deadline:
        current_value = get_bucket_wall_position(wait_timeout_sec=0.3)
        if current_value is None:
            print(f"[СТЕНКА] Нет данных с потенциометра для '{label}'.")
            break

        error = int(target_value) - current_value
        if abs(error) <= tolerance:
            set_bucket_motor(0)
            print(f"[СТЕНКА] '{label}' достигнуто: {current_value}.")
            return True

        direction = increase_direction if error > 0 else -increase_direction
        speed = int(min(max_speed, max(min_speed, abs(error) * kp)))
        set_bucket_motor(direction * speed)
        time.sleep(0.05)

    set_bucket_motor(0)
    print(f"[СТЕНКА] Не удалось довести '{label}' до цели {target_value}. Последнее значение: {get_bucket_wall_position(0.1)}.")
    return False

def run_bucket_wall_timed_test(duration_sec=None, speed=None):
    duration_sec = float(
        duration_sec if duration_sec is not None else global_config.get("bucket_wall_manual_calibration_pulse_sec", 2.0)
    )
    speed = abs(int(speed if speed is not None else global_config.get("bucket_wall_manual_speed", 4095)))

    print(f"[РЎРўР•РќРљРђ] РўРµСЃС‚: РїРѕРґРЅРёРјР°СЋ СЃРѕРІРѕРє РЅР°Р·Р°Рґ РЅР° {duration_sec:.1f} СЃРµРє.")
    before_up, after_up = pulse_bucket_motor(-speed, duration_sec)
    _remember_bucket_wall_direction(-speed, before_up, after_up)

    time.sleep(0.2)

    print(f"[РЎРўР•РќРљРђ] РўРµСЃС‚: РѕРїСѓСЃРєР°СЋ СЃРѕРІРѕРє РІРїРµСЂРµРґ РЅР° {duration_sec:.1f} СЃРµРє.")
    before_down, after_down = pulse_bucket_motor(speed, duration_sec)
    _remember_bucket_wall_direction(speed, before_down, after_down)

    return True

def calibrate_bucket_wall(config):
    search_position = config.get("bucket_wall_search_pot")
    lower_position = config.get("bucket_wall_lower_pot")
    default_answer = "y" if search_position is None or lower_position is None else "n"

    answer = input(
        f"\nХотите откалибровать стенку ковша по потенциометру? (y/n, Enter={default_answer}): "
    ).strip().lower()
    if not answer:
        answer = default_answer

    if answer != "y":
        if search_position is None or lower_position is None:
            print("[СТЕНКА] Калибровка обязательна: не заданы поисковое и нижнее положения.")
            return calibrate_bucket_wall(config)
        return True

    print("\n=== КАЛИБРОВКА СТЕНКИ КОВША ===")
    print("Команды:")
    print("  j - короткий шаг мотором в одну сторону")
    print("  k - короткий шаг мотором в другую сторону")
    print("  p - показать текущее значение потенциометра")
    print("  s - сохранить текущее значение как ПОИСКОВОЕ положение")
    print("  l - сохранить текущее значение как НИЖНЕЕ положение")
    print("  q - завершить калибровку")

    if int(config.get("bucket_wall_manual_speed", 4095)) != 4095:
        config["bucket_wall_manual_speed"] = 4095
        save_config(config)
    manual_speed = int(config.get("bucket_wall_manual_speed", 4095))
    if float(config.get("bucket_wall_manual_calibration_pulse_sec", 2.0)) != 2.0:
        config["bucket_wall_manual_calibration_pulse_sec"] = 2.0
        save_config(config)
    manual_pulse_sec = float(config.get("bucket_wall_manual_calibration_pulse_sec", 2.0))
    search_position = config.get("bucket_wall_search_pot")
    lower_position = config.get("bucket_wall_lower_pot")

    while True:
        current_value = get_bucket_wall_position(wait_timeout_sec=1.0)
        print(f"\n[СТЕНКА] Текущее значение потенциометра: {current_value}")
        command = input("Калибровка стенки > ").strip().lower()

        if command == "j":
            before, after = pulse_bucket_motor(-manual_speed, manual_pulse_sec)
            _remember_bucket_wall_direction(-manual_speed, before, after)
        elif command == "k":
            before, after = pulse_bucket_motor(manual_speed, manual_pulse_sec)
            _remember_bucket_wall_direction(manual_speed, before, after)
        elif command == "t":
            run_bucket_wall_timed_test(manual_pulse_sec, manual_speed)
        elif command == "p":
            continue
        elif command == "s":
            if current_value is None:
                print("[СТЕНКА] Нет сигнала потенциометра.")
                continue
            search_position = current_value
            config["bucket_wall_search_pot"] = search_position
            save_config(config)
            print(f"[СТЕНКА] Поисковое положение сохранено: {search_position}.")
        elif command == "l":
            if current_value is None:
                print("[СТЕНКА] Нет сигнала потенциометра.")
                continue
            lower_position = current_value
            config["bucket_wall_lower_pot"] = lower_position
            save_config(config)
            print(f"[СТЕНКА] Нижнее положение сохранено: {lower_position}.")
        elif command == "q":
            if search_position is None or lower_position is None:
                print("[СТЕНКА] Нужно сохранить и поисковое, и нижнее положение.")
                continue
            break
        else:
            print("Неизвестная команда.")

    ensure_bucket_wall_direction()
    return True

def move_bucket_wall_to_search_position():
    return move_bucket_wall_to_position(global_config.get("bucket_wall_search_pot"), "поисковое положение стенки")

def move_bucket_wall_to_lower_position():
    return move_bucket_wall_to_position(global_config.get("bucket_wall_lower_pot"), "нижнее положение стенки")

def run_bucket_collect_cycle():
    servo_lower_pause_sec = float(global_config.get("bucket_servo_lower_pause_sec", 0.2))
    wall_lower_pause_sec = float(global_config.get("bucket_wall_lower_pause_sec", 0.2))
    servo_join_timeout_sec = float(global_config.get("bucket_servo_move_sec", 0.8)) + 0.8
    settle_after_sec = float(global_config.get("bucket_collect_settle_sec", 0.2))

    try:
        print("[КОВШ] Опускаю серву ковша.")
        set_servo_bucket(down=True, wait=True)
        time.sleep(servo_lower_pause_sec)

        print("[КОВШ] Опускаю стенку ковша к мусору.")
        move_bucket_wall_to_lower_position()
        time.sleep(wall_lower_pause_sec)

        print("[КОВШ] Поднимаю стенку и серву вместе, чтобы закинуть мусор назад.")
        servo_thread = threading.Thread(target=set_servo_bucket, kwargs={"down": False, "wait": True}, daemon=True)
        servo_thread.start()
        move_bucket_wall_to_search_position()
        servo_thread.join(timeout=servo_join_timeout_sec)
        set_servo_bucket(down=True, wait=True)
    finally:
        set_bucket_motor(0)
        time.sleep(settle_after_sec)

def handle_remote_bucket_motor_command(command):
    command = str(command).strip().upper()
    if command in {"DOWN", "LOWER"}:
        threading.Thread(target=move_bucket_wall_to_lower_position, daemon=True).start()
    elif command in {"UP", "SEARCH", "RAISE"}:
        threading.Thread(target=move_bucket_wall_to_search_position, daemon=True).start()
    elif command == "COLLECT":
        threading.Thread(target=run_bucket_collect_cycle, daemon=True).start()
    elif command == "JOG+":
        threading.Thread(
            target=pulse_bucket_motor,
            args=(int(global_config.get("bucket_wall_manual_speed", 4095)),),
            daemon=True,
        ).start()
    elif command == "JOG-":
        threading.Thread(
            target=pulse_bucket_motor,
            args=(-int(global_config.get("bucket_wall_manual_speed", 4095)),),
            daemon=True,
        ).start()
    elif command in {"STOP", "0"}:
        set_bucket_motor(0)
    else:
        print(f"[REMOTE] Неизвестная команда мотору ковша: {command}")

bucket_wall_position_state = "unknown"

def _save_bucket_wall_state(state):
    global bucket_wall_position_state, global_config
    if state not in {"search", "lower", "unknown"}:
        return
    bucket_wall_position_state = state
    global_config["bucket_wall_current_state"] = state
    save_config(global_config)

def _get_bucket_wall_move_duration():
    return float(global_config.get("bucket_wall_move_duration_sec", 2.0))

def _get_bucket_wall_drive_speed():
    return abs(int(global_config.get("bucket_wall_manual_speed", 4095)))

def _run_bucket_wall_motion(direction, label, duration_sec=None):
    duration_sec = float(duration_sec if duration_sec is not None else _get_bucket_wall_move_duration())
    speed = _get_bucket_wall_drive_speed()
    signed_speed = speed if direction > 0 else -speed
    print(f"[BUCKET] {label}: {duration_sec:.1f}s at full power.")
    pulse_bucket_motor(signed_speed, duration_sec)
    return True

def move_bucket_wall_to_search_position():
    global bucket_wall_position_state
    if bucket_wall_position_state == "search":
        print("[BUCKET] Wall is already in search position.")
        return True
    _run_bucket_wall_motion(-1, "Raise wall to search position")
    _save_bucket_wall_state("search")
    return True

def move_bucket_wall_to_lower_position():
    global bucket_wall_position_state
    if bucket_wall_position_state == "lower":
        print("[BUCKET] Wall is already in lowered position.")
        return True
    _run_bucket_wall_motion(1, "Lower wall to pickup position")
    _save_bucket_wall_state("lower")
    return True

def run_bucket_wall_timed_test(duration_sec=None, speed=None):
    duration_sec = float(duration_sec if duration_sec is not None else _get_bucket_wall_move_duration())
    print(f"[BUCKET] Timed test: raise for {duration_sec:.1f}s, then lower for {duration_sec:.1f}s.")
    _run_bucket_wall_motion(-1, "Raise wall test", duration_sec)
    time.sleep(0.2)
    _run_bucket_wall_motion(1, "Lower wall test", duration_sec)
    return True

def calibrate_bucket_wall(config):
    global bucket_wall_position_state, global_config

    config["bucket_wall_manual_speed"] = 4095
    config["bucket_wall_move_duration_sec"] = 2.0

    saved_state = str(config.get("bucket_wall_current_state", "search")).strip().lower()
    if saved_state not in {"search", "lower"}:
        saved_state = "search"

    print("\n=== BUCKET WALL SETUP ===")
    print("The potentiometer is disabled.")
    print("Wall motion now uses fixed timed moves of 2 seconds.")
    print("1 - Search position / raised wall")
    print("2 - Lowered position / pickup wall")
    current_choice = input(f"Current wall position (Enter for {saved_state}): ").strip().lower()

    if current_choice in {"2", "lower", "down", "pickup"}:
        bucket_wall_position_state = "lower"
    elif current_choice in {"1", "search", "up", "raised", ""}:
        bucket_wall_position_state = saved_state if current_choice == "" else "search"
    else:
        bucket_wall_position_state = saved_state

    config["bucket_wall_current_state"] = bucket_wall_position_state
    save_config(config)
    global_config = dict(config)

    test_choice = input("Run timed wall test now? (y/n, Enter=n): ").strip().lower()
    if test_choice == "y":
        run_bucket_wall_timed_test(config.get("bucket_wall_move_duration_sec", 2.0))

    return True

def handle_remote_bucket_motor_command(command):
    command = str(command).strip().upper()
    if command in {"DOWN", "LOWER"}:
        threading.Thread(target=move_bucket_wall_to_lower_position, daemon=True).start()
    elif command in {"UP", "SEARCH", "RAISE"}:
        threading.Thread(target=move_bucket_wall_to_search_position, daemon=True).start()
    elif command == "COLLECT":
        threading.Thread(target=run_bucket_collect_cycle, daemon=True).start()
    elif command in {"TEST", "TIMED"}:
        threading.Thread(target=run_bucket_wall_timed_test, daemon=True).start()
    elif command in {"STOP", "0"}:
        set_bucket_motor(0)
    else:
        print(f"[REMOTE] Unknown bucket motor command: {command}")

def stop_all():
    set_motors(0, 0, 0, 0)
    set_bucket_motor(0)
    with movement_lock:
        global current_mode, current_speed
        current_mode = 0
        current_speed = 0

def slam_thread_function(driver, show_map):
    slam = OnlineFastSlam(show_map=show_map)
    last_time = time.time()
    
    print("SLAM поток запущен.")
    while driver.running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Получаем данные с лидара
        scan = driver.get_latest_scan()
        
        # Расчет псевдо-одометрии
        with movement_lock:
            mode = current_mode
            speed = current_speed
            
        dx = 0.0
        dy = 0.0
        dtheta = 0.0
        
        if mode != 0 and speed > 0:
            # Расчет линейной скорости (м/с)
            # speed / 4095.0 - доля от максимальной скорости
            rpm = MAX_WHEEL_RPM * (speed / 4095.0)
            rps = rpm / 60.0 # оборотов в секунду
            linear_velocity = WHEEL_CIRCUMFERENCE * rps
            
            if mode == 1: # Вперед
                dx = linear_velocity * dt
            elif mode == 2: # Назад
                dx = -linear_velocity * dt
            elif mode == 3: # Влево (разворот на месте)
                # Угловая скорость = (V_right - V_left) / TRACK_WIDTH
                # Правые вперед (V), левые назад (-V)
                v_right = linear_velocity
                v_left = -linear_velocity
                angular_velocity = (v_right - v_left) / ROBOT_TRACK_WIDTH
                dtheta = angular_velocity * dt
            elif mode == 4: # Вправо
                # Левые вперед (V), правые назад (-V)
                v_right = -linear_velocity
                v_left = linear_velocity
                angular_velocity = (v_right - v_left) / ROBOT_TRACK_WIDTH
                dtheta = angular_velocity * dt

        # Обновляем SLAM (передаем также смещение лидара по X и Y)
        slam.process_scan(scan, dx, dy, dtheta, LIDAR_OFFSET_X, LIDAR_OFFSET_Y)
        
        # Спим чтобы SLAM работал примерно 5-10 Гц
        time.sleep(0.1)

def get_lidar_distance(scan, target_angle_deg, cone_half_angle=5):
    """
    Ищет минимальную дистанцию в заданном секторе (target_angle_deg ± cone_half_angle).
    Возвращает дистанцию в метрах или 999.0 если нет данных.
    """
    min_dist = 999.0
    for angle_deg in range(360):
        r = scan[angle_deg]
        if r <= 0.0: continue
        
        rel_angle = (angle_deg - target_angle_deg) % 360
        if rel_angle > 180: rel_angle -= 360
        
        if abs(rel_angle) <= cone_half_angle:
            if r < min_dist:
                min_dist = r
    return min_dist

def get_clearance(scan, target_angle_deg, robot_half_width=0.42): # Увеличили зазор с боков еще на 2см
    """
    Вычисляет свободную дистанцию в направлении target_angle_deg,
    учитывая ширину робота (robot_half_width).
    """
    min_clearance = 8.0
    for angle_deg in range(360):
        r = scan[angle_deg]
        if r <= 0.0: continue
        
        rel_angle = (angle_deg - target_angle_deg) % 360
        if rel_angle > 180: rel_angle -= 360
        rel_rad = math.radians(rel_angle)
        
        y_offset = abs(r * math.sin(rel_rad))
        x_offset = r * math.cos(rel_rad)
        
        # Если точка находится "впереди" на этом луче и в пределах ширины робота
        if x_offset > 0 and y_offset < robot_half_width:
            if x_offset < min_clearance:
                min_clearance = x_offset
    return min_clearance

def calibrate_motors(config):
    global MAX_WHEEL_RPM
    if "MAX_WHEEL_RPM" in config:
        MAX_WHEEL_RPM = config["MAX_WHEEL_RPM"]
        print(f"\n[Конфиг] Загружен MAX_WHEEL_RPM = {MAX_WHEEL_RPM:.1f}")
        
    print("\n=== КАЛИБРОВКА МОТОРОВ ===")
    ans = input("Хотите выполнить калибровку моторов для точной одометрии? (y/n): ").strip().lower()
    if ans != 'y':
        print(f"Калибровка пропущена. Используется MAX_WHEEL_RPM: {MAX_WHEEL_RPM}")
        return

    print("\nВНИМАНИЕ: Поднимите робота над землей (поставьте на подставку), чтобы колеса крутились в воздухе!")
    input("Нажмите ENTER, когда будете готовы начать калибровку...")
    
    test_speed = 2000
    test_duration = 5.0 # Секунды
    
    print(f"\nЗапуск моторов вперед со скоростью {test_speed} на {test_duration} секунд...")
    print("ВНИМАТЕЛЬНО СЧИТАЙТЕ ОБОРОТЫ КОЛЕСА (можно считать обороты одного любого колеса).")
    time.sleep(1) # Небольшая пауза перед стартом
    
    set_motors(test_speed, 0, test_speed, 0)
    time.sleep(test_duration)
    stop_all()
    
    print("\nМоторы остановлены.")
    while True:
        try:
            rotations_str = input("Сколько полных (и дробных, через точку) оборотов сделало колесо? (например, 7.5): ").strip()
            if not rotations_str: continue
            rotations = float(rotations_str)
            break
        except ValueError:
            print("Ошибка: введите число!")
            
    # Расчет RPM для тестовой скорости
    test_rpm = (rotations / test_duration) * 60.0
    # Пропорциональный расчет для максимальной скорости (4095)
    MAX_WHEEL_RPM = test_rpm * (4095.0 / float(test_speed))
    
    print(f"-> Скорость вращения при PWM={test_speed} составила: {test_rpm:.1f} RPM")
    print(f"-> Расчетный MAX_WHEEL_RPM (при PWM=4095) установлен на: {MAX_WHEEL_RPM:.1f} RPM")
    
    config["MAX_WHEEL_RPM"] = MAX_WHEEL_RPM
    save_config(config)
    print("Калибровка успешно завершена и сохранена в config.json!\n")


def autonomous_loop(driver, speed, detector=None):
    print("\n=== ВНИМАНИЕ: ВКЛЮЧЕН УМНЫЙ АВТОПИЛОТ ===")
    print("Робот будет исследовать местность, учитывая свои габариты!")
    time.sleep(2) 
    
    global current_mode, current_speed
    state = "FORWARD"
    
    SAFE_DIST_FRONT = 0.67 # Увеличили дистанцию остановки перед стеной еще на 2см
    
    try:
        while driver.running:
            # --- ЛОГИКА СБОРА МУСОРА (YOLO) ---
            scan = driver.get_latest_scan()
            
            if detector and detector.trash_detected and state not in ["TRASH_APPROACH", "TRASH_COLLECT"]:
                print(f"[АВТОПИЛОТ] МУСОР ОБНАРУЖЕН (Угол: {detector.trash_angle:.1f})! Начинаю сближение.")
                state = "TRASH_APPROACH"
                
            if state == "TRASH_APPROACH":
                dist = get_lidar_distance(scan, detector.trash_angle)
                print(f"[АВТОПИЛОТ] Сближение... Дистанция по лидару: {dist:.2f}м, Угол: {detector.trash_angle:.1f}°")
                
                # Если мусор слишком близко или потерян из виду вблизи (слепая зона)
                if dist < 0.15 or (not detector.trash_detected and dist < 0.4):
                    print("[АВТОПИЛОТ] Мусор в зоне захвата!")
                    state = "TRASH_COLLECT"
                elif not detector.trash_detected and dist >= 0.4:
                    print("[АВТОПИЛОТ] Ложное срабатывание или мусор утерян вдали. Возврат.")
                    state = "FORWARD"
                else:
                    # Подруливание (используем 50% скорости для плавности)
                    if detector.trash_angle > 10:
                        set_motors(speed//2, 0, 0, speed//2) # Вправо
                    elif detector.trash_angle < -10:
                        set_motors(0, speed//2, speed//2, 0) # Влево
                    else:
                        set_motors(speed//2, 0, speed//2, 0) # Прямо
                time.sleep(0.1)
                continue
                
            elif state == "TRASH_COLLECT":
                stop_all()
                time.sleep(0.5) # Даем моторам полностью остановиться перед сменой частоты
                print("[АВТОПИЛОТ] Запускаю ковш!")
                run_bucket_collect_cycle()
                print("[АВТОПИЛОТ] Мусор собран! Возврат к исследованию.")
                state = "FORWARD"
                if detector:
                    detector.trash_detected = False
                continue

            # --- ЛОГИКА ИССЛЕДОВАТЕЛЯ С ЛИДАРОМ ---
            
            clearance_front = get_clearance(scan, 0)
            
            if state == "FORWARD":
                if clearance_front < SAFE_DIST_FRONT:
                    stop_all()
                    time.sleep(0.2)
                    
                    # Ищем лучшее направление (шаг 15 градусов)
                    best_angle = 0
                    max_clear = 0.0
                    
                    for ang in range(-180, 180, 15):
                        if abs(ang) < 30: continue # Прямо и так заблокировано
                        c = get_clearance(scan, ang)
                        if c > max_clear:
                            max_clear = c
                            best_angle = ang
                            
                    print(f"[АВТОПИЛОТ] Препятствие! Лучший коридор на {best_angle}° (свободно {max_clear:.2f}м)")
                    
                    if max_clear < SAFE_DIST_FRONT:
                        state = "REVERSE"
                        print("[АВТОПИЛОТ] Тупик! Включаю задний ход.")
                    elif best_angle < 0:
                        state = "TURN_RIGHT"
                    else:
                        state = "TURN_LEFT"
                else:
                    # Едем прямо
                    with movement_lock:
                        current_mode = 1
                        current_speed = speed
                    set_motors(speed, 0, speed, 0)
                    
            elif state == "TURN_LEFT":
                with movement_lock:
                    current_mode = 3
                    current_speed = speed
                set_motors(speed, 0, 0, speed)
                if get_clearance(scan, 0) > 0.8:
                    stop_all()
                    time.sleep(0.2)
                    state = "FORWARD"
                    
            elif state == "TURN_RIGHT":
                with movement_lock:
                    current_mode = 4
                    current_speed = speed
                set_motors(0, speed, speed, 0)
                if get_clearance(scan, 0) > 0.8:
                    stop_all()
                    time.sleep(0.2)
                    state = "FORWARD"
                    
            elif state == "REVERSE":
                with movement_lock:
                    current_mode = 2
                    current_speed = speed
                set_motors(0, speed, 0, speed)
                
                back_scan = [d for d in scan[150:210] if d > 0.0]
                min_back = min(back_scan) if back_scan else 8.0
                
                if min_back < 0.4 or clearance_front > 0.8:
                    stop_all()
                    time.sleep(0.2)
                    state = "TURN_LEFT" # Случайный выбор для выхода из угла
                
            time.sleep(0.1) 
            
    except KeyboardInterrupt:
        print("\nАвтопилот прерван пользователем!")
    finally:
        stop_all()

def legacy_main_unused():
    print("=== Система управления + FastSLAM + YOLO ===")
    
    # Синхронизируем локальный и глобальный конфиги
    config = global_config
    """Legacy prompt removed: Arduino port must be asked separately from lidar.
        f"Введите порт Arduino ковша (Enter для {config.get('arduino_port', '/dev/ttyACM0')}): "
    ).strip()
    if lidar_port:
        config["lidar_port"] = lidar_port
    """
    # Arduino ковша инициализируем после отдельного явного запроса порта ниже.
    
    # При запуске сразу ставим ковш в нулевое (транспортное) положение
    print("Установка ковша в нулевое положение...")
    # Ковш переводим в ноль после явного выбора порта Arduino ниже.
    
    # Запрос настроек у пользователя
    lidar_port = input(f"Введите порт лидара (Enter для {config.get('lidar_port', '/dev/ttyUSB0')}): ").strip()
    if not lidar_port: lidar_port = config.get('lidar_port', '/dev/ttyUSB0')
    config['lidar_port'] = lidar_port
    arduino_port = prompt_for_serial_port(
        "Arduino ковша",
        saved_port=config.get('arduino_port', '/dev/ttyACM0'),
        forbidden_ports={lidar_port},
    )
    config["arduino_port"] = arduino_port
    init_bucket_arduino(config)
    print("Установка ковша в нулевое положение...")
    set_servo_bucket(down=True)
        
    print("\nГде показывать карту SLAM?")
    print("1 - Показывать в Tailscale (через SSH с пробросом X11 или VNC)")
    print("2 - Показывать на экране RPi на Armbian (физический монитор)")
    print("3 - Не показывать вообще (максимальная скорость)")
    map_choice = input("Ваш выбор (1, 2 или 3): ").strip()
    
    if map_choice == '2':
        os.environ["DISPLAY"] = ":0"
        show_map = True
    elif map_choice == '1':
        show_map = True
    else:
        show_map = False
    
    print("\nГде запускать нейросеть YOLO для сбора мусора?")
    print("1 - На телефоне / ПК (Максимальная скорость, по Wi-Fi)")
    print("2 - Локально на Raspberry Pi (Низкий FPS)")
    print("3 - Отключить сбор мусора")
    yolo_choice = input("Ваш выбор (1, 2 или 3): ").strip()
    
    detector = None
    if yolo_choice == '1' and RemoteTrashListener:
        detector = RemoteTrashListener(
            on_servo_command=handle_remote_servo_command,
            on_motor_command=handle_remote_bucket_motor_command,
        )
        detector.start()
        print("\n[ВНИМАНИЕ] На телефоне (в Pydroid 3) запустите скрипт `yolo_client.py`.")
        
    elif yolo_choice == '2' and TrashDetector:
        models_dir = "models"
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)
            
        available_models = os.listdir(models_dir)
        selected_model = None
        
        if not available_models:
            print("\n[YOLO] В папке 'models' пусто! Пожалуйста, скопируйте туда ваши модели (папки NCNN).")
        else:
            print("\nДоступные модели в папке 'models':")
            for i, m in enumerate(available_models):
                print(f"{i+1} - {m}")
            
            try:
                m_idx = int(input(f"Выберите модель (1-{len(available_models)}): ").strip()) - 1
                if 0 <= m_idx < len(available_models):
                    selected_model = os.path.join(models_dir, available_models[m_idx])
            except ValueError:
                print("Ошибка ввода.")
                
        if selected_model:
            detector = TrashDetector(model_path=selected_model)
            detector.start()
        else:
            print("[YOLO] Модель не выбрана, детектор мусора отключен.")
            
    else:
        print("[YOLO] Сбор мусора отключен.")
    
    print("\nВыберите режим работы:")
    print("1 - Ручной (управление с клавиатуры)")
    print("2 - Автопилот (обход препятствий и сбор мусора)")
    mode_choice = input("Ваш выбор (1 или 2): ").strip()
    
    auto_speed = config.get('auto_speed', 1500)
    if mode_choice == '2':
        speed_str = input(f"Введите скорость автопилота от 0 до 4095 (Enter для {auto_speed}): ").strip()
        if speed_str.isdigit():
            auto_speed = int(speed_str)
        config['auto_speed'] = auto_speed
        
    save_config(config)
    
    # Сначала выполняем калибровку
    calibrate_motors(config)
    
    # Запуск драйвера лидара
    driver = LD06Driver(port=lidar_port)
    driver.start()
    
    if not driver.running:
        print("Ошибка: Лидар не запущен. Убедитесь что он подключен к /dev/ttyUSB0.")
        # Можно позволить продолжить без лидара для тестов, но лучше выйти
        # sys.exit(1)
        
    # Запуск потока SLAM
    slam_thread = threading.Thread(target=slam_thread_function, args=(driver, show_map), daemon=True)
    slam_thread.start()

    if mode_choice == '2':
        # Запуск автопилота
        autonomous_loop(driver, auto_speed, detector)
    else:
        # Ручной режим
        print("\nФормат ввода: 'Режим Скорость' (например, '1 1000').")
        print("Режимы: 1-Вперед, 2-Назад, 3-Влево, 4-Вправо, 0-Остановка")
        print("Скорость: 0 - 4095. Нажмите Ctrl+C для ЭКСТРЕННОЙ ОСТАНОВКИ и выхода.")
        print("Также для быстрой остановки просто нажмите ENTER (пустой ввод) или любую букву.\n")
        
        stop_all()

        try:
            while True:
                cmd = input("Введите команду: ").strip()
                
                # Экстренная остановка при пустом вводе (просто удар по Enter) или вводе буквы 'e'
                if not cmd or cmd.lower() in ['e', 's', 'stop']:
                    stop_all()
                    print("ЭКСТРЕННАЯ ОСТАНОВКА!")
                    continue
                    
                try:
                    parts = cmd.split()
                    mode = int(parts[0])
                    
                    if mode == 0:
                        stop_all()
                        print("Моторы остановлены.")
                        continue
                        
                    speed = int(parts[1]) if len(parts) > 1 else 1000
                    
                    with movement_lock:
                        global current_mode, current_speed
                        current_mode = mode
                        current_speed = speed
                    
                    if mode == 1:
                        print(f"Движение ВПЕРЕД на скорости {speed}")
                        set_motors(speed, 0, speed, 0)
                    elif mode == 2:
                        print(f"Движение НАЗАД на скорости {speed}")
                        set_motors(0, speed, 0, speed)
                    elif mode == 3:
                        print(f"Поворот ВЛЕВО на скорости {speed}")
                        set_motors(speed, 0, 0, speed)
                    elif mode == 4:
                        print(f"Поворот ВПРАВО на скорости {speed}")
                        set_motors(0, speed, speed, 0)
                    else:
                        print("Неизвестный режим! Остановка.")
                        stop_all()
                        
                except ValueError:
                    # Экстренная остановка при случайном вводе любых символов
                    print("Ошибка ввода (введены не числа). ЭКСТРЕННАЯ ОСТАНОВКА!")
                    stop_all()

        except KeyboardInterrupt:
            print("\nВыход из ручного режима...")
            
    # Завершение работы
    stop_all()
    driver.stop()
    close_bucket_arduino()

def main():
    print("=== Система управления + FastSLAM + YOLO ===")

    config = global_config

    lidar_port = prompt_for_serial_port(
        "лидара",
        saved_port=config.get("lidar_port", "/dev/ttyUSB0"),
    )
    config["lidar_port"] = lidar_port

    arduino_port = prompt_for_serial_port(
        "Arduino ковша",
        saved_port=config.get("arduino_port", "/dev/ttyACM0"),
        forbidden_ports={lidar_port},
    )
    config["arduino_port"] = arduino_port
    config["servo_up_angle"] = 0
    config["servo_down_angle"] = 90
    config["bucket_wall_manual_speed"] = 4095
    config["bucket_motor_collect_speed"] = 4095
    config["bucket_motor_reverse_speed"] = -4095
    config["bucket_wall_detect_speed"] = 4095
    config["bucket_wall_max_speed"] = 4095
    camera_port = prompt_for_camera_port(config.get("camera_port", "/dev/video0"))
    config["camera_port"] = camera_port
    start_video_streamer(config)
    save_config(config)

    init_bucket_arduino(config)
    calibrate_bucket_wall(config)

    print("\nУстановка ковша в поисковое положение...")
    move_bucket_wall_to_search_position()
    set_servo_bucket(down=True)

    print("\nГде показывать карту SLAM?")
    print("1 - Показывать в Tailscale (через SSH с X11 или VNC)")
    print("2 - Показывать на экране RPi на Armbian")
    print("3 - Не показывать вообще")
    map_choice = input("Ваш выбор (1, 2 или 3): ").strip()

    if not map_choice:
        map_choice = str(config.get("map_choice", "3"))
    if map_choice not in {"1", "2", "3"}:
        map_choice = str(config.get("map_choice", "3"))
    config["map_choice"] = map_choice

    if map_choice == "2":
        os.environ["DISPLAY"] = ":0"
        show_map = True
    elif map_choice == "1":
        show_map = True
    else:
        show_map = False

    print("\nГде запускать нейросеть YOLO для сбора мусора?")
    print("1 - На телефоне / ПК (максимальная скорость, по Wi-Fi)")
    print("2 - Локально на Raspberry Pi (низкий FPS)")
    print("3 - Отключить сбор мусора")
    yolo_choice = input("Ваш выбор (1, 2 или 3): ").strip()

    if not yolo_choice:
        yolo_choice = str(config.get("yolo_choice", "1"))
    if yolo_choice not in {"1", "2", "3"}:
        yolo_choice = str(config.get("yolo_choice", "1"))
    config["yolo_choice"] = yolo_choice

    detector = None
    if yolo_choice == "1" and RemoteTrashListener:
        detector = RemoteTrashListener(
            on_servo_command=handle_remote_servo_command,
            on_motor_command=handle_remote_bucket_motor_command,
        )
        detector.start()
        print("\n[ВНИМАНИЕ] На телефоне (в Pydroid 3) запустите скрипт `yolo_client.py`.")
    elif yolo_choice == "2" and TrashDetector:
        models_dir = "models"
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)

        available_models = os.listdir(models_dir)
        selected_model = None

        if not available_models:
            print("\n[YOLO] В папке 'models' пусто. Скопируйте туда ваши модели (папки NCNN).")
        else:
            print("\nДоступные модели в папке 'models':")
            for i, model_name in enumerate(available_models):
                print(f"{i + 1} - {model_name}")

            try:
                model_index = int(input(f"Выберите модель (1-{len(available_models)}): ").strip()) - 1
                if 0 <= model_index < len(available_models):
                    selected_model = os.path.join(models_dir, available_models[model_index])
            except ValueError:
                print("Ошибка ввода.")

        if selected_model:
            detector = TrashDetector(model_path=selected_model)
            detector.start()
        else:
            print("[YOLO] Модель не выбрана, детектор мусора отключен.")
    else:
        print("[YOLO] Сбор мусора отключен.")

    print("\nВыберите режим работы:")
    print("1 - Ручной (управление с клавиатуры)")
    print("2 - Автопилот (обход препятствий и сбор мусора)")
    mode_choice = input("Ваш выбор (1 или 2): ").strip()

    if not mode_choice:
        mode_choice = str(config.get("run_mode", "2"))
    if mode_choice not in {"1", "2"}:
        print("[РЕЖИМ] Неизвестный режим, включаю автопилот по умолчанию.")
        mode_choice = "2"
    config["run_mode"] = mode_choice

    if detector and hasattr(detector, "set_allow_text_commands"):
        detector.set_allow_text_commands(mode_choice != "2")

    auto_speed = config.get("auto_speed", 1500)
    if mode_choice == "2":
        speed_str = input(f"Введите скорость автопилота от 0 до 4095 (Enter для {auto_speed}): ").strip()
        if speed_str.isdigit():
            auto_speed = int(speed_str)
        config["auto_speed"] = auto_speed

    save_config(config)
    calibrate_motors(config)

    driver = LD06Driver(port=lidar_port)
    driver.start()

    if not driver.running:
        print("Ошибка: лидар не запущен. Убедитесь, что он подключен к выбранному порту.")

    slam_thread = threading.Thread(target=slam_thread_function, args=(driver, show_map), daemon=True)
    slam_thread.start()

    try:
        if mode_choice == "2":
            autonomous_loop(driver, auto_speed, detector)
        else:
            print("\nФормат ввода: 'Режим Скорость' (например, '1 1000').")
            print("Режимы: 1-Вперед, 2-Назад, 3-Влево, 4-Вправо, 0-Остановка")
            print("Скорость: 0 - 4095. Enter или stop - экстренная остановка.\n")

            print("Manual bucket commands: wall_up, wall_down, scoop_up, scoop_down, bucket_test, collect")
            stop_all()

            while True:
                cmd = input("Введите команду: ").strip()

                if not cmd or cmd.lower() in ["e", "s", "stop"]:
                    stop_all()
                    print("ЭКСТРЕННАЯ ОСТАНОВКА!")
                    continue

                lowered_cmd = cmd.lower()
                if lowered_cmd in ["wall_up", "lift_up", "raise_wall"]:
                    move_bucket_wall_to_search_position()
                    continue
                if lowered_cmd in ["wall_down", "lift_down", "lower_wall"]:
                    move_bucket_wall_to_lower_position()
                    continue
                if lowered_cmd in ["scoop_up", "bucket_up", "servo_up"]:
                    set_servo_bucket(down=False, wait=True)
                    continue
                if lowered_cmd in ["scoop_down", "bucket_down", "servo_down"]:
                    set_servo_bucket(down=True, wait=True)
                    continue
                if lowered_cmd in ["bucket_test", "wall_test", "timed_test"]:
                    run_bucket_wall_timed_test()
                    continue
                if lowered_cmd in ["collect", "bucket_collect"]:
                    run_bucket_collect_cycle()
                    continue

                try:
                    parts = cmd.split()
                    mode = int(parts[0])

                    if mode == 0:
                        stop_all()
                        print("Моторы остановлены.")
                        continue

                    speed = int(parts[1]) if len(parts) > 1 else 1000

                    with movement_lock:
                        global current_mode, current_speed
                        current_mode = mode
                        current_speed = speed

                    if mode == 1:
                        print(f"Движение ВПЕРЕД на скорости {speed}")
                        set_motors(speed, 0, speed, 0)
                    elif mode == 2:
                        print(f"Движение НАЗАД на скорости {speed}")
                        set_motors(0, speed, 0, speed)
                    elif mode == 3:
                        print(f"Поворот ВЛЕВО на скорости {speed}")
                        set_motors(speed, 0, 0, speed)
                    elif mode == 4:
                        print(f"Поворот ВПРАВО на скорости {speed}")
                        set_motors(0, speed, speed, 0)
                    else:
                        print("Неизвестный режим! Остановка.")
                        stop_all()
                except ValueError:
                    print("Ошибка ввода. ЭКСТРЕННАЯ ОСТАНОВКА!")
                    stop_all()
    except KeyboardInterrupt:
        print("\nЗавершение работы по запросу пользователя...")
    finally:
        stop_all()
        driver.stop()
        close_bucket_arduino()
        stop_video_streamer()

def start_video_streamer(config):
    global video_streamer_process

    if video_streamer_process and video_streamer_process.poll() is None:
        print("[VIDEO] Видеостример уже запущен.")
        return

    camera_port = config.get("camera_port", "/dev/video0")
    stream_host = config.get("camera_stream_host", "0.0.0.0")
    stream_port = int(config.get("camera_stream_port", 5000))
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "video_streamer.py")

    command = [
        sys.executable,
        script_path,
        "--camera",
        str(camera_port),
        "--host",
        stream_host,
        "--port",
        str(stream_port),
    ]

    try:
        video_streamer_process = subprocess.Popen(command)
        time.sleep(1.0)
        if video_streamer_process.poll() is None:
            print(f"[VIDEO] Видеостример запущен для {camera_port} на порту {stream_port}.")
        else:
            print("[VIDEO] Видеостример завершился сразу после запуска. Проверьте порт камеры.")
    except Exception as e:
        print(f"[VIDEO] Не удалось запустить видеостример: {e}")
        video_streamer_process = None

def stop_video_streamer():
    global video_streamer_process

    if not video_streamer_process:
        return

    try:
        if video_streamer_process.poll() is None:
            video_streamer_process.terminate()
            video_streamer_process.wait(timeout=3)
    except Exception:
        try:
            video_streamer_process.kill()
        except Exception:
            pass
    finally:
        video_streamer_process = None

if __name__ == "__main__":
    main()
