import time
import board
import busio
from adafruit_pca9685 import PCA9685
import threading
import sys
import math
import json
import os
import glob
import subprocess
import tempfile

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
config_runtime_error = ""


def _set_config_runtime_error(message):
    global config_runtime_error
    config_runtime_error = str(message or "").strip()


def get_config_status():
    return {
        "path": CONFIG_FILE,
        "error": config_runtime_error,
    }


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                _set_config_runtime_error("")
                return data
            _set_config_runtime_error("config.json must contain a JSON object.")
        except Exception as e:
            _set_config_runtime_error(f"Failed to load config.json: {e}")
    return {}

def save_config(cfg):
    global global_config
    sanitized = dict(cfg)
    sanitized.pop("arduino_port", None)
    sanitized.pop("arduino_baudrate", None)
    sanitized.pop("arduino_timeout_sec", None)
    sanitized.pop("arduino_boot_wait_sec", None)
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        fd, temp_path = tempfile.mkstemp(
            prefix="config.",
            suffix=".json.tmp",
            dir=os.path.dirname(CONFIG_FILE),
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(sanitized, f, indent=4, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(temp_path, CONFIG_FILE)
        except Exception:
            try:
                os.unlink(temp_path)
            except OSError:
                pass
            raise
        _set_config_runtime_error("")
    except Exception as e:
        _set_config_runtime_error(f"Failed to save config.json: {e}")
        print(f"[CONFIG] Failed to save config.json: {e}")
    global_config = dict(sanitized)

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
        print(f"\nР”РѕСЃС‚СѓРїРЅС‹Рµ РїРѕСЂС‚С‹ РґР»СЏ {label}:")
        for port in candidates:
            print(f" - {port}")

    while True:
        prompt = f"Р’РІРµРґРёС‚Рµ РїРѕСЂС‚ {label}"
        if saved_port:
            prompt += f" (Enter РґР»СЏ {saved_port})"
        prompt += ": "

        port = input(prompt).strip()
        if not port and saved_port:
            port = saved_port

        if not port:
            print(f"РџРѕСЂС‚ РґР»СЏ {label} РѕР±СЏР·Р°С‚РµР»РµРЅ.")
            continue

        if port in forbidden_ports:
            print(f"РџРѕСЂС‚ {port} СѓР¶Рµ Р·Р°РЅСЏС‚ РґСЂСѓРіРёРј СѓСЃС‚СЂРѕР№СЃС‚РІРѕРј. Р”Р»СЏ {label} РЅСѓР¶РµРЅ РѕС‚РґРµР»СЊРЅС‹Р№ РїРѕСЂС‚.")
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

# Override the prompt so camera address can be changed every start and can`r`n# be entered either as /dev/videoX or as a USB address like 001:010.
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

MOTOR_PCA_DEFAULT_ADDRESS = 0x40
SERVO_PCA_DEFAULT_ADDRESS = 0x42
MOTOR_PCA_FREQUENCY = 1000
SERVO_PCA_FREQUENCY = 50
BUCKET_SERVO_DEFAULT_CHANNEL = 0
BUCKET_SERVO_MIN_PULSE_US = 500
BUCKET_SERVO_MAX_PULSE_US = 2500

i2c = None
pca = None
servo_pca = None
pca_runtime_signature = None
i2c_scan_runtime_error = ""
I2C_SCAN_LOCK_TIMEOUT_SEC = 0.35


def _parse_i2c_address(value, default):
    if value in (None, ""):
        return int(default)
    if isinstance(value, str):
        text = value.strip().lower()
        if not text:
            return int(default)
        base = 16 if text.startswith("0x") else 10
        return int(text, base)
    return int(value)


def _set_i2c_scan_runtime_error(message):
    global i2c_scan_runtime_error
    i2c_scan_runtime_error = str(message or "").strip()


def get_i2c_scan_status():
    return {
        "error": i2c_scan_runtime_error,
    }


def scan_i2c_addresses(lock_timeout_sec=I2C_SCAN_LOCK_TIMEOUT_SEC):
    global i2c
    try:
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)
        deadline = time.time() + max(0.05, float(lock_timeout_sec))
        while not i2c.try_lock():
            if time.time() >= deadline:
                raise TimeoutError("I2C bus lock timeout")
            time.sleep(0.05)
        try:
            addresses = sorted(int(addr) for addr in i2c.scan())
        finally:
            i2c.unlock()
        _set_i2c_scan_runtime_error("")
        return addresses
    except Exception as e:
        _set_i2c_scan_runtime_error(f"I2C scan failed: {e}")
        return []


def get_available_i2c_addresses():
    return [hex(addr) for addr in scan_i2c_addresses()]


def _select_pca_addresses(config=None):
    config = config or global_config
    available = scan_i2c_addresses()
    pca_candidates = [addr for addr in available if addr != 0x70]

    drive_address = None
    servo_address = None
    try:
        drive_address = _parse_i2c_address(
            config.get("drive_pca_address", MOTOR_PCA_DEFAULT_ADDRESS),
            MOTOR_PCA_DEFAULT_ADDRESS,
        )
    except (TypeError, ValueError):
        drive_address = None

    try:
        servo_address = _parse_i2c_address(
            config.get("servo_pca_address", SERVO_PCA_DEFAULT_ADDRESS),
            SERVO_PCA_DEFAULT_ADDRESS,
        )
    except (TypeError, ValueError):
        servo_address = None

    if drive_address not in pca_candidates and pca_candidates:
        drive_address = pca_candidates[0]

    if servo_address not in pca_candidates or servo_address == drive_address:
        servo_address = None
        for candidate in pca_candidates:
            if candidate != drive_address:
                servo_address = candidate
                break

    if drive_address is None:
        drive_address = MOTOR_PCA_DEFAULT_ADDRESS
    if servo_address is None:
        servo_address = SERVO_PCA_DEFAULT_ADDRESS

    return drive_address, servo_address


def _get_drive_pca_address(config=None):
    drive_address, _ = _select_pca_addresses(config)
    return drive_address


def _get_servo_pca_address(config=None):
    _, servo_address = _select_pca_addresses(config)
    return servo_address


def _get_bucket_servo_channel(config=None):
    config = config or global_config
    try:
        channel = int(config.get("bucket_servo_channel", BUCKET_SERVO_DEFAULT_CHANNEL))
    except (TypeError, ValueError):
        channel = BUCKET_SERVO_DEFAULT_CHANNEL
    return max(0, min(15, channel))


def _get_bucket_servo_min_pulse_us(config=None):
    config = config or global_config
    try:
        value = int(config.get("bucket_servo_min_pulse_us", BUCKET_SERVO_MIN_PULSE_US))
    except (TypeError, ValueError):
        value = BUCKET_SERVO_MIN_PULSE_US
    return max(200, min(3000, value))


def _get_bucket_servo_max_pulse_us(config=None):
    config = config or global_config
    try:
        value = int(config.get("bucket_servo_max_pulse_us", BUCKET_SERVO_MAX_PULSE_US))
    except (TypeError, ValueError):
        value = BUCKET_SERVO_MAX_PULSE_US
    return max(_get_bucket_servo_min_pulse_us(config) + 100, min(3200, value))


def _deinit_pca_board(board_instance):
    if not board_instance:
        return
    try:
        board_instance.deinit()
    except Exception:
        pass


def init_pca_controllers(config=None):
    global i2c, pca, servo_pca, pca_runtime_signature

    config = config or global_config
    drive_address = _get_drive_pca_address(config)
    servo_address = _get_servo_pca_address(config)
    runtime_signature = (drive_address, servo_address)

    if pca and servo_pca and pca_runtime_signature == runtime_signature:
        return True

    if drive_address == servo_address:
        print(
            f"[I2C] drive_pca_address ({hex(drive_address)}) and "
            f"servo_pca_address ({hex(servo_address)}) must be different."
        )
        pca = None
        servo_pca = None
        pca_runtime_signature = None
        return False

    try:
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)

        _deinit_pca_board(pca)
        _deinit_pca_board(servo_pca)

        motor_board = PCA9685(i2c, address=drive_address)
        motor_board.frequency = MOTOR_PCA_FREQUENCY

        servo_board = PCA9685(i2c, address=servo_address)
        servo_board.frequency = SERVO_PCA_FREQUENCY

        pca = motor_board
        servo_pca = servo_board
        pca_runtime_signature = runtime_signature
        print(
            f"[I2C] PCA9685 ready: drive={hex(drive_address)}@{MOTOR_PCA_FREQUENCY}Hz, "
            f"servo={hex(servo_address)}@{SERVO_PCA_FREQUENCY}Hz."
        )
        return True
    except Exception as e:
        print(f"I2C/PCA9685 initialization failed (normal on PC): {e}")
        pca = None
        servo_pca = None
        pca_runtime_signature = None
        return False

# РРЅРёС†РёР°Р»РёР·Р°С†РёСЏ I2C С€РёРЅС‹ Рё PCA9685
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 1000
except Exception as e:
    print(f"I2C/PCA9685 РёРЅРёС†РёР°Р»РёР·Р°С†РёСЏ РЅРµ СѓРґР°Р»Р°СЃСЊ (РµСЃР»Рё РІС‹ РЅР° РџРљ, СЌС‚Рѕ РЅРѕСЂРјР°Р»СЊРЅРѕ): {e}")
    pca = None

LEFT_FWD_CHANNELS = [0, 3, 5]
LEFT_REV_CHANNELS = [1, 2, 4]
RIGHT_FWD_CHANNELS = [7, 9, 11]
RIGHT_REV_CHANNELS = [6, 8, 10]

init_pca_controllers(global_config)

# Р“Р»РѕР±Р°Р»СЊРЅС‹Рµ РїРµСЂРµРјРµРЅРЅС‹Рµ СЃРѕСЃС‚РѕСЏРЅРёСЏ РґРІРёР¶РµРЅРёСЏ РґР»СЏ РїСЃРµРІРґРѕ-РѕРґРѕРјРµС‚СЂРёРё
DRIVE_CHANNELS = set(LEFT_FWD_CHANNELS + LEFT_REV_CHANNELS + RIGHT_FWD_CHANNELS + RIGHT_REV_CHANNELS)
BUCKET_MOTOR_DEFAULT_FWD_CHANNEL = 12
BUCKET_MOTOR_DEFAULT_REV_CHANNEL = 13

bucket_servo_lock = threading.Lock()
bucket_servo_release_timer = None
bucket_servo_current_angle = None
bucket_arduino = None
bucket_arduino_lock = threading.Lock()
bucket_arduino_running = False
bucket_arduino_reader_thread = None
last_bucket_pot_value = None
bucket_channel_warning_shown = False
video_streamer_process = None
video_streamer_last_error = ""
video_streamer_last_source = ""
bucket_wall_increase_direction_sign = None
bucket_motor_current_speed = 0

current_mode = 0
current_speed = 0
movement_lock = threading.Lock()
slam_state_lock = threading.Lock()
latest_slam_pose = None
slam_route_recording_active = False
slam_route_last_saved_pose = None

# РџР°СЂР°РјРµС‚СЂС‹ СЂРѕР±РѕС‚Р° (РѕР±РЅРѕРІР»РµРЅС‹ РїРѕ РІР°С€РёРј СЂР°Р·РјРµСЂР°Рј)
WHEEL_CIRCUMFERENCE = 0.39 # Р”Р»РёРЅР° РѕРєСЂСѓР¶РЅРѕСЃС‚Рё РєРѕР»РµСЃР° РІ РјРµС‚СЂР°С… (39 СЃРј)
MAX_WHEEL_RPM = 120.0      # Р—РЅР°С‡РµРЅРёРµ РїРѕ СѓРјРѕР»С‡Р°РЅРёСЋ, Р±СѓРґРµС‚ РїРµСЂРµР·Р°РїРёСЃР°РЅРѕ РїСЂРё РєР°Р»РёР±СЂРѕРІРєРµ
ROBOT_TRACK_WIDTH = 0.55   # РЁРёСЂРёРЅР° РєРѕР»РµСЃРЅРѕР№ Р±Р°Р·С‹ (55 СЃРј)
# Р¦РµРЅС‚СЂ СЂРѕР±РѕС‚Р° = (0, 0). Р—Р°РґРЅРёР№ РїСЂР°РІС‹Р№ СѓРіРѕР» = X: -0.31Рј, Y: -0.275Рј. 
# Р›РёРґР°СЂ РЅР° Р±Р°Р»РєРµ 14СЃРј РѕС‚ Р·Р°РґРЅРµРіРѕ СѓРіР»Р° -> X: -0.31 + 0.14 = -0.17Рј, Y: -0.275Рј
LIDAR_OFFSET_X = -0.17     # РЎРјРµС‰РµРЅРёРµ Р»РёРґР°СЂР° РїРѕ РѕСЃРё X (РЅР°Р·Р°Рґ РѕС‚ С†РµРЅС‚СЂР°)
LIDAR_OFFSET_Y = -0.275    # РЎРјРµС‰РµРЅРёРµ Р»РёРґР°СЂР° РїРѕ РѕСЃРё Y (РІРїСЂР°РІРѕ РѕС‚ С†РµРЅС‚СЂР°)

def set_motors(left_fwd, left_rev, right_fwd, right_rev):
    if not pca and not init_pca_controllers(global_config):
        return
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
    return set_servo_bucket(down=down, wait=True)
    if not pca or not servo: 
        print("[РЎР•Р Р’РћРџР РР’РћР”] РћС€РёР±РєР°: PCA9685 РёР»Рё Р±РёР±Р»РёРѕС‚РµРєР° adafruit_motor РЅРµ РЅР°Р№РґРµРЅС‹!")
        return
        
    # Р’СЂРµРјРµРЅРЅРѕ РїРµСЂРµРєР»СЋС‡Р°РµРј РІСЃСЋ РїР»Р°С‚Сѓ РЅР° 50 Р“С† РґР»СЏ СѓРїСЂР°РІР»РµРЅРёСЏ СЃРµСЂРІРѕРїСЂРёРІРѕРґРѕРј.
    # РўР°Рє РєР°Рє РјРѕС‚РѕСЂС‹ СЃРµР№С‡Р°СЃ РѕСЃС‚Р°РЅРѕРІР»РµРЅС‹ (PWM=0), СЌС‚Рѕ Р±РµР·РѕРїР°СЃРЅРѕ.
    pca.frequency = 50
    
    CHANNEL = 14
    servo_motor = servo.Servo(pca.channels[CHANNEL], min_pulse=500, max_pulse=2500)
    
    state = "РћРџРЈРЎРљРђР®" if down else "РџРћР”РќРРњРђР®"
    print(f"[РЎР•Р Р’РћРџР РР’РћР” - РљР°РЅР°Р» {CHANNEL}] {state} РєРѕРІС€! (Р§Р°СЃС‚РѕС‚Р° РїРµСЂРµРєР»СЋС‡РµРЅР° РЅР° 50 Р“С†)")
    
    # Р§РёС‚Р°РµРј РѕС‚РєР°Р»РёР±СЂРѕРІР°РЅРЅС‹Рµ СѓРіР»С‹ РёР· РєРѕРЅС„РёРіСѓСЂР°С†РёРё
    up_angle = global_config.get("servo_up_angle", 0)
    down_angle = global_config.get("servo_down_angle", 90)
    
    # Р—РЅР°С‡РµРЅРёСЏ СѓРіР»РѕРІ Р±РµСЂРµРј РёР· С„Р°Р№Р»Р° РєРѕРЅС„РёРіСѓСЂР°С†РёРё!
    if down:
        servo_motor.angle = down_angle  # РљРѕРІС€ РѕРїСѓС‰РµРЅ (РїРѕР»РѕР¶РµРЅРёРµ РґР»СЏ СЃР±РѕСЂР°)
    else:
        servo_motor.angle = up_angle    # РљРѕРІС€ РїРѕРґРЅСЏС‚ (РЅСѓР»РµРІРѕРµ/С‚СЂР°РЅСЃРїРѕСЂС‚РЅРѕРµ РїРѕР»РѕР¶РµРЅРёРµ)
        
    # Р–РґРµРј РїРѕРєР° РєРѕРІС€ С„РёР·РёС‡РµСЃРєРё РѕРїСѓСЃС‚РёС‚СЃСЏ/РїРѕРґРЅРёРјРµС‚СЃСЏ
    time.sleep(1.0)
    
    # РћС‚РєР»СЋС‡Р°РµРј РЁРРњ СЃРёРіРЅР°Р» РЅР° СЃРµСЂРІРѕРїСЂРёРІРѕРґ, С‡С‚РѕР±С‹ РѕРЅ РЅРµ РґСЂРѕР¶Р°Р» Рё РЅРµ С‚СЂР°С‚РёР» С‚РѕРє
    pca.channels[CHANNEL].duty_cycle = 0
    
    # Р’РѕР·РІСЂР°С‰Р°РµРј С‡Р°СЃС‚РѕС‚Сѓ РѕР±СЂР°С‚РЅРѕ РЅР° 1000 Р“С† РґР»СЏ РїСЂР°РІРёР»СЊРЅРѕР№ СЂР°Р±РѕС‚С‹ РјРѕС‚РѕСЂРѕРІ РєРѕР»РµСЃ
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
                f"[РљРћР’РЁ] РќРµР±РµР·РѕРїР°СЃРЅР°СЏ РєРѕРЅС„РёРіСѓСЂР°С†РёСЏ РєР°РЅР°Р»РѕРІ bucket motor: {fwd}/{rev}. "
                "РћРЅРё РїРµСЂРµСЃРµРєР°СЋС‚СЃСЏ СЃ С…РѕРґРѕРІС‹РјРё РјРѕС‚РѕСЂР°РјРё, РїРѕСЌС‚РѕРјСѓ РјРѕС‚РѕСЂ РєРѕРІС€Р° РѕС‚РєР»СЋС‡РµРЅ РґРѕ РёСЃРїСЂР°РІР»РµРЅРёСЏ config.json."
            )
            bucket_channel_warning_shown = True
        return None, None
    return fwd, rev

def legacy_init_bucket_arduino_unused(config):
    global bucket_arduino, bucket_arduino_running, bucket_arduino_reader_thread

    if bucket_arduino:
        return True

    if not serial:
        print("[ARDUINO] pyserial РЅРµ СѓСЃС‚Р°РЅРѕРІР»РµРЅ, СѓРїСЂР°РІР»РµРЅРёРµ СЃРµСЂРІРѕР№ РєРѕРІС€Р° С‡РµСЂРµР· Arduino РЅРµРґРѕСЃС‚СѓРїРЅРѕ.")
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
        print(f"[ARDUINO] РџРѕРґРєР»СЋС‡РµРЅ РєРѕРЅС‚СЂРѕР»Р»РµСЂ РєРѕРІС€Р° РЅР° {arduino_port} ({baudrate} Р±РѕРґ).")
        return True
    except Exception as e:
        print(f"[ARDUINO] РќРµ СѓРґР°Р»РѕСЃСЊ РїРѕРґРєР»СЋС‡РёС‚СЊСЃСЏ Рє Arduino РЅР° {arduino_port}: {e}")
        bucket_arduino = None
        bucket_arduino_running = False
        return False

def _bucket_arduino_reader_loop():
    global bucket_arduino_running

    while bucket_arduino_running and bucket_arduino:
        try:
            line = bucket_arduino.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("OK:") or line.startswith("ERR:"):
                print(f"[ARDUINO] {line}")
        except Exception as e:
            if bucket_arduino_running:
                print(f"[ARDUINO] РћС€РёР±РєР° С‡С‚РµРЅРёСЏ Serial: {e}")
            time.sleep(0.2)

def legacy_close_bucket_arduino_unused():
    global bucket_arduino, bucket_arduino_running

    bucket_arduino_running = False
    if bucket_arduino:
        try:
            bucket_arduino.close()
        except Exception:
            pass
    bucket_arduino = None

def legacy_send_bucket_arduino_command_unused(command):
    if not bucket_arduino:
        print(f"[ARDUINO] РљРѕРјР°РЅРґР° '{command}' РїСЂРѕРїСѓС‰РµРЅР°: Arduino РЅРµ РїРѕРґРєР»СЋС‡РµРЅ.")
        return False

    try:
        with bucket_arduino_lock:
            bucket_arduino.write(f"{command}\n".encode("utf-8"))
            bucket_arduino.flush()
        return True
    except Exception as e:
        print(f"[ARDUINO] РќРµ СѓРґР°Р»РѕСЃСЊ РѕС‚РїСЂР°РІРёС‚СЊ РєРѕРјР°РЅРґСѓ '{command}': {e}")
        return False

def legacy_move_bucket_servo_to_angle_unused(target_angle, label, wait=True):
    target_angle = max(0, min(180, int(target_angle)))
    if not send_bucket_arduino_command(f"SERVO:{target_angle}"):
        return False

    print(f"[РЎР•Р Р’РћРџР РР’РћР” - ARDUINO] {label} РєРѕРІС€ РґРѕ {target_angle}В°.")
    if wait:
        time.sleep(float(global_config.get("bucket_servo_move_sec", 0.8)))
    return True

def legacy_set_servo_bucket_via_arduino_unused(down=True, wait=True):
    up_angle = global_config.get("servo_up_angle", 0)
    down_angle = global_config.get("servo_down_angle", 90)
    target_angle = down_angle if down else up_angle
    label = "РћРџРЈРЎРљРђР®" if down else "РџРћР”РќРРњРђР®"
    return _move_bucket_servo_to_angle(target_angle, label, wait=wait)

def is_bucket_servo_controller_ready():
    return servo_pca is not None


def init_bucket_servo_controller(config):
    return init_pca_controllers(config) and servo_pca is not None


def _release_bucket_servo_signal():
    global bucket_servo_release_timer

    bucket_servo_release_timer = None
    if not servo_pca:
        return

    try:
        servo_pca.channels[_get_bucket_servo_channel()].duty_cycle = 0
    except Exception:
        pass


def _cancel_bucket_servo_release():
    global bucket_servo_release_timer

    timer = bucket_servo_release_timer
    bucket_servo_release_timer = None
    if timer:
        timer.cancel()


def _schedule_bucket_servo_release(delay_sec):
    global bucket_servo_release_timer

    _cancel_bucket_servo_release()
    timer = threading.Timer(max(0.05, float(delay_sec)), _release_bucket_servo_signal)
    timer.daemon = True
    bucket_servo_release_timer = timer
    timer.start()


def close_bucket_servo_controller():
    _cancel_bucket_servo_release()
    _release_bucket_servo_signal()


def _bucket_servo_angle_to_duty_cycle(angle, config=None):
    config = config or global_config
    angle = max(0.0, min(180.0, float(angle)))
    min_pulse_us = _get_bucket_servo_min_pulse_us(config)
    max_pulse_us = _get_bucket_servo_max_pulse_us(config)
    pulse_us = min_pulse_us + ((max_pulse_us - min_pulse_us) * (angle / 180.0))
    return max(0, min(65535, int((pulse_us / 20000.0) * 65535)))


def _write_bucket_servo_angle(angle):
    global bucket_servo_current_angle

    angle = max(0, min(180, int(angle)))
    servo_pca.channels[_get_bucket_servo_channel()].duty_cycle = _bucket_servo_angle_to_duty_cycle(angle)
    bucket_servo_current_angle = angle


def init_bucket_arduino(config):
    return init_bucket_servo_controller(config)


def close_bucket_arduino():
    close_bucket_servo_controller()


def send_bucket_arduino_command(command):
    command = str(command).strip().upper()
    if not command.startswith("SERVO:"):
        print(f"[SERVO] Unsupported legacy servo command: {command}")
        return False
    try:
        angle = float(command.split(":", 1)[1])
    except ValueError:
        print(f"[SERVO] Invalid legacy servo command: {command}")
        return False
    return _move_bucket_servo_to_angle(angle, "MOVING", wait=False)


def _move_bucket_servo_to_angle(target_angle, label, wait=True):
    global bucket_servo_current_angle

    if not init_bucket_servo_controller(global_config):
        print("[SERVO] Bucket servo PCA9685 is not available.")
        return False

    target_angle = max(0, min(180, int(target_angle)))
    move_sec = max(0.05, float(global_config.get("bucket_servo_move_sec", 0.8)))
    step_deg = max(1, int(global_config.get("bucket_servo_step_deg", 3)))
    step_delay_sec = max(0.005, float(global_config.get("bucket_servo_step_delay_sec", 0.02)))

    with bucket_servo_lock:
        _cancel_bucket_servo_release()

        start_angle = bucket_servo_current_angle
        if wait and start_angle is not None and start_angle != target_angle:
            direction = 1 if target_angle > start_angle else -1
            for angle in range(start_angle, target_angle, direction * step_deg):
                _write_bucket_servo_angle(angle)
                time.sleep(step_delay_sec)

        _write_bucket_servo_angle(target_angle)
        _schedule_bucket_servo_release(move_sec + 0.1)

    print(
        f"[SERVO - PCA {hex(_get_servo_pca_address())} CH{_get_bucket_servo_channel()}] "
        f"{label} bucket to {target_angle} deg."
    )
    if wait:
        time.sleep(move_sec)
    return True


def set_servo_bucket(down=True, wait=True):
    up_angle = global_config.get("servo_up_angle", 0)
    down_angle = global_config.get("servo_down_angle", 90)
    target_angle = down_angle if down else up_angle
    label = "LOWERING" if down else "RAISING"
    return _move_bucket_servo_to_angle(target_angle, label, wait=wait)


def set_bucket_motor(speed):
    global bucket_motor_current_speed
    if not pca and not init_pca_controllers(global_config):
        print("[РљРћР’РЁ] PCA9685 РЅРµРґРѕСЃС‚СѓРїРЅР°, РјРѕС‚РѕСЂ РєРѕРІС€Р° РЅРµ СѓРїСЂР°РІР»СЏРµС‚СЃСЏ.")
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
    bucket_motor_current_speed = speed
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
            print("[РљРћР’РЁ] РћРїСѓСЃРєР°СЋ РєРѕРІС€ РїРµСЂРµРґ Р·Р°С…РІР°С‚РѕРј РјСѓСЃРѕСЂР°.")
            set_servo_bucket(down=True, wait=True)
            time.sleep(lower_pause_sec)

        print("[РљРћР’РЁ] РџРѕРґРЅРёРјР°СЋ РєРѕРІС€ Рё Р·Р°РїСѓСЃРєР°СЋ СЃС‚РµРЅРєСѓ РєРѕРІС€Р° РѕРґРЅРѕРІСЂРµРјРµРЅРЅРѕ.")
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
        _move_bucket_servo_to_angle(angle, "РЈРЎРўРђРќРђР’Р›РР’РђР®", wait=False)
    except ValueError:
        print(f"[REMOTE] РќРµРёР·РІРµСЃС‚РЅР°СЏ РєРѕРјР°РЅРґР° СЃРµСЂРІРѕРїСЂРёРІРѕРґСѓ: {command}")

def legacy_handle_remote_bucket_motor_pulse_unused(command):
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
        print(f"[REMOTE] РќРµРёР·РІРµСЃС‚РЅР°СЏ РєРѕРјР°РЅРґР° РјРѕС‚РѕСЂСѓ РєРѕРІС€Р°: {command}")

def get_bucket_wall_position(wait_timeout_sec=1.0):
    deadline = time.time() + wait_timeout_sec
    while time.time() < deadline:
        if last_bucket_pot_value is not None:
            return int(last_bucket_pot_value)
        time.sleep(0.05)
    return int(last_bucket_pot_value) if last_bucket_pot_value is not None else None

def _ramp_bucket_motor(target_speed, ramp_sec=None, steps=None):
    global bucket_motor_current_speed

    ramp_sec = float(
        ramp_sec if ramp_sec is not None else global_config.get("bucket_motor_ramp_sec", 0.22)
    )
    steps = max(
        1,
        int(steps if steps is not None else global_config.get("bucket_motor_ramp_steps", 6)),
    )
    start_speed = int(bucket_motor_current_speed)
    target_speed = int(max(-4095, min(4095, target_speed)))

    if ramp_sec <= 0 or steps <= 1 or start_speed == target_speed:
        set_bucket_motor(target_speed)
        return

    step_delay = ramp_sec / steps
    for step_index in range(1, steps + 1):
        ratio = step_index / steps
        interpolated = int(round(start_speed + (target_speed - start_speed) * ratio))
        set_bucket_motor(interpolated)
        time.sleep(step_delay)


def pulse_bucket_motor(speed, duration_sec=None):
    duration_sec = float(
        duration_sec if duration_sec is not None else global_config.get("bucket_motor_pulse_sec", 0.18)
    )
    ramp_sec = float(global_config.get("bucket_motor_ramp_sec", 0.22))
    settle_sec = float(global_config.get("bucket_motor_settle_after_pulse_sec", 0.12))
    before = None
    after = None

    try:
        _ramp_bucket_motor(speed, ramp_sec=ramp_sec)
        hold_sec = max(0.0, duration_sec - ramp_sec)
        if hold_sec > 0:
            time.sleep(hold_sec)
    finally:
        _ramp_bucket_motor(0, ramp_sec=ramp_sec)

    if settle_sec > 0:
        time.sleep(settle_sec)
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

    print("[РЎРўР•РќРљРђ] РќРµ СѓРґР°Р»РѕСЃСЊ РѕРїСЂРµРґРµР»РёС‚СЊ РЅР°РїСЂР°РІР»РµРЅРёРµ РёР·РјРµРЅРµРЅРёСЏ РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂР°. РџСЂРѕРІРµСЂСЊС‚Рµ РїРѕРґРєР»СЋС‡РµРЅРёРµ.")
    return None

def move_bucket_wall_to_position(target_value, label="РЎС‚РµРЅРєР° РєРѕРІС€Р°", timeout_sec=None):
    increase_direction = ensure_bucket_wall_direction()
    if increase_direction not in (-1, 1):
        return False

    if target_value is None:
        print(f"[РЎРўР•РќРљРђ] Р¦РµР»РµРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ '{label}' РЅРµ Р·Р°РґР°РЅРѕ.")
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
            print(f"[РЎРўР•РќРљРђ] РќРµС‚ РґР°РЅРЅС‹С… СЃ РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂР° РґР»СЏ '{label}'.")
            break

        error = int(target_value) - current_value
        if abs(error) <= tolerance:
            set_bucket_motor(0)
            print(f"[РЎРўР•РќРљРђ] '{label}' РґРѕСЃС‚РёРіРЅСѓС‚Рѕ: {current_value}.")
            return True

        direction = increase_direction if error > 0 else -increase_direction
        speed = int(min(max_speed, max(min_speed, abs(error) * kp)))
        set_bucket_motor(direction * speed)
        time.sleep(0.05)

    set_bucket_motor(0)
    print(f"[РЎРўР•РќРљРђ] РќРµ СѓРґР°Р»РѕСЃСЊ РґРѕРІРµСЃС‚Рё '{label}' РґРѕ С†РµР»Рё {target_value}. РџРѕСЃР»РµРґРЅРµРµ Р·РЅР°С‡РµРЅРёРµ: {get_bucket_wall_position(0.1)}.")
    return False

def run_bucket_wall_timed_test(duration_sec=None, speed=None):
    duration_sec = float(
        duration_sec if duration_sec is not None else global_config.get("bucket_wall_manual_calibration_pulse_sec", 2.0)
    )
    speed = abs(int(speed if speed is not None else global_config.get("bucket_wall_manual_speed", 4095)))

    print(f"[Р РЋР СћР вЂўР СњР С™Р С’] Р СћР ВµРЎРѓРЎвЂљ: Р С—Р С•Р Т‘Р Р…Р С‘Р СР В°РЎР‹ РЎРѓР С•Р Р†Р С•Р С” Р Р…Р В°Р В·Р В°Р Т‘ Р Р…Р В° {duration_sec:.1f} РЎРѓР ВµР С”.")
    before_up, after_up = pulse_bucket_motor(-speed, duration_sec)
    _remember_bucket_wall_direction(-speed, before_up, after_up)

    time.sleep(0.2)

    print(f"[Р РЋР СћР вЂўР СњР С™Р С’] Р СћР ВµРЎРѓРЎвЂљ: Р С•Р С—РЎС“РЎРѓР С”Р В°РЎР‹ РЎРѓР С•Р Р†Р С•Р С” Р Р†Р С—Р ВµРЎР‚Р ВµР Т‘ Р Р…Р В° {duration_sec:.1f} РЎРѓР ВµР С”.")
    before_down, after_down = pulse_bucket_motor(speed, duration_sec)
    _remember_bucket_wall_direction(speed, before_down, after_down)

    return True

def calibrate_bucket_wall(config):
    search_position = config.get("bucket_wall_search_pot")
    lower_position = config.get("bucket_wall_lower_pot")
    default_answer = "y" if search_position is None or lower_position is None else "n"

    answer = input(
        f"\nРҐРѕС‚РёС‚Рµ РѕС‚РєР°Р»РёР±СЂРѕРІР°С‚СЊ СЃС‚РµРЅРєСѓ РєРѕРІС€Р° РїРѕ РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂСѓ? (y/n, Enter={default_answer}): "
    ).strip().lower()
    if not answer:
        answer = default_answer

    if answer != "y":
        if search_position is None or lower_position is None:
            print("[РЎРўР•РќРљРђ] РљР°Р»РёР±СЂРѕРІРєР° РѕР±СЏР·Р°С‚РµР»СЊРЅР°: РЅРµ Р·Р°РґР°РЅС‹ РїРѕРёСЃРєРѕРІРѕРµ Рё РЅРёР¶РЅРµРµ РїРѕР»РѕР¶РµРЅРёСЏ.")
            return calibrate_bucket_wall(config)
        return True

    print("\n=== РљРђР›РР‘Р РћР’РљРђ РЎРўР•РќРљР РљРћР’РЁРђ ===")
    print("РљРѕРјР°РЅРґС‹:")
    print("  j - РєРѕСЂРѕС‚РєРёР№ С€Р°Рі РјРѕС‚РѕСЂРѕРј РІ РѕРґРЅСѓ СЃС‚РѕСЂРѕРЅСѓ")
    print("  k - РєРѕСЂРѕС‚РєРёР№ С€Р°Рі РјРѕС‚РѕСЂРѕРј РІ РґСЂСѓРіСѓСЋ СЃС‚РѕСЂРѕРЅСѓ")
    print("  p - РїРѕРєР°Р·Р°С‚СЊ С‚РµРєСѓС‰РµРµ Р·РЅР°С‡РµРЅРёРµ РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂР°")
    print("  s - СЃРѕС…СЂР°РЅРёС‚СЊ С‚РµРєСѓС‰РµРµ Р·РЅР°С‡РµРЅРёРµ РєР°Рє РџРћРРЎРљРћР’РћР• РїРѕР»РѕР¶РµРЅРёРµ")
    print("  l - СЃРѕС…СЂР°РЅРёС‚СЊ С‚РµРєСѓС‰РµРµ Р·РЅР°С‡РµРЅРёРµ РєР°Рє РќРР–РќР•Р• РїРѕР»РѕР¶РµРЅРёРµ")
    print("  q - Р·Р°РІРµСЂС€РёС‚СЊ РєР°Р»РёР±СЂРѕРІРєСѓ")

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
        print(f"\n[РЎРўР•РќРљРђ] РўРµРєСѓС‰РµРµ Р·РЅР°С‡РµРЅРёРµ РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂР°: {current_value}")
        command = input("РљР°Р»РёР±СЂРѕРІРєР° СЃС‚РµРЅРєРё > ").strip().lower()

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
                print("[РЎРўР•РќРљРђ] РќРµС‚ СЃРёРіРЅР°Р»Р° РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂР°.")
                continue
            search_position = current_value
            config["bucket_wall_search_pot"] = search_position
            save_config(config)
            print(f"[РЎРўР•РќРљРђ] РџРѕРёСЃРєРѕРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ СЃРѕС…СЂР°РЅРµРЅРѕ: {search_position}.")
        elif command == "l":
            if current_value is None:
                print("[РЎРўР•РќРљРђ] РќРµС‚ СЃРёРіРЅР°Р»Р° РїРѕС‚РµРЅС†РёРѕРјРµС‚СЂР°.")
                continue
            lower_position = current_value
            config["bucket_wall_lower_pot"] = lower_position
            save_config(config)
            print(f"[РЎРўР•РќРљРђ] РќРёР¶РЅРµРµ РїРѕР»РѕР¶РµРЅРёРµ СЃРѕС…СЂР°РЅРµРЅРѕ: {lower_position}.")
        elif command == "q":
            if search_position is None or lower_position is None:
                print("[РЎРўР•РќРљРђ] РќСѓР¶РЅРѕ СЃРѕС…СЂР°РЅРёС‚СЊ Рё РїРѕРёСЃРєРѕРІРѕРµ, Рё РЅРёР¶РЅРµРµ РїРѕР»РѕР¶РµРЅРёРµ.")
                continue
            break
        else:
            print("РќРµРёР·РІРµСЃС‚РЅР°СЏ РєРѕРјР°РЅРґР°.")

    ensure_bucket_wall_direction()
    return True

def move_bucket_wall_to_search_position():
    return move_bucket_wall_to_position(global_config.get("bucket_wall_search_pot"), "РїРѕРёСЃРєРѕРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ СЃС‚РµРЅРєРё")

def move_bucket_wall_to_lower_position():
    return move_bucket_wall_to_position(global_config.get("bucket_wall_lower_pot"), "РЅРёР¶РЅРµРµ РїРѕР»РѕР¶РµРЅРёРµ СЃС‚РµРЅРєРё")

def run_bucket_collect_cycle():
    servo_lower_pause_sec = float(global_config.get("bucket_servo_lower_pause_sec", 0.2))
    wall_lower_pause_sec = float(global_config.get("bucket_wall_lower_pause_sec", 0.2))
    servo_join_timeout_sec = float(global_config.get("bucket_servo_move_sec", 0.8)) + 0.8
    settle_after_sec = float(global_config.get("bucket_collect_settle_sec", 0.2))

    try:
        print("[РљРћР’РЁ] РћРїСѓСЃРєР°СЋ СЃРµСЂРІСѓ РєРѕРІС€Р°.")
        set_servo_bucket(down=True, wait=True)
        time.sleep(servo_lower_pause_sec)

        print("[РљРћР’РЁ] РћРїСѓСЃРєР°СЋ СЃС‚РµРЅРєСѓ РєРѕРІС€Р° Рє РјСѓСЃРѕСЂСѓ.")
        move_bucket_wall_to_lower_position()
        time.sleep(wall_lower_pause_sec)

        print("[РљРћР’РЁ] РџРѕРґРЅРёРјР°СЋ СЃС‚РµРЅРєСѓ Рё СЃРµСЂРІСѓ РІРјРµСЃС‚Рµ, С‡С‚РѕР±С‹ Р·Р°РєРёРЅСѓС‚СЊ РјСѓСЃРѕСЂ РЅР°Р·Р°Рґ.")
        servo_thread = threading.Thread(target=set_servo_bucket, kwargs={"down": False, "wait": True}, daemon=True)
        servo_thread.start()
        move_bucket_wall_to_search_position()
        servo_thread.join(timeout=servo_join_timeout_sec)
        set_servo_bucket(down=True, wait=True)
    finally:
        set_bucket_motor(0)
        time.sleep(settle_after_sec)

def legacy_handle_remote_bucket_motor_position_unused(command):
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
        print(f"[REMOTE] РќРµРёР·РІРµСЃС‚РЅР°СЏ РєРѕРјР°РЅРґР° РјРѕС‚РѕСЂСѓ РєРѕРІС€Р°: {command}")

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
    print(f"[BUCKET] {label}: {duration_sec:.1f}s with smooth ramp.")
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
    config["bucket_motor_ramp_sec"] = 0.28
    config["bucket_motor_ramp_steps"] = 7

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
        print(f"[REMOTE] Unknown bucket motor command: {command}")

def stop_all():
    set_motors(0, 0, 0, 0)
    set_bucket_motor(0)
    with movement_lock:
        global current_mode, current_speed
        current_mode = 0
        current_speed = 0


def _normalize_angle_rad(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _extract_best_slam_pose(slam):
    try:
        best_particle = max(slam.pf.particles, key=lambda particle: particle.weight)
        if best_particle.xTrajectory and best_particle.yTrajectory:
            theta = 0.0
            if hasattr(best_particle, "prevMatchedReading") and best_particle.prevMatchedReading:
                theta = float(best_particle.prevMatchedReading.get("theta", 0.0))
            return {
                "x": float(best_particle.xTrajectory[-1]),
                "y": float(best_particle.yTrajectory[-1]),
                "theta": theta,
                "time": time.time(),
            }
    except Exception:
        pass

    return {
        "x": float(getattr(slam, "robot_odom_x", 0.0)),
        "y": float(getattr(slam, "robot_odom_y", 0.0)),
        "theta": float(getattr(slam, "robot_odom_theta", 0.0)),
        "time": time.time(),
    }


def get_current_slam_pose():
    with slam_state_lock:
        if latest_slam_pose is None:
            return None
        return dict(latest_slam_pose)


def _get_slam_route_points():
    route_points = global_config.get("slam_route_points", [])
    if isinstance(route_points, list):
        return route_points
    return []


def _save_slam_route_points(route_points):
    global global_config
    global_config["slam_route_points"] = route_points
    save_config(global_config)


def _distance_between_pose_points(a, b):
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def start_slam_route_recording(clear_existing=True):
    global slam_route_recording_active, slam_route_last_saved_pose, global_config

    if clear_existing:
        clear_slam_route()

    with slam_state_lock:
        slam_route_recording_active = True
        current_pose = dict(latest_slam_pose) if latest_slam_pose else None

    slam_route_last_saved_pose = None
    global_config["route_source_mode"] = "slam"
    global_config["slam_route_source_mode"] = "slam"
    save_config(global_config)

    if current_pose is not None:
        _append_slam_route_point_if_needed(current_pose, force=True)

    return True


def stop_slam_route_recording():
    global slam_route_recording_active
    with slam_state_lock:
        slam_route_recording_active = False
    return True


def clear_slam_route():
    global slam_route_last_saved_pose
    slam_route_last_saved_pose = None
    _save_slam_route_points([])
    return True


def _append_slam_route_point_if_needed(pose, force=False):
    global slam_route_last_saved_pose

    route_points = list(_get_slam_route_points())
    min_step_m = float(global_config.get("slam_route_record_step_m", 0.35))
    point = {
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "theta": float(pose.get("theta", 0.0)),
        "time": float(pose.get("time", time.time())),
    }

    if not force and route_points:
        last_point = route_points[-1]
        if _distance_between_pose_points(last_point, point) < min_step_m:
            return False

    route_points.append(point)
    slam_route_last_saved_pose = point
    _save_slam_route_points(route_points)
    return True


def _distance_point_to_segment(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    ab_len_sq = (abx * abx) + (aby * aby)
    if ab_len_sq <= 1e-9:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))
    closest_x = ax + abx * t
    closest_y = ay + aby * t
    return math.hypot(px - closest_x, py - closest_y)


def get_slam_route_distance(pose=None):
    if pose is None:
        pose = get_current_slam_pose()
    if pose is None:
        return None

    route_points = _get_slam_route_points()
    if not route_points:
        return None

    if len(route_points) == 1:
        return _distance_between_pose_points(pose, route_points[0])

    px = float(pose["x"])
    py = float(pose["y"])
    best_distance = None

    for index in range(len(route_points) - 1):
        a = route_points[index]
        b = route_points[index + 1]
        distance = _distance_point_to_segment(
            px,
            py,
            float(a["x"]),
            float(a["y"]),
            float(b["x"]),
            float(b["y"]),
        )
        if best_distance is None or distance < best_distance:
            best_distance = distance

    return best_distance


def get_slam_route_guidance(pose=None):
    if pose is None:
        pose = get_current_slam_pose()
    if pose is None:
        return None

    route_points = _get_slam_route_points()
    if len(route_points) < 2:
        return None

    px = float(pose["x"])
    py = float(pose["y"])
    best_index = 0
    best_distance = None
    for index, point in enumerate(route_points):
        distance = math.hypot(px - float(point["x"]), py - float(point["y"]))
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_index = index

    target_index = min(best_index + 2, len(route_points) - 1)
    target = route_points[target_index]
    dx = float(target["x"]) - px
    dy = float(target["y"]) - py
    target_heading = math.atan2(dy, dx)
    heading_error = _normalize_angle_rad(target_heading - float(pose.get("theta", 0.0)))
    target_distance = math.hypot(dx, dy)

    return {
        "nearest_index": best_index,
        "target_index": target_index,
        "target_distance": target_distance,
        "heading_error": heading_error,
        "corridor_distance": get_slam_route_distance(pose),
    }


def get_route_source_mode():
    route_source_mode = global_config.get("route_source_mode")
    if route_source_mode is None:
        route_source_mode = global_config.get("slam_route_source_mode", "none")

    route_source_mode = str(route_source_mode).strip().lower()
    if route_source_mode not in {"none", "gps", "slam"}:
        route_source_mode = "none"
    return route_source_mode


def get_route_corridor_m():
    try:
        corridor_m = float(global_config.get("route_corridor_m", 3.0))
    except (TypeError, ValueError):
        corridor_m = 3.0
    return max(0.1, corridor_m)


def get_route_state(detector=None):
    route_source_mode = get_route_source_mode()
    corridor_m = get_route_corridor_m()
    state = {
        "source_mode": route_source_mode,
        "corridor_m": corridor_m,
        "route_enabled": False,
        "within_corridor": True,
        "distance_m": None,
        "guidance": None,
        "slam_pose": None,
        "slam_route_points": len(_get_slam_route_points()),
        "route_fresh": False,
    }

    if route_source_mode == "gps":
        route_enabled = bool(detector and getattr(detector, "route_mode_enabled", False))
        route_fresh = bool(detector and getattr(detector, "route_data_fresh", False))
        within_corridor = bool(detector and getattr(detector, "within_route_corridor", False))
        route_distance = getattr(detector, "route_distance_m", None) if detector else None
        state.update(
            {
                "route_enabled": route_enabled,
                "within_corridor": (not route_enabled) or (route_fresh and within_corridor),
                "distance_m": route_distance,
                "route_fresh": route_fresh,
            }
        )
        return state

    if route_source_mode == "slam":
        pose = get_current_slam_pose()
        route_points = _get_slam_route_points()
        route_enabled = len(route_points) >= 2
        route_distance = get_slam_route_distance(pose) if route_enabled else None
        guidance = get_slam_route_guidance(pose) if route_enabled else None
        within_corridor = route_distance is None or route_distance <= corridor_m
        state.update(
            {
                "route_enabled": route_enabled,
                "within_corridor": within_corridor,
                "distance_m": route_distance,
                "guidance": guidance,
                "slam_pose": pose,
                "slam_route_points": len(route_points),
                "route_fresh": pose is not None,
            }
        )
        return state

    return state


def get_slam_route_status():
    pose = get_current_slam_pose()
    with slam_state_lock:
        recording = slam_route_recording_active

    route_points = _get_slam_route_points()
    route_distance = get_slam_route_distance(pose) if route_points else None
    route_guidance = get_slam_route_guidance(pose) if len(route_points) >= 2 else None
    return {
        "recording": recording,
        "points_count": len(route_points),
        "pose": pose,
        "distance_m": route_distance,
        "guidance": route_guidance,
    }


def _cleanup_completed_trash_targets(completed_targets):
    now = time.time()
    expired_ids = [
        target_id
        for target_id, expires_at in completed_targets.items()
        if expires_at <= now
    ]
    for target_id in expired_ids:
        completed_targets.pop(target_id, None)
    return set(completed_targets.keys())


def _remember_completed_trash_target(
    completed_targets,
    target_id,
    cooldown_sec=8.0,
):
    if target_id is None:
        return
    try:
        target_id = int(target_id)
    except (TypeError, ValueError):
        return
    completed_targets[target_id] = time.time() + max(1.0, float(cooldown_sec))


def _get_detector_target(detector, preferred_id=None, exclude_ids=None):
    if detector is None:
        return None

    excluded = set(exclude_ids or [])

    if preferred_id is not None and hasattr(detector, "get_target_by_id"):
        target = detector.get_target_by_id(preferred_id)
        if target is not None:
            return target

    if hasattr(detector, "get_primary_target"):
        return detector.get_primary_target(exclude_ids=excluded)

    if getattr(detector, "trash_detected", False):
        fallback_target_id = getattr(detector, "current_target_id", 1) or 1
        if fallback_target_id in excluded:
            return None
        return {
            "id": fallback_target_id,
            "angle": float(getattr(detector, "trash_angle", 0.0)),
            "in_collection_zone": bool(
                getattr(detector, "trash_in_collection_zone", False)
            ),
            "confidence": float(getattr(detector, "trash_confidence", 0.0)),
            "confirmed_for_ms": int(
                getattr(detector, "trash_confirmed_for_ms", 0) or 0
            ),
        }

    return None


def slam_thread_function(driver, show_map):
    global latest_slam_pose
    slam = OnlineFastSlam(show_map=show_map)
    last_time = time.time()
    
    print("SLAM РїРѕС‚РѕРє Р·Р°РїСѓС‰РµРЅ.")
    while driver.running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # РџРѕР»СѓС‡Р°РµРј РґР°РЅРЅС‹Рµ СЃ Р»РёРґР°СЂР°
        scan = driver.get_latest_scan()
        
        # Р Р°СЃС‡РµС‚ РїСЃРµРІРґРѕ-РѕРґРѕРјРµС‚СЂРёРё
        with movement_lock:
            mode = current_mode
            speed = current_speed
            
        dx = 0.0
        dy = 0.0
        dtheta = 0.0
        
        if mode != 0 and speed > 0:
            # Р Р°СЃС‡РµС‚ Р»РёРЅРµР№РЅРѕР№ СЃРєРѕСЂРѕСЃС‚Рё (Рј/СЃ)
            # speed / 4095.0 - РґРѕР»СЏ РѕС‚ РјР°РєСЃРёРјР°Р»СЊРЅРѕР№ СЃРєРѕСЂРѕСЃС‚Рё
            rpm = MAX_WHEEL_RPM * (speed / 4095.0)
            rps = rpm / 60.0 # РѕР±РѕСЂРѕС‚РѕРІ РІ СЃРµРєСѓРЅРґСѓ
            linear_velocity = WHEEL_CIRCUMFERENCE * rps
            
            if mode == 1: # Р’РїРµСЂРµРґ
                dx = linear_velocity * dt
            elif mode == 2: # РќР°Р·Р°Рґ
                dx = -linear_velocity * dt
            elif mode == 3: # Р’Р»РµРІРѕ (СЂР°Р·РІРѕСЂРѕС‚ РЅР° РјРµСЃС‚Рµ)
                # РЈРіР»РѕРІР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ = (V_right - V_left) / TRACK_WIDTH
                # РџСЂР°РІС‹Рµ РІРїРµСЂРµРґ (V), Р»РµРІС‹Рµ РЅР°Р·Р°Рґ (-V)
                v_right = linear_velocity
                v_left = -linear_velocity
                angular_velocity = (v_right - v_left) / ROBOT_TRACK_WIDTH
                dtheta = angular_velocity * dt
            elif mode == 4: # Р’РїСЂР°РІРѕ
                # Р›РµРІС‹Рµ РІРїРµСЂРµРґ (V), РїСЂР°РІС‹Рµ РЅР°Р·Р°Рґ (-V)
                v_right = -linear_velocity
                v_left = linear_velocity
                angular_velocity = (v_right - v_left) / ROBOT_TRACK_WIDTH
                dtheta = angular_velocity * dt

        # РћР±РЅРѕРІР»СЏРµРј SLAM (РїРµСЂРµРґР°РµРј С‚Р°РєР¶Рµ СЃРјРµС‰РµРЅРёРµ Р»РёРґР°СЂР° РїРѕ X Рё Y)
        slam.process_scan(scan, dx, dy, dtheta, LIDAR_OFFSET_X, LIDAR_OFFSET_Y)
        
        # РЎРїРёРј С‡С‚РѕР±С‹ SLAM СЂР°Р±РѕС‚Р°Р» РїСЂРёРјРµСЂРЅРѕ 5-10 Р“С†
        current_pose = _extract_best_slam_pose(slam)
        with slam_state_lock:
            latest_slam_pose = current_pose
            route_recording_active = slam_route_recording_active
        if route_recording_active:
            _append_slam_route_point_if_needed(current_pose)
        time.sleep(0.1)

def get_lidar_distance(scan, target_angle_deg, cone_half_angle=5):
    """
    РС‰РµС‚ РјРёРЅРёРјР°Р»СЊРЅСѓСЋ РґРёСЃС‚Р°РЅС†РёСЋ РІ Р·Р°РґР°РЅРЅРѕРј СЃРµРєС‚РѕСЂРµ (target_angle_deg В± cone_half_angle).
    Р’РѕР·РІСЂР°С‰Р°РµС‚ РґРёСЃС‚Р°РЅС†РёСЋ РІ РјРµС‚СЂР°С… РёР»Рё 999.0 РµСЃР»Рё РЅРµС‚ РґР°РЅРЅС‹С….
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

def get_clearance(scan, target_angle_deg, robot_half_width=0.42): # РЈРІРµР»РёС‡РёР»Рё Р·Р°Р·РѕСЂ СЃ Р±РѕРєРѕРІ РµС‰Рµ РЅР° 2СЃРј
    """
    Р’С‹С‡РёСЃР»СЏРµС‚ СЃРІРѕР±РѕРґРЅСѓСЋ РґРёСЃС‚Р°РЅС†РёСЋ РІ РЅР°РїСЂР°РІР»РµРЅРёРё target_angle_deg,
    СѓС‡РёС‚С‹РІР°СЏ С€РёСЂРёРЅСѓ СЂРѕР±РѕС‚Р° (robot_half_width).
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
        
        # Р•СЃР»Рё С‚РѕС‡РєР° РЅР°С…РѕРґРёС‚СЃСЏ "РІРїРµСЂРµРґРё" РЅР° СЌС‚РѕРј Р»СѓС‡Рµ Рё РІ РїСЂРµРґРµР»Р°С… С€РёСЂРёРЅС‹ СЂРѕР±РѕС‚Р°
        if x_offset > 0 and y_offset < robot_half_width:
            if x_offset < min_clearance:
                min_clearance = x_offset
    return min_clearance

def calibrate_motors(config):
    global MAX_WHEEL_RPM
    if "MAX_WHEEL_RPM" in config:
        MAX_WHEEL_RPM = config["MAX_WHEEL_RPM"]
        print(f"\n[РљРѕРЅС„РёРі] Р—Р°РіСЂСѓР¶РµРЅ MAX_WHEEL_RPM = {MAX_WHEEL_RPM:.1f}")
        
    print("\n=== РљРђР›РР‘Р РћР’РљРђ РњРћРўРћР РћР’ ===")
    ans = input("РҐРѕС‚РёС‚Рµ РІС‹РїРѕР»РЅРёС‚СЊ РєР°Р»РёР±СЂРѕРІРєСѓ РјРѕС‚РѕСЂРѕРІ РґР»СЏ С‚РѕС‡РЅРѕР№ РѕРґРѕРјРµС‚СЂРёРё? (y/n): ").strip().lower()
    if ans != 'y':
        print(f"РљР°Р»РёР±СЂРѕРІРєР° РїСЂРѕРїСѓС‰РµРЅР°. РСЃРїРѕР»СЊР·СѓРµС‚СЃСЏ MAX_WHEEL_RPM: {MAX_WHEEL_RPM}")
        return

    print("\nР’РќРРњРђРќРР•: РџРѕРґРЅРёРјРёС‚Рµ СЂРѕР±РѕС‚Р° РЅР°Рґ Р·РµРјР»РµР№ (РїРѕСЃС‚Р°РІСЊС‚Рµ РЅР° РїРѕРґСЃС‚Р°РІРєСѓ), С‡С‚РѕР±С‹ РєРѕР»РµСЃР° РєСЂСѓС‚РёР»РёСЃСЊ РІ РІРѕР·РґСѓС…Рµ!")
    input("РќР°Р¶РјРёС‚Рµ ENTER, РєРѕРіРґР° Р±СѓРґРµС‚Рµ РіРѕС‚РѕРІС‹ РЅР°С‡Р°С‚СЊ РєР°Р»РёР±СЂРѕРІРєСѓ...")
    
    test_speed = 2000
    test_duration = 5.0 # РЎРµРєСѓРЅРґС‹
    
    print(f"\nР—Р°РїСѓСЃРє РјРѕС‚РѕСЂРѕРІ РІРїРµСЂРµРґ СЃРѕ СЃРєРѕСЂРѕСЃС‚СЊСЋ {test_speed} РЅР° {test_duration} СЃРµРєСѓРЅРґ...")
    print("Р’РќРРњРђРўР•Р›Р¬РќРћ РЎР§РРўРђР™РўР• РћР‘РћР РћРўР« РљРћР›Р•РЎРђ (РјРѕР¶РЅРѕ СЃС‡РёС‚Р°С‚СЊ РѕР±РѕСЂРѕС‚С‹ РѕРґРЅРѕРіРѕ Р»СЋР±РѕРіРѕ РєРѕР»РµСЃР°).")
    time.sleep(1) # РќРµР±РѕР»СЊС€Р°СЏ РїР°СѓР·Р° РїРµСЂРµРґ СЃС‚Р°СЂС‚РѕРј
    
    set_motors(test_speed, 0, test_speed, 0)
    time.sleep(test_duration)
    stop_all()
    
    print("\nРњРѕС‚РѕСЂС‹ РѕСЃС‚Р°РЅРѕРІР»РµРЅС‹.")
    while True:
        try:
            rotations_str = input("РЎРєРѕР»СЊРєРѕ РїРѕР»РЅС‹С… (Рё РґСЂРѕР±РЅС‹С…, С‡РµСЂРµР· С‚РѕС‡РєСѓ) РѕР±РѕСЂРѕС‚РѕРІ СЃРґРµР»Р°Р»Рѕ РєРѕР»РµСЃРѕ? (РЅР°РїСЂРёРјРµСЂ, 7.5): ").strip()
            if not rotations_str: continue
            rotations = float(rotations_str)
            break
        except ValueError:
            print("РћС€РёР±РєР°: РІРІРµРґРёС‚Рµ С‡РёСЃР»Рѕ!")
            
    # Р Р°СЃС‡РµС‚ RPM РґР»СЏ С‚РµСЃС‚РѕРІРѕР№ СЃРєРѕСЂРѕСЃС‚Рё
    test_rpm = (rotations / test_duration) * 60.0
    # РџСЂРѕРїРѕСЂС†РёРѕРЅР°Р»СЊРЅС‹Р№ СЂР°СЃС‡РµС‚ РґР»СЏ РјР°РєСЃРёРјР°Р»СЊРЅРѕР№ СЃРєРѕСЂРѕСЃС‚Рё (4095)
    MAX_WHEEL_RPM = test_rpm * (4095.0 / float(test_speed))
    
    print(f"-> РЎРєРѕСЂРѕСЃС‚СЊ РІСЂР°С‰РµРЅРёСЏ РїСЂРё PWM={test_speed} СЃРѕСЃС‚Р°РІРёР»Р°: {test_rpm:.1f} RPM")
    print(f"-> Р Р°СЃС‡РµС‚РЅС‹Р№ MAX_WHEEL_RPM (РїСЂРё PWM=4095) СѓСЃС‚Р°РЅРѕРІР»РµРЅ РЅР°: {MAX_WHEEL_RPM:.1f} RPM")
    
    config["MAX_WHEEL_RPM"] = MAX_WHEEL_RPM
    save_config(config)
    print("РљР°Р»РёР±СЂРѕРІРєР° СѓСЃРїРµС€РЅРѕ Р·Р°РІРµСЂС€РµРЅР° Рё СЃРѕС…СЂР°РЅРµРЅР° РІ config.json!\n")


def autonomous_loop(driver, speed, detector=None):
    print("\n=== Р’РќРРњРђРќРР•: Р’РљР›Р®Р§Р•Рќ РЈРњРќР«Р™ РђР’РўРћРџРР›РћРў ===")
    print("Р РѕР±РѕС‚ Р±СѓРґРµС‚ РёСЃСЃР»РµРґРѕРІР°С‚СЊ РјРµСЃС‚РЅРѕСЃС‚СЊ, СѓС‡РёС‚С‹РІР°СЏ СЃРІРѕРё РіР°Р±Р°СЂРёС‚С‹!")
    time.sleep(2) 
    
    global current_mode, current_speed
    state = "FORWARD"
    active_trash_target_id = None
    active_trash_angle = 0.0
    completed_trash_targets = {}
    
    SAFE_DIST_FRONT = 0.67 # РЈРІРµР»РёС‡РёР»Рё РґРёСЃС‚Р°РЅС†РёСЋ РѕСЃС‚Р°РЅРѕРІРєРё РїРµСЂРµРґ СЃС‚РµРЅРѕР№ РµС‰Рµ РЅР° 2СЃРј
    TURN_EXIT_CLEARANCE = 0.72
    TURN_MAX_DURATION_SEC = 1.8
    TARGET_LOSS_REVERSE_COOLDOWN_SEC = 1.5
    TARGET_LOSS_STOP_SEC = 0.8
    turn_started_at = 0.0
    last_target_loss_time = 0.0
    target_loss_stop_until = 0.0
    target_loss_turn_attempts = 0
    turn_started_after_target_loss = False

    def pick_best_turn(scan):
        best_angle = 0
        max_clear = 0.0
        for ang in range(-180, 180, 15):
            if abs(ang) < 30:
                continue
            c = get_clearance(scan, ang)
            if c > max_clear:
                max_clear = c
                best_angle = ang
        next_state = "TURN_RIGHT" if best_angle < 0 else "TURN_LEFT"
        return next_state, best_angle, max_clear
    
    try:
        while driver.running:
            # --- Р›РћР“РРљРђ РЎР‘РћР Рђ РњРЈРЎРћР Рђ (YOLO) ---
            scan = driver.get_latest_scan()
            ignored_target_ids = _cleanup_completed_trash_targets(
                completed_trash_targets
            )
            route_state = get_route_state(detector)
            route_source_mode = route_state["source_mode"]
            route_enabled = bool(route_state["route_enabled"])
            route_within_corridor = bool(route_state["within_corridor"])
            route_guidance = route_state.get("guidance")
            active_target = _get_detector_target(
                detector,
                preferred_id=active_trash_target_id,
                exclude_ids=ignored_target_ids,
            )
            
            if (
                active_target
                and (not route_enabled or route_within_corridor)
                and state not in ["TRASH_APPROACH", "TRASH_COLLECT"]
            ):
                print(f"[РђР’РўРћРџРР›РћРў] РњРЈРЎРћР  РћР‘РќРђР РЈР–Р•Рќ (РЈРіРѕР»: {active_trash_angle:.1f})! РќР°С‡РёРЅР°СЋ СЃР±Р»РёР¶РµРЅРёРµ.")
                active_trash_target_id = active_target.get("id")
                active_trash_angle = float(active_target.get("angle", 0.0))
                state = "TRASH_APPROACH"
                
            if state == "TRASH_APPROACH":
                if route_enabled and not route_within_corridor:
                    print("[AUTOPILOT] Target is outside the active route corridor. Returning to route.")
                    state = "FORWARD"
                    active_trash_target_id = None
                    stop_all()
                    time.sleep(0.1)
                    continue

                target = _get_detector_target(
                    detector,
                    preferred_id=active_trash_target_id,
                    exclude_ids=ignored_target_ids,
                )
                if target is None:
                    replacement_target = _get_detector_target(
                        detector,
                        exclude_ids=ignored_target_ids,
                    )
                    if replacement_target is not None:
                        active_trash_target_id = replacement_target.get("id")
                        target = replacement_target

                if target is None:
                    dist = get_lidar_distance(scan, active_trash_angle)
                    if dist < 0.25:
                        print("[AUTO] Trash is very close, switching to blind collect.")
                        state = "TRASH_COLLECT"
                    else:
                        last_target_loss_time = time.time()
                        target_loss_stop_until = last_target_loss_time + TARGET_LOSS_STOP_SEC
                        target_loss_turn_attempts = 0
                        turn_started_after_target_loss = False
                        state = "FORWARD"
                        active_trash_target_id = None
                        stop_all()
                    time.sleep(0.1)
                    continue

                active_trash_target_id = target.get("id")
                active_trash_angle = float(target.get("angle", 0.0))
                dist = get_lidar_distance(scan, active_trash_angle)
                target_in_zone = bool(target.get("in_collection_zone", False))
                print(f"[РђР’РўРћРџРР›РћРў] РЎР±Р»РёР¶РµРЅРёРµ... Р”РёСЃС‚Р°РЅС†РёСЏ РїРѕ Р»РёРґР°СЂСѓ: {dist:.2f}Рј, РЈРіРѕР»: {active_trash_angle:.1f}В°")
                
                # Р•СЃР»Рё РјСѓСЃРѕСЂ СЃР»РёС€РєРѕРј Р±Р»РёР·РєРѕ РёР»Рё РїРѕС‚РµСЂСЏРЅ РёР· РІРёРґСѓ РІР±Р»РёР·Рё (СЃР»РµРїР°СЏ Р·РѕРЅР°)
                if target_in_zone and dist < 0.40:
                    print("[РђР’РўРћРџРР›РћРў] РњСѓСЃРѕСЂ РІ Р·РѕРЅРµ Р·Р°С…РІР°С‚Р°!")
                    state = "TRASH_COLLECT"
                elif dist < 0.25:
                    print("[AUTO] Trash is very close, switching to blind collect.")
                    state = "TRASH_COLLECT"
                elif dist >= 0.4 and not target_in_zone:
                    print("[РђР’РўРћРџРР›РћРў] Р›РѕР¶РЅРѕРµ СЃСЂР°Р±Р°С‚С‹РІР°РЅРёРµ РёР»Рё РјСѓСЃРѕСЂ СѓС‚РµСЂСЏРЅ РІРґР°Р»Рё. Р’РѕР·РІСЂР°С‚.")
                    last_target_loss_time = time.time()
                    target_loss_stop_until = last_target_loss_time + TARGET_LOSS_STOP_SEC
                    target_loss_turn_attempts = 0
                    turn_started_after_target_loss = False
                    state = "FORWARD"
                else:
                    # РџРѕРґСЂСѓР»РёРІР°РЅРёРµ (РёСЃРїРѕР»СЊР·СѓРµРј 50% СЃРєРѕСЂРѕСЃС‚Рё РґР»СЏ РїР»Р°РІРЅРѕСЃС‚Рё)
                    if active_trash_angle > 10:
                        set_motors(speed//2, 0, 0, speed//2) # Р’РїСЂР°РІРѕ
                    elif active_trash_angle < -10:
                        set_motors(0, speed//2, speed//2, 0) # Р’Р»РµРІРѕ
                    else:
                        set_motors(speed//2, 0, speed//2, 0) # РџСЂСЏРјРѕ
                time.sleep(0.1)
                continue
                
            elif state == "TRASH_COLLECT":
                collected_target_id = active_trash_target_id
                stop_all()
                time.sleep(0.5) # Р”Р°РµРј РјРѕС‚РѕСЂР°Рј РїРѕР»РЅРѕСЃС‚СЊСЋ РѕСЃС‚Р°РЅРѕРІРёС‚СЊСЃСЏ РїРµСЂРµРґ СЃРјРµРЅРѕР№ С‡Р°СЃС‚РѕС‚С‹
                print("[РђР’РўРћРџРР›РћРў] Р—Р°РїСѓСЃРєР°СЋ РєРѕРІС€!")
                run_bucket_collect_cycle()
                print("[РђР’РўРћРџРР›РћРў] РњСѓСЃРѕСЂ СЃРѕР±СЂР°РЅ! Р’РѕР·РІСЂР°С‚ Рє РёСЃСЃР»РµРґРѕРІР°РЅРёСЋ.")
                _remember_completed_trash_target(
                    completed_trash_targets,
                    collected_target_id,
                )
                state = "FORWARD"
                if detector:
                    if hasattr(detector, "ignore_target"):
                        detector.ignore_target(collected_target_id, cooldown_sec=8.0)
                    detector.trash_detected = False
                    if hasattr(detector, "trash_in_collection_zone"):
                        detector.trash_in_collection_zone = False
                active_trash_target_id = None
                next_target = _get_detector_target(
                    detector,
                    exclude_ids=_cleanup_completed_trash_targets(
                        completed_trash_targets
                    ),
                )
                if (
                    next_target is not None
                    and (not route_enabled or route_within_corridor)
                ):
                    active_trash_target_id = next_target.get("id")
                    active_trash_angle = float(next_target.get("angle", 0.0))
                    state = "TRASH_APPROACH"
                continue

            # --- Р›РћР“РРљРђ РРЎРЎР›Р•Р”РћР’РђРўР•Р›РЇ РЎ Р›РР”РђР РћРњ ---
            
            clearance_front = get_clearance(scan, 0)
            now = time.time()

            if state == "FORWARD" and now < target_loss_stop_until:
                stop_all()
                time.sleep(0.1)
                continue
            
            if state == "FORWARD":
                if clearance_front < SAFE_DIST_FRONT:
                    stop_all()
                    time.sleep(0.2)

                    # РС‰РµРј Р»СѓС‡С€РµРµ РЅР°РїСЂР°РІР»РµРЅРёРµ (С€Р°Рі 15 РіСЂР°РґСѓСЃРѕРІ)
                    next_turn_state, best_angle, max_clear = pick_best_turn(scan)
                    print(f"[РђР’РўРћРџРР›РћРў] РџСЂРµРїСЏС‚СЃС‚РІРёРµ! Р›СѓС‡С€РёР№ РєРѕСЂРёРґРѕСЂ РЅР° {best_angle}В° (СЃРІРѕР±РѕРґРЅРѕ {max_clear:.2f}Рј)")

                    if max_clear < SAFE_DIST_FRONT:
                        recently_lost_target = (
                            now - last_target_loss_time < TARGET_LOSS_REVERSE_COOLDOWN_SEC
                        )
                        if recently_lost_target:
                            if target_loss_turn_attempts == 0:
                                state = next_turn_state
                                turn_started_at = now
                                turn_started_after_target_loss = True
                                target_loss_turn_attempts = 1
                                print("[AUTOPILOT] Recently lost target, trying one escape turn without reverse.")
                            else:
                                stop_all()
                                target_loss_stop_until = now + TARGET_LOSS_STOP_SEC
                                print("[AUTOPILOT] Target was lost recently, waiting instead of reversing or spinning.")
                        else:
                            state = "REVERSE"
                            turn_started_after_target_loss = False
                            print("[РђР’РўРћРџРР›РћРў] РўСѓРїРёРє! Р’РєР»СЋС‡Р°СЋ Р·Р°РґРЅРёР№ С…РѕРґ.")
                    else:
                        state = next_turn_state
                        turn_started_at = now
                        turn_started_after_target_loss = False
                else:
                    if now - last_target_loss_time >= TARGET_LOSS_REVERSE_COOLDOWN_SEC:
                        target_loss_turn_attempts = 0
                        turn_started_after_target_loss = False
                    # Р•РґРµРј РїСЂСЏРјРѕ
                    if route_source_mode == "gps" and route_enabled and not route_within_corridor:
                        stop_all()
                        time.sleep(0.1)
                        continue

                    if route_source_mode == "slam" and route_enabled and route_guidance:
                        heading_error_deg = math.degrees(float(route_guidance["heading_error"]))
                        turn_speed = max(700, speed // 2)
                        if heading_error_deg > 10:
                            with movement_lock:
                                current_mode = 3
                                current_speed = turn_speed
                            set_motors(turn_speed, 0, 0, turn_speed)
                            time.sleep(0.1)
                            continue
                        if heading_error_deg < -10:
                            with movement_lock:
                                current_mode = 4
                                current_speed = turn_speed
                            set_motors(0, turn_speed, turn_speed, 0)
                            time.sleep(0.1)
                            continue

                    with movement_lock:
                        current_mode = 1
                        current_speed = speed
                    set_motors(speed, 0, speed, 0)
                    
            elif state == "TURN_LEFT":
                with movement_lock:
                    current_mode = 3
                    current_speed = speed
                set_motors(speed, 0, 0, speed)
                if get_clearance(scan, 0) > TURN_EXIT_CLEARANCE:
                    stop_all()
                    time.sleep(0.2)
                    state = "FORWARD"
                    turn_started_after_target_loss = False
                elif time.time() - turn_started_at > TURN_MAX_DURATION_SEC:
                    stop_all()
                    time.sleep(0.1)
                    state = "FORWARD"
                    if turn_started_after_target_loss:
                        target_loss_stop_until = time.time() + TARGET_LOSS_STOP_SEC
                    turn_started_after_target_loss = False
                    print("[AUTOPILOT] Left turn timeout reached, returning to forward exploration.")
                    
            elif state == "TURN_RIGHT":
                with movement_lock:
                    current_mode = 4
                    current_speed = speed
                set_motors(0, speed, speed, 0)
                if get_clearance(scan, 0) > TURN_EXIT_CLEARANCE:
                    stop_all()
                    time.sleep(0.2)
                    state = "FORWARD"
                    turn_started_after_target_loss = False
                elif time.time() - turn_started_at > TURN_MAX_DURATION_SEC:
                    stop_all()
                    time.sleep(0.1)
                    state = "FORWARD"
                    if turn_started_after_target_loss:
                        target_loss_stop_until = time.time() + TARGET_LOSS_STOP_SEC
                    turn_started_after_target_loss = False
                    print("[AUTOPILOT] Right turn timeout reached, returning to forward exploration.")
                    
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
                    next_turn_state, best_angle, max_clear = pick_best_turn(scan)
                    state = next_turn_state
                    turn_started_at = time.time()
                    turn_started_after_target_loss = False
                    print(f"[AUTOPILOT] Reverse finished, retrying escape turn {best_angle} deg.")
                
            time.sleep(0.1) 
            
    except KeyboardInterrupt:
        print("\nРђРІС‚РѕРїРёР»РѕС‚ РїСЂРµСЂРІР°РЅ РїРѕР»СЊР·РѕРІР°С‚РµР»РµРј!")
    finally:
        stop_all()

def legacy_main_unused():
    print("=== РЎРёСЃС‚РµРјР° СѓРїСЂР°РІР»РµРЅРёСЏ + FastSLAM + YOLO ===")
    
    # РЎРёРЅС…СЂРѕРЅРёР·РёСЂСѓРµРј Р»РѕРєР°Р»СЊРЅС‹Р№ Рё РіР»РѕР±Р°Р»СЊРЅС‹Р№ РєРѕРЅС„РёРіРё
    config = global_config
    """Legacy prompt removed: Arduino port must be asked separately from lidar.
        f"Р’РІРµРґРёС‚Рµ РїРѕСЂС‚ Arduino РєРѕРІС€Р° (Enter РґР»СЏ {config.get('arduino_port', '/dev/ttyACM0')}): "
    ).strip()
    if lidar_port:
        config["lidar_port"] = lidar_port
    """
    # Arduino РєРѕРІС€Р° РёРЅРёС†РёР°Р»РёР·РёСЂСѓРµРј РїРѕСЃР»Рµ РѕС‚РґРµР»СЊРЅРѕРіРѕ СЏРІРЅРѕРіРѕ Р·Р°РїСЂРѕСЃР° РїРѕСЂС‚Р° РЅРёР¶Рµ.
    
    # РџСЂРё Р·Р°РїСѓСЃРєРµ СЃСЂР°Р·Сѓ СЃС‚Р°РІРёРј РєРѕРІС€ РІ РЅСѓР»РµРІРѕРµ (С‚СЂР°РЅСЃРїРѕСЂС‚РЅРѕРµ) РїРѕР»РѕР¶РµРЅРёРµ
    print("РЈСЃС‚Р°РЅРѕРІРєР° РєРѕРІС€Р° РІ РЅСѓР»РµРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ...")
    # РљРѕРІС€ РїРµСЂРµРІРѕРґРёРј РІ РЅРѕР»СЊ РїРѕСЃР»Рµ СЏРІРЅРѕРіРѕ РІС‹Р±РѕСЂР° РїРѕСЂС‚Р° Arduino РЅРёР¶Рµ.
    
    # Р—Р°РїСЂРѕСЃ РЅР°СЃС‚СЂРѕРµРє Сѓ РїРѕР»СЊР·РѕРІР°С‚РµР»СЏ
    lidar_port = input(f"Р’РІРµРґРёС‚Рµ РїРѕСЂС‚ Р»РёРґР°СЂР° (Enter РґР»СЏ {config.get('lidar_port', '/dev/ttyUSB0')}): ").strip()
    if not lidar_port: lidar_port = config.get('lidar_port', '/dev/ttyUSB0')
    config['lidar_port'] = lidar_port
    config["servo_pca_address"] = str(
        config.get("servo_pca_address", hex(SERVO_PCA_DEFAULT_ADDRESS))
    ).strip() or hex(SERVO_PCA_DEFAULT_ADDRESS)
    try:
        config["bucket_servo_channel"] = int(
            config.get("bucket_servo_channel", BUCKET_SERVO_DEFAULT_CHANNEL)
        )
    except (TypeError, ValueError):
        config["bucket_servo_channel"] = BUCKET_SERVO_DEFAULT_CHANNEL
    init_bucket_servo_controller(config)
    print("РЈСЃС‚Р°РЅРѕРІРєР° РєРѕРІС€Р° РІ РЅСѓР»РµРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ...")
    set_servo_bucket(down=True)
        
    print("\nР“РґРµ РїРѕРєР°Р·С‹РІР°С‚СЊ РєР°СЂС‚Сѓ SLAM?")
    print("1 - РџРѕРєР°Р·С‹РІР°С‚СЊ РІ Tailscale (С‡РµСЂРµР· SSH СЃ РїСЂРѕР±СЂРѕСЃРѕРј X11 РёР»Рё VNC)")
    print("2 - РџРѕРєР°Р·С‹РІР°С‚СЊ РЅР° СЌРєСЂР°РЅРµ RPi РЅР° Armbian (С„РёР·РёС‡РµСЃРєРёР№ РјРѕРЅРёС‚РѕСЂ)")
    print("3 - РќРµ РїРѕРєР°Р·С‹РІР°С‚СЊ РІРѕРѕР±С‰Рµ (РјР°РєСЃРёРјР°Р»СЊРЅР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ)")
    map_choice = input("Р’Р°С€ РІС‹Р±РѕСЂ (1, 2 РёР»Рё 3): ").strip()
    
    if map_choice == '2':
        os.environ["DISPLAY"] = ":0"
        show_map = True
    elif map_choice == '1':
        show_map = True
    else:
        show_map = False
    
    print("\nР“РґРµ Р·Р°РїСѓСЃРєР°С‚СЊ РЅРµР№СЂРѕСЃРµС‚СЊ YOLO РґР»СЏ СЃР±РѕСЂР° РјСѓСЃРѕСЂР°?")
    print("1 - РќР° С‚РµР»РµС„РѕРЅРµ / РџРљ (РњР°РєСЃРёРјР°Р»СЊРЅР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ, РїРѕ Wi-Fi)")
    print("2 - Р›РѕРєР°Р»СЊРЅРѕ РЅР° Raspberry Pi (РќРёР·РєРёР№ FPS)")
    print("3 - РћС‚РєР»СЋС‡РёС‚СЊ СЃР±РѕСЂ РјСѓСЃРѕСЂР°")
    yolo_choice = input("Р’Р°С€ РІС‹Р±РѕСЂ (1, 2 РёР»Рё 3): ").strip()
    
    detector = None
    if yolo_choice == '1' and RemoteTrashListener:
        detector = RemoteTrashListener(
            on_servo_command=handle_remote_servo_command,
            on_motor_command=handle_remote_bucket_motor_command,
        )
        detector.start()
        print("\n[Р’РќРРњРђРќРР•] РќР° С‚РµР»РµС„РѕРЅРµ (РІ Pydroid 3) Р·Р°РїСѓСЃС‚РёС‚Рµ СЃРєСЂРёРїС‚ `yolo_client.py`.")
        
    elif yolo_choice == '2' and TrashDetector:
        models_dir = "models"
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)
            
        available_models = os.listdir(models_dir)
        selected_model = None
        
        if not available_models:
            print("\n[YOLO] Р’ РїР°РїРєРµ 'models' РїСѓСЃС‚Рѕ! РџРѕР¶Р°Р»СѓР№СЃС‚Р°, СЃРєРѕРїРёСЂСѓР№С‚Рµ С‚СѓРґР° РІР°С€Рё РјРѕРґРµР»Рё (РїР°РїРєРё NCNN).")
        else:
            print("\nР”РѕСЃС‚СѓРїРЅС‹Рµ РјРѕРґРµР»Рё РІ РїР°РїРєРµ 'models':")
            for i, m in enumerate(available_models):
                print(f"{i+1} - {m}")
            
            try:
                m_idx = int(input(f"Р’С‹Р±РµСЂРёС‚Рµ РјРѕРґРµР»СЊ (1-{len(available_models)}): ").strip()) - 1
                if 0 <= m_idx < len(available_models):
                    selected_model = os.path.join(models_dir, available_models[m_idx])
            except ValueError:
                print("РћС€РёР±РєР° РІРІРѕРґР°.")
                
        if selected_model:
            detector = TrashDetector(model_path=selected_model)
            detector.start()
        else:
            print("[YOLO] РњРѕРґРµР»СЊ РЅРµ РІС‹Р±СЂР°РЅР°, РґРµС‚РµРєС‚РѕСЂ РјСѓСЃРѕСЂР° РѕС‚РєР»СЋС‡РµРЅ.")
            
    else:
        print("[YOLO] РЎР±РѕСЂ РјСѓСЃРѕСЂР° РѕС‚РєР»СЋС‡РµРЅ.")
    
    print("\nР’С‹Р±РµСЂРёС‚Рµ СЂРµР¶РёРј СЂР°Р±РѕС‚С‹:")
    print("1 - Р СѓС‡РЅРѕР№ (СѓРїСЂР°РІР»РµРЅРёРµ СЃ РєР»Р°РІРёР°С‚СѓСЂС‹)")
    print("2 - РђРІС‚РѕРїРёР»РѕС‚ (РѕР±С…РѕРґ РїСЂРµРїСЏС‚СЃС‚РІРёР№ Рё СЃР±РѕСЂ РјСѓСЃРѕСЂР°)")
    mode_choice = input("Р’Р°С€ РІС‹Р±РѕСЂ (1 РёР»Рё 2): ").strip()
    
    auto_speed = config.get('auto_speed', 1500)
    if mode_choice == '2':
        speed_str = input(f"Р’РІРµРґРёС‚Рµ СЃРєРѕСЂРѕСЃС‚СЊ Р°РІС‚РѕРїРёР»РѕС‚Р° РѕС‚ 0 РґРѕ 4095 (Enter РґР»СЏ {auto_speed}): ").strip()
        if speed_str.isdigit():
            auto_speed = int(speed_str)
        config['auto_speed'] = auto_speed
        
    save_config(config)
    
    # РЎРЅР°С‡Р°Р»Р° РІС‹РїРѕР»РЅСЏРµРј РєР°Р»РёР±СЂРѕРІРєСѓ
    calibrate_motors(config)
    
    # Р—Р°РїСѓСЃРє РґСЂР°Р№РІРµСЂР° Р»РёРґР°СЂР°
    driver = LD06Driver(port=lidar_port)
    driver.start()
    
    if not driver.running:
        print("РћС€РёР±РєР°: Р›РёРґР°СЂ РЅРµ Р·Р°РїСѓС‰РµРЅ. РЈР±РµРґРёС‚РµСЃСЊ С‡С‚Рѕ РѕРЅ РїРѕРґРєР»СЋС‡РµРЅ Рє /dev/ttyUSB0.")
        # РњРѕР¶РЅРѕ РїРѕР·РІРѕР»РёС‚СЊ РїСЂРѕРґРѕР»Р¶РёС‚СЊ Р±РµР· Р»РёРґР°СЂР° РґР»СЏ С‚РµСЃС‚РѕРІ, РЅРѕ Р»СѓС‡С€Рµ РІС‹Р№С‚Рё
        # sys.exit(1)
        
    # Р—Р°РїСѓСЃРє РїРѕС‚РѕРєР° SLAM
    slam_thread = threading.Thread(target=slam_thread_function, args=(driver, show_map), daemon=True)
    slam_thread.start()

    if mode_choice == '2':
        # Р—Р°РїСѓСЃРє Р°РІС‚РѕРїРёР»РѕС‚Р°
        autonomous_loop(driver, auto_speed, detector)
    else:
        # Р СѓС‡РЅРѕР№ СЂРµР¶РёРј
        print("\nР¤РѕСЂРјР°С‚ РІРІРѕРґР°: 'Р РµР¶РёРј РЎРєРѕСЂРѕСЃС‚СЊ' (РЅР°РїСЂРёРјРµСЂ, '1 1000').")
        print("Р РµР¶РёРјС‹: 1-Р’РїРµСЂРµРґ, 2-РќР°Р·Р°Рґ, 3-Р’Р»РµРІРѕ, 4-Р’РїСЂР°РІРѕ, 0-РћСЃС‚Р°РЅРѕРІРєР°")
        print("РЎРєРѕСЂРѕСЃС‚СЊ: 0 - 4095. РќР°Р¶РјРёС‚Рµ Ctrl+C РґР»СЏ Р­РљРЎРўР Р•РќРќРћР™ РћРЎРўРђРќРћР’РљР Рё РІС‹С…РѕРґР°.")
        print("РўР°РєР¶Рµ РґР»СЏ Р±С‹СЃС‚СЂРѕР№ РѕСЃС‚Р°РЅРѕРІРєРё РїСЂРѕСЃС‚Рѕ РЅР°Р¶РјРёС‚Рµ ENTER (РїСѓСЃС‚РѕР№ РІРІРѕРґ) РёР»Рё Р»СЋР±СѓСЋ Р±СѓРєРІСѓ.\n")
        
        stop_all()

        try:
            while True:
                cmd = input("Р’РІРµРґРёС‚Рµ РєРѕРјР°РЅРґСѓ: ").strip()
                
                # Р­РєСЃС‚СЂРµРЅРЅР°СЏ РѕСЃС‚Р°РЅРѕРІРєР° РїСЂРё РїСѓСЃС‚РѕРј РІРІРѕРґРµ (РїСЂРѕСЃС‚Рѕ СѓРґР°СЂ РїРѕ Enter) РёР»Рё РІРІРѕРґРµ Р±СѓРєРІС‹ 'e'
                if not cmd or cmd.lower() in ['e', 's', 'stop']:
                    stop_all()
                    print("Р­РљРЎРўР Р•РќРќРђРЇ РћРЎРўРђРќРћР’РљРђ!")
                    continue
                    
                try:
                    parts = cmd.split()
                    mode = int(parts[0])
                    
                    if mode == 0:
                        stop_all()
                        print("РњРѕС‚РѕСЂС‹ РѕСЃС‚Р°РЅРѕРІР»РµРЅС‹.")
                        continue
                        
                    speed = int(parts[1]) if len(parts) > 1 else 1000
                    
                    with movement_lock:
                        global current_mode, current_speed
                        current_mode = mode
                        current_speed = speed
                    
                    if mode == 1:
                        print(f"Р”РІРёР¶РµРЅРёРµ Р’РџР•Р Р•Р” РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(speed, 0, speed, 0)
                    elif mode == 2:
                        print(f"Р”РІРёР¶РµРЅРёРµ РќРђР—РђР” РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(0, speed, 0, speed)
                    elif mode == 3:
                        print(f"РџРѕРІРѕСЂРѕС‚ Р’Р›Р•Р’Рћ РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(speed, 0, 0, speed)
                    elif mode == 4:
                        print(f"РџРѕРІРѕСЂРѕС‚ Р’РџР РђР’Рћ РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(0, speed, speed, 0)
                    else:
                        print("РќРµРёР·РІРµСЃС‚РЅС‹Р№ СЂРµР¶РёРј! РћСЃС‚Р°РЅРѕРІРєР°.")
                        stop_all()
                        
                except ValueError:
                    # Р­РєСЃС‚СЂРµРЅРЅР°СЏ РѕСЃС‚Р°РЅРѕРІРєР° РїСЂРё СЃР»СѓС‡Р°Р№РЅРѕРј РІРІРѕРґРµ Р»СЋР±С‹С… СЃРёРјРІРѕР»РѕРІ
                    print("РћС€РёР±РєР° РІРІРѕРґР° (РІРІРµРґРµРЅС‹ РЅРµ С‡РёСЃР»Р°). Р­РљРЎРўР Р•РќРќРђРЇ РћРЎРўРђРќРћР’РљРђ!")
                    stop_all()

        except KeyboardInterrupt:
            print("\nР’С‹С…РѕРґ РёР· СЂСѓС‡РЅРѕРіРѕ СЂРµР¶РёРјР°...")
            
    # Р—Р°РІРµСЂС€РµРЅРёРµ СЂР°Р±РѕС‚С‹
    stop_all()
    driver.stop()
    close_bucket_arduino()

def main():
    print("=== РЎРёСЃС‚РµРјР° СѓРїСЂР°РІР»РµРЅРёСЏ + FastSLAM + YOLO ===")

    config = global_config

    lidar_port = prompt_for_serial_port(
        "Р»РёРґР°СЂР°",
        saved_port=config.get("lidar_port", "/dev/ttyUSB0"),
    )
    config["lidar_port"] = lidar_port

    config["servo_pca_address"] = str(
        config.get("servo_pca_address", hex(SERVO_PCA_DEFAULT_ADDRESS))
    ).strip() or hex(SERVO_PCA_DEFAULT_ADDRESS)
    try:
        config["bucket_servo_channel"] = int(
            config.get("bucket_servo_channel", BUCKET_SERVO_DEFAULT_CHANNEL)
        )
    except (TypeError, ValueError):
        config["bucket_servo_channel"] = BUCKET_SERVO_DEFAULT_CHANNEL
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

    init_bucket_servo_controller(config)
    calibrate_bucket_wall(config)

    print("\nРЈСЃС‚Р°РЅРѕРІРєР° РєРѕРІС€Р° РІ РїРѕРёСЃРєРѕРІРѕРµ РїРѕР»РѕР¶РµРЅРёРµ...")
    move_bucket_wall_to_search_position()
    set_servo_bucket(down=True)

    print("\nР“РґРµ РїРѕРєР°Р·С‹РІР°С‚СЊ РєР°СЂС‚Сѓ SLAM?")
    print("1 - РџРѕРєР°Р·С‹РІР°С‚СЊ РІ Tailscale (С‡РµСЂРµР· SSH СЃ X11 РёР»Рё VNC)")
    print("2 - РџРѕРєР°Р·С‹РІР°С‚СЊ РЅР° СЌРєСЂР°РЅРµ RPi РЅР° Armbian")
    print("3 - РќРµ РїРѕРєР°Р·С‹РІР°С‚СЊ РІРѕРѕР±С‰Рµ")
    map_choice = input("Р’Р°С€ РІС‹Р±РѕСЂ (1, 2 РёР»Рё 3): ").strip()

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

    print("\nР“РґРµ Р·Р°РїСѓСЃРєР°С‚СЊ РЅРµР№СЂРѕСЃРµС‚СЊ YOLO РґР»СЏ СЃР±РѕСЂР° РјСѓСЃРѕСЂР°?")
    print("1 - РќР° С‚РµР»РµС„РѕРЅРµ / РџРљ (РјР°РєСЃРёРјР°Р»СЊРЅР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ, РїРѕ Wi-Fi)")
    print("2 - Р›РѕРєР°Р»СЊРЅРѕ РЅР° Raspberry Pi (РЅРёР·РєРёР№ FPS)")
    print("3 - РћС‚РєР»СЋС‡РёС‚СЊ СЃР±РѕСЂ РјСѓСЃРѕСЂР°")
    yolo_choice = input("Р’Р°С€ РІС‹Р±РѕСЂ (1, 2 РёР»Рё 3): ").strip()

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
        print("\n[Р’РќРРњРђРќРР•] РќР° С‚РµР»РµС„РѕРЅРµ (РІ Pydroid 3) Р·Р°РїСѓСЃС‚РёС‚Рµ СЃРєСЂРёРїС‚ `yolo_client.py`.")
    elif yolo_choice == "2" and TrashDetector:
        models_dir = "models"
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)

        available_models = os.listdir(models_dir)
        selected_model = None

        if not available_models:
            print("\n[YOLO] Р’ РїР°РїРєРµ 'models' РїСѓСЃС‚Рѕ. РЎРєРѕРїРёСЂСѓР№С‚Рµ С‚СѓРґР° РІР°С€Рё РјРѕРґРµР»Рё (РїР°РїРєРё NCNN).")
        else:
            print("\nР”РѕСЃС‚СѓРїРЅС‹Рµ РјРѕРґРµР»Рё РІ РїР°РїРєРµ 'models':")
            for i, model_name in enumerate(available_models):
                print(f"{i + 1} - {model_name}")

            try:
                model_index = int(input(f"Р’С‹Р±РµСЂРёС‚Рµ РјРѕРґРµР»СЊ (1-{len(available_models)}): ").strip()) - 1
                if 0 <= model_index < len(available_models):
                    selected_model = os.path.join(models_dir, available_models[model_index])
            except ValueError:
                print("РћС€РёР±РєР° РІРІРѕРґР°.")

        if selected_model:
            detector = TrashDetector(model_path=selected_model)
            detector.start()
        else:
            print("[YOLO] РњРѕРґРµР»СЊ РЅРµ РІС‹Р±СЂР°РЅР°, РґРµС‚РµРєС‚РѕСЂ РјСѓСЃРѕСЂР° РѕС‚РєР»СЋС‡РµРЅ.")
    else:
        print("[YOLO] РЎР±РѕСЂ РјСѓСЃРѕСЂР° РѕС‚РєР»СЋС‡РµРЅ.")

    print("\nР’С‹Р±РµСЂРёС‚Рµ СЂРµР¶РёРј СЂР°Р±РѕС‚С‹:")
    print("1 - Р СѓС‡РЅРѕР№ (СѓРїСЂР°РІР»РµРЅРёРµ СЃ РєР»Р°РІРёР°С‚СѓСЂС‹)")
    print("2 - РђРІС‚РѕРїРёР»РѕС‚ (РѕР±С…РѕРґ РїСЂРµРїСЏС‚СЃС‚РІРёР№ Рё СЃР±РѕСЂ РјСѓСЃРѕСЂР°)")
    mode_choice = input("Р’Р°С€ РІС‹Р±РѕСЂ (1 РёР»Рё 2): ").strip()

    if not mode_choice:
        mode_choice = str(config.get("run_mode", "2"))
    if mode_choice not in {"1", "2"}:
        print("[Р Р•Р–РРњ] РќРµРёР·РІРµСЃС‚РЅС‹Р№ СЂРµР¶РёРј, РІРєР»СЋС‡Р°СЋ Р°РІС‚РѕРїРёР»РѕС‚ РїРѕ СѓРјРѕР»С‡Р°РЅРёСЋ.")
        mode_choice = "2"
    config["run_mode"] = mode_choice

    if detector and hasattr(detector, "set_allow_text_commands"):
        detector.set_allow_text_commands(mode_choice != "2")

    auto_speed = config.get("auto_speed", 1500)
    if mode_choice == "2":
        speed_str = input(f"Р’РІРµРґРёС‚Рµ СЃРєРѕСЂРѕСЃС‚СЊ Р°РІС‚РѕРїРёР»РѕС‚Р° РѕС‚ 0 РґРѕ 4095 (Enter РґР»СЏ {auto_speed}): ").strip()
        if speed_str.isdigit():
            auto_speed = int(speed_str)
        config["auto_speed"] = auto_speed

    save_config(config)
    calibrate_motors(config)

    driver = LD06Driver(port=lidar_port)
    driver.start()

    if not driver.running:
        print("РћС€РёР±РєР°: Р»РёРґР°СЂ РЅРµ Р·Р°РїСѓС‰РµРЅ. РЈР±РµРґРёС‚РµСЃСЊ, С‡С‚Рѕ РѕРЅ РїРѕРґРєР»СЋС‡РµРЅ Рє РІС‹Р±СЂР°РЅРЅРѕРјСѓ РїРѕСЂС‚Сѓ.")

    slam_thread = threading.Thread(target=slam_thread_function, args=(driver, show_map), daemon=True)
    slam_thread.start()

    try:
        if mode_choice == "2":
            autonomous_loop(driver, auto_speed, detector)
        else:
            print("\nР¤РѕСЂРјР°С‚ РІРІРѕРґР°: 'Р РµР¶РёРј РЎРєРѕСЂРѕСЃС‚СЊ' (РЅР°РїСЂРёРјРµСЂ, '1 1000').")
            print("Р РµР¶РёРјС‹: 1-Р’РїРµСЂРµРґ, 2-РќР°Р·Р°Рґ, 3-Р’Р»РµРІРѕ, 4-Р’РїСЂР°РІРѕ, 0-РћСЃС‚Р°РЅРѕРІРєР°")
            print("РЎРєРѕСЂРѕСЃС‚СЊ: 0 - 4095. Enter РёР»Рё stop - СЌРєСЃС‚СЂРµРЅРЅР°СЏ РѕСЃС‚Р°РЅРѕРІРєР°.\n")

            print("Manual bucket commands: wall_up, wall_down, scoop_up, scoop_down, bucket_test, collect")
            stop_all()

            while True:
                cmd = input("Р’РІРµРґРёС‚Рµ РєРѕРјР°РЅРґСѓ: ").strip()

                if not cmd or cmd.lower() in ["e", "s", "stop"]:
                    stop_all()
                    print("Р­РљРЎРўР Р•РќРќРђРЇ РћРЎРўРђРќРћР’РљРђ!")
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
                        print("РњРѕС‚РѕСЂС‹ РѕСЃС‚Р°РЅРѕРІР»РµРЅС‹.")
                        continue

                    speed = int(parts[1]) if len(parts) > 1 else 1000

                    with movement_lock:
                        global current_mode, current_speed
                        current_mode = mode
                        current_speed = speed

                    if mode == 1:
                        print(f"Р”РІРёР¶РµРЅРёРµ Р’РџР•Р Р•Р” РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(speed, 0, speed, 0)
                    elif mode == 2:
                        print(f"Р”РІРёР¶РµРЅРёРµ РќРђР—РђР” РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(0, speed, 0, speed)
                    elif mode == 3:
                        print(f"РџРѕРІРѕСЂРѕС‚ Р’Р›Р•Р’Рћ РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(speed, 0, 0, speed)
                    elif mode == 4:
                        print(f"РџРѕРІРѕСЂРѕС‚ Р’РџР РђР’Рћ РЅР° СЃРєРѕСЂРѕСЃС‚Рё {speed}")
                        set_motors(0, speed, speed, 0)
                    else:
                        print("РќРµРёР·РІРµСЃС‚РЅС‹Р№ СЂРµР¶РёРј! РћСЃС‚Р°РЅРѕРІРєР°.")
                        stop_all()
                except ValueError:
                    print("РћС€РёР±РєР° РІРІРѕРґР°. Р­РљРЎРўР Р•РќРќРђРЇ РћРЎРўРђРќРћР’РљРђ!")
                    stop_all()
    except KeyboardInterrupt:
        print("\nР—Р°РІРµСЂС€РµРЅРёРµ СЂР°Р±РѕС‚С‹ РїРѕ Р·Р°РїСЂРѕСЃСѓ РїРѕР»СЊР·РѕРІР°С‚РµР»СЏ...")
    finally:
        stop_all()
        driver.stop()
        close_bucket_arduino()
        stop_video_streamer()

def start_video_streamer(config):
    global video_streamer_process, video_streamer_last_error, video_streamer_last_source

    if video_streamer_process and video_streamer_process.poll() is None:
        print("[VIDEO] Р’РёРґРµРѕСЃС‚СЂРёРјРµСЂ СѓР¶Рµ Р·Р°РїСѓС‰РµРЅ.")
        return

    camera_port = resolve_camera_port(str(config.get("camera_port", "/dev/video0")).strip())
    stream_host = config.get("camera_stream_host", "0.0.0.0")
    stream_port = int(config.get("camera_stream_port", 5000))
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "video_streamer.py")
    video_streamer_last_source = str(camera_port)
    video_streamer_last_error = ""

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
            print(f"[VIDEO] Р’РёРґРµРѕСЃС‚СЂРёРјРµСЂ Р·Р°РїСѓС‰РµРЅ РґР»СЏ {camera_port} РЅР° РїРѕСЂС‚Сѓ {stream_port}.")
        else:
            print("[VIDEO] Р’РёРґРµРѕСЃС‚СЂРёРјРµСЂ Р·Р°РІРµСЂС€РёР»СЃСЏ СЃСЂР°Р·Сѓ РїРѕСЃР»Рµ Р·Р°РїСѓСЃРєР°. РџСЂРѕРІРµСЂСЊС‚Рµ РїРѕСЂС‚ РєР°РјРµСЂС‹.")
    except Exception as e:
        print(f"[VIDEO] РќРµ СѓРґР°Р»РѕСЃСЊ Р·Р°РїСѓСЃС‚РёС‚СЊ РІРёРґРµРѕСЃС‚СЂРёРјРµСЂ: {e}")
        video_streamer_process = None

def get_video_streamer_status():
    active = bool(video_streamer_process and video_streamer_process.poll() is None)
    error = ""
    if not active:
        error = video_streamer_last_error
        if not error and video_streamer_last_source:
            error = (
                f"No live stream from {video_streamer_last_source}. "
                "The selected /dev/videoX may be the wrong node for this camera."
            )
    return {
        "active": active,
        "source": video_streamer_last_source,
        "error": error,
    }


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

