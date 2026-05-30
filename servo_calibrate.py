import json
import os
import time
import glob

try:
    import serial
except ImportError:
    serial = None


CONFIG_FILE = "config.json"


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=4)
    except Exception:
        pass


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


def prompt_for_arduino_port(saved_port=None):
    candidates = find_serial_candidates()
    if candidates:
        print("Доступные последовательные порты:")
        for port in candidates:
            print(f" - {port}")

    while True:
        prompt = "Введите порт Arduino ковша"
        if saved_port:
            prompt += f" (Enter для {saved_port})"
        prompt += ": "

        port = input(prompt).strip()
        if not port and saved_port:
            port = saved_port

        if port:
            return port

        print("Порт Arduino обязателен.")


def send_servo_angle(arduino, angle):
    angle = max(0, min(180, int(angle)))
    arduino.write(f"SERVO:{angle}\n".encode("utf-8"))
    arduino.flush()
    return angle


def main():
    if not serial:
        print("Установите pyserial: pip install pyserial")
        return

    config = load_config()
    arduino_port = prompt_for_arduino_port(config.get("arduino_port", "/dev/ttyACM0"))
    config["arduino_port"] = arduino_port
    save_config(config)
    baudrate = int(config.get("arduino_baudrate", 9600))
    timeout_sec = float(config.get("arduino_timeout_sec", 0.3))
    boot_wait_sec = float(config.get("arduino_boot_wait_sec", 2.0))

    try:
        arduino = serial.Serial(
            arduino_port,
            baudrate,
            timeout=timeout_sec,
            write_timeout=timeout_sec,
        )
        time.sleep(boot_wait_sec)
        arduino.reset_input_buffer()
    except Exception as e:
        print(f"Не удалось подключиться к Arduino на {arduino_port}: {e}")
        return

    up_angle = config.get("servo_up_angle", 0)
    down_angle = config.get("servo_down_angle", 90)
    current_angle = int(config.get("servo_up_angle", 0))

    print(f"Калибровка сервы через Arduino на {arduino_port}.")
    print("Введите угол от 0 до 180.")
    print("Введите 'up', чтобы сохранить текущий угол как верхнее положение.")
    print("Введите 'down', чтобы сохранить текущий угол как нижнее положение.")
    print("Введите 'q' для выхода.")

    current_angle = send_servo_angle(arduino, current_angle)

    try:
        while True:
            val = input(f"Угол (сейчас {current_angle}): ").strip()
            if val.lower() == "q":
                break
            if val.lower() == "up":
                up_angle = current_angle
                config["servo_up_angle"] = up_angle
                save_config(config)
                print(f"Сохранено: верхнее положение = {up_angle}°")
                continue
            if val.lower() == "down":
                down_angle = current_angle
                config["servo_down_angle"] = down_angle
                save_config(config)
                print(f"Сохранено: нижнее положение = {down_angle}°")
                continue

            try:
                angle = float(val)
            except ValueError:
                print("Неверный ввод. Введите число или 'up' / 'down'.")
                continue

            if not 0 <= angle <= 180:
                print("Ошибка: угол должен быть от 0 до 180.")
                continue

            current_angle = send_servo_angle(arduino, angle)
            print(f"Угол установлен на {current_angle}")
    finally:
        try:
            arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
