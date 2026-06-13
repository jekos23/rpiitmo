import json
import os
import time

import board
import busio
from adafruit_pca9685 import PCA9685


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "config.json")

SERVO_PCA_DEFAULT_ADDRESS = 0x41
BUCKET_SERVO_DEFAULT_CHANNEL = 0
BUCKET_SERVO_MIN_PULSE_US = 500
BUCKET_SERVO_MAX_PULSE_US = 2500


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=4)
    except Exception:
        pass


def parse_i2c_address(value, default):
    if value in (None, ""):
        return int(default)
    if isinstance(value, str):
        text = value.strip().lower()
        if not text:
            return int(default)
        base = 16 if text.startswith("0x") else 10
        return int(text, base)
    return int(value)


def clamp_channel(value, default=BUCKET_SERVO_DEFAULT_CHANNEL):
    try:
        channel = int(value)
    except (TypeError, ValueError):
        channel = default
    return max(0, min(15, channel))


def angle_to_duty_cycle(angle, config):
    angle = max(0.0, min(180.0, float(angle)))
    min_pulse_us = int(config.get("bucket_servo_min_pulse_us", BUCKET_SERVO_MIN_PULSE_US))
    max_pulse_us = int(config.get("bucket_servo_max_pulse_us", BUCKET_SERVO_MAX_PULSE_US))
    max_pulse_us = max(min_pulse_us + 100, max_pulse_us)
    pulse_us = min_pulse_us + ((max_pulse_us - min_pulse_us) * (angle / 180.0))
    return max(0, min(65535, int((pulse_us / 20000.0) * 65535)))


def set_servo_angle(servo_board, channel, angle, config):
    angle = max(0, min(180, int(angle)))
    servo_board.channels[channel].duty_cycle = angle_to_duty_cycle(angle, config)
    time.sleep(max(0.05, float(config.get("bucket_servo_move_sec", 0.8))))
    servo_board.channels[channel].duty_cycle = 0
    return angle


def main():
    config = load_config()

    current_address = str(config.get("servo_pca_address", hex(SERVO_PCA_DEFAULT_ADDRESS)))
    entered_address = input(
        f"Servo PCA I2C address [Enter for {current_address}]: "
    ).strip()
    if entered_address:
        current_address = entered_address
    config["servo_pca_address"] = current_address

    current_channel = clamp_channel(
        config.get("bucket_servo_channel", BUCKET_SERVO_DEFAULT_CHANNEL)
    )
    entered_channel = input(
        f"Bucket servo channel [Enter for {current_channel}]: "
    ).strip()
    if entered_channel:
        current_channel = clamp_channel(entered_channel, current_channel)
    config["bucket_servo_channel"] = current_channel

    config.setdefault("servo_up_angle", 0)
    config.setdefault("servo_down_angle", 90)
    save_config(config)

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        servo_board = PCA9685(
            i2c,
            address=parse_i2c_address(current_address, SERVO_PCA_DEFAULT_ADDRESS),
        )
        servo_board.frequency = 50
    except Exception as exc:
        print(f"Failed to initialize servo PCA9685: {exc}")
        return

    up_angle = int(config.get("servo_up_angle", 0))
    down_angle = int(config.get("servo_down_angle", 90))
    current_angle = down_angle

    print("Direct servo calibration via PCA9685.")
    print("Enter angle 0..180, 'up', 'down' or 'q'.")

    current_angle = set_servo_angle(servo_board, current_channel, current_angle, config)

    while True:
        value = input(f"Angle [current {current_angle}]: ").strip().lower()
        if value == "q":
            break
        if value == "up":
            up_angle = current_angle
            config["servo_up_angle"] = up_angle
            save_config(config)
            print(f"Saved top position: {up_angle} deg")
            continue
        if value == "down":
            down_angle = current_angle
            config["servo_down_angle"] = down_angle
            save_config(config)
            print(f"Saved lower/search position: {down_angle} deg")
            continue

        try:
            current_angle = set_servo_angle(
                servo_board,
                current_channel,
                float(value),
                config,
            )
            print(f"Servo moved to {current_angle} deg")
        except ValueError:
            print("Enter a number, 'up', 'down' or 'q'.")


if __name__ == "__main__":
    main()
