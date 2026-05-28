import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import json
import os

CONFIG_FILE = "config.json"

def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except: pass
    return {}

def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=4)
    except: pass

def main():
    print("Инициализация I2C и PCA9685...")
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    
    # ВАЖНО: Для сервоприводов частота должна быть ровно 50 Гц!
    pca.frequency = 50
    
    # Подключаем сервопривод к 14 каналу
    CHANNEL = 14
    servo_motor = servo.Servo(pca.channels[CHANNEL], min_pulse=500, max_pulse=2500)
    
    config = load_config()
    up_angle = config.get("servo_up_angle", 0)
    down_angle = config.get("servo_down_angle", 180)
    
    print(f"Сервопривод на канале {CHANNEL} готов к калибровке.")
    print("Вводите угол от 0 до 180.")
    print("Введите 'up', чтобы сохранить ТЕКУЩИЙ угол как ВЕРХНЕЕ (нулевое) положение.")
    print("Введите 'down', чтобы сохранить ТЕКУЩИЙ угол как НИЖНЕЕ (сбор мусора) положение.")
    print("Введите 'q' для выхода.")
    
    current_angle = 90
    servo_motor.angle = current_angle
    
    while True:
        val = input(f"Угол (сейчас {current_angle}): ").strip()
        if val.lower() == 'q':
            break
        elif val.lower() == 'up':
            up_angle = current_angle
            config["servo_up_angle"] = up_angle
            save_config(config)
            print(f"✅ Сохранено: Верхнее положение = {up_angle}°")
            continue
        elif val.lower() == 'down':
            down_angle = current_angle
            config["servo_down_angle"] = down_angle
            save_config(config)
            print(f"✅ Сохранено: Нижнее положение = {down_angle}°")
            continue
        
        try:
            angle = float(val)
            if 0 <= angle <= 180:
                current_angle = angle
                servo_motor.angle = current_angle
                print(f"Угол установлен на {current_angle}")
            else:
                print("Ошибка: Угол должен быть от 0 до 180.")
        except ValueError:
            print("Неверный ввод. Введите число или 'up' / 'down'.")
            
    # Отключаем ШИМ после выхода
    pca.channels[CHANNEL].duty_cycle = 0
    print("Калибровка завершена.")

if __name__ == "__main__":
    main()
