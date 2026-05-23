import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Инициализация I2C шины и PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)

# Частота ШИМ для моторов (обычно для BTS7960 хорошо подходит 1000 Гц)
pca.frequency = 1000

# === Назначение каналов PCA9685 ===
# Левые моторы (3 штуки)
LEFT_FWD_CHANNELS = [0, 3, 5] # Каналы для движения вперед (например, к RPWM)
LEFT_REV_CHANNELS = [1, 2, 4] # Каналы для движения назад (например, к LPWM)

# Правые моторы (3 штуки)
RIGHT_FWD_CHANNELS = [7, 9, 11]
RIGHT_REV_CHANNELS = [6, 8, 10]

def set_motors(left_fwd, left_rev, right_fwd, right_rev):
    """
    Функция для установки скорости моторов.
    Библиотека Adafruit использует 16-битные значения (0 - 65535).
    Мы будем принимать значения от 0 до 4095 (стандартный 12-битный ШИМ) 
    и масштабировать их.
    """
    # Масштабирование скорости (0-4095) -> (0-65535)
    def scale_speed(speed):
        speed = max(0, min(4095, speed)) # Ограничиваем от 0 до 4095
        return int((speed / 4095.0) * 65535)

    l_fwd_pwm = scale_speed(left_fwd)
    l_rev_pwm = scale_speed(left_rev)
    r_fwd_pwm = scale_speed(right_fwd)
    r_rev_pwm = scale_speed(right_rev)

    # Применяем значения к левым моторам
    for ch in LEFT_FWD_CHANNELS:
        pca.channels[ch].duty_cycle = l_fwd_pwm
    for ch in LEFT_REV_CHANNELS:
        pca.channels[ch].duty_cycle = l_rev_pwm

    # Применяем значения к правым моторам
    for ch in RIGHT_FWD_CHANNELS:
        pca.channels[ch].duty_cycle = r_fwd_pwm
    for ch in RIGHT_REV_CHANNELS:
        pca.channels[ch].duty_cycle = r_rev_pwm

def stop_all():
    set_motors(0, 0, 0, 0)
    print("Моторы остановлены.")

def main():
    print("Система управления моторами запущена!")
    print("Формат ввода: 'Режим Скорость' (например, '1 200').")
    print("Режимы:")
    print("  1 - Вперед")
    print("  2 - Назад")
    print("  3 - Влево (правые вперед, левые назад)")
    print("  4 - Вправо (левые вперед, правые назад)")
    print("  0 - Остановка")
    print("Скорость: от 0 до 4095 (максимум). Нажмите Ctrl+C для выхода.\n")
    
    stop_all()

    try:
        while True:
            cmd = input("Введите команду: ").strip()
            
            # Пропускаем пустой ввод
            if not cmd:
                continue
                
            try:
                parts = cmd.split()
                mode = int(parts[0])
                
                if mode == 0:
                    stop_all()
                    continue
                    
                # Если передан только режим, ставим скорость по умолчанию (например, 1000)
                speed = int(parts[1]) if len(parts) > 1 else 1000
                
                if mode == 1:
                    print(f"Движение ВПЕРЕД на скорости {speed}")
                    set_motors(speed, 0, speed, 0)
                elif mode == 2:
                    print(f"Движение НАЗАД на скорости {speed}")
                    set_motors(0, speed, 0, speed)
                elif mode == 3:
                    print(f"Поворот ВЛЕВО на скорости {speed}")
                    # левые назад, правые вперед
                    set_motors(speed, 0, 0, speed)
                elif mode == 4:
                    print(f"Поворот ВПРАВО на скорости {speed}")
                    # левые вперед, правые назад
		    set_motors(0, speed, speed, 0)
                else:
                    print("Неизвестный режим! Используйте 0, 1, 2, 3 или 4.")
                    
            except ValueError:
                print("Ошибка ввода. Используйте числа, например '1 200'.")

    except KeyboardInterrupt:
        print("\nПрограмма завершена пользователем. Отключение моторов...")
    finally:
        # Всегда отключаем моторы при выходе из программы
        stop_all()

if __name__ == "__main__":
    main()