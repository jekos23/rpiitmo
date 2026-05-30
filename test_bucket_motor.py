import time
import sys

try:
    # Для связи с Arduino
    import serial
    
    # Для управления моторами (если через PCA9685 как основные колеса)
    from adafruit_pca9685 import PCA9685
    import board
    import busio
except ImportError:
    print("Установите библиотеки: pip install pyserial adafruit-circuitpython-pca9685")
    sys.exit(1)

# === 1. НАСТРОЙКА АЦП (ЧЕРЕЗ ARDUINO) ===
# Порт может отличаться (на Raspberry Pi это обычно /dev/ttyUSB0 или /dev/ttyACM0)
arduino_port = "/dev/ttyUSB0" 
try:
    arduino = serial.Serial(arduino_port, 9600, timeout=1)
    # Даем Arduino время на перезагрузку после подключения
    time.sleep(2)
except Exception as e:
    print(f"Не удалось подключиться к Arduino на {arduino_port}: {e}")
    print("Проверьте порт с помощью команды: ls /dev/tty*")
    sys.exit(1)

# === 2. НАСТРОЙКА ДРАЙВЕРА МОТОРА (PCA9685) ===
i2c = busio.I2C(board.SCL, board.SDA)
# Если мотор совка подключен к тому же драйверу, что и колеса, 
# вам нужно указать правильные пины (каналы) для него!
# Например, каналы 8 и 9 для мотора совка:
pca = PCA9685(i2c)
pca.frequency = 50
MOTOR_PIN_FWD = 8  # Укажите свой канал для движения вперед
MOTOR_PIN_BWD = 9  # Укажите свой канал для движения назад

def set_bucket_motor(speed):
    """
    Управляет мотором совка. 
    speed от -4095 (назад) до 4095 (вперед).
    """
    speed = max(-4095, min(4095, int(speed)))
    
    if speed > 0:
        pca.channels[MOTOR_PIN_FWD].duty_cycle = speed * 16
        pca.channels[MOTOR_PIN_BWD].duty_cycle = 0
    elif speed < 0:
        pca.channels[MOTOR_PIN_FWD].duty_cycle = 0
        pca.channels[MOTOR_PIN_BWD].duty_cycle = abs(speed) * 16
    else:
        pca.channels[MOTOR_PIN_FWD].duty_cycle = 0
        pca.channels[MOTOR_PIN_BWD].duty_cycle = 0

def get_bucket_angle():
    """
    Читает значение с Arduino (которое присылает данные потенциометра)
    """
    try:
        # Очищаем буфер, чтобы всегда читать самое свежее значение
        arduino.reset_input_buffer()
        # Читаем строку, убираем пробелы и символы переноса
        line = arduino.readline().decode('utf-8').strip()
        
        if line and line.isdigit():
            raw = int(line)
            # Значение с Arduino analogRead() будет от 0 до 1023
            return raw
    except Exception as e:
        print(f"Ошибка чтения с Arduino: {e}")
    
    return 0

def move_bucket_to_target(target_value, tolerance=500):
    """
    ПИ-регулятор (или просто пропорциональный), который крутит мотор, 
    пока потенциометр не покажет нужное значение.
    """
    print(f"Цель: {target_value}. Начинаю движение...")
    
    Kp = 0.5 # Коэффициент пропорциональности (настроить под свой мотор)
    
    while True:
        current_val = get_bucket_angle()
        error = target_value - current_val
        
        print(f"Текущее значение: {current_val:>6} | Ошибка: {error:>6}")
        
        # Если мы достаточно близки к цели - останавливаемся
        if abs(error) < tolerance:
            print("Цель достигнута!")
            set_bucket_motor(0)
            break
            
        # Рассчитываем скорость в зависимости от того, насколько далеко мы от цели
        speed = error * Kp
        
        # Ограничиваем минимальную скорость, чтобы мотору хватало сил сдвинуться (deadzone)
        if speed > 0 and speed < 1000: speed = 1000
        if speed < 0 and speed > -1000: speed = -1000
        
        set_bucket_motor(speed)
        time.sleep(0.05)

if __name__ == "__main__":
    try:
        print("=== ТЕСТ: ЧТЕНИЕ ПОТЕНЦИОМЕТРА ===")
        # 1. Сначала просто выведем показания 10 раз, чтобы вы увидели значения
        for _ in range(20):
            val = get_bucket_angle()
            print(f"Позиция потенциометра (энкодера): {val}")
            time.sleep(0.5)
            
        print("\n=== ТЕСТ: УПРАВЛЕНИЕ МОТОРОМ ПО ПОТЕНЦИОМЕТРУ ===")
        # 2. Вы можете раскомментировать строку ниже и указать целевое значение,
        #    к которому мотор должен приехать сам:
        # move_bucket_to_target(target_value=15000, tolerance=500)
        
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        set_bucket_motor(0) # Обязательно выключаем мотор при выходе
