import time
import board
import busio
from adafruit_pca9685 import PCA9685
import threading
import sys
import math
import json
import os

from ld06_driver import LD06Driver
from Algorithm.OnlineFastSlam import OnlineFastSlam
try:
    from trash_detector import TrashDetector
except ImportError:
    TrashDetector = None

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
            json.dump(cfg, f)
    except: pass

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

def set_servo_bucket(down=True):
    if not pca: return
    # ВНИМАНИЕ: Для DC моторов у нас pca.frequency = 1000. Сервоприводы обычно требуют 50 Гц!
    # Если подключить сервопривод к 1000 Гц, он может сгореть или не работать.
    # В будущем для ковша понадобится либо отдельный контроллер, либо изменить логику PWM.
    # Это программная заделка.
    state = "ОПУСКАЮ" if down else "ПОДНИМАЮ"
    print(f"[СЕРВОПРИВОД - Канал 15] {state} ковш!")
    # pca.channels[15].duty_cycle = ...

def stop_all():
    set_motors(0, 0, 0, 0)
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

def get_clearance(scan, target_angle_deg, robot_half_width=0.35):
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
    
    SAFE_DIST_FRONT = 0.60
    
    try:
        while driver.running:
            # --- ЛОГИКА СБОРА МУСОРА (YOLO) ---
            if detector and detector.trash_detected:
                stop_all()
                print("[АВТОПИЛОТ] МУСОР ОБНАРУЖЕН! Перехват управления.")
                state = "TRASH_COLLECT"
                
                # Подъезжаем (заглушка)
                set_motors(speed//2, 0, speed//2, 0)
                time.sleep(1.0)
                stop_all()
                
                # Собираем мусор
                set_servo_bucket(down=True)
                time.sleep(2.0)
                set_servo_bucket(down=False)
                time.sleep(1.0)
                
                print("[АВТОПИЛОТ] Мусор собран! Возврат к исследованию.")
                state = "FORWARD"
                detector.trash_detected = False
                continue

            # --- ЛОГИКА ИССЛЕДОВАТЕЛЯ С ЛИДАРОМ ---
            scan = driver.get_latest_scan()
            
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

def main():
    print("=== Система управления + FastSLAM + YOLO ===")
    
    config = load_config()
    
    # Запрос настроек у пользователя
    lidar_port = input(f"Введите порт лидара (Enter для {config.get('lidar_port', '/dev/ttyUSB0')}): ").strip()
    if not lidar_port: lidar_port = config.get('lidar_port', '/dev/ttyUSB0')
    config['lidar_port'] = lidar_port
        
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
    
    use_yolo = input("Запускать камеру с YOLO для сбора мусора? (y/n): ").strip().lower() == 'y'
    detector = None
    if use_yolo and TrashDetector:
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

if __name__ == "__main__":
    main()
