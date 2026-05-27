import cv2
import socket
import json
import time
from ultralytics import YOLO

# ==========================================
# НАСТРОЙКИ (ИЗМЕНИТЕ ПОД ВАШУ СЕТЬ)
# ==========================================
# Если вы используете локальный Wi-Fi через USB-модем, впишите сюда IP малины (например, 192.168.x.x)
# Если используете Tailscale, впишите 'tlabitmopy'
RPI_IP = "192.168.1.X" 

# Название вашей модели (должна лежать в той же папке на телефоне)
MODEL_NAME = "yolov8n.pt" # Можете использовать и NCNN модель
# ==========================================

print("\n========================================")
print("Какую камеру использовать для распознавания?")
print("0 - Встроенная камера телефона")
print("1 - Внешняя USB-вебкамера (вариант А)")
print("2 - Внешняя USB-вебкамера (вариант Б)")
print("========================================")
cam_choice = input("Ваш выбор (0, 1 или 2) [Enter для 0]: ").strip()
try:
    if cam_choice == "":
        CAMERA_INDEX = 0
    else:
        CAMERA_INDEX = int(cam_choice)
except ValueError:
    print("Ошибка ввода. По умолчанию используем встроенную камеру (0).")
    CAMERA_INDEX = 0
UDP_PORT = 5005

print("[INFO] Загрузка модели YOLO...")
try:
    model = YOLO(MODEL_NAME)
    print("[INFO] Модель успешно загружена!")
except Exception as e:
    print(f"[ОШИБКА] Не удалось загрузить модель: {e}")
    exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"[INFO] Подключение к локальной камере (индекс {CAMERA_INDEX})...")
cap = cv2.VideoCapture(CAMERA_INDEX)

print("[INFO] Запуск системы распознавания. Для выхода нажмите 'q'.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ОШИБКА] Нет кадра. Проверьте подключение USB-вебкамеры к телефону!")
        time.sleep(1)
        # Пробуем переподключиться
        cap = cv2.VideoCapture(CAMERA_INDEX)
        continue
        
    # Запускаем распознавание
    results = model(frame, verbose=False)
    
    detected = False
    angle = 0.0
    
    if len(results) > 0 and len(results[0].boxes) > 0:
        box = results[0].boxes[0] # Берем самый уверенный объект
        conf = float(box.conf)
        
        if conf > 0.5: # Уверенность больше 50%
            detected = True
            x1, y1, x2, y2 = box.xyxy[0]
            center_x = (float(x1) + float(x2)) / 2.0
            
            frame_width = frame.shape[1]
            fov_deg = 60.0 # Угол обзора вебкамеры
            angle = ((center_x - (frame_width / 2.0)) / frame_width) * fov_deg
            
            # Рисуем красивую рамку для экрана телефона
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            text = f"TRASH {conf*100:.0f}%  Angle: {angle:.1f}"
            cv2.putText(frame, text, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
    # Отправляем команду автопилоту на малину
    message = json.dumps({"trash_detected": detected, "angle": angle})
    try:
        sock.sendto(message.encode('utf-8'), (RPI_IP, UDP_PORT))
    except Exception as e:
        print(f"[ОШИБКА СЕТИ] Не удалось отправить команду: {e}")
    
    # Показываем видео на экране телефона
    cv2.imshow("Phone YOLO Brain", frame)
    
    # Ждем нажатия 'q' для выхода
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()
sock.close()
