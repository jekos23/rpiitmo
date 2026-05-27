import cv2
import time
import threading

class TrashDetector:
    def __init__(self, model_path=None, camera_index=0):
        self.running = False
        self.trash_detected = False
        self.trash_angle = 0.0 # Угол направления на мусор (относительно центра камеры)
        self.trash_distance = 0.0 # Оценочная дистанция до мусора
        
        self.cap = None
        self.model = None
        
        # Настройка камеры (низкое разрешение для производительности RPi)
        try:
            self.cap = cv2.VideoCapture(camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            self.cap.set(cv2.CAP_PROP_FPS, 15)
        except Exception as e:
            print(f"[YOLO] Ошибка инициализации камеры: {e}")
            
        # Заделка под YOLO
        if model_path:
            try:
                from ultralytics import YOLO
                print(f"[YOLO] Загрузка модели из {model_path}...")
                self.model = YOLO(model_path)
            except ImportError:
                print("[YOLO] ОШИБКА: Не установлена библиотека ultralytics! (pip install ultralytics)")
            except Exception as e:
                print(f"[YOLO] Ошибка загрузки модели: {e}")

    def start(self):
        if not self.cap or not self.cap.isOpened():
            print("[YOLO] Камера недоступна. Детектор мусора отключен.")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._process_loop, daemon=True)
        self.thread.start()
        print("[YOLO] Детектор мусора запущен.")

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()

    def _process_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
                
            detected = False
            angle = 0.0
            
            # --- ИНФЕРЕНС YOLO ---
            if self.model:
                try:
                    # Запускаем распознавание
                    results = self.model(frame, verbose=False)
                    
                    if len(results) > 0 and len(results[0].boxes) > 0:
                        box = results[0].boxes[0]
                        conf = float(box.conf)
                        
                        # Порог уверенности (50%)
                        if conf > 0.5:
                            detected = True
                            
                            # Координаты центра найденного объекта
                            x1, y1, x2, y2 = box.xyxy[0]
                            center_x = (float(x1) + float(x2)) / 2.0
                            
                            frame_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                            if frame_width == 0: frame_width = 320
                            
                            # Примерная оценка угла (исходим из того, что угол обзора камеры ~60 градусов)
                            fov_deg = 60.0
                            angle = ((center_x - (frame_width / 2.0)) / frame_width) * fov_deg
                except Exception as e:
                    print(f"[YOLO] Ошибка во время обработки кадра: {e}")
                    time.sleep(1) 
                
            # Для теста можно возвращать True, если вы захотите протестировать сервопривод
            # detected = False 
            
            self.trash_detected = detected
            self.trash_angle = angle
            
            time.sleep(0.2) # 5 FPS инференс для экономии ресурсов CPU на Raspberry Pi
