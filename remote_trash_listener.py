import socket
import json
import threading
import time

class RemoteTrashListener:
    def __init__(self, port=5005):
        self.port = port
        self.running = False
        
        # Переменные, которые читает main_slam.py
        self.trash_detected = False
        self.trash_angle = 0.0
        
        # Настройка UDP сервера
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Биндим на все интерфейсы, чтобы слушать и локальную сеть, и Tailscale
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(1.0)
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        print(f"[YOLO-REMOTE] Слушаю команды от телефона по UDP на порту {self.port}...")

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()

    def _listen_loop(self):
        last_receive_time = 0
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                message = json.loads(data.decode('utf-8'))
                
                # Обновляем состояние на основе пришедших данных от телефона
                self.trash_detected = message.get("trash_detected", False)
                self.trash_angle = message.get("angle", 0.0)
                
                last_receive_time = time.time()
                
                if self.trash_detected:
                    print(f"[YOLO-REMOTE] Получен сигнал мусора! Угол: {self.trash_angle:.1f}°")
                    
            except socket.timeout:
                # Если 2 секунды нет сообщений от телефона (отключился/завис), сбрасываем флаг
                if time.time() - last_receive_time > 2.0:
                    self.trash_detected = False
            except Exception as e:
                # Игнорируем битые пакеты
                pass
