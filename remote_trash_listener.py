import json
import socket
import threading
import time


class RemoteTrashListener:
    def __init__(self, port=5005, on_servo_command=None, on_motor_command=None):
        self.port = port
        self.running = False
        self.on_servo_command = on_servo_command
        self.on_motor_command = on_motor_command

        self.trash_detected = False
        self.trash_angle = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
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

    def _dispatch_text_command(self, message_text):
        upper_text = message_text.upper()

        if upper_text.startswith("SERVO:"):
            payload = message_text.split(":", 1)[1].strip()
            if self.on_servo_command:
                self.on_servo_command(payload)
            return True

        if upper_text.startswith("MOTOR:"):
            payload = message_text.split(":", 1)[1].strip()
            if self.on_motor_command:
                self.on_motor_command(payload)
            return True

        return False

    def _listen_loop(self):
        last_receive_time = 0
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                message_text = data.decode("utf-8", errors="ignore").strip()
                if not message_text:
                    continue

                if self._dispatch_text_command(message_text):
                    last_receive_time = time.time()
                    continue

                message = json.loads(message_text)
                self.trash_detected = message.get("trash_detected", False)
                self.trash_angle = message.get("angle", 0.0)
                last_receive_time = time.time()

                if self.trash_detected:
                    print(f"[YOLO-REMOTE] Получен сигнал мусора! Угол: {self.trash_angle:.1f}°")

            except socket.timeout:
                if time.time() - last_receive_time > 2.0:
                    self.trash_detected = False
            except Exception:
                pass
