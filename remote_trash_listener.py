import json
import socket
import threading
import time


class RemoteTrashListener:
    def __init__(
        self,
        port=5005,
        on_servo_command=None,
        on_motor_command=None,
        allow_text_commands=False,
    ):
        self.port = port
        self.running = False
        self.on_servo_command = on_servo_command
        self.on_motor_command = on_motor_command
        self.allow_text_commands = bool(allow_text_commands)

        self.trash_detected = False
        self.trash_angle = 0.0
        self.trash_in_collection_zone = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        print(
            f"[YOLO-REMOTE] Listening for phone commands on UDP port {self.port}..."
        )

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()

    def set_allow_text_commands(self, allowed):
        self.allow_text_commands = bool(allowed)

    def _dispatch_text_command(self, message_text):
        if not self.allow_text_commands:
            return False

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
        last_receive_time = 0.0
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                message_text = data.decode("utf-8", errors="ignore").strip()
                if not message_text:
                    continue

                if self._dispatch_text_command(message_text):
                    last_receive_time = time.time()
                    continue

                message = json.loads(message_text)
                self.trash_detected = bool(message.get("trash_detected", False))
                self.trash_angle = float(message.get("angle", 0.0))
                self.trash_in_collection_zone = bool(
                    message.get("in_collection_zone", False)
                )
                last_receive_time = time.time()

                if self.trash_detected:
                    zone_suffix = " [ZONE]" if self.trash_in_collection_zone else ""
                    print(
                        "[YOLO-REMOTE] Trash detected! "
                        f"Angle: {self.trash_angle:.1f} deg{zone_suffix}"
                    )

            except socket.timeout:
                if time.time() - last_receive_time > 2.0:
                    self.trash_detected = False
                    self.trash_in_collection_zone = False
            except Exception:
                pass
