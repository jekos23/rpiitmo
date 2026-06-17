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
        self.trash_confidence = 0.0
        self.trash_confirmed_for_ms = 0
        self.current_target_id = None
        self.targets = []
        self.route_mode_enabled = False
        self.within_route_corridor = True
        self.route_distance_m = None
        self.route_data_fresh = False
        self.position = None
        self.last_packet_time = 0.0
        self._ignored_target_ids = {}
        self._lock = threading.Lock()

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

    def _cleanup_ignored_targets(self, now=None):
        if now is None:
            now = time.time()
        expired_ids = [
            target_id
            for target_id, expires_at in self._ignored_target_ids.items()
            if expires_at <= now
        ]
        for target_id in expired_ids:
            self._ignored_target_ids.pop(target_id, None)

    def _refresh_primary_target_locked(self):
        primary_target = self.targets[0] if self.targets else None
        if primary_target is None:
            self.trash_detected = False
            self.trash_angle = 0.0
            self.trash_in_collection_zone = False
            self.trash_confidence = 0.0
            self.trash_confirmed_for_ms = 0
            self.current_target_id = None
            return

        self.trash_detected = True
        self.trash_angle = float(primary_target.get("angle", 0.0))
        self.trash_in_collection_zone = bool(
            primary_target.get("in_collection_zone", False)
        )
        self.trash_confidence = float(primary_target.get("confidence", 0.0))
        self.trash_confirmed_for_ms = int(
            primary_target.get("confirmed_for_ms", 0) or 0
        )
        self.current_target_id = primary_target.get("id")

    def _normalize_target(self, raw_target, fallback_id=1):
        if not isinstance(raw_target, dict):
            return None

        raw_id = raw_target.get("id", fallback_id)
        try:
            target_id = int(raw_id)
        except (TypeError, ValueError):
            target_id = int(fallback_id)

        try:
            angle = float(raw_target.get("angle", 0.0))
        except (TypeError, ValueError):
            angle = 0.0

        try:
            confidence = float(raw_target.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        try:
            confirmed_for_ms = int(raw_target.get("confirmed_for_ms", 0) or 0)
        except (TypeError, ValueError):
            confirmed_for_ms = 0

        return {
            "id": target_id,
            "angle": angle,
            "in_collection_zone": bool(raw_target.get("in_collection_zone", False)),
            "confidence": confidence,
            "confirmed_for_ms": max(0, confirmed_for_ms),
            "area_ratio": raw_target.get("area_ratio"),
        }

    def _extract_targets(self, message):
        parsed_targets = []
        raw_targets = message.get("targets")
        if isinstance(raw_targets, list):
            for index, raw_target in enumerate(raw_targets, start=1):
                normalized = self._normalize_target(raw_target, fallback_id=index)
                if normalized is not None:
                    parsed_targets.append(normalized)

        if parsed_targets:
            return parsed_targets

        if not bool(message.get("trash_detected", False)):
            return []

        legacy_target = self._normalize_target(
            {
                "id": message.get("target_id", 1),
                "angle": message.get("angle", 0.0),
                "in_collection_zone": message.get("in_collection_zone", False),
                "confidence": message.get("confidence", 0.0),
                "confirmed_for_ms": message.get("confirmed_for_ms", 0),
            },
            fallback_id=1,
        )
        return [legacy_target] if legacy_target is not None else []

    def get_primary_target(self, exclude_ids=None):
        excluded = set(exclude_ids or [])
        with self._lock:
            self._cleanup_ignored_targets()
            for target in self.targets:
                target_id = target.get("id")
                if target_id in excluded or target_id in self._ignored_target_ids:
                    continue
                return dict(target)
        return None

    def get_target_by_id(self, target_id):
        if target_id is None:
            return None
        try:
            target_id = int(target_id)
        except (TypeError, ValueError):
            return None

        with self._lock:
            self._cleanup_ignored_targets()
            if target_id in self._ignored_target_ids:
                return None
            for target in self.targets:
                if target.get("id") == target_id:
                    return dict(target)
        return None

    def ignore_target(self, target_id, cooldown_sec=6.0):
        if target_id is None:
            return
        try:
            target_id = int(target_id)
        except (TypeError, ValueError):
            return

        expires_at = time.time() + max(0.5, float(cooldown_sec))
        with self._lock:
            self._cleanup_ignored_targets()
            self._ignored_target_ids[target_id] = expires_at
            self.targets = [t for t in self.targets if t.get("id") != target_id]
            self._refresh_primary_target_locked()

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
                route_mode_enabled = bool(message.get("route_mode_enabled", False))
                within_route_corridor = bool(
                    message.get("within_route_corridor", not route_mode_enabled)
                )
                route_distance = message.get("route_distance_m")
                try:
                    route_distance_m = (
                        float(route_distance) if route_distance is not None else None
                    )
                except (TypeError, ValueError):
                    route_distance_m = None
                position = message.get("position")
                normalized_position = position if isinstance(position, dict) else None
                parsed_targets = self._extract_targets(message)
                last_receive_time = time.time()
                with self._lock:
                    self.route_mode_enabled = route_mode_enabled
                    self.within_route_corridor = within_route_corridor
                    self.route_distance_m = route_distance_m
                    self.position = normalized_position
                    self.route_data_fresh = True
                    self.last_packet_time = last_receive_time
                    self._cleanup_ignored_targets(last_receive_time)
                    self.targets = [
                        target
                        for target in parsed_targets
                        if target.get("id") not in self._ignored_target_ids
                    ]
                    self._refresh_primary_target_locked()
                    primary_target = dict(self.targets[0]) if self.targets else None

                if primary_target is not None:
                    zone_suffix = (
                        " [ZONE]"
                        if primary_target.get("in_collection_zone", False)
                        else ""
                    )
                    print(
                        "[YOLO-REMOTE] Trash detected! "
                        f"Angle: {float(primary_target.get('angle', 0.0)):.1f} deg{zone_suffix}"
                    )

            except socket.timeout:
                if time.time() - last_receive_time > 2.0:
                    with self._lock:
                        self.targets = []
                        self._refresh_primary_target_locked()
                        self.route_data_fresh = False
                        if self.route_mode_enabled:
                            self.within_route_corridor = False
                            self.route_distance_m = None
            except Exception:
                pass
