import serial
import time
import math
import threading

class LD06Driver:
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.scan_data = [0.0] * 360 # 360 degrees buckets
        self.lock = threading.Lock()
        
    def start(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"[{self.__class__.__name__}] Connected to LD-06 on {self.port}")
        except Exception as e:
            print(f"[{self.__class__.__name__}] Failed to connect to LD-06: {e}")
            self.running = False

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()

    def get_latest_scan(self):
        with self.lock:
            # Return a copy of the current 360 scan data (in meters)
            return list(self.scan_data)

    def _read_loop(self):
        while self.running and self.serial:
            # Read until we find the header 0x54
            header = self.serial.read(1)
            if not header or header[0] != 0x54:
                continue
            
            # Read the rest of the 47-byte packet (46 bytes left)
            packet = self.serial.read(46)
            if len(packet) < 46:
                continue
                
            self._parse_packet(header + packet)

    def _parse_packet(self, packet):
        # packet[0] = 0x54
        # packet[1] = ver_len (usually 0x2C)
        # packet[2:4] = speed
        # packet[4:6] = start_angle
        # packet[6:42] = 12 points (dist, intensity)
        # packet[42:44] = end_angle
        # packet[44:46] = timestamp
        # packet[46] = crc8
        
        start_angle = int.from_bytes(packet[4:6], byteorder='little') / 100.0
        end_angle = int.from_bytes(packet[42:44], byteorder='little') / 100.0
        
        # Handle angle wrap around (e.g. 350 to 10)
        diff = end_angle - start_angle
        if diff < 0:
            diff += 360.0
            
        step = diff / 11.0 if diff > 0 else 0
        
        with self.lock:
            for i in range(12):
                angle = start_angle + (step * i)
                if angle >= 360.0:
                    angle -= 360.0
                    
                idx = 6 + (i * 3)
                dist_mm = int.from_bytes(packet[idx:idx+2], byteorder='little')
                # intensity = packet[idx+2]
                
                # We need the angle mapped to -180 to 180 as FastSLAM expects it
                # The SLAM uses array index from 0 to numSamples.
                # If lidarFOV = 2*pi, it maps index 0 to -pi, and index 359 to +pi.
                # Let's map LD06 angle (0-359) to index 0-359 where index 0 is -180 deg
                # So we shift by 180 degrees
                
                shifted_angle = angle - 180.0
                if shifted_angle < -180.0: shifted_angle += 360.0
                if shifted_angle >= 180.0: shifted_angle -= 360.0
                
                # Convert shifted_angle (-180 to 180) to array index (0 to 359)
                array_index = int((shifted_angle + 180.0)) % 360
                
                # Save distance in meters
                self.scan_data[array_index] = dist_mm / 1000.0

if __name__ == '__main__':
    driver = LD06Driver('COM3') # For windows test
    driver.start()
    try:
        while True:
            scan = driver.get_latest_scan()
            # print front distance
            print(f"Front: {scan[0]:.2f}m, Back: {scan[180]:.2f}m")
            time.sleep(0.5)
    except KeyboardInterrupt:
        driver.stop()
