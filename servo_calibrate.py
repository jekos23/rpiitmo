import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def main():
    print("Инициализация I2C и PCA9685...")
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    
    # ВАЖНО: Для сервоприводов частота должна быть ровно 50 Гц!
    pca.frequency = 50
    
    # Подключаем сервопривод к 14 каналу
    CHANNEL = 14
    servo_motor = servo.Servo(pca.channels[CHANNEL], min_pulse=500, max_pulse=2500)
    
    print(f"Сервопривод на канале {CHANNEL} готов к калибровке.")
    print("Вводите угол от 0 до 180 (или 'q' для выхода).")
    
    while True:
        val = input("Угол (0-180): ")
        if val.lower() == 'q':
            break
        
        try:
            angle = float(val)
            if 0 <= angle <= 180:
                servo_motor.angle = angle
                print(f"Угол установлен на {angle}")
            else:
                print("Ошибка: Угол должен быть от 0 до 180.")
        except ValueError:
            print("Неверный ввод. Введите число.")
            
    # Отключаем ШИМ после выхода
    pca.channels[CHANNEL].duty_cycle = 0
    print("Калибровка завершена.")

if __name__ == "__main__":
    main()
