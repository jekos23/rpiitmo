import time
import sys

# Если вы используете АЦП ADS1115 (через I2C)
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
except ImportError:
    print("Установите библиотеку: pip install adafruit-circuitpython-ads1x15")
    print("Если у вас другой АЦП (например MCP3008), код нужно будет немного изменить.")

def test_potentiometer():
    print("=== ТЕСТ ПОТЕНЦИОМЕТРА СОВКА ===")
    
    try:
        # Инициализация I2C и АЦП
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        
        # Подключение потенциометра к пину A0 на ADS1115
        chan = AnalogIn(ads, ADS.P0)
        
        print("Чтение данных с потенциометра (Пин A0)... Нажмите Ctrl+C для выхода.")
        
        while True:
            # chan.value возвращает сырое значение от 0 до 32767
            # chan.voltage возвращает напряжение
            raw_value = chan.value
            voltage = chan.voltage
            
            # TODO: Позже вы сможете преобразовать raw_value в градусы лопаты
            # Например: degrees = (raw_value / 32767) * 180
            
            print(f"Сырое значение: {raw_value:>6} | Напряжение: {voltage:>5.2f} В")
            time.sleep(0.2)
            
    except Exception as e:
        print(f"\nОшибка при чтении АЦП: {e}")
        print("Убедитесь, что потенциометр подключен к АЦП, а АЦП - к Raspberry Pi по I2C.")

if __name__ == "__main__":
    test_potentiometer()
