import time
from smbus2 import SMBus

ADS1115_ADDRESS = 0x48
CONFIG_REGISTER = 0x01
CONVERSION_REGISTER = 0x00

neutral_voltage = 2.5
acidic_slope = -0.18

bus = SMBus(1)
CONFIG = 0xC283

def configure_ads1115():
    config_high = (CONFIG >> 8) & 0xFF
    config_low = CONFIG & 0xFF
    bus.write_i2c_block_data(ADS1115_ADDRESS, CONFIG_REGISTER, [config_high, config_low])

def read_voltage():
    configure_ads1115()
    time.sleep(0.1)
    data = bus.read_i2c_block_data(ADS1115_ADDRESS, CONVERSION_REGISTER, 2)
    raw_adc = (data[0] << 8) | data[1]
    if raw_adc > 32767:
        raw_adc -= 65536
    voltage = raw_adc * 4.096 / 32768.0
    return voltage

def calculate_ph(voltage):
    ph_value = 7.0 + (voltage - neutral_voltage) / acidic_slope
    return ph_value

try:
    while True:
        voltage = read_voltage()
        ph = calculate_ph(voltage)
        print(f"pH Value: {ph:.2f}")
        time.sleep(1)

except KeyboardInterrupt:
    print("Measurement stopped.")

finally:
    bus.close()
    
