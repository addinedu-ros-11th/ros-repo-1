# ir_sensors.py
import smbus2
import time

class IR:
    DEFAULT_I2C_BUS = 1
    DEFAULT_I2C_ADDRESS = 0x08
    
    REG_IR_SENSORS = [0x98, 0xC8, 0x88] 

    def __init__(self, i2c_bus=DEFAULT_I2C_BUS, i2c_address=DEFAULT_I2C_ADDRESS , wait_time=0.001):
        self.i2c_address = i2c_address
        self.bus = None
        self.wait_time = wait_time
        try:
            self.bus = smbus2.SMBus(i2c_bus)
        except FileNotFoundError:
            print(f"Error: I2C 버스 {i2c_bus}를 찾을 수 없습니다.")
            raise

    def _read_adc_channel(self, register_cmd):
        if not self.bus:
            raise IOError("I2C 버스가 초기화되지 않았습니다.")
        
        try:
            self.bus.write_byte(self.i2c_address, register_cmd)
            time.sleep(self.wait_time) 
            data = self.bus.read_i2c_block_data(self.i2c_address, 0, 2)
            return (data[0] << 4) | (data[1] >> 4)
        except OSError as e:
            print(f"I2C 통신 오류 (주소: 0x{self.i2c_address:02x}): {e}")
            return None

    def read_ir(self):
        values = []
        for reg in self.REG_IR_SENSORS:
            val = self._read_adc_channel(reg)
            if val is None:
                return None
            values.append(val)
        return values
        
    def close(self):
        if self.bus:
            self.bus.close()