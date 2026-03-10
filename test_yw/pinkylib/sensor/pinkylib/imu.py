import smbus2
import time
import struct

class IMU:
    # BNO055 I2C 주소
    BNO055_ADDRESS = 0x28

    # 레지스터 주소 정의
    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_SYS_STAT_ADDR = 0x39
    BNO055_BL_REV_ID_ADDR = 0x3A # Bootloader Status
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_PWR_MODE_ADDR = 0x3E
    BNO055_SYS_TRIGGER_ADDR = 0x3F

    # 데이터 시작 주소
    BNO055_ACC_DATA_X_LSB_ADDR = 0x08
    BNO055_GYR_DATA_X_LSB_ADDR = 0x14
    BNO055_QUAT_DATA_W_LSB_ADDR = 0x20

    # 운용 모드
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_IMU = 0x08
    
    # 전원 모드
    POWER_MODE_NORMAL = 0x00

    def __init__(self, i2c_bus=0):
        try:
            self.bus = smbus2.SMBus(i2c_bus)
        except FileNotFoundError:
            raise RuntimeError(f"I2C 버스 {i2c_bus}를 찾을 수 없습니다. I2C가 활성화되었는지 확인하세요.")

        chip_id = self.bus.read_byte_data(self.BNO055_ADDRESS, self.BNO055_CHIP_ID_ADDR)
        if chip_id != 0xA0:
            raise RuntimeError("BNO055 센서가 감지되지 않았습니다. 연결을 확인하세요.")

        # 1. 시스템 리셋
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_SYS_TRIGGER_ADDR, 0x20)
        
        # 리셋이 완료될 때까지 대기 (Bootloader Status == 0)
        while True:
            try:
                status = self.bus.read_byte_data(self.BNO055_ADDRESS, self.BNO055_BL_REV_ID_ADDR)
                if status == 0:
                    break
            except IOError:
                pass
            time.sleep(0.1)
        print("리셋 완료.")
        time.sleep(0.5)

        # 2. 전원 모드 및 운용 모드 설정
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)
        time.sleep(0.05)
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_OPR_MODE_ADDR, self.OPERATION_MODE_IMU)
        time.sleep(0.05)
        
        # 3. 센서 퓨전 알고리즘이 실행될 때까지 대기 (System Status == 0x05)
        while True:
            status = self.bus.read_byte_data(self.BNO055_ADDRESS, self.BNO055_SYS_STAT_ADDR)
            if status == 0x05:
                break
            self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_OPR_MODE_ADDR, self.OPERATION_MODE_IMU)
            time.sleep(0.1)

    def _bytes_to_int16(self, lsb, msb):
        return struct.unpack('<h', bytes([lsb, msb]))[0]

    def read_imu_data(self):
        """
        가속도, 자이로, 쿼터니언 데이터를 한 번에 읽어와 딕셔너리로 반환합니다.
        """
        data = self.bus.read_i2c_block_data(self.BNO055_ADDRESS, self.BNO055_ACC_DATA_X_LSB_ADDR, 32)

        # 가속도 데이터 파싱 (단위: m/s^2)
        acc_x = self._bytes_to_int16(data[0], data[1]) / 100.0
        acc_y = self._bytes_to_int16(data[2], data[3]) / 100.0
        acc_z = self._bytes_to_int16(data[4], data[5]) / 100.0

        # 자이로 데이터 파싱 (단위: deg/s, ROS 표준은 rad/s)
        gyro_x = self._bytes_to_int16(data[12], data[13]) / 16.0
        gyro_y = self._bytes_to_int16(data[14], data[15]) / 16.0
        gyro_z = self._bytes_to_int16(data[16], data[17]) / 16.0

        # 쿼터니언 데이터 파싱 (단위 없음, 정규화된 값)
        quat_w = self._bytes_to_int16(data[24], data[25]) / 16384.0
        quat_x = self._bytes_to_int16(data[26], data[27]) / 16384.0
        quat_y = self._bytes_to_int16(data[28], data[29]) / 16384.0
        quat_z = self._bytes_to_int16(data[30], data[31]) / 16384.0
        
        return {
            'acceleration': (acc_x, acc_y, acc_z),
            'gyro': (gyro_x, gyro_y, gyro_z),
            'quaternion': (quat_w, quat_x, quat_y, quat_z)
        }

    def close(self):
        self.bus.close()
