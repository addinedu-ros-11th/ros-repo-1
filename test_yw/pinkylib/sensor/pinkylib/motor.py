from dynamixel_sdk import *
import time

#===== 다이나믹셀 제어 테이블 주소 (XM/XH 시리즈 기준) =====
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_VELOCITY      = 104
ADDR_PRESENT_VELOCITY   = 128
ADDR_OPERATING_MODE     = 11

#===== 프로토콜 버전 =====
PROTOCOL_VERSION        = 2.0

#===== 기본 설정 =====
LEFT_MOTOR_ID           = 1
RIGHT_MOTOR_ID          = 2
BAUDRATE                = 1000000
DEVICE_NAME             = "/dev/ttyAMA4"

# 토크 및 속도 값
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
VELOCITY_CONTROL_MODE   = 1

class Motor:
    def __init__(self):
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            raise IOError(f"포트 {DEVICE_NAME} 열기에 실패했습니다.")
        
        if not self.portHandler.setBaudRate(BAUDRATE):
            raise IOError(f"보드레이트 {BAUDRATE} 설정에 실패했습니다.")
            
        self._set_operating_mode(LEFT_MOTOR_ID, VELOCITY_CONTROL_MODE)
        self._set_operating_mode(RIGHT_MOTOR_ID, VELOCITY_CONTROL_MODE)

        self.left_motor_dir = False
        self.right_motor_dir = False

        self.max_velocity_value = 415 

    def _set_operating_mode(self, motor_id, mode):
        """모터의 운용 모드를 설정합니다."""
        self.disable_motor(motor_id)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(f"%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"모터 #{motor_id}를 성공적으로 '속도 제어 모드'로 설정했습니다.")
        self.enable_motor(motor_id)

    def _speed_to_dxl_velocity(self, speed):
        # 속도 범위를 -100 ~ 100으로 제한
        speed = max(-100, min(100, speed))
        return int((speed / 100.0) * self.max_velocity_value)

    def enable_motor(self, motor_id=None):
        ids_to_enable = [LEFT_MOTOR_ID, RIGHT_MOTOR_ID] if motor_id is None else [motor_id]
        for id in ids_to_enable:
            self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_motor(self, motor_id=None):
        ids_to_disable = [LEFT_MOTOR_ID, RIGHT_MOTOR_ID] if motor_id is None else [motor_id]
        for id in ids_to_disable:
            self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            
    def set_left_motor_dir(self, value):
        self.left_motor_dir = bool(value)

    def set_right_motor_dir(self, value):
        self.right_motor_dir = bool(value)

    def set_left_motor(self, L_speed):
        if self.left_motor_dir:
            L_speed = -L_speed
        
        velocity = self._speed_to_dxl_velocity(L_speed)
        self.packetHandler.write4ByteTxRx(self.portHandler, LEFT_MOTOR_ID, ADDR_GOAL_VELOCITY, velocity)

    def set_right_motor(self, R_speed):
        if self.right_motor_dir:
            R_speed = -R_speed
        
        velocity = self._speed_to_dxl_velocity(-R_speed)
        self.packetHandler.write4ByteTxRx(self.portHandler, RIGHT_MOTOR_ID, ADDR_GOAL_VELOCITY, velocity)

    def move(self, L_speed, R_speed):
        self.set_left_motor(L_speed)
        self.set_right_motor(R_speed)

    def stop(self):
        self.move(0, 0)
    
    def close(self):
        self.stop()
        self.disable_motor()
        self.portHandler.closePort()