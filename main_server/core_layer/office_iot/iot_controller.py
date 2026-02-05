import asyncio
from main_server.common.protocols import IoTDevice, IoTCommand

class IoTController:
    """
    사무실 내 조명, 온도 등 IoT 장치를 제어하는 비동기 서비스.
    """
    def __init__(self):
        # 실제 장치와 통신하는 인터페이스 초기화
        print("IoT Controller 초기화 완료 (Async Mock).")
        self.devices = {
            "meeting_room_1_light": {"type": IoTDevice.LIGHT, "status": "OFF"},
            "office_thermostat": {"type": IoTDevice.THERMOSTAT, "value": 23}
        }

    async def control_device(self, device_id: str, command: IoTCommand, value=None):
        """
        특정 장치에 제어 명령을 비동기적으로 보냅니다.
        """
        if device_id not in self.devices:
            return {"status": "error", "message": "Device not found."}
        
        # 실제 HW 통신을 흉내내기 위한 약간의 지연
        await asyncio.sleep(0.1)
        
        device = self.devices[device_id]
        
        if command == IoTCommand.TURN_ON and device['type'] == IoTDevice.LIGHT:
            device['status'] = "ON"
            print(f"'{device_id}' 조명을 켭니다.")
            return {"status": "success", "device_status": device}

        elif command == IoTCommand.TURN_OFF and device['type'] == IoTDevice.LIGHT:
            device['status'] = "OFF"
            print(f"'{device_id}' 조명을 끕니다.")
            return {"status": "success", "device_status": device}
            
        elif command == IoTCommand.SET_VALUE and device['type'] == IoTDevice.THERMOSTAT:
            device['value'] = value
            print(f"'{device_id}' 온도를 {value}°C로 설정합니다.")
            return {"status": "success", "device_status": device}
            
        return {"status": "error", "message": "Invalid command for the device."}

    async def get_devices_status(self):
        await asyncio.sleep(0.05) # DB 조회 등 I/O 작업을 흉내
        return self.devices
