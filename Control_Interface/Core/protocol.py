# protocol.py
import struct
from collections import deque
from typing import List, Optional, Tuple

# ===================== 数据类 =====================
class MotorData:
    __slots__ = ('pos', 'vel', 'acc', 'status')
    def __init__(self, pos: float, vel: float, acc: float, status: int):
        self.pos = pos      # mm
        self.vel = vel      # mm/s
        self.acc = acc      # mm/s²
        self.status = status  # 0:停止, 1:运行

class SensorData:
    __slots__ = ('pitch', 'roll', 'yaw')
    def __init__(self, pitch: float, roll: float, yaw: float):
        self.pitch = pitch  # deg
        self.roll = roll    # deg
        self.yaw = yaw      # deg

class DeviceStatus:
    __slots__ = ('num_motors', 'num_sensors', 'motors', 'sensors',
                 'scale', 'bend_angle', 'sys_state')
    def __init__(self, num_motors: int, num_sensors: int,
                 motors: List[MotorData], sensors: List[SensorData],
                 scale: float, bend_angle: float, sys_state: int):
        self.num_motors = num_motors
        self.num_sensors = num_sensors
        self.motors = motors
        self.sensors = sensors
        self.scale = scale
        self.bend_angle = bend_angle
        self.sys_state = sys_state

# ===================== 滤波器 =====================
class DataFilter:
    """中值滤波 + 限幅滤波"""
    def __init__(self, window_size: int = 3, max_change_rate: dict = None):
        self.window_size = window_size
        self.max_change = max_change_rate or {
            'pos': 20.0, 'vel': 50.0, 'acc': 100.0, 'angle': 30.0
        }
        self._motor_buffers = []   # 每个电机 [pos_queue, vel_queue, acc_queue]
        self._sensor_buffers = []  # 每个传感器 [pitch_queue, roll_queue, yaw_queue]
        self._prev_motor = []      # 上一次滤波后的值，用于限幅
        self._prev_sensor = []

    def _median(self, queue: deque, new_val: float) -> float:
        queue.append(new_val)
        if len(queue) > self.window_size:
            queue.popleft()
        if len(queue) < self.window_size:
            return new_val
        sorted_vals = sorted(queue)
        return sorted_vals[len(sorted_vals)//2]

    def _limit(self, old: float, new: float, max_change: float) -> float:
        diff = new - old
        if abs(diff) > max_change:
            return old + (max_change if diff > 0 else -max_change)
        return new

    def apply_motor(self, idx: int, pos: float, vel: float, acc: float) -> Tuple[float, float, float]:
        while len(self._motor_buffers) <= idx:
            self._motor_buffers.append([deque(maxlen=self.window_size) for _ in range(3)])
            self._prev_motor.append([0.0, 0.0, 0.0])
        buffers = self._motor_buffers[idx]
        prev = self._prev_motor[idx]

        fpos = self._median(buffers[0], pos)
        fvel = self._median(buffers[1], vel)
        facc = self._median(buffers[2], acc)

        fpos = self._limit(prev[0], fpos, self.max_change['pos'])
        fvel = self._limit(prev[1], fvel, self.max_change['vel'])
        facc = self._limit(prev[2], facc, self.max_change['acc'])

        self._prev_motor[idx] = [fpos, fvel, facc]
        return fpos, fvel, facc

    def apply_sensor(self, idx: int, pitch: float, roll: float, yaw: float) -> Tuple[float, float, float]:
        while len(self._sensor_buffers) <= idx:
            self._sensor_buffers.append([deque(maxlen=self.window_size) for _ in range(3)])
            self._prev_sensor.append([0.0, 0.0, 0.0])
        buffers = self._sensor_buffers[idx]
        prev = self._prev_sensor[idx]

        fpitch = self._median(buffers[0], pitch)
        froll  = self._median(buffers[1], roll)
        fyaw   = self._median(buffers[2], yaw)

        max_angle = self.max_change['angle']
        fpitch = self._limit(prev[0], fpitch, max_angle)
        froll  = self._limit(prev[1], froll,  max_angle)
        fyaw   = self._limit(prev[2], fyaw,   max_angle)

        self._prev_sensor[idx] = [fpitch, froll, fyaw]
        return fpitch, froll, fyaw

    def reset(self):
        self._motor_buffers.clear()
        self._sensor_buffers.clear()
        self._prev_motor.clear()
        self._prev_sensor.clear()

# ===================== 协议解析器 =====================
class ProtocolParser:
    @staticmethod
    def parse_frame(frame: bytes, apply_filter: bool = False, filter_obj: DataFilter = None) -> Optional[DeviceStatus]:
        if len(frame) < 5 or frame[0] != 0xBB:
            return None
        func = frame[1]
        if func != 0x02:   # 只处理状态反馈帧
            return None
        payload = frame[3:-1]
        if len(payload) < 2:
            return None

        esp_m, esp_s = payload[0], payload[1]
        offset = 2

        motors = []
        for _ in range(esp_m):
            if offset + 7 > len(payload):
                break
            x, y, z = struct.unpack_from('>hhh', payload, offset)
            offset += 6
            status = payload[offset]
            offset += 1
            motors.append(MotorData(x/100.0, y/100.0, z/100.0, status))

        sensors = []
        for _ in range(esp_s):
            if offset + 6 > len(payload):
                break
            pitch, roll, yaw = struct.unpack_from('>hhh', payload, offset)
            offset += 6
            sensors.append(SensorData(pitch/100.0, roll/100.0, yaw/100.0))

        scale = bend = 0.0
        sys_state = 0
        if offset + 5 <= len(payload):
            scale = struct.unpack_from('>h', payload, offset)[0] / 100.0
            offset += 2
            bend = struct.unpack_from('>h', payload, offset)[0] / 100.0
            offset += 2
            sys_state = payload[offset]

        if apply_filter and filter_obj is not None:
            filtered_motors = []
            for i, m in enumerate(motors):
                fp, fv, fa = filter_obj.apply_motor(i, m.pos, m.vel, m.acc)
                filtered_motors.append(MotorData(fp, fv, fa, m.status))
            motors = filtered_motors

            filtered_sensors = []
            for i, s in enumerate(sensors):
                fp, fr, fy = filter_obj.apply_sensor(i, s.pitch, s.roll, s.yaw)
                filtered_sensors.append(SensorData(fp, fr, fy))
            sensors = filtered_sensors

        return DeviceStatus(
            num_motors=esp_m,
            num_sensors=esp_s,
            motors=motors,
            sensors=sensors,
            scale=scale,
            bend_angle=bend,
            sys_state=sys_state
        )