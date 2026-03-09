"""
UDP packet structures for ESP32 Robot (Python equivalent of C++ structs)
"""
import struct
from typing import NamedTuple, Tuple

class LidarMeasurement(NamedTuple):
    signal_strength: int
    distance_raw: int

    @staticmethod
    def from_bytes(data: bytes) -> 'LidarMeasurement':
        # 1 byte signal, 2 bytes distance (big-endian)
        signal, dist = struct.unpack('>BH', data)
        return LidarMeasurement(signal, dist)

class LidarHeader(NamedTuple):
    start_byte: int
    total_length: int
    protocol_version: int
    packet_type: int
    data_header: int
    data_length: int
    motor_speed: int
    zero_offset: int
    start_angle: int

    @staticmethod
    def from_bytes(data: bytes) -> 'LidarHeader':
        # >B H B B B H B h H (big-endian)
        fields = struct.unpack('>B H B B B H B h H', data)
        return LidarHeader(*fields)

class TelemetryPacket(NamedTuple):
    identifier: int
    ip: Tuple[int, int, int, int]
    tcp_port: int
    left_hall: int
    right_hall: int

    @staticmethod
    def from_bytes(data: bytes) -> 'TelemetryPacket':
        # Формат: 0xBB (1) + IP (4) + TCP port (2) + left (4) + right (4) = 15 байт
        # IP и port – big-endian, счётчики – little-endian
        if len(data) < 15:
            raise ValueError("Packet too short")
        ident = data[0]
        ip = tuple(data[1:5])
        tcp_port = (data[5] << 8) | data[6]
        left = int.from_bytes(data[7:11], 'little')
        right = int.from_bytes(data[11:15], 'little')
        return TelemetryPacket(ident, ip, tcp_port, left, right)
    
def get_checksum(packet: bytes) -> int:
    # Last 2 bytes are checksum (big-endian)
    return (packet[-2] << 8) | packet[-1]

def calc_checksum(packet: bytes) -> int:
    # Sum all bytes except last 2
    return sum(packet[:-2]) & 0xFFFF
