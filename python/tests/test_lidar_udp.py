#!/bin/bash
# Bash тест для UDP сервера

# Отправить тестовый LiDAR пакет на 8888 порт
# Формат: 0xFA angle_L angle_H dist_L dist_H quality checksum

python3 << 'EOF'
import socket
import struct

# Создать тестовый пакет
packets = []
for angle_raw in range(0, 360*64, 64):  # Каждый градус
    distance_mm = 1000  # 1 метр
    quality = 200
    
    data = bytearray()
    data.append(0xFA)  # Header
    data.append(angle_raw & 0xFF)  # Angle low
    data.append((angle_raw >> 8) & 0xFF)  # Angle high
    data.append(distance_mm & 0xFF)  # Distance low
    data.append((distance_mm >> 8) & 0xFF)  # Distance high
    data.append(quality)  # Quality
    
    # Checksum
    checksum = sum(data[:6]) & 0xFF
    data.append(checksum)
    
    packets.append(bytes(data))

# Отправить на UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for pkt in packets:
    sock.sendto(pkt, ('127.0.0.1', 8888))
    
print(f"✓ Отправлено {len(packets)} пакетов LiDAR")
sock.close()
EOF
