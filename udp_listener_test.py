#!/usr/bin/env python3
"""
Тестовая программа для прослушивания UDP порта
и оценки расстояний вокруг робота по LiDAR.
"""

import socket
import sys
import time
import math

# Масштаб дистанции такой же, как в lidar_udp_server_new.py:
# 1 unit = 0.25 мм → 0.00025 м
DISTANCE_SCALE = 0.00025


def _init_lidar_state():
    return {
        "accum_ranges": [float("inf")] * 360,
        "last_start_angle": None,
        "packets_in_rev": 0,
        "last_packet_hex": "",
    }


def _fill_ranges_from_packet(packet, ranges, start_angle_deg, data_len):
    """Заполнить массив расстояний по одному LiDAR пакету."""
    length = len(packet)
    if data_len < 5:
        return

    num_samples = (data_len - 5) // 3
    if num_samples <= 0:
        return

    angle_step_deg = 22.5 / num_samples
    data_start = 13

    for i in range(num_samples):
        offset = data_start + i * 3
        # не заходим на контрольную сумму в конце
        if offset + 3 > length - 2:
            break

        signal = packet[offset]
        distance_raw = (packet[offset + 1] << 8) | packet[offset + 2]
        distance_m = distance_raw * DISTANCE_SCALE

        angle_deg = start_angle_deg + angle_step_deg * i
        # сдвиг +180°: физический 0° (фронт) → индекс 180
        angle_index = int(round(angle_deg + 180.0)) % 360

        if signal > 0 and 0.01 <= distance_m <= 30.0:
            if distance_m < ranges[angle_index]:
                ranges[angle_index] = distance_m


def _print_lidar_summary(state):
    """Печать последнего пакета и дистанций по четырём направлениям."""
    ranges = state["accum_ranges"]

    def sector_min_cm(indices):
        vals = [ranges[i] for i in indices if math.isfinite(ranges[i])]
        if not vals:
            return None
        return int(round(min(vals) * 100.0))

    # Сектора вокруг робота.
    # Судя по данным, сейчас препятствия попадают в сектор, который мы
    # раньше называли "слева", поэтому считаем его "вперёд робота".
    #
    # front_indices  ≈ бывший "левый" сектор
    # right_indices  ≈ бывший "передний"
    # back_indices   ≈ бывший "правый"
    # left_indices   ≈ бывший "задний"
    front_cm = sector_min_cm(range(255, 286))  # вперёд
    right_cm = sector_min_cm(range(165, 196))  # вправо
    back_cm = sector_min_cm(range(75, 106))    # назад
    left_back_indices = list(range(345, 360)) + list(range(0, 16))
    left_cm = sector_min_cm(left_back_indices) # влево

    def fmt(v):
        return f"{v} см" if v is not None else "---"

    print("\n=== Полный оборот LiDAR ===")
    hex_str = state["last_packet_hex"]
    if hex_str:
        length_bytes = len(hex_str) // 2
        short_hex = hex_str[:80] + ("..." if len(hex_str) > 80 else "")
        print(f"Последний LiDAR пакет ({length_bytes} байт): {short_hex}")

    print("Дистанции вокруг робота (см):")
    print(f"  Вперёд : {fmt(front_cm)}")
    print(f"  Справа : {fmt(right_cm)}")
    print(f"  Сзади  : {fmt(back_cm)}")
    print(f"  Слева  : {fmt(left_cm)}")


def _handle_lidar_packet(data, state):
    """Обработка одного LiDAR пакета с аккумуляцией по обороту."""
    if len(data) < 13:
        return

    # Все многобайтные поля — BIG-ENDIAN
    length = (data[1] << 8) | data[2]
    data_len = (data[6] << 8) | data[7]
    start_angle_raw = (data[11] << 8) | data[12]
    start_angle_deg = start_angle_raw * 0.01

    # Определение начала нового оборота
    last_start = state["last_start_angle"]
    if last_start is not None:
        delta = start_angle_deg - last_start
        if delta < -180.0 and state["packets_in_rev"] >= 4:
            _print_lidar_summary(state)
            state["accum_ranges"] = [float("inf")] * 360
            state["packets_in_rev"] = 0

    state["last_start_angle"] = start_angle_deg

    _fill_ranges_from_packet(data, state["accum_ranges"], start_angle_deg, data_len)
    state["last_packet_hex"] = data.hex()
    state["packets_in_rev"] += 1


def main():
    port = 4444
    
    print(f"=== UDP Listener Test ===")
    print(f"Слушаю порт {port}...")
    print(f"Нажми Ctrl+C для выхода")
    print("Каждый полный оборот лидара → сводка по расстояниям (см).")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        sock.settimeout(1.0)  # Таймаут 1 секунда
        lidar_state = _init_lidar_state()

        packet_count = 0
        start_time = time.time()
        
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                packet_count += 1
                current_time = time.time()
                elapsed = current_time - start_time

                # Определяем тип пакета
                if len(data) >= 1:
                    if data[0] == 0xAA:
                        _handle_lidar_packet(data, lidar_state)
                    elif data[0] == 0xBB:
                        if len(data) >= 23:
                            try:
                                ip = ".".join(str(b) for b in data[1:5])
                                tcp_port = (data[5] << 8) | data[6]
                                print(f"[{elapsed:.2f}s] Telemetry от {addr}: IP={ip}, TCP порт={tcp_port}")
                            except:
                                pass
                    elif data.startswith(b'ROBOT_STATUS'):
                        try:
                            status_str = data.decode("utf-8", errors="ignore")
                        except Exception:
                            status_str = repr(data)
                        print(f"[{elapsed:.2f}s] ROBOT_STATUS: {status_str}")
                    else:
                        # Можно раскомментировать при необходимости детального дебага
                        # print(f"[{elapsed:.2f}s] Неизвестный пакет ({len(data)} байт) от {addr}")
                        pass
                
            except socket.timeout:
                # Нет данных - продолжаем
                if packet_count > 0:
                    print(f".", end="", flush=True)
                continue
            except Exception as e:
                print(f"Ошибка: {e}")
                break
                
    except KeyboardInterrupt:
        print("\n\nОстановлено пользователем")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        sock.close()
        
    print(f"\nВсего получено пакетов: {packet_count}")

if __name__ == '__main__':
    main()
