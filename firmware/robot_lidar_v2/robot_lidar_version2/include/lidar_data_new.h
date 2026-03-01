#ifndef LIDAR_DATA_NEW_H
#define LIDAR_DATA_NEW_H

#include <cstdint>
#include <cstring>

// ===== LIDAR PACKET FORMAT (НОВЫЙ ФОРМАТ) =====
// Заголовок: 0xAA
// Длина пакета (2 байта, little endian)
// Версия протокола (1 байт)
// Тип пакета (1 байт)
// Заголовок данных: 0xAD
// Длина данных (2 байта, little endian)
// Скорость мотора (1 байт)
// Сдвиг нуля (2 байта, little endian, со знаком)
// Стартовый угол (2 байта, little endian)
// Данные: [уровень сигнала (1 байт) + расстояние (2 байта)] * N
// Контрольная сумма (2 байта, little endian)

#pragma pack(1)
struct LidarPacketNew {
  uint8_t header;              // 0xAA
  uint16_t packet_length;      // длина пакета (little-endian)
  uint8_t protocol_version;    // версия протокола
  uint8_t packet_type;         // тип пакета
  uint8_t data_header;         // 0xAD
  uint16_t data_length;        // длина данных (little-endian)
  uint8_t motor_speed;         // скорость мотора * 0.05 об/с
  int16_t zero_offset;         // сдвиг нуля * 0.01° (со знаком)
  uint16_t start_angle;        // стартовый угол * 0.01° (little-endian)
  // Данные: [signal_level (1 байт) + distance (2 байта)] * N
  // Контрольная сумма: 2 байта (little-endian)
};
#pragma pack()

// ===== UTILITY FUNCTIONS =====

inline uint16_t calculateChecksum(const uint8_t* data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFFFF;
}

inline bool validatePacket(const uint8_t* packet, size_t length) {
  if (length < 15) return false; // Минимальная длина пакета
  
  // Проверка заголовка
  if (packet[0] != 0xAA) return false;
  
  // Проверка контрольной суммы
  uint16_t received_checksum = (packet[length-1] << 8) | packet[length-2];
  uint16_t calculated_checksum = 0;
  
  for (size_t i = 0; i < length - 2; i++) {
    calculated_checksum += packet[i];
  }
  
  return (calculated_checksum == received_checksum);
}

inline size_t getPacketLength(const uint8_t* packet) {
  if (packet[0] != 0xAA) return 0;
  return (packet[2] << 8) | packet[1];
}

inline bool isLidarPacket(const uint8_t* packet, size_t length) {
  return (length >= 15 && 
          packet[0] == 0xAA && 
          packet[5] == 0xAD && 
          validatePacket(packet, length));
}

#endif // LIDAR_DATA_NEW_H