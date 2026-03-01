#ifndef UDP_HANDLER_NEW_H
#define UDP_HANDLER_NEW_H

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #include <WiFiUdp.h>
#else
  class WiFiUDP {
  public:
    void begin(int port) {}
    void beginPacket(const char* addr, int port) { return 0; }
    size_t write(const uint8_t* buffer, size_t size) { return size; }
    int endPacket() { return 1; }
    void stop() {}
  };
#endif

#include "lidar_data_new.h"

// ===== UDP BROADCAST SERVER (НОВЫЙ ФОРМАТ) =====
class UdpHandlerNew {
private:
  WiFiUDP udp;
  int broadcast_port;
  const char* broadcast_addr;
  bool is_initialized;

public:
  UdpHandlerNew(int port = 4444, const char* addr = "255.255.255.255") 
    : broadcast_port(port), broadcast_addr(addr), is_initialized(false) {}

  void begin() {
    if (udp.begin(broadcast_port)) {
      is_initialized = true;
      Serial.printf("[✓] UDP broadcast инициализирован на порту %d (новый формат)\n", broadcast_port);
    } else {
      Serial.printf("[✗] Ошибка инициализации UDP на порту %d\n", broadcast_port);
      is_initialized = false;
    }
  }

  void stop() {
    if (is_initialized) {
      udp.stop();
      is_initialized = false;
    }
  }

  bool sendPacket(const LidarPacketNew& packet) {
    if (!is_initialized) return false;

    if (udp.beginPacket(broadcast_addr, broadcast_port)) {
      udp.write((const uint8_t*)&packet, sizeof(LidarPacketNew));
      return (udp.endPacket() > 0);
    }
    return false;
  }

  bool sendRawData(const uint8_t* data, size_t length) {
    if (!is_initialized) return false;

    if (udp.beginPacket(broadcast_addr, broadcast_port)) {
      udp.write(data, length);
      return (udp.endPacket() > 0);
    }
    return false;
  }

  bool isInitialized() const { return is_initialized; }
};

#endif // UDP_HANDLER_NEW_H