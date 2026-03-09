// ===============================================
// ESP32 MOBILE LIDAR ROBOT - MAIN FIRMWARE v2.10
// Direct PWM motor control (no PID)
// Lidar speed control with moving average and step correction
// UDP telemetry, enhanced display logs
// ===============================================

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #include <Arduino.h>
  #include <WiFi.h>
  #include <Preferences.h>
  #include "lwip/sockets.h"
  #include "lwip/netdb.h"
  #include <fcntl.h>
  #include <cstdint>
  #include <errno.h>
  #include "include/udp_handler_new.h"
  #include "include/display_handler.h"
#endif

// ===== PIN DEFINITIONS =====
#define PIN_Rm_pwm_CCW  32
#define PIN_Rm_pwm_CW   33
#define PIN_Lm_pwm_CCW  25
#define PIN_Lm_pwm_CW   26
#define PIN_LidarM_pwm  27
#define PIN_Lm_CW       14
#define PIN_Lm_CCW      12
#define PIN_Rm_CW       21
#define PIN_Rm_CCW      2
#define PIN_HALL_LEFT   22
#define PIN_HALL_RIGHT  13
#define PIN_BATTERY     34

#define PWM_FREQ        2000
#define PWM_RESOLUTION  8           // 8-bit for drive motors
#define LIDAR_PWM_RESOLUTION 8      // 8-bit for lidar motor (16-bit was not working)
#define LIDAR_START_PWM 127         // Initial PWM to start lidar rotation (must be within 125-160)
#define LIDAR_PWM_MIN   120         // Minimum allowed PWM when lidar is enabled
#define LIDAR_PWM_MAX   160         // Maximum allowed PWM when lidar is enabled

// Moving average settings for lidar speed filtering
#define SPEED_SMA_WINDOW 30         // Number of samples for moving average

// ===== STRUCTURES =====
struct Settings {
  char ssid[32];
  char password[64];
  uint16_t tcp_port;
  uint16_t udp_port;
};

// ===== GLOBAL VARIABLES =====
Preferences preferences;
Settings settings;
String ssid = "";
String password = "";
int tcpPort = 3333;
int udpPort = 4444;

volatile int32_t hall_left_counter = 0;
volatile int32_t hall_right_counter = 0;

float current_battery_voltage = 0.0;
unsigned long last_telemetry = 0;
unsigned long last_status = 0;
unsigned long last_updateLidarSpeed = 0;


// Lidar simple control
bool lidar_enabled = false;               // Lidar motor state
float lidar_target_speed = 5.0;            // Desired speed in rps (user adjustable)
float lidar_motor_speed = 0.0;             // Raw measured speed from packets
float lidar_current_pwm = 0.0;             // Current PWM output (0-255)

// Moving average buffer
float speed_buffer[SPEED_SMA_WINDOW];      // Circular buffer for speed samples
int speed_buffer_index = 0;                 // Current index in buffer
bool speed_buffer_filled = false;           // True after first full buffer
float filtered_speed = 0.0;                 // Current filtered speed (moving average)

int server_socket = -1;
int client_socket = -1;
bool client_connected = false;
String tcp_recv_buffer = "";

UdpHandlerNew udpHandler(4444, "255.255.255.255");

unsigned long lidar_packet_count = 0;
unsigned long lidar_error_count = 0;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// ===== MOTOR CLASS =====
class Motor {
private:
  int m_speed, m_direction;
  int pin_pwm_ccw, pin_pwm_cw, pin_ccw, pin_cw;
  bool initialized;
  char motor_name[8];

public:
  Motor(int pwmccw, int pwmcw, int ccw, int cw, const char *name) {
    pin_pwm_ccw = pwmccw;
    pin_pwm_cw = pwmcw;
    pin_ccw = ccw;
    pin_cw = cw;
    initialized = false;
    m_speed = 0;
    m_direction = 0;
    strncpy(motor_name, name, sizeof(motor_name) - 1);
    motor_name[sizeof(motor_name) - 1] = '\0';
  }

  void init() {
    if (initialized) return;
    pinMode(pin_ccw, OUTPUT);
    pinMode(pin_cw, OUTPUT);
    digitalWrite(pin_ccw, HIGH);
    digitalWrite(pin_cw, HIGH);
    pinMode(pin_pwm_ccw, OUTPUT);
    pinMode(pin_pwm_cw, OUTPUT);
    digitalWrite(pin_pwm_ccw, LOW);
    digitalWrite(pin_pwm_cw, LOW);
    ledcAttach(pin_pwm_ccw, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(pin_pwm_ccw, 0);
    ledcAttach(pin_pwm_cw, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(pin_pwm_cw, 0);
    initialized = true;
  }

  void setSpeed(int value) {
    if (!initialized) init();
    if (value > 255) value = 255;
    if (value < -255) value = -255;
    int new_speed = abs(value);
    int new_direction = (value > 0) ? 1 : (value < 0) ? -1 : 0;
    if (new_speed == m_speed && new_direction == m_direction) return;
    m_speed = new_speed;
    m_direction = new_direction;

    if (value < 0) {
      digitalWrite(pin_cw, HIGH);
      digitalWrite(pin_ccw, LOW);
      ledcWrite(pin_pwm_cw, 0);
      ledcWrite(pin_pwm_ccw, m_speed);   // 8-bit value
      Serial.printf("[MOTOR:%s] REVERSE PWM=%d\n", motor_name, m_speed);
    } else if (value > 0) {
      digitalWrite(pin_ccw, HIGH);
      digitalWrite(pin_cw, LOW);
      ledcWrite(pin_pwm_ccw, 0);
      ledcWrite(pin_pwm_cw, m_speed);     // 8-bit value
      Serial.printf("[MOTOR:%s] FORWARD PWM=%d\n", motor_name, m_speed);
    } else {
      digitalWrite(pin_ccw, HIGH);
      digitalWrite(pin_cw, HIGH);
      ledcWrite(pin_pwm_ccw, 0);
      ledcWrite(pin_pwm_cw, 0);
      Serial.printf("[MOTOR:%s] STOPPED\n", motor_name);
    }
  }

  int getSpeed() { return initialized ? m_speed * m_direction : 0; }
  int getDir() { return m_direction; }
};

Motor leftMotor(PIN_Lm_pwm_CCW, PIN_Lm_pwm_CW, PIN_Lm_CCW, PIN_Lm_CW, "LEFT");
Motor rightMotor(PIN_Rm_pwm_CCW, PIN_Rm_pwm_CW, PIN_Rm_CCW, PIN_Rm_CW, "RIGHT");

volatile uint32_t lastEventHallLeft = 0;
volatile uint32_t lastEventHallRight = 0;

int32_t getLeftCounter()  { return hall_left_counter; }
int32_t getRightCounter() { return hall_right_counter; }

volatile int prevDirLeft = 0;
volatile int prevDirRight = 0;

const uint32_t debounceInterval = 1000;

void IRAM_ATTR hallLeftISR() {
  if (digitalRead(PIN_HALL_LEFT) == 0) {
    uint32_t now = micros();
    if (now - lastEventHallLeft > debounceInterval) {
      int Dir = leftMotor.getDir();
      if ((Dir == 1) || ((Dir == 0) && (prevDirLeft == 1))) {
        hall_left_counter++;
      } else if ((Dir == -1) || ((Dir == 0) && (prevDirLeft == -1))) {
        hall_left_counter--;
      }
      prevDirLeft = (Dir == 0) ? prevDirLeft : Dir;
      lastEventHallLeft = now;
    }
  }
}

void IRAM_ATTR hallRightISR() {
  if (digitalRead(PIN_HALL_RIGHT) == 0) {
    uint32_t now = micros();
    if (now - lastEventHallRight > debounceInterval) {
      int Dir = rightMotor.getDir();
      if ((Dir == 1) || ((Dir == 0) && (prevDirRight == 1))) {
        hall_right_counter++;
      } else if ((Dir == -1) || ((Dir == 0) && (prevDirRight == -1))) {
        hall_right_counter--;
      }
      prevDirRight = (Dir == 0) ? prevDirRight : Dir;
      lastEventHallRight = now;
    }
  }
}

// ===== TELEMETRY =====
void sendTelemetryUDP() {
  int32_t left = getLeftCounter();
  int32_t right = getRightCounter();
  uint8_t buffer[24]; // telemetry buffer
  int idx = 0;
  buffer[idx++] = 0xBB;
  IPAddress ip = WiFi.localIP();
  buffer[idx++] = ip[0]; buffer[idx++] = ip[1]; buffer[idx++] = ip[2]; buffer[idx++] = ip[3];
  buffer[idx++] = (tcpPort >> 8) & 0xFF;
  buffer[idx++] = tcpPort & 0xFF;
  // pack 32-bit counters (little-endian)
  buffer[idx++] =  left        & 0xFF;
  buffer[idx++] = (left >> 8) & 0xFF;
  buffer[idx++] = (left >> 16) & 0xFF;
  buffer[idx++] = (left >> 24) & 0xFF;
  buffer[idx++] =  right       & 0xFF;
  buffer[idx++] = (right >> 8) & 0xFF;
  buffer[idx++] = (right >> 16) & 0xFF;
  buffer[idx++] = (right >> 24) & 0xFF;
  udpHandler.sendRawData(buffer, idx);
}

// ===== SETTINGS =====
void loadSettings() {
  preferences.begin("robot_config", true);
  if (preferences.getBytesLength("settings") == sizeof(Settings)) {
    preferences.getBytes("settings", &settings, sizeof(Settings));
  } else {
    strcpy(settings.ssid, "esp32_robot");
    strcpy(settings.password, "password123");
    settings.tcp_port = 3333;
    settings.udp_port = 4444;
    preferences.end();
    saveSettings();
    return;
  }
  preferences.end();
  ssid = String(settings.ssid);
  password = String(settings.password);
  tcpPort = settings.tcp_port;
  udpPort = settings.udp_port;
  Serial.printf("[SETTINGS] SSID=%s TCP=%d UDP=%d\n", ssid.c_str(), tcpPort, udpPort);
}

void saveSettings() {
  preferences.begin("robot_config", false);
  preferences.putBytes("settings", &settings, sizeof(Settings));
  preferences.end();
}

// ===== TCP SERVER =====
void initTcpServer() {
  if (server_socket >= 0) close(server_socket);
  server_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket < 0) {Serial.println("[TCP] Socket error"); return; }
  int opt = 1;
  setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  fcntl(server_socket, F_SETFL, fcntl(server_socket, F_GETFL, 0) | O_NONBLOCK);
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(tcpPort);
  if (bind(server_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(server_socket, 4) < 0) {
    Serial.println("[TCP] Bind/Listen error");
    close(server_socket);
    server_socket = -1;
    return;
  }
  Serial.printf("[TCP] Server port %d\n", tcpPort);
}

void processTcpServer() {
  if (client_socket < 0 && server_socket >= 0) {
    struct sockaddr_in caddr;
    socklen_t clen = sizeof(caddr);
    int ns = accept(server_socket, (struct sockaddr *)&caddr, &clen);
    if (ns >= 0) {
      client_socket = ns;
      client_connected = true;
      Serial.printf("[TCP] New client connected from %s:%d\n",
                    inet_ntoa(caddr.sin_addr), ntohs(caddr.sin_port));
      DisplayHandler::addLogTCP("Client connected");
    } else {
      int err = errno;
      if (err != EAGAIN && err != EWOULDBLOCK) {
        Serial.printf("[TCP] accept error: %d\n", err);
      }
    }
  }
  if (client_socket >= 0 && client_connected) {
    char tmp[256];
    int r = recv(client_socket, tmp, sizeof(tmp) - 1, 0);
    if (r > 0) {
      tmp[r] = 0;
      tcp_recv_buffer += String(tmp);
      int nl;
      while ((nl = tcp_recv_buffer.indexOf('\n')) != -1) {
        handleTcpCommand(tcp_recv_buffer.substring(0, nl));
        tcp_recv_buffer = tcp_recv_buffer.substring(nl + 1);
      }
    } else if (r == 0) {
      close(client_socket);
      client_socket = -1;
      client_connected = false;
      Serial.println("[TCP] Client disconnected");
      DisplayHandler::addLogTCP("Client disconnected");
    }
  }
}

void handleTcpCommand(const String &command) {
  String cmd = command; cmd.trim();
  String cmdLower = cmd; cmdLower.toLowerCase();
  Serial.printf("[TCP] %s\n", cmd.c_str());

  if (cmdLower.startsWith("set_motor_left:")) {
    int speed = cmd.substring(cmd.indexOf(':') + 1).toInt();
    if (speed >= -255 && speed <= 255) {
      leftMotorSpeed = speed;
      leftMotor.setSpeed(speed);
      DisplayHandler::addLogInfo("Left motor: " + String(speed));
    } else {
      DisplayHandler::addLogWarning("Invalid left motor speed: " + String(speed));
    }
  } else if (cmdLower.startsWith("set_motor_right:")) {
    int speed = cmd.substring(cmd.indexOf(':') + 1).toInt();
    if (speed >= -255 && speed <= 255) {
      rightMotorSpeed = speed;
      rightMotor.setSpeed(speed);
      DisplayHandler::addLogInfo("Right motor: " + String(speed));
    } else {
      DisplayHandler::addLogWarning("Invalid right motor speed: " + String(speed));
    }
  } else if (cmdLower.startsWith("setspeed:")) {
    // Format: setspeed: left,right   (e.g., setspeed: 100,-50)
    String params = cmd.substring(cmd.indexOf(':') + 1);
    int commaIndex = params.indexOf(',');
    if (commaIndex != -1) {
      String leftStr = params.substring(0, commaIndex);
      String rightStr = params.substring(commaIndex + 1);
      leftStr.trim(); rightStr.trim();
      int leftVal = leftStr.toInt();
      int rightVal = rightStr.toInt();
      if (leftVal >= -255 && leftVal <= 255 && rightVal >= -255 && rightVal <= 255) {
        leftMotorSpeed = leftVal;
        rightMotorSpeed = rightVal;
        leftMotor.setSpeed(leftVal);
        rightMotor.setSpeed(rightVal);
        DisplayHandler::addLogInfo("Motors set: L=" + String(leftVal) + " R=" + String(rightVal));
      } else {
        DisplayHandler::addLogWarning("Invalid motor speeds: L=" + String(leftVal) + " R=" + String(rightVal));
      }
    } else {
      DisplayHandler::addLogWarning("Invalid setspeed format. Use: setspeed: left,right");
    }
  } else if (cmdLower == "disable_motors") {
    leftMotorSpeed = 0; rightMotorSpeed = 0;
    leftMotor.setSpeed(0); rightMotor.setSpeed(0);
    DisplayHandler::addLogSuccess("Motors disabled");
  } else if (cmdLower == "lidar_on") {
    lidar_enabled = true;
    lidar_current_pwm = LIDAR_START_PWM;
    ledcWrite(PIN_LidarM_pwm, LIDAR_START_PWM);  // initial PWM to start rotation
    // Reset moving average buffer
    speed_buffer_index = 0;
    speed_buffer_filled = false;
    DisplayHandler::addLogInfo("Lidar enabled, target " + String(lidar_target_speed, 1) + " rps, initial PWM set to " + String(LIDAR_START_PWM));
    // После долгих тестов выяснилось, что двойной отправка lidar_on сбивает пакеты сканирования
    // Поэтому при повторном включении (после переподключения) НЕ перезапускаем лидар
    if (lidar_packet_count > 0) {
      DisplayHandler::addLogWarning("Lidar already running, skipping restart to avoid stream drop");
      return; // Не перезапускаем, если уже были пакеты
    }
  } else if (cmdLower == "lidar_off") {
    lidar_enabled = false;
    ledcWrite(PIN_LidarM_pwm, 0);
    lidar_motor_speed = 0.0;
    lidar_current_pwm = 0.0;
    filtered_speed = 0.0;
    // Сброс счётчика пакетов при выключении, чтобы можно было снова включить
    lidar_packet_count = 0;
    DisplayHandler::addLogInfo("Lidar disabled");
  } else if (cmdLower == "status") {
    String resp = String("STATUS:L=") + String(getLeftCounter()) + ",R=" + String(getRightCounter()) +
      ",BAT=" + String((analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 4.4, 2) + "V" +
      ",LIDAR=" + (lidar_enabled ? "ON" : "OFF") +
      ",SPD=" + String(lidar_motor_speed, 2) + "rps" +
      ",FILT=" + String(filtered_speed, 2) + "rps" +
      ",TARGET=" + String(lidar_target_speed, 2) + "rps" +
      ",PWM=" + String(lidar_current_pwm, 1) + "\n";
    send(client_socket, resp.c_str(), resp.length(), 0);
  } else if (cmdLower == "reset") {
    DisplayHandler::addLogWarning("System reset via TCP");
    delay(500);
    ESP.restart();
  } else {
    // Unknown command notification
    String resp = "UNKNOWN COMMAND: " + cmd + "\n";
    send(client_socket, resp.c_str(), resp.length(), 0);
    Serial.println("[TCP] Unknown command: " + cmd);
    DisplayHandler::addLogTCP("Unknown: " + cmd);
  }
}

// ===== LIDAR SIMPLE CONTROL =====
void filter() {
  if (!lidar_enabled) return;

  // Add new raw speed to circular buffer
  speed_buffer[speed_buffer_index] = lidar_motor_speed;
  speed_buffer_index = (speed_buffer_index + 1) % SPEED_SMA_WINDOW;
  if (!speed_buffer_filled && speed_buffer_index == 0) {
    speed_buffer_filled = true;
  }

  // Calculate moving average
  float sum = 0.0;
  int count = speed_buffer_filled ? SPEED_SMA_WINDOW : speed_buffer_index;
  if (count == 0) return; // no data yet
  for (int i = 0; i < count; i++) {
    sum += speed_buffer[i];
  }
  filtered_speed = sum / count;
}

void updateLidarSpeed() {
  if (!lidar_enabled) return;
  if (!speed_buffer_filled) return;

  float error = lidar_target_speed - filtered_speed;
  const float threshold = 0.1;

  if (error > threshold) {
    lidar_current_pwm += 1;
    if (lidar_current_pwm > LIDAR_PWM_MAX) lidar_current_pwm = LIDAR_PWM_MAX;
  } else if (error < -threshold) {
    lidar_current_pwm -= 1;
    if (lidar_current_pwm < LIDAR_PWM_MIN) lidar_current_pwm = LIDAR_PWM_MIN;
  }

  ledcWrite(PIN_LidarM_pwm, (uint32_t)round(lidar_current_pwm));
}



void processLidarData() {
  static uint8_t buffer[256];
  static int idx = 0;
  static bool packetStarted = false;
  static int expectedLength = 0;
  static uint32_t lastByteTime = 0;
  int packetsProcessed = 0;
  const int MAX_PACKETS_PER_CALL = 1;

  while (Serial2.available() && (packetsProcessed < MAX_PACKETS_PER_CALL)) {
    uint8_t byte = Serial2.read();
    lastByteTime = millis();

    if (!packetStarted) {
      if (byte == 0xAA) {
        packetStarted = true;
        idx = 0;
        buffer[idx++] = byte;
        expectedLength = 0;
      }
      continue;
    }

    if (idx >= (int)sizeof(buffer)) {
      packetStarted = false;
      idx = 0;
      expectedLength = 0;
      if (byte == 0xAA) {
        packetStarted = true;
        buffer[idx++] = byte;
      }
      continue;
    }

    buffer[idx++] = byte;

    if (expectedLength == 0 && idx >= 3) {
      expectedLength = (buffer[1] << 8) | buffer[2];
      if (expectedLength < 3 || expectedLength > (int)sizeof(buffer)) {
        packetStarted = false;
        idx = 0;
        expectedLength = 0;
        if (byte == 0xAA) {
          packetStarted = true;
          buffer[idx++] = byte;
        }
        continue;
      }
    }

    if (expectedLength > 0 && idx >= expectedLength) {
      if (buffer[0] == 0xAA) {
        // --- Extract lidar motor speed if the packet contains valid data ---
        // According to protocol: byte 5 must be 0xAD (data header) and packet must be at least 9 bytes long
        if (expectedLength >= 9 && buffer[5] == 0xAD) {
          uint8_t speed_byte = buffer[8];
          lidar_motor_speed = speed_byte * 0.05f;
          // Call the simple control routine with the new speed
          filter();
        }

        if (udpHandler.sendRawData(buffer, expectedLength)) {
          lidar_packet_count++;
        } else {
          lidar_error_count++;
          DisplayHandler::addLogError("Lidar send error");
        }
      } else {
        lidar_error_count++;
        DisplayHandler::addLogError("Lidar packet corrupted (first byte not AA)");
      }

      packetStarted = false;
      idx = 0;
      expectedLength = 0;
      packetsProcessed++;
    }
  }

  if (packetStarted && (millis() - lastByteTime > 100)) {
    packetStarted = false;
    idx = 0;
    expectedLength = 0;
  }
}

void initLidar() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  ledcAttach(PIN_LidarM_pwm, PWM_FREQ, LIDAR_PWM_RESOLUTION); // 8-bit
  ledcWrite(PIN_LidarM_pwm, 0);  // start with motor off
  udpHandler.begin();
  Serial.println("[LIDAR] Ready");
  DisplayHandler::addLogSuccess("Lidar ready");
}

// ===== SERIAL =====
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();

  if (cmd == "-help") {
    Serial.println("-help -status -restart -left <pwm> -right <pwm> -both <left> <right> -stop");
    Serial.println("-lidar_on -lidar_off -target <rps>");
    Serial.println("Note: PID commands (-kp, -ki, -kd) are ignored (simple step control used).");
  } else if (cmd == "-status") {
    Serial.printf("WiFi: %s | L=%d R=%d | Bat=%.2fV | Lidar: %s, speed=%.2f/%.2f rps (filt=%.2f), PWM=%.1f\n",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "NC",
                  leftMotor.getSpeed(), rightMotor.getSpeed(),
                  (analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 4.4,
                  lidar_enabled ? "ON" : "OFF",
                  lidar_motor_speed, lidar_target_speed,
                  filtered_speed, lidar_current_pwm);
  } else if (cmd == "-restart") {
    DisplayHandler::addLogWarning("System reset via Serial");
    ESP.restart();
  } else if (cmd.startsWith("-left ")) {
    int s = cmd.substring(6).toInt();
    if (s >= -255 && s <= 255) {
      leftMotorSpeed = s;
      leftMotor.setSpeed(s);
      DisplayHandler::addLogSerial("Left: " + String(s));
    } else {
      DisplayHandler::addLogWarning("Invalid left PWM: " + String(s));
    }
  } else if (cmd.startsWith("-right ")) {
    int s = cmd.substring(7).toInt();
    if (s >= -255 && s <= 255) {
      rightMotorSpeed = s;
      rightMotor.setSpeed(s);
      DisplayHandler::addLogSerial("Right: " + String(s));
    } else {
      DisplayHandler::addLogWarning("Invalid right PWM: " + String(s));
    }
  } else if (cmd.startsWith("-both ")) {
    // Format: -both left right  (space separated)
    String params = cmd.substring(6); // after "-both "
    int spaceIndex = params.indexOf(' ');
    if (spaceIndex != -1) {
      String leftStr = params.substring(0, spaceIndex);
      String rightStr = params.substring(spaceIndex + 1);
      leftStr.trim(); rightStr.trim();
      int leftVal = leftStr.toInt();
      int rightVal = rightStr.toInt();
      if (leftVal >= -255 && leftVal <= 255 && rightVal >= -255 && rightVal <= 255) {
        leftMotorSpeed = leftVal;
        rightMotorSpeed = rightVal;
        leftMotor.setSpeed(leftVal);
        rightMotor.setSpeed(rightVal);
        DisplayHandler::addLogSerial("Both: L=" + String(leftVal) + " R=" + String(rightVal));
      } else {
        DisplayHandler::addLogWarning("Invalid both speeds: L=" + String(leftVal) + " R=" + String(rightVal));
      }
    } else {
      DisplayHandler::addLogWarning("Invalid -both format. Use: -both left right");
    }
  } else if (cmd == "-stop") {
    leftMotorSpeed = 0; rightMotorSpeed = 0;
    leftMotor.setSpeed(0); rightMotor.setSpeed(0);
    DisplayHandler::addLogSerial("Stop");
  } else if (cmd == "-lidar_on") {
    lidar_enabled = true;
    lidar_current_pwm = LIDAR_START_PWM;
    ledcWrite(PIN_LidarM_pwm, LIDAR_START_PWM);
    // Reset moving average buffer
    speed_buffer_index = 0;
    speed_buffer_filled = false;
    DisplayHandler::addLogSerial("Lidar ON, target " + String(lidar_target_speed, 1) + " rps, initial PWM set to " + String(LIDAR_START_PWM));
  } else if (cmd == "-lidar_off") {
    lidar_enabled = false;
    ledcWrite(PIN_LidarM_pwm, 0);
    lidar_motor_speed = 0.0;
    lidar_current_pwm = 0.0;
    filtered_speed = 0.0;
    DisplayHandler::addLogSerial("Lidar OFF");
  } else if (cmd.startsWith("-target ")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 0.0f && val <= 10.0f) {
      lidar_target_speed = val;
      DisplayHandler::addLogSerial("Target speed set to " + String(val) + " rps");
    } else {
      DisplayHandler::addLogWarning("Target speed out of range (0-10)");
    }
  } else if (cmd.startsWith("-kp ") || cmd.startsWith("-ki ") || cmd.startsWith("-kd ")) {
    // Ignore PID commands � simple step control does not use them
    DisplayHandler::addLogWarning("PID commands not used (simple step control active)");
  } else {
    // Unknown serial command notification
    Serial.println("[SERIAL] Unknown command: " + cmd);
    DisplayHandler::addLogSerial("Unknown: " + cmd);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("\n=== ESP32 Robot v2.10 (Simple Lidar Step Control) ===\n");
  DisplayHandler::init();
  DisplayHandler::displayStatus("v2.10");
  leftMotor.init(); rightMotor.init();
  loadSettings();
  WiFi.mode(WIFI_STA); WiFi.begin(ssid.c_str(), password.c_str());
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP().toString());
    DisplayHandler::addLogSuccess("WiFi connected");
  } else {
    Serial.println("WiFi failed");
    DisplayHandler::addLogError("WiFi failed");
  }

  // Hall sensors have external 10k pull-ups, so use INPUT_PULLUP to be safe
  pinMode(PIN_HALL_LEFT, INPUT_PULLUP);
  pinMode(PIN_HALL_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_LEFT), hallLeftISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_RIGHT), hallRightISR, FALLING);

  initLidar();
  initTcpServer();
  Serial.println("\nSystem ready!\n");
  DisplayHandler::addLogSuccess("System ready");
}

// ===== LOOP =====
void loop() {
  processTcpServer();
  processLidarData();
  handleSerialCommands();

  // Send telemetry every 320 ms
  if (millis() - last_telemetry > 320) {
    if (WiFi.status() == WL_CONNECTED) {
      sendTelemetryUDP();
    }
    last_telemetry = millis();
  }

  // Print status every 5 seconds
  if (millis() - last_status > 5000) {
    float batteryVoltage = (analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 4.4; // Voltage divider correction
    Serial.printf("[STATUS] L=%ld R=%ld Bat=%.2fV | Lidar: %s, speed=%.2f/%.2f rps (filt=%.2f), PWM=%.1f\n",
                  getLeftCounter(), getRightCounter(), batteryVoltage,
                  lidar_enabled ? "ON" : "OFF", lidar_motor_speed, lidar_target_speed,
                  filtered_speed, lidar_current_pwm);
    last_status = millis();
  }
  if (millis() - last_updateLidarSpeed > 2000) {
    updateLidarSpeed();
    last_updateLidarSpeed = millis();
  }
}
