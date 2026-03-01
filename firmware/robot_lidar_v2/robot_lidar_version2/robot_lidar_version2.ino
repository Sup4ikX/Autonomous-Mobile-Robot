// ===============================================
// ESP32 MOBILE LIDAR ROBOT - MAIN FIRMWARE v2.9
// Direct PWM motor control (no PID)
// 64-bit hall counters as two 32-bit halves (manual atomic access)
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
//#include "include/lidar_data_new.h"
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
#define PWM_RESOLUTION  8


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

int lidarCurrentDuty = 0;
float current_battery_voltage = 0.0;
unsigned long last_broadcast = 0;

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
      ledcWrite(pin_pwm_ccw, m_speed);
      Serial.printf("[MOTOR:%s] REVERSE PWM=%d\n", motor_name, m_speed);
    } else if (value > 0) {
      digitalWrite(pin_ccw, HIGH);
      digitalWrite(pin_cw, LOW);
      ledcWrite(pin_pwm_ccw, 0);
      ledcWrite(pin_pwm_cw, m_speed);
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
  if (bind(server_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(server_socket, 1) < 0) {
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
      Serial.println("[TCP] Client connected");
      DisplayHandler::addLogTCP("Client connected");
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
  } else if (cmdLower.startsWith("set_lidar_pwm:")) {
    int pwm = cmd.substring(cmd.indexOf(':') + 1).toInt();
    if (pwm >= 0 && pwm <= 255) {
      lidarCurrentDuty = pwm;
      ledcWrite(PIN_LidarM_pwm, pwm);
      DisplayHandler::addLogInfo("Lidar PWM: " + String(pwm));
    } else {
      DisplayHandler::addLogWarning("Invalid lidar PWM: " + String(pwm));
    }
  } else if (cmdLower == "status") {
    String resp = String("STATUS:L=") + String(getLeftCounter()) + ",R=" + String(getRightCounter()) + ",BAT=" + String((analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 2, 2) + "V\n";
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

// ===== LIDAR =====
void processLidarData() {
  static uint8_t buffer[256];                // increased buffer to hold at least one full packet
  static int idx = 0;                         // current fill index in buffer
  static bool packetStarted = false;          // whether we are inside a packet
  static int expectedLength = 0;               // expected total packet length (from header)
  static uint32_t lastByteTime = 0;            // timestamp of last received byte (for timeout)

  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    lastByteTime = millis();

    // If not currently building a packet, look for the start byte 0xAA
    if (!packetStarted) {
      if (byte == 0xAA) {
        packetStarted = true;
        idx = 0;
        buffer[idx++] = byte;
        expectedLength = 0;                     // length not yet known
      }
      // ignore any other bytes outside a packet
      continue;
    }

    // Guard against buffer overflow
    if (idx >= (int)sizeof(buffer)) {
      // Buffer full without completing a packet ? reset and try to re-sync
      packetStarted = false;
      idx = 0;
      expectedLength = 0;
      // If the current byte itself is a new start, handle it immediately
      if (byte == 0xAA) {
        packetStarted = true;
        buffer[idx++] = byte;
      }
      continue;
    }

    // Store the byte
    buffer[idx++] = byte;

    // Determine expected packet length if we haven't yet and have at least 3 bytes (header: AA + length L/H)
    if (expectedLength == 0 && idx >= 3) {
      // Length is stored as big-endian in bytes 1 and 2 (after AA)
      expectedLength = (buffer[1] << 8) | buffer[2];

      // Validate the claimed length (must be at least 3 and within buffer size)
      if (expectedLength < 3 || expectedLength > (int)sizeof(buffer)) {
        // Invalid length ? discard everything and resync
        packetStarted = false;
        idx = 0;
        expectedLength = 0;
        // If the current byte could be a new start, process it now
        if (byte == 0xAA) {
          packetStarted = true;
          buffer[idx++] = byte;
        }
        continue;
      }
    }

    // If we have collected the complete packet
    if (expectedLength > 0 && idx >= expectedLength) {
      // Verify the first byte is still AA (should be, but check)
      if (buffer[0] == 0xAA) {
        if (udpHandler.sendRawData(buffer, expectedLength)) {
          lidar_packet_count++;
        } else {
          lidar_error_count++;
          DisplayHandler::addLogError("Lidar send error");
        }
      } else {
        // This should not happen if sync is correct; log as error
        lidar_error_count++;
        DisplayHandler::addLogError("Lidar packet corrupted (first byte not AA)");
      }

      // Reset for the next packet
      packetStarted = false;
      idx = 0;
      expectedLength = 0;
      // Continue reading possible remaining bytes in the UART buffer
    }
  }

  // Timeout: if we are in the middle of a packet and no new data arrives for 100 ms,
  // assume the packet is incomplete and reset to avoid waiting forever.
  if (packetStarted && (millis() - lastByteTime > 100)) {
    packetStarted = false;
    idx = 0;
    expectedLength = 0;
    // Optionally log a timeout event (can be too verbose, uncomment if needed)
    // DisplayHandler::addLogWarning("Lidar packet timeout");
  }
}

void initLidar() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  ledcAttach(PIN_LidarM_pwm, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PIN_LidarM_pwm, lidarCurrentDuty);
  udpHandler.begin();
  Serial.println("[LIDAR] Ready");
  DisplayHandler::addLogSuccess("Lidar ready");
}

// ===== SERIAL =====
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();

  if (cmd == "-help") {
    Serial.println("-help -status -restart -left <pwm> -right <pwm> -both <left> <right> -stop -lidar <pwm>");
  } else if (cmd == "-status") {
    Serial.printf("WiFi: %s | L=%d R=%d | Bat=%.2fV\n",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "NC",
                  leftMotor.getSpeed(), rightMotor.getSpeed(), (analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 4.4);


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
  } else if (cmd.startsWith("-lidar ")) {
    int p = cmd.substring(7).toInt();
    if (p >= 0 && p <= 255) {
      lidarCurrentDuty = p;
      ledcWrite(PIN_LidarM_pwm, p);
      DisplayHandler::addLogSerial("Lidar: " + String(p));
    } else {
      DisplayHandler::addLogWarning("Invalid lidar PWM: " + String(p));
    }
  } else {
    // Unknown serial command notification
    Serial.println("[SERIAL] Unknown command: " + cmd);
    DisplayHandler::addLogSerial("Unknown: " + cmd);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("\n=== ESP32 Robot v2.9 (Direct PWM, 32+32 hall counters, enhanced display logs) ===\n");
  DisplayHandler::init();
  DisplayHandler::displayStatus("v2.9");
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

  // Hall sensors have external 10k pull-ups, so use INPUT only
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

  if (millis() - last_broadcast > 320) {
    if (WiFi.status() == WL_CONNECTED) {
      float batteryVoltage = (analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 4.4; // Voltage divider correction
      Serial.printf("[STATUS] L=%ld R=%ld Bat=%.2fV\n", getLeftCounter(), getRightCounter(), batteryVoltage);
      sendTelemetryUDP();
    }
    last_broadcast = millis();
  }
  delay(10);
}
