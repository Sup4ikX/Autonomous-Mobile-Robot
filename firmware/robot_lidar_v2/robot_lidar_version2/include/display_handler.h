#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// Colors in RGB565 format
#define COLOR_BG           0x0266    // #026670 - темный фон
#define COLOR_HEADER       0x9FED    // #9FEDD7 - заголовок  
#define COLOR_TEXT         0xFEF9    // #FEF9C7 - основной текст
#define COLOR_WARNING      0xFCE1    // #FCE181 - предупреждения
#define COLOR_FOOTER       0xEDEA    // #EDEAE5 - футер

// Дополнительные для логирования
#define COLOR_RED          0xF800    // Красный для ошибок
#define COLOR_GREEN        0x07E0    // Зеленый для успеха  
#define COLOR_PURPLE       0x91FF    // Фиолетовый для Serial
#define COLOR_CYAN         0x07FF    // Голубой для TCP
#define COLOR_ORANGE       0xFD20    // Оранжевый для дебага

#define MAX_LOG_LINES      7         // УВЕЛИЧИЛИ с 7 до 8!
#define MAX_CHARS_PER_LINE 24        // Символов в линии
#define LOG_BUFFER_SIZE    15        // Всего храним в буфере

class DisplayHandler {
private:
  static TFT_eSPI tft;
  static bool initialized;
  
  // Оптимизированная структура для хранения логов
  struct LogEntry {
    char text[MAX_CHARS_PER_LINE + 1]; // +1 для null-terminator
    uint16_t color;
  };
  
  static LogEntry log_buffer[LOG_BUFFER_SIZE];
  static uint8_t log_start_idx;
  static uint8_t log_count;
  
  static char current_status[MAX_CHARS_PER_LINE + 1];
  static char current_error[MAX_CHARS_PER_LINE + 1];
  static bool has_error;

public:
  // ===== PUBLIC METHODS =====
  
  static void init() {
    if (initialized) return;
    
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(COLOR_BG);
    
    drawHeader();
    drawFooter();
    
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextSize(1);
    tft.setCursor(2, 22);
    tft.print("READY");
    
    // Инициализация буферов
    log_start_idx = 0;
    log_count = 0;
    memset(current_status, 0, sizeof(current_status));
    memset(current_error, 0, sizeof(current_error));
    has_error = false;
    
    initialized = true;
    
    Serial.println("[✓] Display 80x160 initialized");
    addLog("SYSTEM START", COLOR_TEXT);
  }
  
  static void update() {
    // Nothing to do here
  }
  
  static void redraw() {
    if (!initialized) return;
    
    tft.fillScreen(COLOR_BG);
    drawHeader();
    drawStatusArea();
    drawTerminal();
    drawFooter();
  }
  
  // ===== LOG METHODS =====
  
  static void addLog(const char* message, uint16_t color = COLOR_TEXT) {
    if (!initialized) {
      Serial.printf("[LOG] %s\n", message);
      return;
    }
    
    // Форматируем таймстамп
    static char time_buf[6];
    unsigned long total_sec = millis() / 1000;
    unsigned long minutes = total_sec / 60;
    unsigned long seconds = total_sec % 60;
    
    if (minutes > 0) {
      snprintf(time_buf, sizeof(time_buf), "%lu:%02lu", minutes, seconds);
    } else {
      snprintf(time_buf, sizeof(time_buf), "%lus", seconds);
    }
    
    // Создаем полную строку
    char full_msg[32];
    snprintf(full_msg, sizeof(full_msg), "%s %s", time_buf, message);
    
    // Обрезаем до нужной длины
    char display_msg[MAX_CHARS_PER_LINE + 1];
    strncpy(display_msg, full_msg, MAX_CHARS_PER_LINE);
    display_msg[MAX_CHARS_PER_LINE] = '\0';
    
    // Заполняем пробелами
    size_t len = strlen(display_msg);
    while (len < MAX_CHARS_PER_LINE) {
      display_msg[len++] = ' ';
    }
    display_msg[MAX_CHARS_PER_LINE] = '\0';
    
    Serial.printf("[LOG] %s\n", full_msg);
    
    // Добавляем в буфер (новое сообщение в конец)
    uint8_t idx = (log_start_idx + log_count) % LOG_BUFFER_SIZE;
    strncpy(log_buffer[idx].text, display_msg, MAX_CHARS_PER_LINE + 1);
    log_buffer[idx].color = color;
    
    if (log_count < LOG_BUFFER_SIZE) {
      log_count++;
    } else {
      // Буфер полный, сдвигаем начало
      log_start_idx = (log_start_idx + 1) % LOG_BUFFER_SIZE;
    }
    
    drawTerminal();
  }
  
  // String версии для удобства
  static void addLog(const String &message, uint16_t color = COLOR_TEXT) {
    addLog(message.c_str(), color);
  }
  
  static void addLogInfo(const String &message) {
    addLog(message.c_str(), COLOR_TEXT);
  }
  
  static void addLogTCP(const String &message) {
    char buf[32];
    snprintf(buf, sizeof(buf), "T:%s", message.c_str());
    addLog(buf, COLOR_CYAN);
  }
  
  static void addLogSerial(const String &message) {
    char buf[32];
    snprintf(buf, sizeof(buf), "S:%s", message.c_str());
    addLog(buf, COLOR_PURPLE);
  }
  
  static void addLogSuccess(const String &message) {
    char buf[32];
    snprintf(buf, sizeof(buf), "✓ %s", message.c_str());
    addLog(buf, COLOR_GREEN);
  }
  
  static void addLogWarning(const String &message) {
    char buf[32];
    snprintf(buf, sizeof(buf), "! %s", message.c_str());
    addLog(buf, COLOR_WARNING);
  }
  
  static void addLogError(const String &message) {
    char buf[32];
    snprintf(buf, sizeof(buf), "✗ %s", message.c_str());
    addLog(buf, COLOR_RED);
  }
  
  static void addLogDebug(const String &message) {
    char buf[32];
    snprintf(buf, sizeof(buf), "D:%s", message.c_str());
    addLog(buf, COLOR_ORANGE);
  }
  
  // ===== STATUS METHODS =====
  
  static void displayStatus(const char* status) {
    if (!initialized) return;
    
    strncpy(current_status, status, MAX_CHARS_PER_LINE);
    current_status[MAX_CHARS_PER_LINE] = '\0';
    has_error = false;
    
    drawStatusArea();
    
    char log_msg[32];
    snprintf(log_msg, sizeof(log_msg), "Status: %s", status);
    addLog(log_msg, COLOR_GREEN);
  }
  
  static void displayStatus(const String &status) {
    displayStatus(status.c_str());
  }
  
  static void displayError(const char* error) {
    if (!initialized) return;
    
    strncpy(current_error, error, MAX_CHARS_PER_LINE);
    current_error[MAX_CHARS_PER_LINE] = '\0';
    has_error = true;
    
    drawStatusArea();
    addLogError(error);
  }
  
  static void displayError(const String &error) {
    displayError(error.c_str());
  }
  
  // ===== UTILITY METHODS =====
  
  static void clearLogs() {
    log_start_idx = 0;
    log_count = 0;
    drawTerminal();
    addLogInfo("Logs cleared");
  }
  
  static int getLogCount() {
    return log_count;
  }
  
  static bool isInitialized() {
    return initialized;
  }

private:
  // ===== PRIVATE DRAWING METHODS =====
  
  static void drawHeader() {
    tft.fillRect(0, 0, 80, 20, COLOR_HEADER);
    tft.setTextColor(COLOR_BG, COLOR_HEADER);
    tft.setTextSize(1);
    tft.setCursor(2, 6);
    tft.print("ESP32 LOGS");
  }
  
  static void drawStatusArea() {
    // УМЕНЬШИЛИ статусную область до 12 пикселей
    tft.fillRect(0, 22, 80, 12, COLOR_BG);
    
    const char* text = has_error ? current_error : current_status;
    uint16_t color = has_error ? COLOR_RED : COLOR_GREEN;
    
    if (text[0] != '\0') {
      tft.setTextColor(color, COLOR_BG);
      tft.setTextSize(1);
      tft.setCursor(2, 24); // Сместили немного вниз
      
      // Выводим с заполнением пробелами
      tft.print(text);
      size_t len = strlen(text);
      while (len++ < MAX_CHARS_PER_LINE) {
        tft.print(' ');
      }
    }
  }
  
  static void drawTerminal() {
    // УВЕЛИЧИЛИ область логов до 102 пикселей (было 95)
    tft.fillRect(0, 36, 80, 102, COLOR_BG);
    
    int y = 38; // Начало области логов
    
    // Определяем сколько строк рисовать
    int lines_to_draw = log_count;
    if (lines_to_draw > MAX_LOG_LINES) {
      lines_to_draw = MAX_LOG_LINES;
    }
    
    // Рисуем снизу вверх: новые сообщения внизу
    for (int i = 0; i < lines_to_draw; i++) {
      // Вычисляем индекс: начинаем с последнего сообщения
      int buffer_idx = (log_start_idx + log_count - 1 - i) % LOG_BUFFER_SIZE;
      if (buffer_idx < LOG_BUFFER_SIZE) {
        // Новые сообщения внизу: i=0 (самое новое) рисуется внизу
        int draw_y = y + (lines_to_draw - 1 - i) * 9;
        drawLogLine(log_buffer[buffer_idx], draw_y);
      }
    }
  }
  
  static void drawLogLine(const LogEntry &entry, int y) {
    if (y < 36 || y > 138) return; // Обновленные границы
    
    tft.setTextColor(entry.color, COLOR_BG);
    tft.setCursor(2, y);
    tft.print(entry.text);
  }
  
  static void drawFooter() {
    tft.fillRect(0, 140, 80, 20, COLOR_FOOTER);
    tft.setTextColor(COLOR_BG, COLOR_FOOTER);
    tft.setTextSize(1);
    
    // Левый бок - количество логов
    tft.fillRect(2, 146, 25, 8, COLOR_FOOTER);
    tft.setCursor(2, 146);
    tft.print(String(log_count) + "L");
    
    // Правый бок - время работы
    unsigned long total_sec = millis() / 1000;
    unsigned long minutes = total_sec / 60;
    unsigned long seconds = total_sec % 60;
    
    char time_buf[8];
    if (minutes > 0) {
      if (minutes > 99) {
        snprintf(time_buf, sizeof(time_buf), ">%lum", minutes);
      } else {
        snprintf(time_buf, sizeof(time_buf), "%lu:%02lu", minutes, seconds);
      }
    } else {
      snprintf(time_buf, sizeof(time_buf), "%lus", seconds);
    }
    
    tft.fillRect(60, 146, 20, 8, COLOR_FOOTER);
    int text_width = strlen(time_buf) * 6;
    tft.setCursor(80 - text_width - 2, 146);
    tft.print(time_buf);
  }
};

// ===== STATIC VARIABLE INITIALIZATION =====
TFT_eSPI DisplayHandler::tft = TFT_eSPI();
bool DisplayHandler::initialized = false;

DisplayHandler::LogEntry DisplayHandler::log_buffer[LOG_BUFFER_SIZE];
uint8_t DisplayHandler::log_start_idx = 0;
uint8_t DisplayHandler::log_count = 0;

char DisplayHandler::current_status[MAX_CHARS_PER_LINE + 1] = "";
char DisplayHandler::current_error[MAX_CHARS_PER_LINE + 1] = "";
bool DisplayHandler::has_error = false;

#endif // DISPLAY_HANDLER_H
