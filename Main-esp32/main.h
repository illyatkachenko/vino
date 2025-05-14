#ifndef MAIN_H_
#define MAIN_H_

// Раскомментируйте следующую строку, если датчики не подключены (для разработки)
 #define IGNORE_MISSING_SENSORS_FOR_DEVELOPMENT

#include <Arduino.h> // Для базовых типов Arduino
#include <WebServer.h> // WebServer используется в web_server_handlers.cpp

extern WebServer server; // Определен в Main-esp32.ino

// Глобальные состояния
extern bool system_power_enabled;
extern bool ap_mode_active; // Флаг, что устройство в режиме точки доступа
extern bool g_preferences_operational; // Флаг состояния Preferences

// Функции логирования (определены в Main-esp32.ino)
// Используем префикс app_ чтобы избежать конфликтов с системными логами ESP-IDF
void app_log_e(const char* tag, const char* format, ...);
void app_log_w(const char* tag, const char* format, ...);
void app_log_i(const char* tag, const char* format, ...);
void app_log_d(const char* tag, const char* format, ...);

// Другие общие объявления, если нужны

// Функция переключения питания системы (определена в Main-esp32.ino)
void toggleSystemPower(bool fromWeb = false);
// Геттер для состояния питания системы
bool isSystemPowerEnabled();

// Геттер для состояния AP режима (если нужен другим модулям)
// bool isApModeActive();

#endif // MAIN_H_