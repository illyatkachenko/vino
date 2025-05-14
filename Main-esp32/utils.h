#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h> // Для String и millis()

// void log_w(const char* tag, const char* format, ...);
// void log_i(const char* tag, const char* format, ...);
// void log_d(const char* tag, const char* format, ...);

// Прототип для инициализации утилит (например, времени старта)
void initUtils();

// Прототип для получения строки времени работы
String getUptimeString();
uint32_t getUptimeSeconds(); // Объявление для получения секунд
// Note: Logging functions (app_log_x) are declared in main.h and defined in Main-esp32.ino

#endif // UTILS_H