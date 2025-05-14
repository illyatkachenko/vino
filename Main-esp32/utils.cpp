#include "utils.h"
#include "main.h" // Для доступа к определениям уровней логирования CURRENT_LOG_LEVEL и Serial
#include <stdarg.h>
#include <stdio.h>  // Для vsnprintf

static unsigned long systemStartTime_utils = 0; // Static to this file, инициализируется в initUtils()

// Функции логирования, если мы решим их полностью перенести сюда.
// Пока что они также есть в main.c. Если они остаются в main.c, этот блок можно удалить.
// Однако, если они будут здесь, то в main.c их определения нужно будет удалить, оставив только вызовы.
// Для простоты, предположим, что они остаются в main.c, а utils.c предоставляет только getUptimeString и initUtils.
// Если же ты хочешь централизовать логирование здесь, то раскомментируй и удали из main.c.

/*
static void log_printf_utils(const char* level, const char* tag, const char* format, va_list args) {
    char buffer[512];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.printf("[%s] %s: %s\n", level, tag, buffer);
}

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_ERROR
void log_e(const char* tag, const char* format, ...) { va_list args; va_start(args, format); log_printf_utils("E", tag, format, args); va_end(args); }
#else
void log_e(const char* tag, const char* format, ...) {}
#endif

// ... и так далее для log_w, log_i, log_d ...
*/

void initUtils() {
    systemStartTime_utils = millis();
    // log_i("UTILS", "Utils initialized, system start time set."); // Если log_i доступна
    Serial.println("[I] UTILS: Utils initialized, system start time set.");
}

String getUptimeString() {
    unsigned long uptime_seconds = (millis() - systemStartTime_utils) / 1000;
    int days = uptime_seconds / 86400;
    int hours = (uptime_seconds % 86400) / 3600;
    int minutes = (uptime_seconds % 3600) / 60;
    int seconds = uptime_seconds % 60;

    char buffer[100];
    if (days > 0) {
        snprintf(buffer, sizeof(buffer), "%d дн. %02d:%02d:%02d", days, hours, minutes, seconds);
    } else {
        snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, seconds);
    }
    return String(buffer);
}