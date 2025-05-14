#ifndef ESP_NOW_PROTOCOL_H
#define ESP_NOW_PROTOCOL_H
#include "error_handler.h" // Включаем для SystemErrorCode_t

#include <stdint.h> // Для uint8_t, uint16_t и т.д.

// --- Определения для ESP-NOW ---
#define ESP_NOW_CHANNEL 1 // Канал должен быть одинаковым на обоих устройствах

// Типы команд, отправляемых С ЭКРАНА на ГЛАВНЫЙ ESP32
typedef enum {
    CMD_SET_TEMPERATURE = 0,
    CMD_SET_VOLUME = 1,
    CMD_START_PROCESS = 2,
    CMD_PAUSE = 3,
    CMD_RESUME = 4,
    CMD_STOP_PROCESS = 5, // Убедитесь, что это значение уникально
    CMD_ACK_ERROR = 6,    // Можно добавить, если ошибки требуют явного подтверждения с экрана
    // CMD_HEARTBEAT_SCREEN, // Опционально: для экрана, чтобы сигнализировать, что он жив
} command_type_t;

typedef struct struct_command {
    command_type_t cmd_type;
    int value; // Для температуры, объема или других данных
} struct_command_t;

// Состояния системы, отправляемые С ГЛАВНОГО ESP32 на ЭКРАН
typedef enum {
    STATE_UNKNOWN = 0,
    STATE_LOADING,          // Главный ESP32 загружается
    STATE_HELLO,            // Главный ESP32 готов, приветствие
    STATE_IDLE,             // Готов к вводу пользователя или старту
    STATE_SETTINGS_TEMP,    // (Экран управляет, главный подтверждает) - возможно, не нужен как отдельное состояние
    STATE_SETTINGS_VOLUME,  // (Экран управляет, главный подтверждает) - возможно, не нужен как отдельное состояние
    STATE_COOLING,
    STATE_READY,            // Охлажден и готов к наливу
    STATE_FILLING,
    STATE_PAUSED,
    STATE_COMPLETE,
    STATE_ERROR,
    STATE_GOODBYE,          // Завершение работы или процесса
    // STATE_MAIN_ESP_REBOOTING, // Для отображения экраном "переподключение"
} system_state_t;

typedef struct struct_status {
    float current_temperature_out; // Температура на выходе (tOut)
    float current_temperature_in;  // Температура на входе (tIn), если есть
    int current_volume_ml;         // Текущий налитый объем во время FILLING
    int target_volume_ml;          // Целевой объем для текущего цикла
    system_state_t system_state;
    SystemErrorCode_t error_code; // Используем тип из error_handler.h
    int current_setpoint_temperature; // Подтвержденная/текущая уставка температуры
    int current_setpoint_volume;      // Подтвержденная/текущая уставка объема
    uint32_t uptime_seconds;      // Опционально: для отображения времени работы главного модуля
    char version_main_esp[16];    // Опционально: для отображения версии прошивки главного модуля
} struct_status_t;

#endif // ESP_NOW_PROTOCOL_H
