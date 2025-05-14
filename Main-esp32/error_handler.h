#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H


#include "freertos/FreeRTOS.h" // Для portMUX_TYPE
#include <Arduino.h> // Для String

// Размер буфера для хранения последнего сообщения об ошибке
#define ERROR_HANDLER_LAST_MSG_BUFFER_SIZE 128
#define PREFS_ERROR_LOG_NAMESPACE "fatal_log" // Пространство имен для хранения фатальных ошибок


// Коды ошибок (расширяемый список)
enum SystemErrorCode {
    NO_ERROR = 0,
    INFO_SYSTEM_RECOVERED = 1,
    WARN_SENSOR_UNSTABLE = 10, WARN_LOW_MEMORY = 11, WARN_ESP_NOW_SEND_FAIL = 20, WARN_PREFS_READ_FAIL = 30, WARN_CAL_DATA_INVALID_LOAD = 31,
    INPUT_VALIDATION_ERROR = 100, WIFI_ERROR = 101, ESP_NOW_INIT_ERROR = 102, ESP_NOW_PEER_ERROR = 103, PREFERENCES_ERROR = 104, CALIBRATION_ERROR = 105, LOGIC_ERROR = 110,
    CRIT_TEMP_SENSOR_IN_FAIL = 200, CRIT_TEMP_SENSOR_COOLER_FAIL = 201, CRIT_TEMP_SENSOR_OUT_FAIL = 202, CRIT_FLOW_SENSOR_FAIL = 203, CRIT_MOTOR_FAIL = 204, CRIT_DOSING_TIMEOUT = 205, CRIT_PRECOOL_TIMEOUT = 206, CRIT_FILESYSTEM_ERROR = 210, CRIT_HEAP_EXHAUSTED = 211,
    FATAL_WDT_RESET = 250, FATAL_BROWN_OUT = 251, FATAL_UNHANDLED_EXCEPTION = 252, FATAL_WIFI_FAIL = 253, FATAL_PREFS_CORRUPT = 254
}; // Оставляем SystemErrorCode для обратной совместимости, если где-то используется без _t
typedef enum SystemErrorCode SystemErrorCode_t; // Определяем SystemErrorCode_t

// --- Для хранения списка ошибок ---
#define MAX_STORED_ERRORS 20 // Максимальное количество хранимых сообщений об ошибках
typedef struct {
    SystemErrorCode_t code; // Теперь SystemErrorCode_t определен выше
    char message[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE]; // Теперь ERROR_HANDLER_LAST_MSG_BUFFER_SIZE определен выше
    unsigned long timestamp;
} StoredError_t;


extern SystemErrorCode_t current_system_error;
extern char last_error_msg_buffer_internal[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE]; // Должно совпадать с именем в .c
extern unsigned long last_error_time;
extern portMUX_TYPE error_handler_mutex; // Объявляем мьютекс

// Объявления функций
void setSystemError(SystemErrorCode errorCode, const char* message); // Изменено с String на const char*
void clearSystemError();
void initErrorHandler(); // Для инициализации переменных, если нужно
SystemErrorCode getSystemErrorCode(); // Геттер для текущего кода ошибки
const char* getSystemErrorMessage(); // Геттер для последнего сообщения об ошибке
const char* getSystemErrorCodeString(SystemErrorCode code);
int getStoredErrors(StoredError_t* buffer, int max_errors); // Для получения списка ошибок
void clearAllStoredErrors(); // Для очистки списка ошибок (может понадобиться)

#endif // ERROR_HANDLER_H