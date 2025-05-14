#include "error_handler.h"
#include "config_manager.h" // Для доступа к config.errorCount и saveConfig()
#include <Preferences.h>     // Для работы с Preferences
#include "motor_control.h"  // Для остановки мотора
#include "dosing_logic.h"   // Для изменения состояния дозирования
#include "calibration_logic.h" // Для getCalibrationModeState
// #include "compressor_control.h" // Если будет такой модуль для compressorOff()
#include "main.h"           // Для функций логирования log_x
#include "sensors.h"        // Для compressorOff() и compressorRunning
#include "localization.h"   // For _T()

// Внешние переменные, которые будут использоваться здесь
// extern Config config; // Доступно через config_manager.h
// extern bool motor_running_auto; // Доступно через motor_control.h
// extern bool motor_running_manual; // Доступно через motor_control.h
// extern bool compressorRunning; // Доступно через sensors.h
// extern DosingState current_dosing_state; // Доступно через dosing_logic.h
// extern unsigned long dosing_state_start_time; // Доступно через dosing_logic.h
// extern bool calibration_stopped_by_timeout; // Эта переменная локальна для calibration_logic.c

// Определения глобальных переменных из error_handler.h
SystemErrorCode_t current_system_error = NO_ERROR;
// String last_error_msg = "No active error."; // Заменено на char массив
char last_error_msg_buffer_internal[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE]; // Initialized in initErrorHandler
unsigned long last_error_time = 0;
// extern void compressorOff(); // Из sensors.h, уже включен

// Мьютекс для защиты глобальных переменных этого модуля
portMUX_TYPE error_handler_mutex = portMUX_INITIALIZER_UNLOCKED;

// --- Для хранения списка ошибок ---
static StoredError_t stored_errors_buffer[MAX_STORED_ERRORS];
static int stored_errors_count = 0;
static int stored_errors_next_idx = 0; // Для кольцевого буфера

void initErrorHandler() {
    portENTER_CRITICAL(&error_handler_mutex);
    current_system_error = NO_ERROR;
    strncpy(last_error_msg_buffer_internal, _T(L_NO_ACTIVE_ERROR_MSG), sizeof(last_error_msg_buffer_internal) - 1);
    last_error_msg_buffer_internal[sizeof(last_error_msg_buffer_internal) - 1] = '\0'; // Ensure null termination
    last_error_time = 0;
    portEXIT_CRITICAL(&error_handler_mutex);
    // Очищаем буфер сохраненных ошибок при инициализации
    stored_errors_count = 0;
    stored_errors_next_idx = 0;
}

void setSystemError(SystemErrorCode_t errorCode, const char* message) {
    portENTER_CRITICAL(&error_handler_mutex);
    if (current_system_error == errorCode && errorCode != NO_ERROR) { // Не логируем повторно ту же ошибку, если она уже активна
        portEXIT_CRITICAL(&error_handler_mutex);
        return;
    }
    current_system_error = errorCode;
    unsigned long current_time_ms = millis(); // Получаем время один раз
    strncpy(last_error_msg_buffer_internal, message, sizeof(last_error_msg_buffer_internal) - 1);
    last_error_msg_buffer_internal[sizeof(last_error_msg_buffer_internal) - 1] = '\0';
    last_error_time = current_time_ms;

    // Добавляем ошибку в кольцевой буфер
    stored_errors_buffer[stored_errors_next_idx].code = errorCode;
    strncpy(stored_errors_buffer[stored_errors_next_idx].message, message, ERROR_HANDLER_LAST_MSG_BUFFER_SIZE - 1);
    stored_errors_buffer[stored_errors_next_idx].message[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE - 1] = '\0';
    stored_errors_buffer[stored_errors_next_idx].timestamp = current_time_ms;
    stored_errors_next_idx = (stored_errors_next_idx + 1) % MAX_STORED_ERRORS;
    if (stored_errors_count < MAX_STORED_ERRORS) {
        stored_errors_count++;
    }
    portEXIT_CRITICAL(&error_handler_mutex);


    // Доступ к config должен быть потокобезопасным, если config изменяется из разных задач.
    // Предполагаем, что config_manager обеспечивает это или saveConfig() вызывается из одного потока.
    config.errorCount++; // Увеличиваем счетчик ошибок в глобальной структуре config
    strncpy(config.lastErrorMsgBuffer, message, sizeof(config.lastErrorMsgBuffer)-1);
    config.lastErrorMsgBuffer[sizeof(config.lastErrorMsgBuffer)-1] = '\0';

    // Используем функции логирования log_x, которые определены в main.c и доступны глобально
    if (errorCode >= CRIT_TEMP_SENSOR_IN_FAIL) {
        app_log_e("ERROR_HANDLER", "System Error %d: %s", errorCode, message);
    } else if (errorCode >= INPUT_VALIDATION_ERROR) {
        app_log_w("ERROR_HANDLER", "System Warning/Error %d: %s", errorCode, message);
    } else {
        app_log_i("ERROR_HANDLER", "System Info/Message %d: %s", errorCode, message);
    }

    // Проверяем, является ли ошибка фатальной (предполагаем, что все коды >= FATAL_WDT_RESET фатальны)
    if (errorCode >= FATAL_WDT_RESET) { 
        log_e("FATAL_HANDLER", "FATAL ERROR %d detected. Saving code and preparing for restart.", errorCode);
        Preferences error_prefs;
        if (error_prefs.begin(PREFS_ERROR_LOG_NAMESPACE, false)) { // R/W mode
            error_prefs.putUInt("last_fatal", (uint32_t)errorCode);
            error_prefs.end();
            app_log_i("FATAL_HANDLER", "Fatal error code %d saved to NVS.", errorCode);
        } else {
            app_log_e("FATAL_HANDLER", "Failed to open NVS to save fatal error code.");
        }
    }

    if (errorCode >= CRIT_TEMP_SENSOR_IN_FAIL) {
        app_log_w("ERROR_HANDLER", "Critical/Fatal error (%d), stopping active processes.", errorCode);
        // bool cal_stopped_by_timeout_local = false; // Локальная переменная для проверки состояния калибровки - удалена, т.к. не используется
        if (isMotorRunningAuto() || isMotorRunningManual()) { // Используем геттеры
            if (getCalibrationModeState() && errorCode == CALIBRATION_ERROR) {
                 // В calibration_logic.c, calibration_stopped_by_timeout устанавливается ПЕРЕД setSystemError
                 // Здесь мы не можем напрямую проверить calibration_stopped_by_timeout из другого модуля без extern
                 // Если ошибка CALIBRATION_ERROR и режим калибровки активен, предполагаем, что это может быть таймаут.
                 // Более надежно было бы передавать флаг таймаута в setSystemError или иметь геттер.
                 // Пока что, если это ошибка калибровки в режиме калибровки, не останавливаем мотор здесь,
                 // так как handleCalibrationLogic() уже должен был его остановить.
                 app_log_d("ERROR_HANDLER", "Calibration error in calibration mode, motor stop handled by calibration logic.");
            } else {
                 // digitalWrite(ENABLE_PIN, HIGH); // Предполагается, что ENABLE_PIN доступен или через функцию
                 stopMotor(); // Используем функцию из motor_control.h
                 // motor_running_auto = false; // stopMotor() уже сбрасывает эти флаги
                 // motor_running_manual = false; // stopMotor() уже сбрасывает эти флаги
                 app_log_w("ERROR_HANDLER", "Motor stopped due to critical error.");
            }
        }
        if (isCompressorRunning()) { // Используем геттер
            compressorOff();
            app_log_w("ERROR_HANDLER", "Compressor turned off due to critical error.");
        }
        // Удален блок прямого изменения current_dosing_state.
        // Модуль dosing_logic должен сам среагировать на current_system_error
        // в своем цикле handleDosingState() и перейти в состояние ошибки.
        app_log_d("ERROR_HANDLER", "Critical error set. Dosing logic should handle state transition if active.");
    }
    saveConfig(); // Сохраняем обновленный config (счетчик ошибок, сообщение)

    if (errorCode >= FATAL_WDT_RESET) { // Если ошибка была фатальной
        delay(5000);
        ESP.restart();
    }
}

void clearSystemError() {
    portENTER_CRITICAL(&error_handler_mutex);
    if (current_system_error == NO_ERROR) {
        portEXIT_CRITICAL(&error_handler_mutex);
        return;
    }
    // Сохраняем старые значения для лога перед очисткой
    SystemErrorCode_t prev_error = current_system_error;
    char prev_msg_copy[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE];
    strncpy(prev_msg_copy, last_error_msg_buffer_internal, sizeof(prev_msg_copy) -1);
    prev_msg_copy[sizeof(prev_msg_copy)-1] = '\0';

    current_system_error = NO_ERROR;
    strncpy(last_error_msg_buffer_internal, _T(L_NO_ACTIVE_ERROR_MSG), sizeof(last_error_msg_buffer_internal) -1);
    last_error_msg_buffer_internal[sizeof(last_error_msg_buffer_internal) -1] = '\0';
    portEXIT_CRITICAL(&error_handler_mutex);
    app_log_i("ERROR_HANDLER", "Clearing system error. Previous error: %d (%s)", prev_error, prev_msg_copy);

    strncpy(config.lastErrorMsgBuffer, _T(L_NO_ACTIVE_ERROR_MSG), sizeof(config.lastErrorMsgBuffer)-1);
    config.lastErrorMsgBuffer[sizeof(config.lastErrorMsgBuffer)-1] = '\0'; // Убедимся, что строка завершена null-терминатором
    saveConfig();
}

SystemErrorCode_t getSystemErrorCode() {
    SystemErrorCode_t code;
    portENTER_CRITICAL(&error_handler_mutex);
    code = current_system_error;
    portEXIT_CRITICAL(&error_handler_mutex);
    return code;
}

const char* getSystemErrorMessage() {
    const char* msg;
    portENTER_CRITICAL(&error_handler_mutex);
    msg = last_error_msg_buffer_internal; // Возвращаем указатель на внутренний буфер
    portEXIT_CRITICAL(&error_handler_mutex);
    return msg; // Внимание: возвращает указатель на внутренний буфер, не изменять извне!
}

const char* getSystemErrorCodeString(SystemErrorCode_t code) {
    switch (code) {
        case NO_ERROR: return _T(L_ERROR_DESC_NO_ERROR);
        // Example: Add more specific error codes here
        // case WIFI_ERROR: return _T(L_ERROR_DESC_WIFI_ERROR);
        // case CALIBRATION_ERROR: return _T(L_ERROR_DESC_CALIBRATION_ERROR);
        // ... other critical errors
        // case CRIT_TEMP_SENSOR_IN_FAIL: return _T(L_ERROR_DESC_CRIT_TEMP_SENSOR_IN_FAIL);
        // case CRIT_TEMP_SENSOR_OUT_FAIL: return _T(L_ERROR_DESC_CRIT_TEMP_SENSOR_OUT_FAIL);
        // ... etc.
        default: return _T(L_UNKNOWN_ERROR_CODE_DESC_MSG);
    }
}

int getStoredErrors(StoredError_t* buffer, int max_errors_to_get) {
    int num_to_copy = 0;
    portENTER_CRITICAL(&error_handler_mutex);
    if (buffer && max_errors_to_get > 0) {
        num_to_copy = (stored_errors_count < max_errors_to_get) ? stored_errors_count : max_errors_to_get;
        // Копируем в обратном порядке, чтобы новые ошибки были первыми
        for (int i = 0; i < num_to_copy; i++) {
            int source_idx = (stored_errors_next_idx - 1 - i + MAX_STORED_ERRORS) % MAX_STORED_ERRORS;
            buffer[i] = stored_errors_buffer[source_idx];
        }
    }
    portEXIT_CRITICAL(&error_handler_mutex);
    return num_to_copy;
}

void clearAllStoredErrors() {
    portENTER_CRITICAL(&error_handler_mutex);
    stored_errors_count = 0;
    stored_errors_next_idx = 0;
    portEXIT_CRITICAL(&error_handler_mutex);
    app_log_i("ERROR_HANDLER", "All stored error messages cleared.");
}