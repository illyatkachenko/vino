#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/ledc.h>
#include <LittleFS.h>               // <-- ДОБАВЛЕНО: Для файловой системы
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_log.h"   // <-- ДОБАВЛЕНО: Для подробного логирования ESP-IDF
#include <esp_event.h> // <-- ДОБАВЛЕНО: Для обработки событий Wi-Fi
#include "esp_task_wdt.h"           // Для Task Watchdog Timer
#include "soc/timer_group_struct.h" // Для системного таймера и сторожевого таймера (хотя напрямую не используются для программного WDT)
#include "soc/timer_group_reg.h"    // Для системного таймера и сторожевого таймера
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"          // Для vTaskDelay и xTaskCreate
#include "freertos/semphr.h"        // Для критических секций
#include <DNSServer.h>              // <-- Для Captive Portal (пока не используем, но готовим)
#include <nvs_flash.h>              // Для явной инициализации NVS
#include <limits.h>
#include <stdarg.h>                 // For variadic log functions
#include <math.h>                   // For isnan()
#include <Preferences.h>            // Убедимся, что Preferences включен

#include "main.h"                // Включаем для согласованности объявлений
// --- Include New Module Headers ---
#include "config_manager.h"
#include "error_handler.h"
#include "dosing_logic.h"
#include "calibration_logic.h"
#include "esp_now_handler.h"
#include "pid_controller.h"
#include "web_server_handlers.h" // Added
#include "motor_control.h"       // Added (was missing from previous includes)
#include "utils.h"               // Added for getUptimeString and initUtils
#include "sensors.h"             // Added (was missing from previous includes)
#include "hardware_pins.h"       // <-- ДОБАВЛЕНО: Единый файл с определениями пинов
#include "sensitive_config.h"    // <-- ДОБАВЛЕНО: Для getWifiSsid/Password
#include "localization.h" // <-- ДОБАВЛЕНО: Для локализации
#include "button_handler.h"      // <-- ДОБАВЛЕНО: Для обработки кнопок

// --- Firmware Version ---
const char* MAIN_FIRMWARE_VERSION = "4.3.1"; // Define the main firmware version

// --- Logging Configuration ---
#define LOG_LEVEL_NONE      0
#define LOG_LEVEL_ERROR     1
#define LOG_LEVEL_WARN      2
#define LOG_LEVEL_INFO      3
#define LOG_LEVEL_DEBUG     4
#define CURRENT_LOG_LEVEL LOG_LEVEL_INFO // Установите желаемый уровень логирования

const char* LOG_FILENAME = "/system_log.txt"; // Имя файла для логов
const unsigned long MAX_LOG_FILE_SIZE_BYTES = 500 * 1024; // Максимальный размер лог-файла (500 КБ)
bool g_littlefs_mounted = false;              // Флаг успешного монтирования LittleFS

// Logging functions
static void log_printf(const char* level, const char* tag, const char* format, va_list args) { // Made static as it's used by log_x in this file only now
    static bool fs_log_fail_logged_once = false; // Объявляем здесь, чтобы была доступна во всей функции
    char buffer[512];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.printf("[%s] %s: %s\n", level, tag, buffer);
    if (g_littlefs_mounted) {
        // Проверяем размер файла перед записью
        File logFileCheck = LittleFS.open(LOG_FILENAME, "r");
        if (logFileCheck) {
            if (logFileCheck.size() > MAX_LOG_FILE_SIZE_BYTES) {
                logFileCheck.close(); // Закрываем перед удалением
                if (LittleFS.remove(LOG_FILENAME)) {
                    Serial.printf("[W] FS_LOG: Log file %s was too large (%u bytes) and has been removed.\n", LOG_FILENAME, logFileCheck.size());
                    // Сбрасываем флаг ошибки логирования в ФС
                    fs_log_fail_logged_once = false;
                } else {
                     Serial.printf("[E] FS_LOG: Failed to remove large log file %s.\n", LOG_FILENAME);
                }
            } else {
                logFileCheck.close(); // Закрываем, если размер в норме
            }
        } // Если открыть для чтения не удалось, пробуем открыть для записи ниже.

        File logFile = LittleFS.open(LOG_FILENAME, "a"); // Открываем файл в режиме добавления
        if (logFile) {
            logFile.printf("[%s] %s: %s\n", level, tag, buffer);
            logFile.close();
        } else {
            // Осторожно с логированием ошибок файловой системы, чтобы не вызвать рекурсию
            // Можно выводить в Serial только один раз или использовать флаг
                Serial.printf("[E] FS_LOG: Failed to open %s for appending.\n", LOG_FILENAME);
                fs_log_fail_logged_once = true;
            }
        } // Closes if (g_littlefs_mounted)
} // Closes log_printf

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_ERROR
void app_log_e(const char* tag, const char* format, ...) { va_list args; va_start(args, format); log_printf("E", tag, format, args); va_end(args); }
#else
void app_log_e(const char* tag, const char* format, ...) {}
#endif

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_WARN
void app_log_w(const char* tag, const char* format, ...) { va_list args; va_start(args, format); log_printf("W", tag, format, args); va_end(args); }
#else
void app_log_w(const char* tag, const char* format, ...) {}
#endif

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_INFO
void app_log_i(const char* tag, const char* format, ...) { va_list args; va_start(args, format); log_printf("I", tag, format, args); va_end(args); }
#else
void app_log_i(const char* tag, const char* format, ...) {}
#endif

#if CURRENT_LOG_LEVEL >= LOG_LEVEL_DEBUG
void app_log_d(const char* tag, const char* format, ...) { va_list args; va_start(args, format); log_printf("D", tag, format, args); va_end(args); }
#else
void app_log_d(const char* tag, const char* format, ...) {}
#endif

// --- Configuration ---
WebServer server(80);

// GPIO Definitions теперь в hardware_pins.h
// #define BTN_FWD             32
// #define BTN_REV             33
// #define BTN_RESET           18
// #define BTN_POWER           5
// #define MOSFET_POWER_PIN    2

// Button Debounce Variables
// const unsigned long debounce_delay_ms = 50; // Перенесено в button_handler.cpp
// int btn_fwd_state = HIGH, btn_rev_state = HIGH, btn_reset_state = HIGH, btn_power_state = HIGH; // Перенесено в button_handler.cpp
bool ap_mode_active = false; // Определение флага режима AP

// AP Mode Configuration
const char* AP_SSID = "Main-ESP32-Setup";
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer; // Объявляем DNS сервер
const byte DNS_PORT = 53; // Стандартный порт DNS 


// int last_btn_fwd_state = HIGH, last_btn_rev_state = HIGH, last_btn_reset_state = HIGH, last_btn_power_state = HIGH; // Перенесено в button_handler.cpp
// unsigned long last_btn_fwd_change = 0, last_btn_rev_change = 0, last_btn_reset_change = 0, last_btn_power_change = 0; // Перенесено в button_handler.cpp

// System State Variables
bool system_power_enabled = false;
bool reset_requested = false;
bool g_preferences_operational = true; // По умолчанию считаем, что Preferences работают
unsigned long reset_request_time = 0;
const unsigned long reset_delay_ms = 5000; // Hold reset button for 5 seconds to restart
// bool reset_requested = false; // Перенесено в button_handler.cpp
// unsigned long reset_request_time = 0; // Перенесено в button_handler.cpp

unsigned long last_wifi_reconnect = 0; // Для контроля переподключения WiFi

// Error Handling variables are now in error_handler.c/error_handler.h
// Dosing State Machine variables are now in dosing_logic.c/dosing_logic.h
// ESP-NOW variables are now in esp_now_handler.c/esp_now_handler.h
unsigned long last_esp_now_status_send_time = 0;
const unsigned long ESP_NOW_STATUS_INTERVAL_MS = 2000; // Send status every 2 seconds

// Watchdog Timer
const int WDT_TIMEOUT_S = 30; // Таймаут сторожевого таймера в секундах

// --- Forward Declarations (Прототипы функций) ---
// Most forward declarations are now in respective .h files
// void handleButtons(); // Теперь объявлена в button_handler.h

// --- WiFi Event Handler ---
// Функция, которая будет вызываться при различных событиях Wi-Fi
// Изменяем сигнатуру, чтобы принимать event_data
void WiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
    app_log_i("WIFI_EVENT", "WiFi Event: %d", event);

    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_START:
            app_log_i("WIFI_EVENT", "STA Start");
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP: // Event 111 - STA mode stopped
            Serial.flush();
            Serial.println(">>>> DEBUG: STA_STOP_CASE_REACHED <<<<");
            Serial.printf("[SERIAL_DIRECT] WIFI_EVENT: STA Stopped. Event ID: %d\n", event);
            delay(10);
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.flush(); // Попытка сбросить буфер перед выводом
            Serial.println(">>>> DEBUG: STA_DISCONNECTED_CASE_REACHED <<<<");
            // Закомментируем доступ к info на всякий случай, если он вызывает сбой до вывода
            // Accessing info.wifi_sta_disconnected should be safe here
            Serial.printf("[SERIAL_DIRECT] WIFI_EVENT: STA Disconnected. Reason code: %d\n", info.wifi_sta_disconnected.reason);
            delay(10); // Небольшая задержка для вывода
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            app_log_i("WIFI_EVENT", "STA Connected");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            app_log_i("WIFI_EVENT", "STA Got IP: %s", WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_AP_START:
            app_log_i("WIFI_EVENT", "AP Start");
            break;
        case ARDUINO_EVENT_WIFI_AP_STOP:
            app_log_i("WIFI_EVENT", "AP Stop");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            app_log_i("WIFI_EVENT", "AP STA Connected");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            app_log_i("WIFI_EVENT", "AP STA Disconnected");
            break;
        default:
            break;
    }
}
// toggleSystemPower declared in main.h

// Переместим определение WiFiEvent выше, чтобы оно было доступно при регистрации
// в setup(). Если оно уже выше, то все в порядке.
// Убедимся, что WiFi.onEvent(WiFiEvent); вызывается в setup()
// ПОСЛЕ WiFi.mode(WIFI_STA); или любой другой инициализации Wi-Fi,
// но ДО WiFi.begin(). Хотя, обычно, регистрация события может быть и раньше.
// Для Arduino ESP32 core, регистрация до WiFi.begin() должна работать.
// В твоем коде WiFi.onEvent(WiFiEvent); уже стоит в правильном месте в setup().

// --- Function Implementations ---
// Функции, которые остаются в main.c или будут вынесены в utils.c / button_handler.c

// manualMotorForward, manualMotorReverse, stopManualMotor, handleMotorStepping - теперь в motor_control.c

void toggleSystemPower(bool fromWeb) {
    system_power_enabled = !system_power_enabled;
    digitalWrite(MOSFET_POWER_PIN, system_power_enabled ? HIGH : LOW);
    app_log_i("SYSTEM", "System Power Toggled to: %s", system_power_enabled ? "ON" : "OFF"); // Исправлен формат спецификатора

    if (!system_power_enabled) {
        if (isMotorRunningManual()) stopManualMotor(); // Используем геттер
        if (isMotorRunningAuto()) stopDosingCycle(false); // Используем геттер
        if (isCompressorRunning()) compressorOff(); // Используем геттер
        if (getCalibrationModeState()) { // Используем геттер
             app_log_w("SYSTEM", "System powered OFF during active calibration. Stopping calibration."); // Исправлен формат спецификатора
             stopMotor(); // из motor_control.h
             setCalibrationModeState(false); // из calibration_logic.h
        }
        app_log_i("SYSTEM", "System powered OFF. All processes stopped."); // Исправлен формат спецификатора
    } else {
        app_log_i("SYSTEM", "System powered ON.");
    }
    config.systemPowerStateSaved = system_power_enabled;
    saveConfig();
    if (fromWeb) { server.sendHeader("Location", "/"); server.send(303); }
}

// Определение функции isSystemPowerEnabled
bool isSystemPowerEnabled() {
    return system_power_enabled;
}

// String getUptimeString() moved to utils.c

// handlePidControl - теперь в pid_controller.c
// All web handlers are now in web_server_handlers.c

// --- Setup and Loop ---

void setup() {
    Serial.begin(115200);
    esp_log_level_set("wifi", ESP_LOG_VERBOSE);    // Включаем подробное логирование Wi-Fi
    esp_log_level_set("event", ESP_LOG_VERBOSE);   // Включаем подробное логирование системных событий

    app_log_i("DEBUG_CONSTS", "Value of ARDUINO_EVENT_WIFI_STA_DISCONNECTED is: %d, ARDUINO_EVENT_WIFI_STA_STOP is: %d", ARDUINO_EVENT_WIFI_STA_DISCONNECTED, ARDUINO_EVENT_WIFI_STA_STOP); // Проверка значения константы

    // Инициализация LittleFS
    if (!LittleFS.begin(true)) { // true - форматировать, если не удалось смонтировать
        // Cannot use _T here if localization is not yet initialized
        Serial.println("[E] SETUP: LittleFS Mount Failed! Log to file will be disabled.");
        g_littlefs_mounted = false;
    } else {
        g_littlefs_mounted = true;
        // app_log_i can be used now if it writes to Serial primarily, or if LittleFS is ok for this one message
        app_log_i("SETUP", "LittleFS mounted successfully."); 
    }
    unsigned long setup_start_time = millis();
    while (!Serial && (millis() - setup_start_time < 2000)); // Ждем Serial не более 2 сек

      // --- НАЧАЛО ВРЕМЕННОГО КОДА ДЛЯ ОЧИСТКИ NVS ---
       // !!! ВНИМАНИЕ: Раскомментируйте этот блок ТОЛЬКО ДЛЯ ОДНОЙ ПРОШИВКИ, чтобы очистить NVS.
    // !!! Затем СНОВА ЗАКОММЕНТИРУЙТЕ его и прошейте еще раз.
    /*
    app_log_w("SETUP", "!!! ВНИМАНИЕ: Принудительная очистка NVS через 5 секунд !!!");
    delay(5000);
    if (nvs_flash_erase() == ESP_OK) {
        app_log_i("SETUP", "NVS успешно очищен.");
    } else {
        app_log_e("SETUP", "Ошибка при очистке NVS.");
    }
    app_log_w("SETUP", "!!! Очистка NVS завершена. Перезагрузка через 3 секунды. ЗАКОММЕНТИРУЙТЕ КОД ОЧИСТКИ ПЕРЕД СЛЕДУЮЩЕЙ ПРОШИВКОЙ !!!");
    delay(3000);
    ESP.restart();*/
    // --- КОНЕЦ ВРЕМЕННОГО КОДА ДЛЯ ОЧИСТКИ NVS ---

    // Явная инициализация NVS в самом начале
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS может быть поврежден или содержать старую версию. Попробуем стереть и инициализировать заново.
        app_log_w("SETUP", "NVS: No free pages or new version found, attempting to erase NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase()); // Стираем, если нужно
        nvs_err = nvs_flash_init();         // Инициализируем снова
    }
    ESP_ERROR_CHECK(nvs_err); // Проверяем результат инициализации NVS. При ошибке здесь будет остановка.
    if(nvs_err == ESP_OK) app_log_i("SETUP", "NVS initialized successfully by explicit call."); else app_log_e("SETUP", "NVS explicit init failed: %s", esp_err_to_name(nvs_err));

    // Попытка создать/проверить пространство имен "fatal_log" в режиме R/W
    // Это должно помочь, если проблема в том, что пространство имен не создано корректно.
    Preferences temp_prefs_rw_check;
    if (g_preferences_operational && !temp_prefs_rw_check.begin(PREFS_ERROR_LOG_NAMESPACE, false)) { // false для read-write
        app_log_e("SETUP_DBG", "CRITICAL: Не удалось открыть/создать пространство имен '%s' в режиме R/W. Проблема с NVS.", PREFS_ERROR_LOG_NAMESPACE);
        g_preferences_operational = false; // Отмечаем, что Preferences не работают
    } else if (g_preferences_operational) {
        app_log_i("SETUP_DBG", "Пространство имен '%s' успешно открыто/создано в режиме R/W.", PREFS_ERROR_LOG_NAMESPACE);
        temp_prefs_rw_check.end(); // Закрываем сессию R/W
    }


    // app_log_i("SETUP", "System Starting (v4.2 - Main Module Sync, Diagnostics Added)...");
    Serial.println("[I] SETUP: System Starting (v4.3 - Web Handlers Modularized)...");

    initUtils(); // Initialize systemStartTime in utils.c
    
    // Попытка деинициализировать TWDT перед повторной инициализацией
    // Это может помочь, если TWDT был инициализирован системой ранее с другими параметрами.
    esp_err_t deinit_result = esp_task_wdt_deinit();
    if (deinit_result == ESP_OK) {
        app_log_i("SETUP", "TWDT deinitialized successfully before re-init.");
    } else if (deinit_result == ESP_ERR_INVALID_STATE) {
        app_log_i("SETUP", "TWDT was not previously initialized, no deinit needed.");
        // Это нормальное состояние, если TWDT не был инициализирован ранее.
    } else {
        app_log_w("SETUP", "Failed to deinitialize TWDT, error: %s. Proceeding with init.", esp_err_to_name(deinit_result));
    }

    // Конфигурируем и инициализируем Task Watchdog Timer (Новый API)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000, 
        .trigger_panic = true, // При срабатывании WDT вызывать панику
        // .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // For ESP-IDF v4.x
        // .subscribe_idle_tasks = true // For ESP-IDF v5.x
    };
    if (esp_task_wdt_init(&twdt_config) != ESP_OK) { 
        app_log_e("SETUP", "Task WDT init failed!"); 
    } else {
        if (esp_task_wdt_add(NULL) != ESP_OK) { app_log_e("SETUP", "Failed to add current task (loop) to WDT!"); }
        else { app_log_i("SETUP", "TWDT initialized and current task added."); }
    }

    app_log_i("SETUP_DBG", "--- STARTING MINIMAL SETUP FOR DIAGNOSTICS ---");

    // Проверка на наличие сохраненной фатальной ошибки
    Preferences error_prefs_check;
    if (error_prefs_check.begin(PREFS_ERROR_LOG_NAMESPACE, true)) { // R/O mode
        uint32_t last_fatal_code = error_prefs_check.getUInt("last_fatal", 0); // Используем SystemErrorCode
        error_prefs_check.end(); // Закрываем R/O сессию
        if (last_fatal_code != 0) {
            // _T might not be fully ready if language from config isn't loaded yet.
            // Using direct Serial print for very early critical messages might be safer, or ensure localization_init_default_lang() is called.
            // Assuming localization_init_default_lang() will be called before this point.
            app_log_e("REBOOT_INFO", _T(L_REBOOTED_DUE_TO_FATAL_ERROR), last_fatal_code);
            // Очищаем флаг
            Preferences error_prefs_clear; // Новый экземпляр для записи
            if (error_prefs_clear.begin(PREFS_ERROR_LOG_NAMESPACE, false)) { // R/W mode
                error_prefs_clear.remove("last_fatal");
                error_prefs_clear.end();
                app_log_i("REBOOT_INFO", _T(L_FATAL_ERROR_FLAG_CLEARED_MSG));
            } else {
                 app_log_e("REBOOT_INFO", _T(L_PREFS_OPEN_FAIL_CLEAR_FATAL_FLAG));
            }
        }
    } else {
        app_log_w("REBOOT_INFO", _T(L_PREFS_OPEN_FAIL_CHECK_FATAL_FLAG), PREFS_ERROR_LOG_NAMESPACE);
        g_preferences_operational = false; // Отмечаем, что Preferences не работают
    }

    app_log_i("SETUP_DBG", "Preferences check for 'err_log' completed. g_preferences_operational: %s", g_preferences_operational ? "true" : "false");

 // ВРЕМЕННО ОТКЛЮЧАЕМ БОЛЬШУЮ ЧАСТЬ ИНИЦИАЛИЗАЦИЙ ДЛЯ ДИАГНОСТИКИ  // <--- УБИРАЕМ НАЧАЛО КОММЕНТАРИЯ БЛОКА

    localization_init_default_lang(); // Initialize with default language (e.g., "en" or "ru")
    loadConfig(); // Загружаем конфигурацию, включая system_power_enabled and config.currentLanguage
    set_current_language(config.currentLanguage); // Set the language from config

    // Инициализация модулей
    initErrorHandler();
    initMotor();      // Из motor_control.c
    initSensors();    // Из sensors.c (включая Flow Sensor interrupt и Compressor pin)
    initDosingLogic(); // Из dosing_logic.c
    initCalibrationLogic(); // Из calibration_logic.c
    initPidController(); // Из pid_controller.c
    initSensitiveConfig(); // Инициализация модуля чувствительных настроек

    // Инициализация пинов кнопок (важно, если handleButtons() вызывается в loop)
    pinMode(BTN_FWD, INPUT_PULLUP);
    pinMode(BTN_REV, INPUT_PULLUP);
    pinMode(BTN_RESET, INPUT_PULLUP);
    pinMode(BTN_POWER, INPUT_PULLUP);
    pinMode(MOSFET_POWER_PIN, OUTPUT);
    // Устанавливаем состояние MOSFET_POWER_PIN в соответствии с загруженным system_power_enabled


#ifndef IGNORE_MISSING_SENSORS_FOR_DEVELOPMENT
    if (!tempOutSensorFound && getSystemErrorCode() != CRIT_TEMP_SENSOR_OUT_FAIL) { // Проверяем ошибку еще раз после loadConfig
        setSystemError(CRIT_TEMP_SENSOR_OUT_FAIL, "CRITICAL: Output temp sensor not found after config load!");
    }
#else
    if (!tempOutSensorFound) {
        app_log_w("SETUP_DEV", "Output temperature sensor not found, but IGNORED for development.");
        // Можно добавить здесь установку tOut и tOut_filtered в какое-то безопасное значение по умолчанию, если это нужно для другой логики
    }
#endif

    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent); // <-- ПЕРЕМЕЩЕНО: Регистрируем обработчик событий Wi-Fi ПОСЛЕ установки режима
    const char* ssid = getWifiSsid();         // <-- Используем геттер
    const char* password = getWifiPassword(); // <-- Используем геттер
    app_log_i("WIFI_PRE_BEGIN", "Attempting WiFi.begin with SSID: '%s' (len: %d), Password_len: %d", ssid, strlen(ssid), strlen(password));
    WiFi.begin(ssid, password); // <--- ДОБАВЛЕН ЯВНЫЙ ВЫЗОВ WiFi.begin()
    app_log_i("WIFI", "Connecting to WiFi: %s", ssid); // Исправлен формат спецификатора
    unsigned long wifi_connect_start = millis();
    int wifi_retries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_retries < 20) { // Уменьшено количество попыток для ускорения загрузки в случае проблем
        delay(500);
        Serial.print(".");
        wifi_retries++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        app_log_i("WIFI", "Connected! IP Address: %s", WiFi.localIP().toString().c_str()); // Исправлен формат спецификатора
        last_wifi_reconnect = millis();
    } else {
        app_log_e("WIFI", "Failed to connect to WiFi in STA mode after %d retries. Switching to AP mode.", wifi_retries); // Исправлен формат спецификатора
        ap_mode_active = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0)); // Установка IP для AP
        if (WiFi.softAP(AP_SSID)) { // Запускаем AP без пароля
            app_log_i("WIFI_AP", "Access Point '%s' started. IP: %s", AP_SSID, apIP.toString().c_str()); // Исправлен формат спецификатора
            app_log_i("WIFI_AP", _T(L_AP_CONFIGURE_WIFI_NAVIGATE), apIP.toString().c_str());
            // Запускаем DNS сервер для Captive Portal
            if (dnsServer.start(DNS_PORT, "*", apIP)) {
                app_log_i("DNS", "DNS server started for Captive Portal."); // Исправлен формат спецификатора
            } else {
                app_log_e("DNS", "Failed to start DNS server!");
            }
        } else {
            app_log_e("WIFI_AP", "Failed to start Access Point!");
            setSystemError(WIFI_ERROR, "Failed to start AP mode."); // Критическая ошибка, если AP не стартует
        }
    }

    // Initialize ESP-NOW after Wi-Fi interface is up (either STA attempted or AP started)
    initEspNow(); // Из esp_now_handler.c (должен быть после loadConfig для MAC-адреса)

    initWebServerUtils();   // Initialize CSRF token
    setupWebServerRoutes(); // Setup all server.on() routes and server.begin()
                            // setupWebServerRoutes теперь будет учитывать ap_mode_active
    // systemStartTime = millis(); // Moved to initUtils()

 // КОНЕЦ ВРЕМЕННО ОТКЛЮЧЕННОГО БЛОКА // <--- УБИРАЕМ КОНЕЦ КОММЕНТАРИЯ БЛОКА

    // Если блок выше закомментирован, но handleButtons() используется, инициализируем пины кнопок здесь:
    // pinMode(BTN_FWD, INPUT_PULLUP); // Теперь это будет внутри раскомментированного блока
    // pinMode(BTN_REV, INPUT_PULLUP);
    // pinMode(BTN_RESET, INPUT_PULLUP);
    // pinMode(BTN_POWER, INPUT_PULLUP);
    // pinMode(MOSFET_POWER_PIN, OUTPUT);
    // digitalWrite(MOSFET_POWER_PIN, system_power_enabled ? HIGH : LOW); // Состояние питания будет установлено из loadConfig или по умолчанию

    app_log_i("SETUP", "System Ready. Free heap: %u", ESP.getFreeHeap());
} // <--- УДАЛИТЕ ЭТОТ КОММЕНТАРИЙ, ЕСЛИ ОН ЕСТЬ
void loop() {
    esp_task_wdt_reset(); // Сбрасываем сторожевой таймер в каждой итерации

    if (!ap_mode_active && WiFi.status() != WL_CONNECTED) { // Переподключаемся только если не в AP режиме
        if (millis() - last_wifi_reconnect > 30000) { // Попытка переподключения каждые 30 секунд
            app_log_w("WIFI", "WiFi disconnected. Attempting to reconnect..."); // Исправлен формат спецификатора
            WiFi.disconnect(true); // true для очистки предыдущей конфигурации
            delay(100); // Небольшая задержка перед новой попыткой
            const char* ssid = getWifiSsid();         // <-- Используем геттер
            const char* password = getWifiPassword(); // <-- Используем геттер
            app_log_i("WIFI_PRE_BEGIN", "Attempting WiFi.begin with SSID: '%s' (len: %d), Password_len: %d", ssid, strlen(ssid), strlen(password));
            WiFi.begin(ssid, password); // <--- ДОБАВЛЕН ЯВНЫЙ ВЫЗОВ WiFi.begin() ДЛЯ ПЕРЕПОДКЛЮЧЕНИЯ
            last_wifi_reconnect = millis();
            if(getSystemErrorCode() != WIFI_ERROR && getSystemErrorCode() != ESP_NOW_INIT_ERROR && getSystemErrorCode() != ESP_NOW_PEER_ERROR) {
                 // Устанавливаем ошибку WiFi только если нет более специфичной ошибки ESP-NOW
                 // setSystemError(WIFI_ERROR, "WiFi disconnected, attempting reconnect.");
            }
        }
    } else if (!ap_mode_active && WiFi.status() == WL_CONNECTED) { // WiFi подключен в режиме STA
        if (getSystemErrorCode() == WIFI_ERROR) { // Если была ошибка WiFi, но сейчас подключено
            app_log_i("WIFI", "WiFi reconnected. Clearing WiFi error."); // Исправлен формат спецификатора
            clearSystemError();
        }
        // Логика проверки и передобавления пира ESP-NOW теперь в esp_now_handler
        // esp_now_handler сам будет пытаться восстановить пир при необходимости,
        // например, перед отправкой или по таймеру.
        // В main.c можно периодически вызывать функцию проверки из esp_now_handler, если это нужно для логики main.
        // ensureEspNowPeer(); // Пример вызова функции из esp_now_handler, если она там реализована
        }
    // Обработка HTTP клиента должна быть здесь, чтобы она выполнялась и для STA, и для AP (если включено ниже)
    // server.handleClient(); // Перемещено в условные блоки ниже


    // Если в режиме AP, то основной цикл логики устройства (дозирование, ESP-NOW и т.д.) не выполняется
    // или выполняется ограниченно. Сейчас мы его пропускаем.
    if (!ap_mode_active) {
        handleButtons();
        // Логика для режима STA
        // ... (существующий код для режима STA) ...

        handleFlowSensor();
        handleDosingState();
        handleCalibrationLogic();
        // handleTempLogic() теперь вызывается после dosing_logic, чтобы dosing_logic мог сначала обновить свое состояние,
        // а затем sensors.c мог учесть это состояние при управлении компрессором.
        handleTempLogic(); // Читает температуры и управляет общим температурным режимом
        handleMotorStepping();
        // ESP-NOW статус отправляется из esp_now_handler.c по таймеру или по событию
        if (esp_now_peer_added && WiFi.status() == WL_CONNECTED && millis() - last_esp_now_status_send_time >= ESP_NOW_STATUS_INTERVAL_MS) {
            sendSystemStatusEspNow();
            last_esp_now_status_send_time = millis(); // Исправлен формат спецификатора
        }

        DosingState_t local_dosing_state_for_pid; // Используем DosingState_t, так как dosing_logic.h будет обновлен
        portENTER_CRITICAL(&dosing_state_mutex);
        local_dosing_state_for_pid = current_dosing_state;
        portEXIT_CRITICAL(&dosing_state_mutex);

        // Вызов PID-регулятора, если он включен и система в подходящем состоянии
        if (getIsPidTempControlEnabled() && system_power_enabled && (local_dosing_state_for_pid == DOSING_STATE_RUNNING || local_dosing_state_for_pid == DOSING_STATE_STARTING)) {
            handlePidControl();
        }
    }

    // --- Web Server and DNS Handling ---
    server.handleClient(); // Обрабатываем HTTP-запросы клиента (должно вызываться в каждой итерации loop)

    if (ap_mode_active) { // Мы в режиме AP
        // dnsServer.processNextRequest(); // ОСТАВЛЯЕМ ЗАКОММЕНТИРОВАННЫМ ДЛЯ ДИАГНОСТИКИ
        // app_log_d("LOOP_AP", "AP Mode Loop iteration"); // Опциональное логирование для режима AP
    }
    // --- End Web Server and DNS Handling ---

    vTaskDelay(pdMS_TO_TICKS(10)); // Небольшая задержка для стабильности и WDT
}
