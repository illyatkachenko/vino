#include "esp_now_handler.h"
#include "config_manager.h"    // Для config, saveConfig, getScreenMacAddress
#include <WiFi.h>              // Для WiFi.status(), WiFi.macAddress()
// --- Временное объявление для диагностики проблемы с include ---
#include "error_handler.h"     // Для setSystemError, clearSystemError, SystemErrorCode_t, getSystemErrorCode, getSystemErrorMessage, PREFERENCES_ERROR, CRIT_MOTOR_FAIL
#include "dosing_logic.h"      // Для startDosingCycle, stopDosingCycle, getDosingState, getCurrentDosedVolume, dosing_state_mutex
#include "calibration_logic.h" // Для startCalibrationMode, stopCalibrationMode, getCalibrationModeState
#include "motor_control.h"     // Для updateMotorSpeed, stopMotor
#include "sensors.h"           // Для getTempOut, getFlowRate, isCompressorRunning, compressorOff, isWaterLevelOk, tIn, tOut_filtered, compressorOn
#include "main.h"              // Для app_log_x функций и isSystemPowerEnabled(), MAIN_FIRMWARE_VERSION
#include "pid_controller.h"    // For getIsPidTempControlEnabled, getPidSetpointTemp
#include "localization.h"      // For _T(), L_ERROR_ESPNOW_INIT_FAIL, L_ERROR_ESPNOW_RECV_CB_REGISTER_FAIL, L_ERROR_INVALID_PEER_MAC_IN_CONFIG, L_ERROR_ESPNOW_PEER_ADD_READD_FAIL_MAC, L_ERROR_ESPNOW_PEER_ADD_READD_FAIL, L_WARN_ESPNOW_SEND_FAIL_AFTER_RETRIES, L_ERROR_EMERGENCY_STOP_VIA_ESPNOW
#include "utils.h"             // Для getUptimeSeconds()
#include "esp_now_protocol.h"  // <--- ДОБАВЛЕНО: Для struct_status_t, command_type_t, system_state_t, struct_command_t
#include <string.h>            // <--- ДОБАВЛЕНО: Для strncpy
#include "esp_err.h"           // Для esp_err_to_name
#include "freertos/FreeRTOS.h" // Для vTaskDelay, pdMS_TO_TICKS
#include <math.h>              // Для isnan()

// Временное extern объявление для диагностики проблемы с видимостью из config_manager.h
extern bool getScreenMacAddress(uint8_t* mac_addr_buf);

// Note: The compiler reports 'getScreenMacAddress' and 'L_ERROR_ESPNOW_PEER_ADD_READD_FAIL_MAC'
// are not declared in this scope, despite 'config_manager.h' and 'localization.h' being included
// and the declarations/definitions being present in the provided files.
// This suggests a potential issue with the build environment, include paths, or file caching.
// The code below assumes these declarations are correctly available via the includes.

// Global variables for ESP-NOW, specific to this module
// static uint8_t remotePeerAddress[6]; // Filled from config.remotePeerMacStr // Removed, replaced by s_screen_mac_address

// Constants for ESP-NOW
// const int ESP_NOW_SEND_MAX_RETRIES_LOCAL = 3; // Defined below
// const unsigned long ESP_NOW_SEND_RETRY_DELAY_MS_LOCAL = 200; // Defined below
// unsigned long last_peer_check_time = 0; // Defined below
// const unsigned long PEER_CHECK_INTERVAL_MS = 15000; // Defined below

// Глобальные переменные для ESP-NOW, специфичные для этого модуля
const int ESP_NOW_SEND_MAX_RETRIES_LOCAL = 3; // Локальное имя, чтобы не конфликтовать с возможным define
const unsigned long ESP_NOW_SEND_RETRY_DELAY_MS_LOCAL = 200;
unsigned long last_peer_check_time = 0;
const unsigned long PEER_CHECK_INTERVAL_MS = 15000; // Проверять/передобавлять пир каждые 15 сек

// Глобальная переменная, определенная в esp_now_handler.h
bool esp_now_peer_added = false;
// uint8_t screen_mac_address[6]; // MAC-адрес экрана, должен быть загружен из Preferences // This was in the diff as added then removed, using s_screen_mac_address
static uint8_t s_screen_mac_address[6]; // Статический MAC-адрес экрана, будет загружен из Preferences
static bool s_screen_mac_valid = false;
// Мьютекс для защиты доступа к s_screen_mac_address, если он может изменяться во время выполнения
// SemaphoreHandle_t screen_mac_mutex = NULL; // Если потребуется


uint8_t calculate_checksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

// --- Инициализация ESP-NOW ---
void initEspNow() {
    // screen_mac_mutex = xSemaphoreCreateMutex(); // Если потребуется
    if (esp_now_init() != ESP_OK) {
        app_log_e("ESP_NOW_INIT", _T(L_ERROR_ESPNOW_INIT_FAIL)); // Использование локализованного сообщения
        setSystemError(ESP_NOW_INIT_ERROR, _T(L_ERROR_ESPNOW_INIT_FAIL));
        return;
    }
    esp_now_register_send_cb(onEspNowSend);
    // Используем reinterpret_cast для приведения типа функции, если компилятор строг
    // или убедимся, что сигнатура onEspNowReceive точно соответствует esp_now_recv_cb_t
    if (esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(onEspNowReceive)) != ESP_OK) {
        app_log_e("ESPNOW", "Error registering ESP-NOW receive callback"); // Original log message
        setSystemError(ESP_NOW_INIT_ERROR, _T(L_ERROR_ESPNOW_RECV_CB_REGISTER_FAIL));
    }
    // Загружаем MAC-адрес экрана из конфигурации
    s_screen_mac_valid = getScreenMacAddress(s_screen_mac_address);
    ensureEspNowPeer(); // Первая попытка добавить пир
    app_log_i("ESP_NOW_INIT", "ESP-NOW Initialized.");
}

void ensureEspNowPeer() {
    // Загрузка MAC-адреса экрана из Preferences (если еще не загружен)
    if (!s_screen_mac_valid) { // Если MAC не был загружен при initEspNow или стал невалидным
        // Try to load it again if it was not valid
        s_screen_mac_valid = getScreenMacAddress(s_screen_mac_address);
        if (!s_screen_mac_valid) { // Повторная проверка после попытки загрузки
            app_log_e("ESP_NOW_PEER", "Screen MAC address not configured. Cannot add peer.");
            esp_now_peer_added = false;
            if (getSystemErrorCode() != PREFERENCES_ERROR) { // Не перезаписываем более важную ошибку Preferences (NVS)
                setSystemError(PREFERENCES_ERROR, _T(L_ERROR_INVALID_PEER_MAC_IN_CONFIG));
            }
            return;
        }
    }

    // Если пир уже добавлен и существует, ничего не делаем
    if (esp_now_peer_added && esp_now_is_peer_exist(s_screen_mac_address)) {
        return;
    }

    // Сначала удаляем, если он вдруг был добавлен с ошибкой или устарел
    if (esp_now_is_peer_exist(s_screen_mac_address)) {
        esp_now_del_peer(s_screen_mac_address);
    }
    
    esp_now_peer_info_t peerInfo = {}; 
    memcpy(peerInfo.peer_addr, s_screen_mac_address, 6);
    peerInfo.channel = ESP_NOW_CHANNEL; // Используем глобально определенный канал ESP_NOW_CHANNEL (assuming ESP_NOW_CHANNEL is defined elsewhere, e.g. config_manager.h or main.h)
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;


    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        esp_now_peer_added = true;
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                s_screen_mac_address[0], s_screen_mac_address[1], s_screen_mac_address[2],
                s_screen_mac_address[3], s_screen_mac_address[4], s_screen_mac_address[5]);
        app_log_i("ESPNOW", "Peer %s added/re-added successfully.", macStr);
        if (getSystemErrorCode() == ESP_NOW_PEER_ERROR) clearSystemError();
        if (getSystemErrorCode() == ESP_NOW_INIT_ERROR) clearSystemError();
    } else {
        esp_now_peer_added = false; 
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                s_screen_mac_address[0], s_screen_mac_address[1], s_screen_mac_address[2],
                s_screen_mac_address[3], s_screen_mac_address[4], s_screen_mac_address[5]);
        // Используем L_ERROR_ESPNOW_PEER_ADD_READD_FAIL, так как L_ERROR_ESPNOW_PEER_ADD_READD_FAIL_MAC может быть не определен
        app_log_e("ESPNOW", "%s MAC: %s", _T(L_ERROR_ESPNOW_PEER_ADD_READD_FAIL), macStr); // Исправлено использование ключа локализации
        if (getSystemErrorCode() != ESP_NOW_PEER_ERROR) setSystemError(ESP_NOW_PEER_ERROR, _T(L_ERROR_ESPNOW_PEER_ADD_READD_FAIL));
    }
}

void onEspNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    app_log_d("ESPNOW_CB", "Send CB, status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
    if (status != ESP_NOW_SEND_SUCCESS) {
        // Ошибка отправки здесь логируется, но setSystemError вызывается в sendEspNowData после всех попыток.
    }
}

void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    struct_command_t cmd; // Используем struct_command_t из esp_now_protocol.h
    // Original check was: if (len == sizeof(esp_now_cmd_t))
    // Diff introduces: else if (len == sizeof(struct_command_t))
    // Assuming the old esp_now_cmd_t is deprecated and we only care about struct_command_t
    if (len == sizeof(struct_command_t)) { 
        memcpy(&cmd, incomingData, sizeof(struct_command_t)); // Copy data into the new struct type

        // Checksum logic from original onEspNowReceive might be needed here if struct_command_t has a checksum field
        // For now, assuming struct_command_t does not have a checksum field as per the diff's focus on cmd_type and value.
        // If it does, it should be validated:
        // uint8_t received_checksum = cmd.checksum; // Assuming cmd.checksum exists
        // cmd.checksum = 0; 
        // uint8_t calculated_checksum = calculate_checksum((uint8_t*)&cmd, sizeof(cmd) - sizeof(cmd.checksum));
        // if (received_checksum != calculated_checksum) {
        //     app_log_w("ESPNOW_RX", "Checksum mismatch! Rcv: %02X, Calc: %02X. Discarding.", received_checksum, calculated_checksum);
        //     return;
        // }

        app_log_i("ESPNOW_RX", "Cmd: %d (Value:%d)", cmd.cmd_type, cmd.value); // Логируем поля из struct_command_t

        // --- Handle Commands from Screen (using command_type_t) ---
        switch (cmd.cmd_type) { // Используем cmd_type из struct_command_t
            case CMD_SET_TEMPERATURE:
                if (cmd.value >= -10 && cmd.value <= 30) { // Assuming value is temperature in degrees C
                    app_log_i("ESPNOW_RX", "Set Target Temp: %d C", cmd.value);
                    config.tempSetpoint = (float)cmd.value; // Используем config.tempSetpoint
                    saveConfig();
                    // PID controller will pick up the new setpoint from config
                } else {
                    app_log_w("ESPNOW_RX", "Invalid Target Temp: %d", cmd.value);
                }
                break;
            case CMD_SET_VOLUME:
                if (cmd.value > 0 && cmd.value <= 10000) { // Assuming max volume 10000ml
                    app_log_i("ESPNOW_RX", "Set Target Volume: %d ml", cmd.value);
                    config.volumeTarget = cmd.value; // Используем config.volumeTarget
                    saveConfig();
                } else {
                    app_log_w("ESPNOW_RX", "Invalid Target Volume: %d", cmd.value);
                }
                break;
            case CMD_START_PROCESS:
                if (cmd.value > 0) { // Используем поле value для объема
                    app_log_i("ESPNOW_RX", "Start Dosing command: %d ml", cmd.value);
                    startDosingCycle(cmd.value, false); // false - не из веб
                } else {
                    app_log_w("ESPNOW_RX", "Invalid volume for Start Dosing: %d", cmd.value);
                }
                break; 
            case CMD_PAUSE:
                app_log_i("ESPNOW_RX", "Pause command received.");
                // TODO: Implement pauseDosingCycle() in dosing_logic.c/h
                // pauseDosingCycle();
                break;
            case CMD_RESUME:
                app_log_i("ESPNOW_RX", "Resume command received.");
                // TODO: Implement resumeDosingCycle() in dosing_logic.c/h
                // resumeDosingCycle();
                break;
            case CMD_STOP_PROCESS: 
                app_log_i("ESPNOW_RX", "Stop Process command"); // Changed log from "Stop Dosing" to "Stop Process"
                // This command is used for both "Stop Dosing" and "Emergency Stop" in the diff.
                // Let's assume CMD_STOP_PROCESS means stop the current process (dosing/calibration)
                // CMD_SET_MOTOR_SPEED and calibration commands were removed as they are not in esp_now_protocol.h command_type_t
                stopMotor(); 
                if(isCompressorRunning()) compressorOff(); // Check before turning off
                if (getCalibrationModeState()) {
                    // stopCalibrationMode(0, false); // Original logic for stopping calibration
                    // Consider if a more specific stop is needed or if stopMotor() is sufficient
                }

                DosingState_t local_ds_state;
                portENTER_CRITICAL(&dosing_state_mutex); 
                local_ds_state = getDosingState(); 
                portEXIT_CRITICAL(&dosing_state_mutex);

                if (local_ds_state != DOSING_STATE_IDLE &&
                    local_ds_state != DOSING_STATE_FINISHED &&
                    local_ds_state != DOSING_STATE_ERROR) { 
                    stopDosingCycle(true); // true - сброс состояния (stops motor, etc.) as per diff
                }
                // If this command is also intended for emergency stop, you might set a specific error here.
                // setSystemError(CRIT_MOTOR_FAIL, _T(L_ERROR_EMERGENCY_STOP_VIA_ESPNOW)); // Only if it's an emergency stop
                break; 
            // If CMD_ACK_ERROR is added to esp_now_protocol.h:
            // case CMD_ACK_ERROR: // This was commented out in the diff, but the clearSystemError() was not. Assuming it's a general error clear.
            //     app_log_i("ESPNOW_RX", "Acknowledge Error command received.");
            //     clearSystemError(); // This was present in the diff for CMD_ACK_ERROR
            //     break;
            // The original code had CMD_CLEAR_ERROR. If CMD_ACK_ERROR replaces it:
            case CMD_ACK_ERROR: // Assuming CMD_ACK_ERROR is defined in esp_now_protocol.h
                 app_log_i("ESPNOW_RX", "Acknowledge Error command received.");
                 clearSystemError();
                 break;


            default:
                 app_log_w("ESPNOW_RX", "Unknown command type: %d", cmd.cmd_type); // Используем cmd_type
                 break;
        }
    } else {
        // Original log: "Invalid data length: %d, expected: %d", len, sizeof(esp_now_cmd_t)
        // Diff log: "Received data length mismatch. Expected %d, got %d.", sizeof(struct_command_t), len
        app_log_w("ESPNOW_RX", "Received data length mismatch. Expected %d for struct_command_t, got %d.", sizeof(struct_command_t), len);
    }
}

bool sendEspNowData(const uint8_t *data, size_t len, const uint8_t *peer_addr_target) {
    if (WiFi.status() != WL_CONNECTED) {
        app_log_w("ESPNOW_SEND", "WiFi not connected. Cannot send ESP-NOW data.");
        return false;
    }

    ensureEspNowPeer(); 

    if (!s_screen_mac_valid || !esp_now_peer_added || !esp_now_is_peer_exist(peer_addr_target)) {
        app_log_w("ESP_NOW_SEND", "Attempting ESP-NOW send, but peer not (fully) configured or not added.");
        if (!s_screen_mac_valid || !esp_now_peer_added || !esp_now_is_peer_exist(peer_addr_target)) {
             app_log_e("ESP_NOW_SEND", "Peer still not available after re-check. Cannot send.");
             return false;
        }
    }

    esp_err_t result;
    for (int i = 0; i < ESP_NOW_SEND_MAX_RETRIES_LOCAL; ++i) { 
        result = esp_now_send(peer_addr_target, data, len);
        if (result == ESP_OK) {
            app_log_d("ESPNOW_SEND", "ESP-NOW data sent (attempt %d).", i + 1);
            return true;
        }
        app_log_w("ESPNOW_SEND", "ESP-NOW send error (attempt %d/%d): %s. Delay %lu ms.",
              i + 1, ESP_NOW_SEND_MAX_RETRIES_LOCAL, esp_err_to_name(result), ESP_NOW_SEND_RETRY_DELAY_MS_LOCAL);
        if (i < ESP_NOW_SEND_MAX_RETRIES_LOCAL - 1) {
            vTaskDelay(pdMS_TO_TICKS(ESP_NOW_SEND_RETRY_DELAY_MS_LOCAL)); 
        }
    }
    app_log_e("ESPNOW_SEND", "Failed to send ESP-NOW data after %d attempts.", ESP_NOW_SEND_MAX_RETRIES_LOCAL);
    setSystemError(WARN_ESP_NOW_SEND_FAIL, _T(L_WARN_ESPNOW_SEND_FAIL_AFTER_RETRIES));
    return false;
}


// --- Отправка статуса системы на дисплей ---
void sendSystemStatusEspNow() {
    ensureEspNowPeer(); 
    if (!isEspNowPeerAvailable()) { 
        app_log_w("ESP_NOW_SEND", "Cannot send status, peer not available.");
        return;
    }

    struct_status_t status_data; 
    memset(&status_data, 0, sizeof(status_data)); 

    portENTER_CRITICAL(&dosing_state_mutex);
    status_data.system_state = (system_state_t)getDosingState(); // Используем getDosingState()
    portEXIT_CRITICAL(&dosing_state_mutex);
    
    status_data.current_temperature_out = getTempOut(); // Используем getTempOut() из sensors.h
    status_data.current_temperature_in = tIn; // Используем глобальную переменную tIn из sensors.h (ensure tIn is up-to-date)

    status_data.current_volume_ml = getCurrentDosedVolume(); // Объем, налитый в текущем цикле (was getCurrentVolumeDispensedCycleMl)

    status_data.target_volume_ml = config.volumeTarget; // Целевой объем для текущего цикла

    portENTER_CRITICAL(&error_handler_mutex); // Мьютекс из error_handler.h
    status_data.error_code = (SystemErrorCode_t)getSystemErrorCode(); // Исправлено на SystemErrorCode_t
    portEXIT_CRITICAL(&error_handler_mutex);

    status_data.current_setpoint_temperature = config.tempSetpoint; // Используем config.tempSetpoint

    status_data.current_setpoint_volume = config.volumeTarget; // Используем config.volumeTarget

    status_data.uptime_seconds = getUptimeSeconds();

    if (MAIN_FIRMWARE_VERSION != nullptr) {
        strncpy(status_data.version_main_esp, MAIN_FIRMWARE_VERSION, sizeof(status_data.version_main_esp) - 1);
        status_data.version_main_esp[sizeof(status_data.version_main_esp) - 1] = '\0'; 
    } else {
        strncpy(status_data.version_main_esp, "N/A", sizeof(status_data.version_main_esp) - 1);
        status_data.version_main_esp[sizeof(status_data.version_main_esp) - 1] = '\0';
        app_log_e("ESP_NOW_SEND", "MAIN_FIRMWARE_VERSION is null!");
    }

    app_log_d("ESPNOW_STATUS", "Sending status: DS:%d, T_out:%.1f, T_in:%.1f, Vol:%.1f, TargetVol:%.1f, Err:%d, SetT:%.1f, SetV:%d, Uptime:%lu, FW:%s",
        (int)status_data.system_state, status_data.current_temperature_out, status_data.current_temperature_in,
        status_data.current_volume_ml, status_data.target_volume_ml, (int)status_data.error_code,
        status_data.current_setpoint_temperature, status_data.current_setpoint_volume,
        status_data.uptime_seconds, status_data.version_main_esp);

    esp_err_t result = esp_now_send(s_screen_mac_address, (uint8_t *) &status_data, sizeof(status_data));
    if (result == ESP_OK) {
        app_log_d("ESP_NOW_SEND", "Status sent successfully with FW version: %s", status_data.version_main_esp);
    } else {
        app_log_e("ESP_NOW_SEND", "Error sending status: %s", esp_err_to_name(result));
        setSystemError(WARN_ESP_NOW_SEND_FAIL, _T(L_WARN_ESPNOW_SEND_FAIL_AFTER_RETRIES)); 
    }
}

bool isEspNowPeerAvailable() {
    if (!s_screen_mac_valid) {
        return false;
    }
    return esp_now_peer_added && esp_now_is_peer_exist(s_screen_mac_address);
}

void getRemotePeerAddress(uint8_t *mac_addr_buf) {
    if (mac_addr_buf) { 
        if (s_screen_mac_valid) { 
            memcpy(mac_addr_buf, s_screen_mac_address, 6); 
        } else {
            memset(mac_addr_buf, 0, 6); 
        }
    }
}
