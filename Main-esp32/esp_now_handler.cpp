#include "esp_now_handler.h"
#include "config_manager.h" // Для config.remotePeerMacStr, config.tempSetpoint, config.volumeTarget, config.motorSpeed, saveConfig
#include <WiFi.h>           // Для WiFi.status() и WL_CONNECTED
#include "error_handler.h"  // Для setSystemError, clearSystemError, SystemErrorCode, getSystemErrorCode, getSystemErrorMessage
#include "dosing_logic.h"   // Для startDosingCycle, stopDosingCycle, getDosingState, getCurrentDosedVolume, dosing_state_mutex
#include "calibration_logic.h" // Для startCalibrationMode, stopCalibrationMode, getCalibrationModeState
#include "motor_control.h"  // Для updateMotorSpeed, stopMotor
#include "sensors.h"        // Для getTempOut, getFlowRate, isCompressorRunning, compressorOff, getWaterLevelSwitchStatus
#include "main.h"           // Для app_log_x функций и isSystemPowerEnabled()
#include "pid_controller.h" // For getIsPidTempControlEnabled, getPidSetpointTemp
#include "localization.h"   // For _T()

// Global variables for ESP-NOW, specific to this module
static uint8_t remotePeerAddress[6]; // Filled from config.remotePeerMacStr
esp_now_peer_info_t peerInfo;
bool esp_now_peer_added = false; // Definition of the global variable

// Constants for ESP-NOW

// Глобальные переменные для ESP-NOW, специфичные для этого модуля
const int ESP_NOW_SEND_MAX_RETRIES_LOCAL = 3; // Локальное имя, чтобы не конфликтовать с возможным define
const unsigned long ESP_NOW_SEND_RETRY_DELAY_MS_LOCAL = 200;
unsigned long last_peer_check_time = 0;
const unsigned long PEER_CHECK_INTERVAL_MS = 15000; // Проверять/передобавлять пир каждые 15 сек

uint8_t calculate_checksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        app_log_e("ESPNOW", "Error initializing ESP-NOW");
        setSystemError(ESP_NOW_INIT_ERROR, _T(L_ERROR_ESPNOW_INIT_FAIL));
        return;
    }
    esp_now_register_send_cb(onEspNowSend);
    // Используем reinterpret_cast для приведения типа функции, если компилятор строг
    // или убедимся, что сигнатура onEspNowReceive точно соответствует esp_now_recv_cb_t
    if (esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(onEspNowReceive)) != ESP_OK) {
        app_log_e("ESPNOW", "Error registering ESP-NOW receive callback");
        setSystemError(ESP_NOW_INIT_ERROR, _T(L_ERROR_ESPNOW_RECV_CB_REGISTER_FAIL));
    }

    ensureEspNowPeer(); // Первая попытка добавить пир
}

void ensureEspNowPeer() {
    // Если пир уже добавлен и существует, ничего не делаем
    if (esp_now_peer_added && esp_now_is_peer_exist(remotePeerAddress)) {
        return;
    }

    // Установка MAC-адреса пира из config
    if (sscanf(config.remotePeerMacStr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
               &remotePeerAddress[0], &remotePeerAddress[1], &remotePeerAddress[2], 
               &remotePeerAddress[3], &remotePeerAddress[4], &remotePeerAddress[5]) == 6) {
        memcpy(peerInfo.peer_addr, remotePeerAddress, 6);
        peerInfo.channel = 0; // 0 для автоматического выбора канала
        peerInfo.ifidx = WIFI_IF_STA;
        peerInfo.encrypt = false;

        // Сначала удаляем, если он вдруг был добавлен с ошибкой или устарел
        if (esp_now_is_peer_exist(remotePeerAddress)) {
            esp_now_del_peer(remotePeerAddress);
        }

        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            esp_now_peer_added = true;
            app_log_i("ESPNOW", "Peer %s added/re-added successfully.", config.remotePeerMacStr);
            if (getSystemErrorCode() == ESP_NOW_PEER_ERROR) clearSystemError();
            // Принудительно очищаем ошибку ESP_NOW_INIT_ERROR, если она была, т.к. пир добавлен
            if (getSystemErrorCode() == ESP_NOW_INIT_ERROR) clearSystemError();
        } else {
            esp_now_peer_added = false; // Явно указываем, что добавление не удалось
            app_log_e("ESPNOW", "Failed to add/re-add peer: %s", config.remotePeerMacStr);
            if (getSystemErrorCode() != ESP_NOW_PEER_ERROR) setSystemError(ESP_NOW_PEER_ERROR, _T(L_ERROR_ESPNOW_PEER_ADD_READD_FAIL));
        }
    } else {
        app_log_e("ESPNOW", "Failed to parse remotePeerMacStr from config: %s", config.remotePeerMacStr);
        setSystemError(PREFERENCES_ERROR, _T(L_ERROR_INVALID_PEER_MAC_IN_CONFIG));
    }
}

void onEspNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    app_log_d("ESPNOW_CB", "Send CB, status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
    if (status != ESP_NOW_SEND_SUCCESS) {
        // Ошибка отправки здесь логируется, но setSystemError вызывается в sendEspNowData после всех попыток.
    }
}

void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    esp_now_cmd_t cmd;
    if (len == sizeof(esp_now_cmd_t)) { // Проверяем размер полученных данных
        memcpy(&cmd, incomingData, sizeof(esp_now_cmd_t));

        uint8_t received_checksum = cmd.checksum;
        cmd.checksum = 0; // Обнуляем поле checksum в локальной копии для расчета
        uint8_t calculated_checksum = calculate_checksum((uint8_t*)&cmd, sizeof(cmd) - sizeof(cmd.checksum)); // Минус поле checksum

        if (received_checksum != calculated_checksum) {
            app_log_w("ESPNOW_RX", "Checksum mismatch! Rcv: %02X, Calc: %02X. Discarding.", received_checksum, calculated_checksum);
            return;
        }

        app_log_i("ESPNOW_RX", "Cmd: %d (Vol:%d, Temp:%.1f, Speed:%d, ActualVol:%.1f, CS_OK)", cmd.command, cmd.volume, cmd.temp, cmd.speed, cmd.actualVolume);

        switch (cmd.command) {
            case CMD_REQ_STATUS: // Запрос статуса (можно инициировать отправку статуса)
                app_log_i("ESPNOW_RX", "Status request received. Sending current status.");
                sendSystemStatusEspNow();
                break;
            case 1: // Старт дозирования
                if (cmd.volume > 0) {
                    app_log_i("ESPNOW_RX", "Start Dosing command: %d ml", cmd.volume);
                    startDosingCycle(cmd.volume, false); // false - не из веб
                } else {
                    app_log_w("ESPNOW_RX", "Invalid volume for Start Dosing: %d", cmd.volume);
                }
                break;
            case CMD_STOP_DOSING: // Стоп дозирования
                app_log_i("ESPNOW_RX", "Stop Dosing command");
                stopDosingCycle(false); // false - не из веб
                break;
            case CMD_SET_TEMP_SETPOINT: // Обновить уставку температуры
                if (!isnan(cmd.temp) && cmd.temp >= -10.0f && cmd.temp <= 30.0f) { // Добавлена проверка на NaN
                    app_log_i("ESPNOW_RX", "Set Temp Setpoint command: %.1f C", cmd.temp);
                    config.tempSetpoint = cmd.temp;
                    saveConfig(); // Сохраняем изменение
                    // Если PID активен, он подхватит новую уставку
                } else {
                    app_log_w("ESPNOW_RX", "Invalid temp for Setpoint: %.1f", cmd.temp);
                }
                break;
            case CMD_SET_MOTOR_SPEED: // Обновить скорость мотора
                if (cmd.speed >= 0 && cmd.speed <= 2000) { 
                    app_log_i("ESPNOW_RX", "Set Motor Speed command: %d steps/sec", cmd.speed);
                    updateMotorSpeed(cmd.speed); // Эта функция обновит config.motorSpeed
                    saveConfig(); // Сохраняем изменение
                } else {
                    app_log_w("ESPNOW_RX", "Invalid speed for Motor: %d", cmd.speed);
                }
                break;
            case CMD_START_MOTOR_CAL: // Старт калибровки (ml/step мотора)
                app_log_i("ESPNOW_RX", "Start Motor Calibration command (target hint: %.1f ml)", cmd.actualVolume);
                startCalibrationMode(cmd.actualVolume > 0 ? cmd.actualVolume : 100.0f, false);
                break;
            case CMD_STOP_MOTOR_CAL: // Стоп калибровки (ml/step мотора)
                if (!isnan(cmd.actualVolume) && cmd.actualVolume > 0) { // Добавлена проверка на NaN
                    app_log_i("ESPNOW_RX", "Stop Motor Calibration command, actual vol: %.1f ml", cmd.actualVolume);
                    stopCalibrationMode(cmd.actualVolume, false);
                } else {
                     app_log_w("ESPNOW_RX", "Invalid actual volume for Stop Motor Calibration: %.1f", cmd.actualVolume);
                }
                break;
            // TODO: Добавить команды для калибровки датчика потока, если нужно
            // case 7: // Старт калибровки датчика потока
            // case 8: // Стоп калибровки датчика потока
            case CMD_EMERGENCY_STOP: // Экстренная остановка
                app_log_w("ESPNOW_RX", "EMERGENCY STOP command received!");
                stopMotor();
                if(isCompressorRunning()) compressorOff(); // Check before turning off
                if (getCalibrationModeState()) {
                    setCalibrationModeState(false); // Эта функция управляет своим мьютексом
                }

                DosingState_t local_ds_state;
                portENTER_CRITICAL(&dosing_state_mutex); // Защищаем чтение current_dosing_state
                local_ds_state = getDosingState(); // Use getter if current_dosing_state is not extern
                portEXIT_CRITICAL(&dosing_state_mutex);

                if (local_ds_state != DOSING_STATE_IDLE &&
                    local_ds_state != DOSING_STATE_FINISHED &&
                    local_ds_state != DOSING_STATE_ERROR) { // Используем локальную переменную
                    stopDosingCycle(false); // Эта функция управляет изменением состояния и мьютексом
                }
                setSystemError(CRIT_MOTOR_FAIL, _T(L_ERROR_EMERGENCY_STOP_VIA_ESPNOW));
                break;
            case CMD_CLEAR_ERROR: // Сброс ошибки
                app_log_i("ESPNOW_RX", "Clear Error command received.");
                clearSystemError();
                break;

            // default: // Удалено, чтобы избежать ошибки "jump to case label" если компилятор строг
            //     app_log_w("ESPNOW_RX", "Unknown command: %d", cmd.command);
            //     break;
        }
    } else {
        app_log_w("ESPNOW_RX", "Invalid data length: %d, expected: %d", len, sizeof(esp_now_cmd_t));
    }
}

bool sendEspNowData(const uint8_t *data, size_t len, const uint8_t *peer_addr_target) {
    if (WiFi.status() != WL_CONNECTED) {
        app_log_w("ESPNOW_SEND", "WiFi not connected. Cannot send ESP-NOW data.");
        return false;
    }

    ensureEspNowPeer(); // Убедимся, что пир добавлен

    if (!esp_now_peer_added) {
        app_log_w("ESPNOW_SEND", "Attempting ESP-NOW send, but peer not added.");
        return false;
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
            vTaskDelay(pdMS_TO_TICKS(ESP_NOW_SEND_RETRY_DELAY_MS_LOCAL)); // Исправлен макрос
        }
    }
    app_log_e("ESPNOW_SEND", "Failed to send ESP-NOW data after %d attempts.", ESP_NOW_SEND_MAX_RETRIES_LOCAL);
    setSystemError(WARN_ESP_NOW_SEND_FAIL, _T(L_WARN_ESPNOW_SEND_FAIL_AFTER_RETRIES));
    return false;
}

void sendSystemStatusEspNow() {
    if (WiFi.status() != WL_CONNECTED) {
        // app_log_d("ESPNOW_STATUS", "WiFi not connected. Skipping status send."); // Можно логировать, если часто
        return;
    }
    // ensureEspNowPeer(); // Проверка пира будет в sendEspNowData
    esp_now_status_t s_msg;

    // Dosing State
    portENTER_CRITICAL(&dosing_state_mutex);
    s_msg.status_dosing_state = (uint8_t)getDosingState(); // Use getter
    portEXIT_CRITICAL(&dosing_state_mutex);

    // Sensor Data
    s_msg.tempOut = getTempOut();
    s_msg.flowRate = getFlowRate();
    s_msg.volumeDispensedCycle_ml = (int)round(getCurrentDosedVolume()); // Use getter

    // Error State
    portENTER_CRITICAL(&error_handler_mutex); // Мьютекс из error_handler.c
    s_msg.errorCode = (uint8_t)getSystemErrorCode(); // Use getter
    strncpy(s_msg.lastErrorMessage, getSystemErrorMessage(), MAX_ESP_NOW_ERROR_MSG_LEN); // Используем геттер
    s_msg.lastErrorMessage[MAX_ESP_NOW_ERROR_MSG_LEN -1] = '\0'; // Ensure null termination
    portEXIT_CRITICAL(&error_handler_mutex);

    // System States
    s_msg.systemPower = isSystemPowerEnabled(); // Use getter
    s_msg.compressorState = isCompressorRunning(); // Use getter
    s_msg.calibrationActive = getCalibrationModeState(); // Эта функция управляет своим мьютексом
    s_msg.wifiConnected = (WiFi.status() == WL_CONNECTED);
    s_msg.waterLevelOk = getWaterLevelSwitchStatus(); // Используем геттер
    s_msg.reserved = 0;

    // Чтение config.* предполагается безопасным для статусного сообщения,
    // т.к. изменения конфигурации обычно не происходят с высокой частотой одновременно с отправкой статуса.
    // If PID is active, send its setpoint, otherwise the general config setpoint
    if (getIsPidTempControlEnabled()) {
        s_msg.currentTempSetpoint = getPidSetpointTemp(); // Используем геттер
    } else {
        s_msg.currentTempSetpoint = config.tempSetpoint; // Or a dedicated user setpoint if PID is off
    }
    s_msg.currentVolumeTarget = config.volumeTarget; // This is likely target_dosing_volume_ml from dosing_logic

    s_msg.checksum = 0; // Обнуляем для расчета
    s_msg.checksum = calculate_checksum((uint8_t*)&s_msg, sizeof(s_msg) - sizeof(s_msg.checksum));

    app_log_d("ESPNOW_STATUS", "Sending status: DS:%d, T:%.1f, FR:%.1f, VD:%.0f, Err:%d (%s), Pwr:%d, Comp:%d, Cal:%d, SetT:%.1f, SetV:%d, CS:%02X",
        s_msg.status_dosing_state, s_msg.tempOut, s_msg.flowRate, s_msg.volumeDispensedCycle_ml, s_msg.errorCode,
        s_msg.lastErrorMessage, s_msg.systemPower, s_msg.compressorState, s_msg.calibrationActive, s_msg.currentTempSetpoint, s_msg.currentVolumeTarget, s_msg.checksum);
    sendEspNowData((uint8_t*)&s_msg, sizeof(s_msg), remotePeerAddress);
}

bool isEspNowPeerAvailable() {
    return esp_now_peer_added && esp_now_is_peer_exist(remotePeerAddress);
}

void getRemotePeerAddress(uint8_t *mac_addr_buf) {
    if (mac_addr_buf) {
        memcpy(mac_addr_buf, remotePeerAddress, sizeof(remotePeerAddress));
    }
}