#include "config_manager.h"
#include <Preferences.h>
#include "error_handler.h" // Для setSystemError, last_error_msg_buffer_internal
#include "main.h"          // Для log_x, system_power_enabled
#include "localization.h"  // For _T()
#include "motor_control.h" // Для updateMotorSpeed()
#include <stdio.h>         // Для sscanf
#include <string.h>        // Для strncpy, strcmp, strlen
#include "pid_controller.h" // Для setPidCoefficients
// Внешние переменные теперь доступны через соответствующие .h файлы
// extern String last_error_msg; // Доступно через error_handler.h
// extern bool system_power_enabled; // Доступно через main.h (предполагается)
// extern void updateMotorSpeed(int speed); // Доступно через motor_control.h
// extern uint8_t remotePeerAddress[]; // Это было некорректно, remotePeerAddress - static в esp_now_handler.c

// Определяем глобальную переменную config
Config config;

// Функции логирования доступны через main.h (который должен быть включен, если они здесь используются)
// Если main.h не включен, а функции логирования нужны, то #include "main.h"

void saveConfig() {
    log_i("PREFS", "Saving config...");
    Preferences preferences;
    // if (!preferences.begin(CONFIG_NAMESPACE, false)) { // Пример, как НЕ НАДО делать, если setSystemError вызывает saveConfig
    //     log_e("PREFS", "Error opening preferences for writing.");
    //     setSystemError(PREFERENCES_ERROR, "Failed to open Prefs for write"); 
    //     return;
    // }
    // Безопаснее:
     if (!preferences.begin(CONFIG_NAMESPACE, false)) {
         log_e("PREFS", "CRITICAL: Error opening preferences for writing. Config NOT saved.");
         // НЕ вызываем setSystemError отсюда, чтобы избежать рекурсии,
         // так как setSystemError сама вызывает saveConfig.
         // Ошибка будет залогирована, но система продолжит работу с текущей конфигурацией в памяти.
         return;
     }
    preferences.putFloat("tempSet", config.tempSetpoint);
    preferences.putInt("volTarget", config.volumeTarget);
    preferences.putInt("motorSpd", config.motorSpeed);
    preferences.putFloat("mlPerStep", config.mlPerStep);
    preferences.putFloat("flowMlPP", config.flowMlPerPulse);
    preferences.putString("peerMAC", config.remotePeerMacStr);
    preferences.putUChar("wifiChan", config.wifiChannel); // Сохраняем канал WiFi
    preferences.putULong("totalVol", config.totalVolumeDispensed);
    preferences.putULong("compTime", config.compressorRunTime);
    preferences.putInt("compStarts", config.compressorStartCount);
    preferences.putInt("totalCycles", config.totalDosingCycles);
    preferences.putInt("errCount", config.errorCount);
    preferences.putString("lastErrStr", last_error_msg_buffer_internal); // Используем буфер из error_handler
    preferences.putFloat("pidKp", config.pidKp); 
    preferences.putFloat("pidKi", config.pidKi);
    preferences.putFloat("pidKd", config.pidKd);
    preferences.putBool("sysPower", config.systemPowerStateSaved);
    preferences.putString("curr_lang", config.currentLanguage); // Сохраняем язык
    preferences.end();
    log_i("PREFS", "Config saved.");
}

void loadConfig() {
    log_i("PREFS", "Loading config...");
    bool defaults_applied_this_load = false;
    Preferences preferences;

    if (!preferences.begin(CONFIG_NAMESPACE, true)) {
        log_w("PREFS", "Error opening preferences. Using ALL defaults.");
        defaults_applied_this_load = true;
        // ... (установка всех полей config значениями по умолчанию) ...
        config.tempSetpoint = 4.0f;
        config.volumeTarget = 100;
        // ... и т.д.
        config.motorSpeed = 100;
        config.flowMlPerPulse = 0.2f;
        strncpy(config.remotePeerMacStr, "N/A", sizeof(config.remotePeerMacStr)-1);
        config.remotePeerMacStr[sizeof(config.remotePeerMacStr)-1] = '\0'; // Добавлено для безопасности
        config.remotePeerMacStr[sizeof(config.remotePeerMacStr)-1] = '\0';
        config.pidKp = 20.0f; // Default Kp
        config.pidKi = 0.5f;  // Default Ki
        strncpy(last_error_msg_buffer_internal, _T(L_ERROR_PREFS_DEFAULTS_APPLIED_OPEN_FAIL), sizeof(last_error_msg_buffer_internal) -1);
        last_error_msg_buffer_internal[sizeof(last_error_msg_buffer_internal)-1] = '\0';

        strncpy(config.currentLanguage, DEFAULT_LANGUAGE, sizeof(config.currentLanguage) - 1); // Устанавливаем язык по умолчанию
        config.currentLanguage[sizeof(config.currentLanguage)-1] = '\0';
        config.pidKd = 5.0f;  // Default Kd
        config.wifiChannel = 1; // Канал WiFi по умолчанию
        
    } else {
        config.tempSetpoint = preferences.getFloat("tempSet", 4.0f);
        config.volumeTarget = preferences.getInt("volTarget", 100); // <--- ИСПРАВЛЕНО
        config.motorSpeed = preferences.getInt("motorSpd", 100);     // <--- ИСПРАВЛЕНО
        config.mlPerStep = preferences.getFloat("mlPerStep", 0.01f); // Пример значения по умолчанию
        config.flowMlPerPulse = preferences.getFloat("flowMlPP", 0.2f);
        String mac_str_loaded = preferences.getString("peerMAC", "N/A");
        strncpy(config.remotePeerMacStr, mac_str_loaded.c_str(), sizeof(config.remotePeerMacStr)-1);
        config.pidKp = preferences.getFloat("pidKp", 20.0f); 
        config.pidKi = preferences.getFloat("pidKi", 0.5f);
        config.pidKd = preferences.getFloat("pidKd", 5.0f);
        config.remotePeerMacStr[sizeof(config.remotePeerMacStr)-1] = '\0';
        
        String loaded_err_str = preferences.getString("lastErrStr", "None");
        strncpy(last_error_msg_buffer_internal, loaded_err_str.c_str(), sizeof(last_error_msg_buffer_internal) -1);
        last_error_msg_buffer_internal[sizeof(last_error_msg_buffer_internal)-1] = '\0';

        config.totalVolumeDispensed = preferences.getULong("totalVol", 0);
        config.compressorRunTime = preferences.getULong("compTime", 0);
        config.compressorStartCount = preferences.getInt("compStarts", 0);
        config.totalDosingCycles = preferences.getInt("totalCycles", 0);
        config.errorCount = preferences.getInt("errCount", 0);

        config.systemPowerStateSaved = preferences.getBool("sysPower", false);
        
        String loaded_lang_str = preferences.getString("curr_lang", DEFAULT_LANGUAGE); // Загружаем язык
        strncpy(config.currentLanguage, loaded_lang_str.c_str(), sizeof(config.currentLanguage) - 1);
        config.currentLanguage[sizeof(config.currentLanguage) - 1] = '\0';
        preferences.end();

        // ... (валидация загруженных значений и применение defaults_applied_this_load = true при необходимости) ...
        if (isnan(config.flowMlPerPulse) || config.flowMlPerPulse <= 0.000001f || config.flowMlPerPulse > 10.0f) {
            log_w("PREFS", "Invalid flowMlPerPulse loaded (%.6f). Setting default: 0.2", config.flowMlPerPulse);
            config.flowMlPerPulse = 0.2f;
            defaults_applied_this_load = true;
        }
        if (isnan(config.mlPerStep) || config.mlPerStep <= 0.0000001f || config.mlPerStep > 1.0f) {
            log_w("PREFS", "Invalid mlPerStep loaded (%.6f). Setting default: 0.01", config.mlPerStep);
            config.mlPerStep = 0.01f; // Пример значения по умолчанию
            defaults_applied_this_load = true;
        }
        // Добавьте другие проверки валидности для загруженных значений, если необходимо
        if (config.motorSpeed < 0 || config.motorSpeed > 2000) {
            log_w("PREFS", "Invalid motorSpeed loaded (%d). Setting default: 100", config.motorSpeed);
            config.motorSpeed = 100;
            defaults_applied_this_load = true;
        }
        if (isnan(config.tempSetpoint) || config.tempSetpoint < -10.0f || config.tempSetpoint > 30.0f) {
            log_w("PREFS", "Invalid tempSetpoint loaded (%.1f). Setting default: 4.0", config.tempSetpoint);
            config.tempSetpoint = 4.0f;
            defaults_applied_this_load = true;
        }
        if (isnan(config.pidKp) || isnan(config.pidKi) || isnan(config.pidKd) || config.pidKp < 0 || config.pidKi < 0 || config.pidKd < 0) { // Примерная проверка
            log_w("PREFS", "Invalid PID coefficients loaded (Kp:%.2f, Ki:%.2f, Kd:%.2f). Setting defaults.", config.pidKp, config.pidKi, config.pidKd);
            config.pidKp = 20.0f; config.pidKi = 0.5f; config.pidKd = 5.0f;
            defaults_applied_this_load = true;
        }
        if (strcmp(config.currentLanguage, "ru") != 0 && strcmp(config.currentLanguage, "en") != 0) {
            log_w("PREFS", "Invalid currentLanguage loaded ('%s'). Setting default: '%s'", config.currentLanguage, DEFAULT_LANGUAGE);
            strncpy(config.currentLanguage, DEFAULT_LANGUAGE, sizeof(config.currentLanguage) - 1);
            config.currentLanguage[sizeof(config.currentLanguage) - 1] = '\0';
            defaults_applied_this_load = true;
        }
        if (config.wifiChannel == 0 || config.wifiChannel > 13) { // Каналы WiFi обычно 1-13
            log_w("PREFS", "Invalid wifiChannel loaded (%u). Setting default: 1", config.wifiChannel);
            config.wifiChannel = 1;
        }
    }

    strncpy(config.lastErrorMsgBuffer, last_error_msg_buffer_internal, sizeof(config.lastErrorMsgBuffer)-1);
    config.lastErrorMsgBuffer[sizeof(config.lastErrorMsgBuffer)-1] = '\0';

    if (defaults_applied_this_load) {
        log_i("PREFS", "Defaults were applied. Saving current (corrected) config.");
        saveConfig(); // Сохраняем, если были применены значения по умолчанию
    }

    system_power_enabled = config.systemPowerStateSaved; // Обновляем глобальную system_power_enabled
    // digitalWrite(MOSFET_POWER_PIN, system_power_enabled ? HIGH : LOW); // Это должно быть в main.c или где есть доступ к MOSFET_POWER_PIN
    updateMotorSpeed(config.motorSpeed);
    // Обновляем коэффициенты в PID контроллере значениями из конфигурации
    setPidCoefficients(config.pidKp, config.pidKi, config.pidKd); 

    // Парсинг MAC-адреса пира и обновление remotePeerAddress теперь происходит внутри esp_now_handler.c
    // при вызове initEspNow() или ensureEspNowPeer(), которые используют config.remotePeerMacStr.
    log_i("PREFS", "Config loaded. System power state: %s", system_power_enabled ? "ON" : "OFF");
    // ... (логирование остальных загруженных параметров) ...
}

void performFactoryReset() {
    log_w("PREFS", "Performing factory reset...");
    Preferences preferences;
    if (preferences.begin(CONFIG_NAMESPACE, false)) {
        preferences.clear();
        preferences.end();
        log_i("PREFS", "Main config namespace '%s' cleared.", CONFIG_NAMESPACE);
    } else {
        log_e("PREFS", "Failed to open preferences for clearing main config.");
    }
    // Здесь можно добавить очистку других пространств имен Preferences, если они есть,
    // например, лог ошибок, если он хранится отдельно и его тоже нужно сбрасывать.
    // Preferences error_log_prefs;
    // if (error_log_prefs.begin(PREFS_ERROR_LOG_NAMESPACE, false)) { ... }
}

void resetStats() {
    log_w("STATS", "Resetting statistics...");
    config.totalVolumeDispensed = 0;
    config.compressorRunTime = 0;
    config.compressorStartCount = 0;
    config.totalDosingCycles = 0;
    config.errorCount = 0; // Также сбрасываем общий счетчик ошибок
    strncpy(config.lastErrorMsgBuffer, _T(L_INFO_STATISTICS_RESET_MSG), sizeof(config.lastErrorMsgBuffer) - 1);
    config.lastErrorMsgBuffer[sizeof(config.lastErrorMsgBuffer) - 1] = '\0'; // Обновляем и глобальный буфер в error_handler
    strncpy(last_error_msg_buffer_internal, _T(L_INFO_STATISTICS_RESET_MSG), sizeof(last_error_msg_buffer_internal) -1);
    last_error_msg_buffer_internal[sizeof(last_error_msg_buffer_internal)-1] = '\0';

    saveConfig(); // Сохраняем изменения
    log_i("STATS", "Statistics have been reset.");
}

bool getScreenMacAddress(uint8_t* mac_addr_buf) {
    if (mac_addr_buf == nullptr) {
        app_log_e("CONFIG_MAC", "Null buffer passed to getScreenMacAddress");
        return false;
    }
    // config.remotePeerMacStr должен быть уже загружен функцией loadConfig()
    // и содержать строку вида "XX:XX:XX:XX:XX:XX"
    int matched = sscanf(config.remotePeerMacStr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                         &mac_addr_buf[0], &mac_addr_buf[1], &mac_addr_buf[2],
                         &mac_addr_buf[3], &mac_addr_buf[4], &mac_addr_buf[5]);
    if (matched == 6) {
        // Дополнительная проверка на "пустой" или невалидный MAC, если строка была "N/A" или пустая
        if (strcmp(config.remotePeerMacStr, "N/A") == 0 || strlen(config.remotePeerMacStr) < 17) {
             app_log_w("CONFIG_MAC", "Screen MAC address is 'N/A' or too short in config: '%s'", config.remotePeerMacStr);
             memset(mac_addr_buf, 0, 6); // Заполняем нулями, чтобы было понятно, что MAC невалиден
             return false;
        }
        // Можно добавить проверку на нулевой MAC, если "00:00:00:00:00:00" считается невалидным
        app_log_i("CONFIG_MAC", "Screen MAC successfully parsed: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr_buf[0], mac_addr_buf[1], mac_addr_buf[2], mac_addr_buf[3], mac_addr_buf[4], mac_addr_buf[5]);
        return true;
    }
    app_log_w("CONFIG_MAC", "Failed to parse screen MAC address from config string: '%s'", config.remotePeerMacStr);
    memset(mac_addr_buf, 0, 6); // Заполняем нулями при ошибке парсинга
    return false;
}