#include "dosing_logic.h"
#include "config_manager.h" // Для config.volumeTarget, config.mlPerStep, config.tempSetpoint
#include "error_handler.h"  // Для setSystemError, current_system_error
#include "sensors.h"        // Для tOut, tOut_filtered, compressorOn, compressorOff
#include "motor_control.h"  // Для updateMotorSpeed, motor_dir, ENABLE_PIN, motor_running_auto, steps_target_dosing, steps_taken_dosing
#include "calibration_logic.h" // Для getCalibrationModeState
#include "pid_controller.h" // Для getIsPidTempControlEnabled() и других функций управления PID
#include "main.h"           // Для system_power_enabled и функций логирования
#include "localization.h"   // For _T()

// Определения глобальных переменных из dosing_logic.h
DosingState_t current_dosing_state = DOSING_STATE_IDLE;
unsigned long dosing_state_start_time = 0;
// Мьютекс для current_dosing_state и dosing_state_start_time
portMUX_TYPE dosing_state_mutex = portMUX_INITIALIZER_UNLOCKED;

volatile float volume_dispensed_cycle = 0; // Добавляем volatile в определение

// Внешние переменные
// extern Config config; // Доступно через config_manager.h
// extern bool system_power_enabled; // Доступно через main.h
// extern float tOut; // Доступно через sensors.h
// extern float tOut_filtered; // Доступно через sensors.h
// extern SystemErrorCode current_system_error; // Доступно через error_handler.h
// extern bool motor_running_auto; // Доступно через motor_control.h
// extern long steps_target_dosing; // Доступно через motor_control.h
// extern long steps_taken_dosing;  // Доступно через motor_control.h
// Для No-Flow Timeout из sensors.c
// extern bool checking_for_flow; // Доступно через sensors.h
// extern unsigned long motor_start_time_with_no_flow; // Доступно через sensors.h

// PID-контроллер управляется через функции из pid_controller.h
// extern bool getIsPidTempControlEnabled(); // Из pid_controller.h, уже включен

// Пины ENABLE_PIN и DIR_PIN управляются из motor_control.c
// Мьютекс для volume_dispensed_cycle
portMUX_TYPE volume_dispensed_mutex = portMUX_INITIALIZER_UNLOCKED;

void initDosingLogic() {
    portENTER_CRITICAL(&dosing_state_mutex);
    current_dosing_state = DOSING_STATE_IDLE;
    dosing_state_start_time = millis();
    portEXIT_CRITICAL(&dosing_state_mutex);

    portENTER_CRITICAL(&volume_dispensed_mutex);
    volume_dispensed_cycle = 0;
    portEXIT_CRITICAL(&volume_dispensed_mutex);
    log_i("DOSING", "Dosing Logic Initialized.");
}

void startDosingCycle(int volumeML, bool fromWeb) {
    // ... (реализация как в mainbuidv4.c) ...
    // Важно: setSystemError, getCalibrationModeState, system_power_enabled, config.mlPerStep
    // будут доступны через .h файлы или extern.    
    DosingState_t local_current_dosing_state;
    portENTER_CRITICAL(&dosing_state_mutex);
    local_current_dosing_state = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    if (getCalibrationModeState()) { // getCalibrationModeState() из calibration_logic.h
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_CAL_ACTIVE_CANNOT_DOSE));
        // if (fromWeb) server.send(400, "text/plain", "Calibration active"); // Веб-ответы остаются в web_server_handlers.c
        return;
    }
    if (!system_power_enabled) {
        setSystemError(LOGIC_ERROR, _T(L_ERROR_SYS_NOT_POWERED_CANNOT_DOSE));
        return;
    }
    if (local_current_dosing_state != DOSING_STATE_IDLE && local_current_dosing_state != DOSING_STATE_FINISHED && local_current_dosing_state != DOSING_STATE_ERROR) {
        setSystemError(LOGIC_ERROR, _T(L_ERROR_DOSING_CYCLE_BUSY_OR_ERROR));
        return;
    }
    // Для дозирования по объему через датчик потока, важна калибровка датчика потока.
    // Калибровка mlPerStep для мотора становится менее критичной для точности объема, но важна для скорости.
    if (config.flowMlPerPulse <= 0.000001f) { 
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_FLOW_SENSOR_NOT_CALIBRATED));
        return;
    }
    if (config.mlPerStep <= 0.000001f) { // Предупреждение, если скорость мотора важна
        log_w("DOSING", "Motor (mlPerStep) not calibrated. Dosing by flow sensor, but base speed control might be inaccurate.");
    }

    log_i("DOSING", "Starting dosing cycle request for %d ml. (FromWeb: %s)", volumeML, fromWeb ? "true" : "false");
    config.volumeTarget = volumeML;
    // steps_target_dosing = (long)((float)volumeML / config.mlPerStep); // Если контроль по шагам
    steps_taken_dosing = 0;
    portENTER_CRITICAL(&volume_dispensed_mutex);
    volume_dispensed_cycle = 0; // Сбрасываем объем по датчику потока
    portEXIT_CRITICAL(&volume_dispensed_mutex);
    log_dosing_state_change(DOSING_STATE_REQUESTED);
    clearSystemError(); // Сбрасываем предыдущие ошибки (если это нужно)
    // Ответ веб-серверу должен быть в вызывающей функции в main.c (handleStartDosing)
}

void stopDosingCycle(bool fromWeb) {
    log_i("DOSING", "Stop dosing cycle requested (fromWeb: %s)", fromWeb ? "true" : "false");
    DosingState_t local_current_dosing_state;

    if (motor_running_auto) {
        log_i("DOSING", "Motor was running auto, stopping motor.");
        stopMotor(); // Используем функцию из motor_control.h
        checking_for_flow = false; // Останавливаем проверку на отсутствие потока
    }

    portENTER_CRITICAL(&dosing_state_mutex);
    local_current_dosing_state = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    if (local_current_dosing_state != DOSING_STATE_IDLE && local_current_dosing_state != DOSING_STATE_FINISHED && local_current_dosing_state != DOSING_STATE_ERROR) {
        log_dosing_state_change(DOSING_STATE_STOPPING); // Переходим в состояние остановки
    }
}

// Вспомогательная функция для логирования смены состояния
void log_dosing_state_change(DosingState_t new_state) {
    DosingState_t old_state;

    portENTER_CRITICAL(&dosing_state_mutex);
    old_state = current_dosing_state;
    current_dosing_state = new_state;
    dosing_state_start_time = millis();
    portEXIT_CRITICAL(&dosing_state_mutex);

    log_i("DOSING_SM", "State change: %s -> %s", getDosingStateString(old_state), getDosingStateString(new_state));
}

void handleDosingState() {
    // ... (реализация конечного автомата, как в mainbuidv4.c) ...
    // Важно: tOut_filtered, config.tempSetpoint, compressorOn/Off, updateMotorSpeed,
    // motor_dir, ENABLE_PIN, motor_running_auto, volume_dispensed_cycle, config.volumeTarget,
    // setSystemError, config.totalDosingCycles, config.totalVolumeDispensed, saveConfig
    // будут доступны через .h файлы или extern.
    // PID-логика (pid_setpoint_temp и т.д.) также будет доступна.
    unsigned long c_ms = millis();
    float temp_to_check = (tOut_filtered == -127.0f) ? tOut : tOut_filtered; // Используем фильтрованную, если доступна
    const float temp_hysteresis_dosing = 1.0f; // Гистерезис для старта дозирования    
    const unsigned long PRECOOL_TIMEOUT_MS = 5 * 60 * 1000; // 5 минут на предохлаждение
    const unsigned long MAX_DOSING_DURATION_MS = 15 * 60 * 1000; // 15 минут максимальная длительность дозирования
    DosingState local_current_dosing_state; // Переименовано для ясности
    unsigned long local_dosing_state_start_time;

    portENTER_CRITICAL(&dosing_state_mutex);
    local_current_dosing_state = current_dosing_state;
    local_dosing_state_start_time = dosing_state_start_time; // Копируем под мьютексом
    portEXIT_CRITICAL(&dosing_state_mutex);

    switch (local_current_dosing_state) {
        case DOSING_STATE_IDLE: { // Добавлены скобки
            // Ничего не делаем, ожидаем запроса
            break;
        }
        case DOSING_STATE_REQUESTED: {
            if (!system_power_enabled) {
                log_w("DOSING_SM", "System not powered, cannot proceed from REQUESTED. Returning to IDLE.");
                setSystemError(LOGIC_ERROR, _T(L_ERROR_DOSING_REQUESTED_SYS_NOT_POWERED));
                log_dosing_state_change(DOSING_STATE_IDLE);
                break;
            }
            if (getSystemErrorCode() == CRIT_TEMP_SENSOR_OUT_FAIL) { // Используем getSystemErrorCode()
                log_e("DOSING_SM", "Output temp sensor failed, cannot proceed from REQUESTED. -> STOPPING (will lead to ERROR)");
                // setSystemError не нужен, он уже должен быть установлен
                log_dosing_state_change(DOSING_STATE_STOPPING); // STOPPING корректно обработает ошибку и перейдет в ERROR
            } else if (temp_to_check <= (config.tempSetpoint + temp_hysteresis_dosing) && temp_to_check != -127.0f) {
                log_i("DOSING_SM", "Temp OK (%.1fC <= %.1fC + %.1fC). -> STARTING", temp_to_check, config.tempSetpoint, temp_hysteresis_dosing);
                log_dosing_state_change(DOSING_STATE_STARTING);
                // Инициализация PID происходит в enablePidTempControl()
            } else { // temp_to_check > setpoint или датчик не готов (-127.0f)
                log_i("DOSING_SM", "Temp High (%.1fC > %.1fC) or sensor not ready. -> PRE_COOLING", temp_to_check, config.tempSetpoint);
                compressorOn();
                log_dosing_state_change(DOSING_STATE_PRE_COOLING);
            }
            break; // End of DOSING_STATE_REQUESTED // Добавлены скобки
        }
        case DOSING_STATE_PRE_COOLING: {
            if (!system_power_enabled) { log_dosing_state_change(DOSING_STATE_STOPPING); break; }
            if (getSystemErrorCode() == CRIT_TEMP_SENSOR_OUT_FAIL) { // Используем getSystemErrorCode()
                log_w("DOSING_SM", "Temp sensor failed during PRE_COOLING. -> STARTING (will likely fail)");
                log_dosing_state_change(DOSING_STATE_STARTING);
                break;
            }
            if (temp_to_check <= config.tempSetpoint && temp_to_check != -127.0f) {
                log_i("DOSING_SM", "Pre-cooling complete (%.1fC <= %.1fC). -> STARTING", temp_to_check, config.tempSetpoint);
                log_dosing_state_change(DOSING_STATE_STARTING); // Используем локальную копию
            } else if (c_ms - local_dosing_state_start_time > PRECOOL_TIMEOUT_MS) {
                log_e("DOSING_SM", "Pre-cooling timeout! Temp: %.1fC, Setpoint: %.1fC", temp_to_check, config.tempSetpoint);
                setSystemError(CRIT_PRECOOL_TIMEOUT, _T(L_ERROR_PRECOOLING_TIMEOUT));
                log_dosing_state_change(DOSING_STATE_STOPPING);
            } else {
                compressorOn(); // Убедимся, что компрессор включен
            }
            break; // End of DOSING_STATE_PRE_COOLING
        }
        case DOSING_STATE_STARTING: { // Добавлены скобки
            if (!system_power_enabled) { log_dosing_state_change(DOSING_STATE_STOPPING); break; }
            if (getSystemErrorCode() == CRIT_TEMP_SENSOR_OUT_FAIL) { // Используем getSystemErrorCode()
                 log_e("DOSING_SM", "Cannot start dosing, output temp sensor failed.");
                 setSystemError(CRIT_TEMP_SENSOR_OUT_FAIL, _T(L_ERROR_DOSING_START_ABORTED_TOUT_FAIL));
                 log_dosing_state_change(DOSING_STATE_ERROR);
                 break;
            }
            if (temp_to_check > (config.tempSetpoint + temp_hysteresis_dosing) && temp_to_check != -127.0f) {
                log_w("DOSING_SM", "Temp too high to start dosing (%.1fC > %.1fC). -> PRE_COOLING", temp_to_check, config.tempSetpoint);
                log_dosing_state_change(DOSING_STATE_PRE_COOLING);
                break;
            }

            log_i("DOSING_SM", "Starting motor for dosing. Target: %d ml. Speed: %d steps/s.", config.volumeTarget, config.motorSpeed);
            steps_taken_dosing = 0;
            portENTER_CRITICAL(&volume_dispensed_mutex);
            volume_dispensed_cycle = 0;
            portEXIT_CRITICAL(&volume_dispensed_mutex);
            updateMotorSpeed(config.motorSpeed);
            if (config.motorSpeed > 0) {
                 // ENABLE_PIN управляется в motor_control.c через handleMotorStepping
                 // или через явный вызов функции включения мотора, если бы она была.
                 // Сейчас motor_control.c сам включит ENABLE_PIN при handleMotorStepping, если step_interval_us != 0
                 motor_running_auto = true;
                 checking_for_flow = true; // Активируем проверку на отсутствие потока
                 motor_start_time_with_no_flow = millis(); // Запоминаем время старта для таймаута
            } else {
                log_w("DOSING_SM", "Motor speed is 0. Cannot start dosing. -> ERROR");
                setSystemError(LOGIC_ERROR, _T(L_ERROR_MOTOR_SPEED_ZERO_CANNOT_DOSE));
                log_dosing_state_change(DOSING_STATE_ERROR);
                break;
            }
            
            if (getIsPidTempControlEnabled()) {
                log_i("DOSING_SM", "PID Temperature Control is active for this dosing cycle.");
                // enablePidTempControl(true) уже должен был быть вызван извне (веб, ESP-NOW)
                // и он уже инициализирует PID (уставка, сброс интеграла/ошибки)
            }
            log_dosing_state_change(DOSING_STATE_RUNNING);
            break; // End of DOSING_STATE_STARTING // Добавлены скобки
        }
        case DOSING_STATE_RUNNING: {
            if (!system_power_enabled || !motor_running_auto) {
                log_w("DOSING_SM", "System off or motor stopped externally during RUNNING. -> STOPPING");
                log_dosing_state_change(DOSING_STATE_STOPPING);
                break;
            }
            SystemErrorCode err_code_running = getSystemErrorCode(); // Используем getSystemErrorCode()
            if (err_code_running != NO_ERROR && err_code_running != WARN_ESP_NOW_SEND_FAIL) {
                log_e("DOSING_SM", "System error %d occurred during RUNNING. -> STOPPING", err_code_running);
                log_dosing_state_change(DOSING_STATE_STOPPING);
                break;
            }

            float current_volume_dispensed_local;
            portENTER_CRITICAL(&volume_dispensed_mutex);
            current_volume_dispensed_local = volume_dispensed_cycle;
            portEXIT_CRITICAL(&volume_dispensed_mutex);

            if (current_volume_dispensed_local >= (float)config.volumeTarget) {
                log_i("DOSING_SM", "Target Volume Reached (Flow: %.2f ml / Target: %d ml). -> STOPPING", current_volume_dispensed_local, config.volumeTarget);
                log_dosing_state_change(DOSING_STATE_STOPPING);
            } else if (c_ms - local_dosing_state_start_time > MAX_DOSING_DURATION_MS) { // Используем локальную копию
                log_e("DOSING_SM", "Max dosing duration timeout! Dispensed: %.2f ml / Target: %d ml", current_volume_dispensed_local, config.volumeTarget);
                setSystemError(CRIT_DOSING_TIMEOUT, _T(L_ERROR_MAX_DOSING_DURATION_TIMEOUT));
                log_dosing_state_change(DOSING_STATE_STOPPING);
            }
            break; // End of DOSING_STATE_RUNNING // Добавлены скобки
        }
        case DOSING_STATE_STOPPING: {
            log_i("DOSING_SM", "Stopping motor and compressor (if running).");
            stopMotor();
            compressorOff();
            checking_for_flow = false; // Деактивируем проверку на отсутствие потока
            motor_running_auto = false;
            
            SystemErrorCode err_code_stopping = getSystemErrorCode(); // Используем getSystemErrorCode()
            if (err_code_stopping != NO_ERROR && err_code_stopping != WARN_ESP_NOW_SEND_FAIL &&
                err_code_stopping != CRIT_PRECOOL_TIMEOUT && err_code_stopping != CRIT_DOSING_TIMEOUT &&
                err_code_stopping != CRIT_FLOW_SENSOR_FAIL) { // Если ошибка не связана с таймаутами или датчиком потока
                log_dosing_state_change(DOSING_STATE_ERROR);
            } else {
                log_dosing_state_change(DOSING_STATE_FINISHED);
            }
            break; // End of DOSING_STATE_STOPPING // Добавлены скобки
        }
        case DOSING_STATE_FINISHED: {
            float final_volume_dispensed;
            portENTER_CRITICAL(&volume_dispensed_mutex);
            final_volume_dispensed = volume_dispensed_cycle;
            portEXIT_CRITICAL(&volume_dispensed_mutex);

            log_i("DOSING_SM", "Dosing cycle finished. Volume dispensed: %.2f ml. Steps: %ld.", final_volume_dispensed, steps_taken_dosing);
            config.totalDosingCycles++;
            config.totalVolumeDispensed += (unsigned long)round(final_volume_dispensed);
            saveConfig();
            log_dosing_state_change(DOSING_STATE_IDLE);
            break; // End of DOSING_STATE_FINISHED // Добавлены скобки
        }
        case DOSING_STATE_ERROR: {
            log_e("DOSING_SM", "Dosing cycle ended in ERROR state. Last system error: %d", getSystemErrorCode()); // Используем getSystemErrorCode()
            stopMotor();
            compressorOff();
            checking_for_flow = false; // Деактивируем проверку на отсутствие потока
            motor_running_auto = false;
            // Остаемся в ERROR до сброса ошибки или нового запроса
            break; // End of DOSING_STATE_ERROR // Добавлены скобки
        }
        default: {
            log_e("DOSING_SM", "Unknown dosing state: %d. Resetting to IDLE.", current_dosing_state);
            log_dosing_state_change(DOSING_STATE_IDLE);
            break;
        }
    }
}

DosingState_t getDosingState() {
    DosingState_t state;
    portENTER_CRITICAL(&dosing_state_mutex);
    state = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);
    return state;
}

float getCurrentDosedVolume() {
    float volume;
    portENTER_CRITICAL(&volume_dispensed_mutex);
    volume = volume_dispensed_cycle;
    portEXIT_CRITICAL(&volume_dispensed_mutex);
    return volume;
}

const char* getDosingStateString(DosingState_t state) {
    switch (state) {
        case DOSING_STATE_IDLE: return _T(L_DOSING_STATE_IDLE);
        case DOSING_STATE_REQUESTED: return _T(L_DOSING_STATE_REQUESTED);
        case DOSING_STATE_PRE_COOLING: return _T(L_DOSING_STATE_PRE_COOLING);
        case DOSING_STATE_STARTING: return _T(L_DOSING_STATE_STARTING);
        case DOSING_STATE_RUNNING: return _T(L_DOSING_STATE_RUNNING);
        case DOSING_STATE_STOPPING: return _T(L_DOSING_STATE_STOPPING);
        case DOSING_STATE_PAUSED: return _T(L_DOSING_STATE_PAUSED);
        case DOSING_STATE_FINISHED: return _T(L_DOSING_STATE_FINISHED);
        case DOSING_STATE_ERROR: return _T(L_DOSING_STATE_ERROR);
        default: return _T(L_DOSING_STATE_UNKNOWN);
    }
}