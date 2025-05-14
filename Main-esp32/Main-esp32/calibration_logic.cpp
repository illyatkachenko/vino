#include "calibration_logic.h"
#include "config_manager.h" // Для config.mlPerStep, config.flowMlPerPulse, saveConfig
#include "error_handler.h"  // Для setSystemError, current_system_error
#include "motor_control.h"  // Для motor_running_auto, stopMotor, manualMotorForward, stopManualMotor
#include "dosing_logic.h"   // Для current_dosing_state
#include "main.h"           // Для system_power_enabled и функций логирования
#include "localization.h"   // For _T()

// Определения глобальных переменных из calibration_logic.h
bool _calibrationMode = false;
portMUX_TYPE cal_mode_mutex = portMUX_INITIALIZER_UNLOCKED;
unsigned long calibration_start_time = 0;
bool calibration_stopped_by_timeout = false;

long steps_taken_calibration = 0;
portMUX_TYPE motor_cal_steps_mutex = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long flow_pulses_calibration = 0;
portMUX_TYPE cal_pulse_mutex = portMUX_INITIALIZER_UNLOCKED;

// Внешние переменные
// extern Config config; // Доступно через config_manager.h
// extern bool system_power_enabled; // Доступно через main.h
// extern bool motor_running_auto; // Из motor_control.h
// extern DosingState current_dosing_state; // Из dosing_logic.h
// extern SystemErrorCode current_system_error; // Из error_handler.h

// Функции логирования доступны через main.h
// extern void log_i(const char* tag, const char* format, ...);
// extern void log_w(const char* tag, const char* format, ...);
// extern void log_e(const char* tag, const char* format, ...);

const unsigned long CALIBRATION_MODE_TIMEOUT_MS = 5 * 60 * 1000; // 5 минут

void initCalibrationLogic() {
    _calibrationMode = false;
    calibration_stopped_by_timeout = false;
    steps_taken_calibration = 0;
    flow_pulses_calibration = 0;
}

void setCalibrationModeState(bool state) {
    portENTER_CRITICAL(&cal_mode_mutex);
    _calibrationMode = state;
    portEXIT_CRITICAL(&cal_mode_mutex);
}

bool getCalibrationModeState() {
    bool state;
    portENTER_CRITICAL(&cal_mode_mutex);
    state = _calibrationMode;
    portEXIT_CRITICAL(&cal_mode_mutex);
    return state;
}

void startCalibrationMode(float targetVolume, bool fromWeb) {
    // ... (реализация как в mainbuidv4.c, с использованием getCalibrationModeState, setCalibrationModeState) ...
    // Важно: setSystemError, system_power_enabled, motor_running_auto, current_dosing_state
    // будут доступны через .h файлы или extern.
    // manualMotorForward() будет вызываться из motor_control.h
    if (getCalibrationModeState()) {
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_CALIBRATION_ALREADY_ACTIVE));
        return;
    }
    if (!system_power_enabled) {
        setSystemError(LOGIC_ERROR, _T(L_ERROR_SYS_NOT_POWERED_CANNOT_CALIBRATE));
        return;
    }

    DosingState local_dosing_state_check;
    portENTER_CRITICAL(&dosing_state_mutex); // Мьютекс из dosing_logic.h
    local_dosing_state_check = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    if (motor_running_auto || (local_dosing_state_check != DOSING_STATE_IDLE && local_dosing_state_check != DOSING_STATE_FINISHED && local_dosing_state_check != DOSING_STATE_ERROR)) {
        setSystemError(LOGIC_ERROR, _T(L_ERROR_DOSING_CYCLE_ACTIVE_CANNOT_CALIBRATE));
        return;
    }

    // ... (другие проверки) ...
    log_i("CAL", "Starting calibration mode (target hint: %.1f ml). Motor control is manual.", targetVolume);
    setCalibrationModeState(true);
    calibration_start_time = millis();
    calibration_stopped_by_timeout = false;
    
    portENTER_CRITICAL(&motor_cal_steps_mutex);
    steps_taken_calibration = 0;
    portEXIT_CRITICAL(&motor_cal_steps_mutex);

    portENTER_CRITICAL(&cal_pulse_mutex);
    flow_pulses_calibration = 0; // Сбрасываем и счетчик импульсов
    portEXIT_CRITICAL(&cal_pulse_mutex);

    // Пользователь должен будет вручную управлять мотором (например, через кнопки или веб-интерфейс)
    // manualMotorForward(); // Пример, если нужно автоматически запустить
}

void stopCalibrationMode(float actualVolume, bool fromWeb) { // Для калибровки мотора (mlPerStep)
    // ... (реализация как в mainbuidv4.c) ...
    // Важно: getCalibrationModeState, setCalibrationModeState, steps_taken_calibration,
    // config.mlPerStep, saveConfig, setSystemError
    if (!getCalibrationModeState()) {
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_CALIBRATION_NOT_ACTIVE_MOTOR));
        return;
    }
    stopManualMotor(); // Останавливаем мотор, если он работал в ручном режиме

    long final_steps;
    portENTER_CRITICAL(&motor_cal_steps_mutex);
    final_steps = steps_taken_calibration;
    portEXIT_CRITICAL(&motor_cal_steps_mutex);

    log_i("CAL_MOTOR", "Motor Calibration Stopped. Actual Volume: %.1f ml, Steps: %ld", actualVolume, final_steps);
    if (final_steps > 0 && actualVolume > 0 && !isnan(actualVolume)) {
        config.mlPerStep = actualVolume / (float)final_steps;
        log_i("CAL_MOTOR", "New mlPerStep: %.6f", config.mlPerStep);
        saveConfig();
    } else {
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_INVALID_MOTOR_CALIBRATION_DATA));
    }
    setCalibrationModeState(false); // Выходим из режима калибровки здесь
}

void stopFlowCalibrationMode(float actualVolume, bool fromWeb) { // Для калибровки датчика потока (flowMlPerPulse)
    if (!getCalibrationModeState()) {
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_CALIBRATION_NOT_ACTIVE_FLOW));
        return;
    }
    stopManualMotor(); // Останавливаем мотор

    unsigned long final_pulses;
    portENTER_CRITICAL(&cal_pulse_mutex);
    final_pulses = flow_pulses_calibration;
    portEXIT_CRITICAL(&cal_pulse_mutex);

    log_i("CAL_FLOW", "Flow Calibration Stopped. Actual Volume: %.1f ml, Pulses: %lu", actualVolume, final_pulses);
    if (final_pulses > 0 && actualVolume > 0 && !isnan(actualVolume)) {
        config.flowMlPerPulse = actualVolume / (float)final_pulses;
        log_i("CAL_FLOW", "New flowMlPerPulse: %.6f", config.flowMlPerPulse);
        saveConfig();
    } else {
        setSystemError(CALIBRATION_ERROR, _T(L_ERROR_INVALID_FLOW_CALIBRATION_DATA));
    }
    setCalibrationModeState(false); // Выходим из режима калибровки здесь
}

void handleCalibrationLogic() {
    // ... (реализация таймаута, как в mainbuidv4.c) ...
    // Важно: getCalibrationModeState, setCalibrationModeState, stopMotor, setSystemError
    if (getCalibrationModeState()) {
        // Таймаут должен срабатывать всегда, если режим калибровки активен слишком долго,
        // независимо от того, запущен ли мотор вручную в данный момент.
        if (millis() - calibration_start_time > CALIBRATION_MODE_TIMEOUT_MS) {
            log_w("CAL", "Calibration mode timed out! Stopping motor and exiting calibration.");
            stopMotor(); // Общая остановка мотора
            setCalibrationModeState(false);
            calibration_stopped_by_timeout = true;
            // Используем getSystemErrorCode() для потокобезопасного чтения
            if (getSystemErrorCode() == NO_ERROR || getSystemErrorCode() == CALIBRATION_ERROR) {
                 setSystemError(CALIBRATION_ERROR, _T(L_ERROR_CALIBRATION_TIMED_OUT_MOTOR_STOPPED));
            }
        }
    }
}