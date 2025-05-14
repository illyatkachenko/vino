#include "motor_control.h"
#include "config_manager.h" // Для config.motorSpeed, config.mlPerStep
#include "error_handler.h"  // Для setSystemError
#include "pid_controller.h" // Для getIsPidTempControlEnabled()
#include "calibration_logic.h" // Для getCalibrationModeState, steps_taken_calibration
#include "main.h"           // Для system_power_enabled и функций логирования
#include "hardware_pins.h"  // <-- ДОБАВЛЕНО: Единый файл с определениями пинов

// Пины DIR_PIN, STEP_PIN, ENABLE_PIN теперь определены в hardware_pins.h
// #define DIR_PIN             16
// #define STEP_PIN            17
// #define ENABLE_PIN          4

// Определения глобальных переменных из motor_control.h
bool motor_running_auto = false;
bool motor_running_manual = false;
bool motor_dir = HIGH; // HIGH - forward
float current_steps_per_sec = 0;
unsigned long step_interval_us = 0;
long steps_taken_dosing = 0;
long steps_target_dosing = 0;

// Мьютекс для защиты состояния мотора (флаги, скорость, интервал)
portMUX_TYPE motor_state_mutex = portMUX_INITIALIZER_UNLOCKED;

// Внешние переменные
// extern Config config; // Доступно через config_manager.h
// extern bool system_power_enabled; // Доступно через main.h
// extern float volume_dispensed_cycle; // Доступно через dosing_logic.h (если нужно здесь)
// extern portMUX_TYPE motor_cal_steps_mutex; // Из calibration_logic.h
// extern long steps_taken_calibration;      // Из calibration_logic.h
// extern bool pid_temp_control_enabled;     // Заменено на getIsPidTempControlEnabled()

// Функции логирования доступны через main.h
// extern void log_i(const char* tag, const char* format, ...);
// extern void log_w(const char* tag, const char* format, ...);
// extern void log_d(const char* tag, const char* format, ...);

unsigned long last_step_time = 0;

void initMotor() {
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); // Мотор выключен по умолчанию
    log_i("MOTOR", "Motor pins initialized. ENABLED (HIGH).");
}

void updateMotorSpeed(int speedSetting) {
    // ... (реализация как в mainbuidv4.c, с учетом pid_temp_control_enabled) ...
    if (speedSetting < 0) {
        log_w("MOTOR", "Invalid speed setting: %d. Setting to 0.", speedSetting);
        speedSetting = 0;
    }
    config.motorSpeed = speedSetting; // Сохраняем запрошенную пользователем скорость

    portENTER_CRITICAL(&motor_state_mutex);
    if (!getIsPidTempControlEnabled() || speedSetting == 0) {
        current_steps_per_sec = (float)config.motorSpeed;
        if (current_steps_per_sec == 0) {
            step_interval_us = 0;
            // log_i("MOTOR", "Motor speed set to 0 (PID off or explicit stop)."); // Может быть избыточным
        } else {
            step_interval_us = (unsigned long)(1000000.0f / current_steps_per_sec);
        }
        log_i("MOTOR", "Speed (PID off or explicit 0) updated to: %.1f steps/sec, interval: %lu us", current_steps_per_sec, step_interval_us);
    } else {
        log_i("MOTOR", "PID is active. Base speed set to %d. PID will control actual steps/sec.", config.motorSpeed);
        // PID будет управлять current_steps_per_sec и step_interval_us напрямую
    }
    portEXIT_CRITICAL(&motor_state_mutex);
}

void updateMotorSpeedFromPid(float steps_sec) {
    portENTER_CRITICAL(&motor_state_mutex);
    current_steps_per_sec = steps_sec;
    if (current_steps_per_sec < 0.01f) { // Считаем 0, если очень мало
        current_steps_per_sec = 0;
        step_interval_us = 0; // Мотор остановлен
    } else {
        step_interval_us = (unsigned long)(1000000.0f / current_steps_per_sec);
    }
    portEXIT_CRITICAL(&motor_state_mutex);
    // Эта функция напрямую меняет current_steps_per_sec и step_interval_us
}

void handleMotorStepping() {
    // ... (реализация как в mainbuidv4.c) ...
    // volume_dispensed_cycle теперь обновляется в handleFlowSensor()
    // steps_taken_dosing все еще считаем для информации или резерва
    bool local_motor_running_auto;
    bool local_motor_running_manual;
    unsigned long local_step_interval_us;

    portENTER_CRITICAL(&motor_state_mutex);
    local_motor_running_auto = motor_running_auto;
    local_motor_running_manual = motor_running_manual;
    local_step_interval_us = step_interval_us;
    portEXIT_CRITICAL(&motor_state_mutex);

    // getCalibrationModeState() использует свой мьютекс
    if (!local_motor_running_auto && !local_motor_running_manual && !getCalibrationModeState()) {
        if (digitalRead(ENABLE_PIN) == LOW) { // Если мотор был включен, но не должен работать
            digitalWrite(ENABLE_PIN, HIGH);
            log_d("MOTOR", "Motor disabled (no active mode).");
        }
        return;
    }

    if (local_step_interval_us == 0) { // Скорость 0, мотор не должен шагать
        if (digitalRead(ENABLE_PIN) == LOW) digitalWrite(ENABLE_PIN, HIGH);
        return;
    }

    unsigned long current_micros = micros();
    if (current_micros - last_step_time >= local_step_interval_us) {
        last_step_time = current_micros;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(2); // Короткий импульс для шага
        digitalWrite(STEP_PIN, LOW);

        // Читаем motor_running_auto и motor_running_manual еще раз, если они могли измениться
        // другим потоком между первой проверкой и этим местом.
        // Однако, для простоты и учитывая, что handleMotorStepping вызывается часто,
        // можно использовать ранее прочитанные локальные значения, если вероятность изменения мала.
        // Для большей строгости, можно снова прочитать под мьютексом.
        // В данном случае, используем локальные значения, так как основной контроль уже прошел.

        if (local_motor_running_auto) {
            steps_taken_dosing++;
            // volume_dispensed_cycle = (float)steps_taken_dosing * config.mlPerStep; // УДАЛЕНО, теперь по датчику потока
        } else if (getCalibrationModeState() && local_motor_running_manual) { // Шаги для калибровки считаются только в ручном режиме калибровки
            portENTER_CRITICAL(&motor_cal_steps_mutex);
            steps_taken_calibration++;
            portEXIT_CRITICAL(&motor_cal_steps_mutex);
        }
    }
}

void manualMotorForward() {
    bool local_motor_running_auto;
    portENTER_CRITICAL(&motor_state_mutex);
    local_motor_running_auto = motor_running_auto;
    portEXIT_CRITICAL(&motor_state_mutex);

    if (!system_power_enabled || local_motor_running_auto) return;

    log_i("MOTOR_MAN", "Manual Forward");
    portENTER_CRITICAL(&motor_state_mutex);
    motor_dir = HIGH;
    portEXIT_CRITICAL(&motor_state_mutex);
    digitalWrite(DIR_PIN, motor_dir);
    updateMotorSpeed(config.motorSpeed > 0 ? config.motorSpeed : 100); // Используем текущую или базовую скорость
    digitalWrite(ENABLE_PIN, LOW);
    portENTER_CRITICAL(&motor_state_mutex);
    motor_running_manual = true;
    portEXIT_CRITICAL(&motor_state_mutex);
}

void manualMotorReverse() {
    bool local_motor_running_auto;
    portENTER_CRITICAL(&motor_state_mutex);
    local_motor_running_auto = motor_running_auto;
    portEXIT_CRITICAL(&motor_state_mutex);

    if (!system_power_enabled || local_motor_running_auto) return;

    log_i("MOTOR_MAN", "Manual Reverse");
    portENTER_CRITICAL(&motor_state_mutex);
    motor_dir = LOW;
    portEXIT_CRITICAL(&motor_state_mutex);
    digitalWrite(DIR_PIN, motor_dir);
    updateMotorSpeed(config.motorSpeed > 0 ? config.motorSpeed : 100);
    digitalWrite(ENABLE_PIN, LOW);
    portENTER_CRITICAL(&motor_state_mutex);
    motor_running_manual = true;
    portEXIT_CRITICAL(&motor_state_mutex);
}

void stopManualMotor() {
    bool local_motor_running_manual;
    portENTER_CRITICAL(&motor_state_mutex);
    local_motor_running_manual = motor_running_manual;
    portEXIT_CRITICAL(&motor_state_mutex);

    if (local_motor_running_manual) {
        log_i("MOTOR_MAN", "Manual Stop");
        digitalWrite(ENABLE_PIN, HIGH);
        portENTER_CRITICAL(&motor_state_mutex);
        motor_running_manual = false;
        portEXIT_CRITICAL(&motor_state_mutex);
    }
}

void stopMotor() { // Общая функция остановки
    digitalWrite(ENABLE_PIN, HIGH);
    portENTER_CRITICAL(&motor_state_mutex);
    motor_running_auto = false;
    motor_running_manual = false;
    // current_steps_per_sec = 0; // Можно также сбросить скорость здесь, если это требуется
    // step_interval_us = 0;
    portEXIT_CRITICAL(&motor_state_mutex);
    log_i("MOTOR", "Motor stopped (general stop).");
}

bool isMotorEnabled() {
    return digitalRead(ENABLE_PIN) == LOW;
}

int getMotorDirection() {
    int local_dir;
    portENTER_CRITICAL(&motor_state_mutex);
    local_dir = motor_dir;
    portEXIT_CRITICAL(&motor_state_mutex);
    return local_dir;
}

bool isMotorRunningManual() {
    bool status;
    portENTER_CRITICAL(&motor_state_mutex);
    status = motor_running_manual;
    portEXIT_CRITICAL(&motor_state_mutex);
    return status;
}

bool isMotorRunningAuto() {
    bool status;
    portENTER_CRITICAL(&motor_state_mutex);
    status = motor_running_auto;
    portEXIT_CRITICAL(&motor_state_mutex);
    return status;
}