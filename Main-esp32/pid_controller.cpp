#include "pid_controller.h"
#include "config_manager.h" // Для config.tempSetpoint
#include "sensors.h"        // Для tOut (или tOut_filtered)
#include "motor_control.h"  // Для current_steps_per_sec, step_interval_us
#include "dosing_logic.h"   // Для current_dosing_state

#include "main.h"           // Для system_power_enabled и функций логирования
// Статические переменные модуля PID
static bool pid_temp_control_enabled_static = false;
static float pid_kp_static = 20.0f;
static float pid_ki_static = 0.5f;
static float pid_kd_static = 5.0f;
static float pid_setpoint_temp_static = 4.0f; // Начальное значение
static float pid_integral_static = 0.0f;
static float pid_previous_error_static = 0.0f;
static unsigned long pid_last_time_static = 0;

// Мьютекс для защиты статических переменных PID
portMUX_TYPE pid_params_mutex = portMUX_INITIALIZER_UNLOCKED;

// Константы PID (определяем здесь)
const int PID_MIN_MOTOR_SPEED = 30;  // Минимальная скорость мотора при PID управлении
const int PID_MAX_MOTOR_SPEED = 800; // Максимальная скорость мотора при PID управлении
const int PID_BASE_MOTOR_SPEED = 150; // Базовая скорость, от которой PID будет отталкиваться (может быть = config.motorSpeed)

// Внешние переменные
// extern Config config; // Доступно через config_manager.h
// extern bool system_power_enabled; // Доступно через main.h
// extern DosingState current_dosing_state; // Доступно через dosing_logic.h
// extern float tOut_filtered; // Используем отфильтрованную температуру для PID, доступно через sensors.h
// extern float current_steps_per_sec; // Управляется PID, доступно через motor_control.h
// extern unsigned long step_interval_us; // Управляется PID, доступно через motor_control.h

// Функции логирования доступны через main.h
// extern void log_i(const char* tag, const char* format, ...);
// extern void log_d(const char* tag, const char* format, ...);

// Getters for PID parameters
bool getIsPidTempControlEnabled() {
    bool status;
    portENTER_CRITICAL(&pid_params_mutex);
    status = pid_temp_control_enabled_static;
    portEXIT_CRITICAL(&pid_params_mutex);
    return status;
}
float getPidKp() {
    float val;
    portENTER_CRITICAL(&pid_params_mutex);
    val = pid_kp_static;
    portEXIT_CRITICAL(&pid_params_mutex);
    return val;
}
float getPidKi() {
    float val;
    portENTER_CRITICAL(&pid_params_mutex);
    val = pid_ki_static;
    portEXIT_CRITICAL(&pid_params_mutex);
    return val;
}
float getPidKd() {
    float val;
    portENTER_CRITICAL(&pid_params_mutex);
    val = pid_kd_static;
    portEXIT_CRITICAL(&pid_params_mutex);
    return val;
}
// Имя функции изменено на getPidSetpointTemp, чтобы соответствовать вызову
float getPidSetpointTemp() {
    float val;
    portENTER_CRITICAL(&pid_params_mutex);
    val = pid_setpoint_temp_static;
    portEXIT_CRITICAL(&pid_params_mutex);
    return val;
}

void initPidController() {
    portENTER_CRITICAL(&pid_params_mutex);
    pid_temp_control_enabled_static = false; // По умолчанию выключен
    pid_setpoint_temp_static = config.tempSetpoint; // Инициализируем уставкой из конфига
    pid_integral_static = 0.0f;
    pid_previous_error_static = 0.0f;
    pid_last_time_static = millis();
    // Загрузка коэффициентов из config при инициализации
    pid_kp_static = config.pidKp;
    pid_ki_static = config.pidKi;
    pid_kd_static = config.pidKd;
    portEXIT_CRITICAL(&pid_params_mutex);
    log_i("PID", "PID Controller initialized. Kp=%.2f, Ki=%.2f, Kd=%.2f", pid_kp_static, pid_ki_static, pid_kd_static);
}

void enablePidTempControl(bool enable) {
    portENTER_CRITICAL(&pid_params_mutex);
    pid_temp_control_enabled_static = enable;
    if (enable) {
        pid_setpoint_temp_static = config.tempSetpoint; // Устанавливаем текущую уставку
        pid_integral_static = 0.0f;
        pid_previous_error_static = 0.0f;
        pid_last_time_static = millis();
    }
    portEXIT_CRITICAL(&pid_params_mutex);
    log_i("PID", "PID Temperature Control %s. Setpoint: %.1f C", enable ? "ENABLED" : "DISABLED", pid_setpoint_temp_static);
}

void setPidCoefficients(float kp, float ki, float kd) {
    portENTER_CRITICAL(&pid_params_mutex);
    pid_kp_static = kp;
    pid_ki_static = ki;
    pid_kd_static = kd;
    portEXIT_CRITICAL(&pid_params_mutex);
    log_i("PID", "PID Coefficients updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", pid_kp_static, pid_ki_static, pid_kd_static);
    // Обновляем config, но НЕ сохраняем здесь. Сохранение должно управляться извне.
    if (fabs(config.pidKp - kp) > 0.001f || fabs(config.pidKi - ki) > 0.001f || fabs(config.pidKd - kd) > 0.001f) {
        config.pidKp = kp;
        config.pidKi = ki;
        config.pidKd = kd;
        // saveConfig(); // <--- УДАЛИТЬ ВЫЗОВ saveConfig() ОТСЮДА
        log_i("PID", "PID Coefficients in config struct updated. Save externally if needed.");
    }
}

void handlePidControl() {
    // Читаем состояние PID и dosing_state под мьютексами
    bool local_pid_enabled;
    DosingState local_dosing_state;
    float local_pid_setpoint;
    float local_pid_kp, local_pid_ki, local_pid_kd;
    float local_pid_integral, local_pid_previous_error;
    unsigned long local_pid_last_time;

    portENTER_CRITICAL(&pid_params_mutex);
    local_pid_enabled = pid_temp_control_enabled_static;
    local_pid_setpoint = pid_setpoint_temp_static;
    local_pid_kp = pid_kp_static;
    local_pid_ki = pid_ki_static;
    local_pid_kd = pid_kd_static;
    local_pid_integral = pid_integral_static;
    local_pid_previous_error = pid_previous_error_static;
    local_pid_last_time = pid_last_time_static;
    portEXIT_CRITICAL(&pid_params_mutex);

    portENTER_CRITICAL(&dosing_state_mutex); // Мьютекс из dosing_logic.h
    local_dosing_state = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    if (!local_pid_enabled || !system_power_enabled ||
        !(local_dosing_state == DOSING_STATE_RUNNING || local_dosing_state == DOSING_STATE_STARTING)) {
        return;
    }

    unsigned long current_time = millis();
    float dt = (float)(current_time - local_pid_last_time) / 1000.0f;

    if (dt <= 0.001f) { // Слишком маленький интервал времени
        return;
    }

    // Обновляем уставку PID, если она изменилась в config
    if (fabs(local_pid_setpoint - config.tempSetpoint) > 0.01f) {
        portENTER_CRITICAL(&pid_params_mutex);
        pid_setpoint_temp_static = config.tempSetpoint;
        local_pid_setpoint = pid_setpoint_temp_static; // Обновляем локальную копию
        // Сброс интеграла при смене уставки может быть полезен
        pid_integral_static = 0.0f;
        local_pid_integral = 0.0f;
        portEXIT_CRITICAL(&pid_params_mutex);
        log_i("PID", "Setpoint updated to %.1f C", local_pid_setpoint);
    }

    float current_temp = tOut_filtered; // Используем отфильтрованную температуру

    if (current_temp == -127.0f) { // Датчик не вернул валидное значение
        log_w("PID", "Invalid temperature (%.1f C) for PID control. Skipping.", current_temp);
        return;
    }

    float error = local_pid_setpoint - current_temp;

    // Пропорциональный член
    float p_term = local_pid_kp * error;

    // Интегральный член (с anti-windup)
    local_pid_integral += local_pid_ki * error * dt;
    local_pid_integral = constrain(local_pid_integral, -200.0f, 200.0f); // Ограничение интеграла (anti-windup)

    // Дифференциальный член
    float derivative = (error - local_pid_previous_error) / dt;
    float d_term = local_pid_kd * derivative;

    float pid_output_correction = p_term + local_pid_integral + d_term;
    int base_speed = config.motorSpeed > 0 ? config.motorSpeed : PID_BASE_MOTOR_SPEED;
    int new_motor_speed = constrain(base_speed + (int)round(pid_output_correction), PID_MIN_MOTOR_SPEED, PID_MAX_MOTOR_SPEED);

    updateMotorSpeedFromPid((float)new_motor_speed); // Эта функция должна быть в motor_control.h/c

    log_d("PID_TEMP", "Tset:%.1f, Tcur:%.1f, Err:%.2f, P:%.2f, I(sum):%.2f, D:%.2f, OutCorr:%.2f, BaseSpd:%d, NewSpeed:%d (%.1f st/s)",
          local_pid_setpoint, current_temp, error, p_term, local_pid_integral, d_term, pid_output_correction, base_speed, new_motor_speed, current_steps_per_sec);

    // Сохраняем обновленные значения PID состояния
    portENTER_CRITICAL(&pid_params_mutex);
    pid_integral_static = local_pid_integral;
    pid_previous_error_static = error;
    pid_last_time_static = current_time;
    portEXIT_CRITICAL(&pid_params_mutex);
}