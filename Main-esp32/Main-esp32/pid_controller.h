#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config_manager.h" // Для Config
#include "sensors.h"        // Для tOut_filtered
#include "motor_control.h"  // Для current_steps_per_sec, step_interval_us, updateMotorSpeedFromPid
#include "dosing_logic.h"   // Для DosingState, current_dosing_state

// Мьютекс для защиты статических переменных PID (определен в .c файле)
extern portMUX_TYPE pid_params_mutex;

void initPidController(); // Инициализация переменных PID
void handlePidControl();  // Основная функция расчета и применения PID
void enablePidTempControl(bool enable); // Включение/выключение PID
float getPidSetpointTemp(); // Объявление геттера для уставки PID
void setPidCoefficients(float kp, float ki, float kd); // Установка коэффициентов

// Функции для получения текущих значений PID (для отображения в UI и т.д.)
bool getIsPidTempControlEnabled();
float getPidKp();
float getPidKi();
float getPidKd(); // Объявление геттера
float getPidSetpoint();

#endif // PID_CONTROLLER_H