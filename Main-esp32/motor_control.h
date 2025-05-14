#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
// Определения пинов мотора теперь находятся в hardware_pins.h

// Определения для направления мотора
#define MOTOR_DIR_FORWARD HIGH
#define MOTOR_DIR_REVERSE LOW

// Глобальные переменные, связанные с мотором (объявляем как extern)
extern bool motor_running_auto;
extern bool motor_running_manual;
extern bool motor_dir; // HIGH - forward, LOW - reverse
extern float current_steps_per_sec;
extern unsigned long step_interval_us;
extern long steps_taken_dosing;
extern long steps_target_dosing;


// Мьютекс для защиты состояния мотора (определен в .c файле)
extern portMUX_TYPE motor_state_mutex;

// Функции
void initMotor();
void updateMotorSpeed(int speedSetting); // Устанавливает базовую скорость
void updateMotorSpeedFromPid(float steps_sec); // Устанавливает скорость от PID
void handleMotorStepping(); // Основная функция для генерации шагов
void manualMotorForward();
void manualMotorReverse();
void stopManualMotor();
void stopMotor(); // Общая функция остановки мотора
bool isMotorEnabled(); // Проверяет, включен ли мотор (ENABLE_PIN)
bool isMotorRunningManual(); // Геттер для motor_running_manual
bool isMotorRunningAuto(); // Геттер для motor_running_auto
int getMotorDirection(); // Возвращает текущее направление мотора

#endif // MOTOR_CONTROL_H