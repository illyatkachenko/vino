#ifndef CALIBRATION_LOGIC_H
#define CALIBRATION_LOGIC_H

#include <Arduino.h>

extern bool _calibrationMode; // Защищено мьютексом
extern portMUX_TYPE cal_mode_mutex;
extern unsigned long calibration_start_time;
extern bool calibration_stopped_by_timeout;

extern long steps_taken_calibration; // Шаги, подсчитанные во время калибровки мотора
extern portMUX_TYPE motor_cal_steps_mutex;

extern volatile unsigned long flow_pulses_calibration; // Импульсы, подсчитанные во время калибровки потока
extern portMUX_TYPE cal_pulse_mutex;

void initCalibrationLogic();
void handleCalibrationLogic(); // Обработка таймаута калибровки
void startCalibrationMode(float targetVolume, bool fromWeb = false);
void stopCalibrationMode(float actualVolume, bool fromWeb = false); // Для калибровки мотора (mlPerStep)
void stopFlowCalibrationMode(float actualVolume, bool fromWeb = false); // Новая функция для калибровки датчика потока (flowMlPerPulse)

void setCalibrationModeState(bool state);
bool getCalibrationModeState();

#endif // CALIBRATION_LOGIC_H