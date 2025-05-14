#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <OneWire.h> // Если бы использовались DS18B20, но для ADC не нужны
#include <DallasTemperature.h> // Если бы использовались DS18B20, но для ADC не нужны
#include "hardware_pins.h" // <-- ДОБАВЛЕНО: Единый файл с определениями пинов

// --- GPIO Pin Definitions --- теперь в hardware_pins.h
// #define TEMP_IN_ADC_PIN         34
// #define TEMP_COOLER_ADC_PIN     35
// #define TEMP_OUT_ADC_PIN        36
// #define FLOW_SENSOR_PIN         25
// #define COMPRESSOR_PIN          26

// --- ADC and NTC Thermistor Parameters ---
#define ADC_RESOLUTION          4095.0f  // Разрешение АЦП (12-бит для ESP32)
#define ADC_VREF                3.3f     // Опорное напряжение АЦП (или откалиброванное значение)
#define SERIES_RESISTOR         10000.0f // Сопротивление последовательного резистора в делителе (Ом)
#define NOMINAL_RESISTANCE      10000.0f // Номинальное сопротивление NTC-термистора при номинальной температуре (Ом)
#define B_COEFFICIENT           3950.0f  // B-коэффициент NTC-термистора
#define NOMINAL_TEMPERATURE_C   25.0f    // Номинальная температура для NTC (градусы Цельсия)

// --- Sensor Logic Parameters ---
#define TEMP_ERROR_THRESHOLD         5   // Количество последовательных ошибок для критического отказа датчика T_out
#define TEMP_ANY_ERROR_THRESHOLD     10  // Количество последовательных ошибок для некритических датчиков (T_in, T_cooler)
#define FLOW_CHECK_INTERVAL_MS       1000 // Интервал проверки датчика потока (мс)
#define NO_FLOW_TIMEOUT_MS_CONST     10000 // Таймаут отсутствия потока при работающем моторе (мс)
#define COMPRESSOR_POST_ACTIVE_DELAY_MS 15000 // Задержка для общего управления компрессором после активности дозатора (мс)

// --- Global Variables (declarations) ---
extern bool tempInSensorFound;
extern bool tempCoolerSensorFound;
extern bool tempOutSensorFound;
extern bool compressorRunning;

extern float tIn;
extern float tCool;
extern float tOut;
extern float tOut_filtered;

extern int consecutive_temp_in_errors;
extern int consecutive_temp_cooler_errors;
extern int consecutive_temp_out_errors;

extern volatile unsigned long flow_pulse_count; // Счетчик импульсов с датчика потока
extern portMUX_TYPE flow_pulse_mutex;          // Мьютекс для защиты flow_pulse_count
extern float current_flow_rate_ml_per_min;     // Текущий рассчитанный расход

extern bool checking_for_flow;                 // Флаг, что активна проверка на отсутствие потока
extern unsigned long motor_start_time_with_no_flow; // Время старта мотора для проверки no-flow

// --- Function Prototypes ---
void initSensors();
void handleTempLogic();
float getFilteredTempOut();
void IRAM_ATTR flowPulse(); // Обработчик прерывания датчика потока
float getTempOut(); // Геттер для tOut
float getFlowRate(); // Геттер для current_flow_rate_ml_per_min
bool getWaterLevelSwitchStatus(); // Геттер для состояния датчика уровня воды
void handleFlowSensor();
void compressorOn();
void compressorOff();
bool isCompressorRunning(); // Геттер для состояния компрессора
float adcToTemperature(int adcValue, const char* sensorName); // Сделаем ее доступной, если вдруг понадобится извне

#endif // SENSORS_H