#include "sensors.h"
#include "error_handler.h"
#include "config_manager.h" // Для config.flowMlPerPulse
#include "motor_control.h"  // Для motor_running_auto, motor_running_manual
#include "dosing_logic.h"   // Для current_dosing_state, volume_dispensed_cycle
#include "calibration_logic.h" // Для getCalibrationModeState
#include "main.h"           // Для system_power_enabled и функций логирования

// Пины определены в sensors.h

// Определения глобальных переменных из sensors.h
// Объекты OneWire и DallasTemperature больше не нужны

bool tempInSensorFound = false;
bool compressorRunning = false; // Определение переменной
bool tempCoolerSensorFound = false;
bool tempOutSensorFound = false;
float tIn = -127.0, tCool = -127.0, tOut = -127.0;
float tOut_filtered = -127.0;
int consecutive_temp_in_errors = 0;
int consecutive_temp_cooler_errors = 0;
int consecutive_temp_out_errors = 0;

volatile unsigned long flow_pulse_count = 0;
portMUX_TYPE flow_pulse_mutex = portMUX_INITIALIZER_UNLOCKED;
float current_flow_rate_ml_per_min = 0;
bool checking_for_flow = false;
static unsigned long lastCompressorStartTime = 0; // Для подсчета времени работы компрессора
unsigned long motor_start_time_with_no_flow = 0;
// Для No-Flow Timeout
// NO_FLOW_TIMEOUT_MS_CONST определена в sensors.h
// Для управления компрессором после активного дозирования
static unsigned long last_dosing_active_or_error_time = 0; // Время, когда дозирование было последний раз активно или в ошибке
// COMPRESSOR_POST_ACTIVE_DELAY_MS определена в sensors.h
// Внешние переменные, которые будут использоваться здесь
// extern Config config; // Доступно через config_manager.h
// extern bool motor_running_auto; // Доступно через motor_control.h
// extern bool motor_running_manual; // Доступно через motor_control.h
// extern bool system_power_enabled; // Доступно через main.h
// extern DosingState current_dosing_state; // Из dosing_logic.h
// extern float volume_dispensed_cycle;     // Из dosing_logic.h

// Функции логирования доступны через main.h
// extern void log_i(const char* tag, const char* format, ...);
// extern void log_w(const char* tag, const char* format, ...);
// extern void log_e(const char* tag, const char* format, ...);
// extern void log_d(const char* tag, const char* format, ...);

// Состояние для неблокирующего чтения температуры (упрощено для ADC)
// enum TempReadState { TEMP_READ_IDLE, TEMP_READ_REQUEST, TEMP_READ_WAIT, TEMP_READ_PROCESS }; // Больше не нужен сложный автомат
// TempReadState current_temp_read_state = TEMP_READ_IDLE; // Больше не нужен сложный автомат
// unsigned long temp_request_time = 0; // Больше не нужен
// const unsigned long TEMP_CONVERSION_TIME_MS = 800; // Больше не нужен
// TEMP_ERROR_THRESHOLD и TEMP_ANY_ERROR_THRESHOLD определены в sensors.h

void initSensors() {
    log_i("SENSORS", "Initializing temperature sensors...");
        // sensorIn.begin(); // Удалено, так как DallasTemperature больше не используется
    // Инициализация ADC пинов (уже сделана в глобальной области видимости для pinMode)
    pinMode(TEMP_IN_ADC_PIN, INPUT);
    pinMode(TEMP_COOLER_ADC_PIN, INPUT);
    pinMode(TEMP_OUT_ADC_PIN, INPUT);
    log_i("SENSORS", "Temp ADC pins initialized: IN_ADC=%d, COOLER_ADC=%d, OUT_ADC=%d", TEMP_IN_ADC_PIN, TEMP_COOLER_ADC_PIN, TEMP_OUT_ADC_PIN);

    log_i("SENSORS_INIT", "Performing initial temperature sensor check...");

    float initial_tOut = adcToTemperature(analogRead(TEMP_OUT_ADC_PIN), "Initial T_Out");
    if (initial_tOut == -127.0f) {
        tempOutSensorFound = false;
        log_w("SENSORS_INIT", "Initial check: T_Out sensor NOT DETECTED or error.");
    } else {
        tempOutSensorFound = true;
        tOut = initial_tOut; // Сохраняем начальное валидное значение
        tOut_filtered = tOut; // Инициализируем фильтр
        log_i("SENSORS_INIT", "Initial T_Out: %.2f C", tOut);
    }

    float initial_tIn = adcToTemperature(analogRead(TEMP_IN_ADC_PIN), "Initial T_In");
    if (initial_tIn == -127.0f) {
        tempInSensorFound = false;
        log_w("SENSORS_INIT", "Initial check: T_In sensor NOT DETECTED or error.");
    } else {
        tempInSensorFound = true;
        tIn = initial_tIn;
        log_i("SENSORS_INIT", "Initial T_In: %.2f C", tIn);
    }

    float initial_tCool = adcToTemperature(analogRead(TEMP_COOLER_ADC_PIN), "Initial T_Cooler");
    if (initial_tCool == -127.0f) {
        tempCoolerSensorFound = false;
        log_w("SENSORS_INIT", "Initial check: T_Cooler sensor NOT DETECTED or error.");
    } else {
        tempCoolerSensorFound = true;
        tCool = initial_tCool;
        log_i("SENSORS_INIT", "Initial T_Cooler: %.2f C", tCool);
    }
    
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulse, FALLING);
    log_i("SENSORS", "Flow sensor interrupt attached to pin %d.", FLOW_SENSOR_PIN);

    pinMode(COMPRESSOR_PIN, OUTPUT);
    digitalWrite(COMPRESSOR_PIN, LOW); // Компрессор выключен по умолчанию
    compressorRunning = false;
    log_i("SENSORS", "Compressor pin %d initialized.", COMPRESSOR_PIN);
}

// Вспомогательная функция для преобразования ADC в температуру
float adcToTemperature(int adcValue, const char* sensorName) {
    if (adcValue <= 0 || adcValue >= ADC_RESOLUTION) { // Проверка на обрыв или КЗ
        log_w("TEMP_ADC", "%s: ADC value out of range (%d). Assuming sensor error.", sensorName, adcValue);
        return -127.0f; // Значение ошибки
    }

    float Vout = (float)adcValue * (ADC_VREF / ADC_RESOLUTION);
    // Проверка на случай, если Vout равен ADC_VREF (делитель не работает)
    if (Vout >= ADC_VREF - 0.01f) { // Небольшой допуск
        log_w("TEMP_ADC", "%s: Vout (%.2fV) is too close to Vref. Check wiring/resistor.", sensorName, Vout);
        return -127.0f;
    }
    float Rthermistor = (Vout * SERIES_RESISTOR) / (ADC_VREF - Vout);

    if (Rthermistor <= 0) {
         log_w("TEMP_ADC", "%s: Calculated Rthermistor is invalid (%.2f Ohm).", sensorName, Rthermistor);
         return -127.0f;
    }

    float steinhart;
    steinhart = Rthermistor / NOMINAL_RESISTANCE;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= B_COEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_TEMPERATURE_C + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C

    if (isnan(steinhart) || isinf(steinhart) || steinhart < -50.0f || steinhart > 150.0f) {
        log_w("TEMP_ADC", "%s: Calculated temperature out of plausible range (%.2f C). ADC: %d, R_therm: %.0f", sensorName, steinhart, adcValue, Rthermistor);
        return -127.0f;
    }
    return steinhart;
}

void handleTempLogic() {
    unsigned long c_ms = millis();
    // Чтение и обработка температур с ADC
    // Temp Out
    float temp_val_out = adcToTemperature(analogRead(TEMP_OUT_ADC_PIN), "T_Out");
    if (temp_val_out != -127.0f) {
        tOut = temp_val_out;
        tOut_filtered = getFilteredTempOut(); // Используем функцию фильтрации
        consecutive_temp_out_errors = 0; // Используем getSystemErrorCode()
        if (getSystemErrorCode() == CRIT_TEMP_SENSOR_OUT_FAIL) {
            clearSystemError();
            log_i("TEMP", "Temp Out sensor recovered. Error cleared.");
        }
    } else {
        consecutive_temp_out_errors++;
        // tOut уже -127.0f из adcToTemperature
        // tOut_filtered = -127.0f; // Сбросить фильтр при ошибке
    }

    // Temp In
    float temp_val_in = adcToTemperature(analogRead(TEMP_IN_ADC_PIN), "T_In");
    if (temp_val_in != -127.0f) {
        tIn = temp_val_in;
        consecutive_temp_in_errors = 0; // Используем getSystemErrorCode()
        if (getSystemErrorCode() == CRIT_TEMP_SENSOR_IN_FAIL) {
            clearSystemError();
            log_i("TEMP", "Temp In sensor recovered. Error cleared.");
        }
    } else {
        consecutive_temp_in_errors++;
    }

    // Temp Cooler
    float temp_val_cooler = adcToTemperature(analogRead(TEMP_COOLER_ADC_PIN), "T_Cooler");
    if (temp_val_cooler != -127.0f) {
        tCool = temp_val_cooler;
        consecutive_temp_cooler_errors = 0; // Используем getSystemErrorCode()
        if (getSystemErrorCode() == CRIT_TEMP_SENSOR_COOLER_FAIL) {
            clearSystemError();
            log_i("TEMP", "Temp Cooler sensor recovered. Error cleared.");
        }
    } else {
        consecutive_temp_cooler_errors++;
    }

    // Логика установки ошибок (остается похожей)
#ifndef IGNORE_MISSING_SENSORS_FOR_DEVELOPMENT
    if (consecutive_temp_out_errors >= TEMP_ERROR_THRESHOLD && getSystemErrorCode() != CRIT_TEMP_SENSOR_OUT_FAIL) { // Используем getSystemErrorCode()
        setSystemError(CRIT_TEMP_SENSOR_OUT_FAIL, "CRIT: Output temp sensor failed!");
        tOut_filtered = -127.0f; // Явно устанавливаем ошибку и для фильтрованного значения
    }
    if (consecutive_temp_in_errors >= TEMP_ANY_ERROR_THRESHOLD && getSystemErrorCode() != CRIT_TEMP_SENSOR_IN_FAIL) { // Используем getSystemErrorCode()
        setSystemError(CRIT_TEMP_SENSOR_IN_FAIL, "CRIT: Input temp sensor failed!"); // Можно сделать менее критичным, если T_in не так важен
    }
    if (consecutive_temp_cooler_errors >= TEMP_ANY_ERROR_THRESHOLD && getSystemErrorCode() != CRIT_TEMP_SENSOR_COOLER_FAIL) { // Используем getSystemErrorCode()
        setSystemError(CRIT_TEMP_SENSOR_COOLER_FAIL, "CRIT: Cooler temp sensor failed!"); // Аналогично
    }
#else
    // В режиме разработки, логируем предупреждения, но не устанавливаем критические ошибки для датчиков
    if (consecutive_temp_out_errors >= TEMP_ERROR_THRESHOLD && getSystemErrorCode() != CRIT_TEMP_SENSOR_OUT_FAIL) {
        app_log_w("TEMP_DEV", "Output temp sensor failed (%d errors), but IGNORED for development.", consecutive_temp_out_errors);
        tOut_filtered = -127.0f; // Все еще полезно сбросить фильтр
    }
    if (consecutive_temp_in_errors >= TEMP_ANY_ERROR_THRESHOLD && getSystemErrorCode() != CRIT_TEMP_SENSOR_IN_FAIL) {
        app_log_w("TEMP_DEV", "Input temp sensor failed (%d errors), but IGNORED for development.", consecutive_temp_in_errors);
    }
    if (consecutive_temp_cooler_errors >= TEMP_ANY_ERROR_THRESHOLD && getSystemErrorCode() != CRIT_TEMP_SENSOR_COOLER_FAIL) {
        app_log_w("TEMP_DEV", "Cooler temp sensor failed (%d errors), but IGNORED for development.", consecutive_temp_cooler_errors);
    }
#endif

    log_d("TEMP", "Temp In: %.1f C, Cooler: %.1f C, Out: %.1f C (F: %.1f C)", tIn, tCool, tOut, tOut_filtered);

    DosingState local_dosing_state;
    portENTER_CRITICAL(&dosing_state_mutex); // Мьютекс из dosing_logic.h
    local_dosing_state = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    // Обновляем время последней активности дозатора или ошибки
    if (local_dosing_state != DOSING_STATE_IDLE && local_dosing_state != DOSING_STATE_FINISHED) {
        last_dosing_active_or_error_time = c_ms;
    }

    // Управление компрессором
    // Общее управление компрессором по температуре активно, только если:
    // 1. Система включена.
    // 2. Нет критической ошибки датчика T_out.
    // 3. Состояние дозирования - IDLE или FINISHED (т.е. dosing_logic не управляет компрессором активно).
    bool allow_general_compressor_control =
        system_power_enabled && // system_power_enabled - bool, чтение атомарно
        getSystemErrorCode() != CRIT_TEMP_SENSOR_OUT_FAIL && // Используем getSystemErrorCode()
        (local_dosing_state == DOSING_STATE_IDLE || local_dosing_state == DOSING_STATE_FINISHED);

    if (allow_general_compressor_control) {
        // Проверяем, не было ли дозирование активно или в ошибке совсем недавно
        if (last_dosing_active_or_error_time != 0 && (c_ms - last_dosing_active_or_error_time < COMPRESSOR_POST_ACTIVE_DELAY_MS)) {
            // Дозирование было недавно активно/в ошибке. Общее управление компрессором приостановлено.
            // dosing_logic должен был установить состояние компрессора (например, выключить).
            // Мы здесь ничего не делаем, чтобы не мешать.
            log_d("SENSORS_COMP", "Post-dosing/error delay active (%lu ms remaining). General compressor control paused.", COMPRESSOR_POST_ACTIVE_DELAY_MS - (c_ms - last_dosing_active_or_error_time));
        } else {
            // Задержка прошла, или дозирование не было активно/в ошибке в последнее время.
            if (last_dosing_active_or_error_time != 0) { // Сбрасываем флаг времени, если задержка только что закончилась
                log_d("SENSORS_COMP", "Post-dosing/error delay ended. Resuming general compressor control.");
                last_dosing_active_or_error_time = 0;
            }

            // Используем отфильтрованную температуру для управления
            float control_temp = (tOut_filtered == -127.0f) ? tOut : tOut_filtered; // Если фильтр еще не готов, используем сырую
            if (control_temp != -127.0f) { // Убедимся, что есть валидное значение
                const float temp_hysteresis_general = 0.5f; // Гистерезис для общего управления
                if (control_temp > (config.tempSetpoint + temp_hysteresis_general)) {
                    compressorOn();
                } else if (control_temp < config.tempSetpoint) {
                    compressorOff();
                }
            }
        } // Используем getSystemErrorCode()
    } else if (!system_power_enabled || getSystemErrorCode() == CRIT_TEMP_SENSOR_OUT_FAIL) {
        // Жесткое выключение компрессора, если система выключена или критическая ошибка датчика,
        // независимо от состояния дозирования или задержек.
        if (compressorRunning) {
            compressorOff();
        }
    }
}

float getFilteredTempOut() {
    // ... (реализация фильтра Калмана или простого скользящего среднего для tOut, как в mainbuidv4.c) ...
    // Пока просто вернем tOut, если фильтр не реализован подробно
    static float alpha = 0.2; // Коэффициент сглаживания для экспоненциального скользящего среднего
    if (tOut_filtered == -127.0f && tOut != -127.0f) { // Инициализация фильтра
        // Это условие сработает, если tOut_filtered был сброшен в -127.0f из-за ошибки,
        // а затем tOut снова стал валидным.
        tOut_filtered = tOut;
    } else if (tOut == -127.0f) { // Если сырое значение ошибочно, сбрасываем фильтр
        // Не меняем tOut_filtered здесь. Если ошибка постоянная,
        // handleTempLogic установит tOut_filtered в -127.0f.
    } else if (tOut != -127.0f && tOut_filtered != -127.0f) { // Обновляем, только если оба значения валидны
        tOut_filtered = alpha * tOut + (1 - alpha) * tOut_filtered;
    }
    return tOut_filtered;
}

void IRAM_ATTR flowPulse() {
    portENTER_CRITICAL_ISR(&flow_pulse_mutex);
    flow_pulse_count++;
    if (getCalibrationModeState()) { // Если в режиме калибровки, считаем импульсы и для нее
        portENTER_CRITICAL_ISR(&cal_pulse_mutex); // Используем отдельный мьютекс для калибровочных импульсов
        flow_pulses_calibration++;
        portEXIT_CRITICAL_ISR(&cal_pulse_mutex);
    }
    portEXIT_CRITICAL_ISR(&flow_pulse_mutex);
}

void handleFlowSensor() {
    // ... (реализация логики датчика потока, как в mainbuidv4.c) ...
    // Важно: setSystemError будет вызываться из error_handler.h
    // config.flowMlPerPulse будет браться из config_manager.h (через extern Config config)
    // motor_running_auto, motor_running_manual, getCalibrationModeState() будут доступны через extern или .h файлы
    static unsigned long last_flow_check_time_local = 0; // Локальная переменная для этого модуля
    if (millis() - last_flow_check_time_local >= FLOW_CHECK_INTERVAL_MS) {
        unsigned long p_snap;
        portENTER_CRITICAL(&flow_pulse_mutex);
        p_snap = flow_pulse_count;
        flow_pulse_count = 0;
        portEXIT_CRITICAL(&flow_pulse_mutex);

        const float ml_per_pulse = config.flowMlPerPulse;
        if (FLOW_CHECK_INTERVAL_MS == 0 || ml_per_pulse <= 0.000001f) {
            current_flow_rate_ml_per_min = 0.0f;
        } else {
            float flow_rate_ml_per_sec = ((float)p_snap * ml_per_pulse) / (FLOW_CHECK_INTERVAL_MS / 1000.0f);
            current_flow_rate_ml_per_min = flow_rate_ml_per_sec * 60.0f;
        }
        
        // Обновляем объем, выданный в текущем цикле дозирования
        // motor_running_auto - bool, чтение атомарно
        DosingState local_dosing_state;
        portENTER_CRITICAL(&dosing_state_mutex); // Мьютекс из dosing_logic.h
        local_dosing_state = current_dosing_state;
        portEXIT_CRITICAL(&dosing_state_mutex);

        if (motor_running_auto && local_dosing_state == DOSING_STATE_RUNNING) {
            portENTER_CRITICAL(&volume_dispensed_mutex); // Мьютекс из dosing_logic.h
            volume_dispensed_cycle += (float)p_snap * ml_per_pulse;
            portEXIT_CRITICAL(&volume_dispensed_mutex);
        }

        // Логика No-Flow Timeout
        // motor_running_auto - bool, чтение атомарно
        // local_dosing_state уже прочитан выше
        if (motor_running_auto && local_dosing_state == DOSING_STATE_RUNNING) {
            if (p_snap > 0) {
                // Если поток обнаружен, сбрасываем таймер (или флаг) no-flow
                if (checking_for_flow) { // checking_for_flow устанавливается в true в dosing_logic.c при старте мотора
                    log_d("FLOW", "Flow detected during no-flow check window.");
                    checking_for_flow = false; // Поток есть, первичная проверка пройдена
                }
            } else { // p_snap == 0 (нет импульсов)
                if (checking_for_flow) { // Если активна проверка на отсутствие потока после старта мотора
                    if (millis() - motor_start_time_with_no_flow > NO_FLOW_TIMEOUT_MS_CONST) {
                        log_e("FLOW", "NO FLOW TIMEOUT! Motor running for %lu ms but no flow detected.", NO_FLOW_TIMEOUT_MS_CONST);
                        setSystemError(CRIT_FLOW_SENSOR_FAIL, "No flow detected after motor start.");
                        checking_for_flow = false; // Остановить проверку, чтобы не спамить ошибками
                    }
                }
            }
        } else { // Мотор не в авто-режиме или не в состоянии DOSING_STATE_RUNNING
            if (checking_for_flow) {
                checking_for_flow = false; // Сбросить флаг, если дозирование остановлено
            }
        }

        // Локальная копия для безопасного чтения volume_dispensed_cycle
        float local_volume_dispensed_cycle;
        portENTER_CRITICAL(&volume_dispensed_mutex);
        local_volume_dispensed_cycle = volume_dispensed_cycle;
        portEXIT_CRITICAL(&volume_dispensed_mutex);

        log_d("FLOW", "Rate: %.2f ml/min, Pulses in interval: %lu, Dispensed this cycle: %.2f ml", current_flow_rate_ml_per_min, p_snap, local_volume_dispensed_cycle);
        last_flow_check_time_local = millis();
    }
}

void compressorOn() {
    if (!compressorRunning && system_power_enabled) {
        digitalWrite(COMPRESSOR_PIN, HIGH);
        compressorRunning = true;
        config.compressorStartCount++;
        lastCompressorStartTime = millis(); // Раскомментировано для отслеживания времени работы
        log_i("COMPRESSOR", "Compressor ON");
    }
}

void compressorOff() {
    if (compressorRunning) {
        digitalWrite(COMPRESSOR_PIN, LOW);
        compressorRunning = false;
        if (lastCompressorStartTime > 0) { // Убедимся, что время старта было зафиксировано
           config.compressorRunTime += (millis() - lastCompressorStartTime);
           lastCompressorStartTime = 0; // Сбрасываем для следующего цикла
        }
        log_i("COMPRESSOR", "Compressor OFF");
    }
}

bool isCompressorRunning() {
    // Чтение bool атомарно, мьютекс не обязателен для простого чтения
    return compressorRunning;
}

// Определения геттеров
float getTempOut() {
    // Простое чтение float обычно атомарно на ESP32, но для tOut,
    // который может обновляться в handleTempLogic, лучше использовать мьютекс, если бы он был.
    // В данном случае, предполагаем, что чтение достаточно безопасно для статусного сообщения.
    return tOut;
}

float getFlowRate() {
    return current_flow_rate_ml_per_min; // Аналогично tOut
}

bool getWaterLevelSwitchStatus() {
    // TODO: Реализовать логику чтения датчика уровня воды, если он есть.
    return true; // Заглушка, всегда возвращает true (вода есть)
}