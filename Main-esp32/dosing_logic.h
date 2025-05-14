#ifndef DOSING_LOGIC_H
#define DOSING_LOGIC_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h" // Для portMUX_TYPE
// Состояния дозирования (можно оставить в main.h или перенести сюда, если используется только здесь и в main.c)
enum DosingState {
    DOSING_STATE_IDLE = 0,
    DOSING_STATE_REQUESTED,
    DOSING_STATE_PRE_COOLING,
    DOSING_STATE_STARTING,
    DOSING_STATE_RUNNING,
    DOSING_STATE_STOPPING,
    DOSING_STATE_PAUSED, // Пока не используется активно
    DOSING_STATE_FINISHED,
    DOSING_STATE_ERROR
}; // Оставляем DosingState для обратной совместимости, если где-то используется без _t
typedef enum DosingState DosingState_t; // Определяем DosingState_t

extern DosingState_t current_dosing_state;
extern portMUX_TYPE dosing_state_mutex; // Мьютекс для current_dosing_state и dosing_state_start_time
extern unsigned long dosing_state_start_time;

// Объем, выданный в текущем цикле (обновляется датчиком потока)
extern volatile float volume_dispensed_cycle; // volatile, так как может обновляться из ISR (датчик потока)
extern portMUX_TYPE volume_dispensed_mutex;   // Мьютекс для volume_dispensed_cycle
DosingState_t getDosingState(); // Геттер для current_dosing_state
float getCurrentDosedVolume();  // Геттер для volume_dispensed_cycle
const char* getDosingStateString(DosingState_t state); // Новая функция

void initDosingLogic();
void handleDosingState();
void startDosingCycle(int volumeML, bool fromWeb = false);
void stopDosingCycle(bool fromWeb = false); // Объявление функции
void log_dosing_state_change(DosingState_t new_state); // Объявление функции

#endif // DOSING_LOGIC_H