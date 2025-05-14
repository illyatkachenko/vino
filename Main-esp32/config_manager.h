#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include "error_handler.h" // Для ERROR_HANDLER_LAST_MSG_BUFFER_SIZE

#define CONFIG_NAMESPACE "app_config"
#define DEFAULT_LANGUAGE "ru" // или "en"

typedef struct {
    float tempSetpoint;
    int volumeTarget;
    int motorSpeed;
    float mlPerStep;
    float flowMlPerPulse;
    char remotePeerMacStr[18]; // "XX:XX:XX:XX:XX:XX" + null
    unsigned long totalVolumeDispensed;
    unsigned long compressorRunTime; // в миллисекундах
    int compressorStartCount;
    int totalDosingCycles;
    int errorCount;
    char lastErrorMsgBuffer[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE]; // Буфер для последнего сообщения об ошибке
    float pidKp;
    float pidKi;
    float pidKd;
    bool systemPowerStateSaved; // Сохраненное состояние питания
    char currentLanguage[3]; // "ru" или "en"
} Config;

extern Config config; // Делаем структуру config доступной глобально

void saveConfig();
void loadConfig();
void performFactoryReset();
void resetStats();

#endif // CONFIG_MANAGER_H
