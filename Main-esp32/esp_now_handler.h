#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include <Arduino.h>
#include <esp_now.h>
#include "error_handler.h" // For ERROR_HANDLER_LAST_MSG_BUFFER_SIZE

typedef struct __attribute__((packed)) esp_now_cmd_s {
    uint8_t command;        // Код команды (0-255)
    int16_t volume;         // Целевой объем (мл), может быть отрицательным для спец. команд
    float temp;             // Целевая температура (градусы C)
    int16_t speed;          // Скорость мотора (шаги/сек)
    float actualVolume;     // Фактический объем (для калибровки)
    uint8_t checksum;       // Простая контрольная сумма (XOR всех байт до checksum)
} esp_now_cmd_t;

#define MAX_ESP_NOW_ERROR_MSG_LEN (ERROR_HANDLER_LAST_MSG_BUFFER_SIZE > 60 ? 60 : ERROR_HANDLER_LAST_MSG_BUFFER_SIZE - 1) // Ограничиваем длину сообщения для ESP-NOW

// Статус, отправляемый на дисплей (СИНХРОНИЗИРОВАННАЯ СТРУКТURA)
typedef struct __attribute__((packed)) esp_now_status_s {
    uint8_t status_dosing_state;    // Текущее состояние дозирования (из DosingState enum)
    float tempOut;                  // Температура на выходе (градусы C)
    float flowRate;                 // Текущий расход (мл/мин)
    int16_t volumeDispensedCycle_ml;// Объем, выданный в текущем цикле (мл)
    uint8_t errorCode;              // Код текущей ошибки системы (из SystemErrorCode enum)
    // Группируем битовые поля для лучшей совместимости и потенциального устранения предупреждения
    uint8_t systemPower   : 1;      // Питание системы (0=OFF, 1=ON)
    uint8_t compressorState : 1;    // Состояние компрессора (0=OFF, 1=ON)
    uint8_t calibrationActive : 1;  // Режим калибровки активен (0=OFF, 1=ON)
    uint8_t wifiConnected : 1;      // WiFi подключен
    uint8_t waterLevelOk : 1;       // Состояние датчика уровня воды (1=OK, 0=LOW)
    uint8_t reserved      : 3;      // Зарезервировано для будущих флагов (5 флагов + 3 бита = 1 байт)
    float currentTempSetpoint;      // Текущая уставка температуры
    int16_t currentVolumeTarget;    // Текущий целевой объем
    char lastErrorMessage[MAX_ESP_NOW_ERROR_MSG_LEN + 1]; // Текстовое сообщение последней ошибки
    uint8_t checksum;               // Простая контрольная сумма (XOR всех байт до checksum)
} esp_now_status_t;

extern bool esp_now_peer_added; // Объявляем глобальную переменную

void initEspNow();
void onEspNowSend(const uint8_t *mac_addr, esp_now_send_status_t status);
void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
bool sendEspNowData(const uint8_t *data, size_t len, const uint8_t *peer_addr); // Отправка с повторами
void sendSystemStatusEspNow();
bool isEspNowPeerAvailable(); // Проверяет, существует ли пир
void getRemotePeerAddress(uint8_t *mac_addr_buf); // Копирует MAC-адрес пира (буфер должен быть 6 байт) - Объявление
void ensureEspNowPeer(); // Пытается добавить пир, если он не добавлен или потерян

#endif // ESP_NOW_HANDLER_H