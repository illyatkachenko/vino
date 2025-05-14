#ifndef SENSITIVE_CONFIG_H
#define SENSITIVE_CONFIG_H

#include <Arduino.h>
#include <Preferences.h> // Для использования Preferences

// Namespace для хранения чувствительных данных
#define SECRETS_NAMESPACE "secrets"

// Максимальные размеры буферов для строк
#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64
#define MAX_USERNAME_LEN 32

// Объявления функций
void initSensitiveConfig(); // Инициализация модуля и загрузка из NVS
void saveSensitiveConfig(); // Сохранение текущих чувствительных настроек в NVS

// Геттеры для доступа к данным
const char* getWifiSsid();
const char* getWifiPassword();
const char* getWebUsername();
const char* getWebPassword();

// Сеттеры для обновления данных перед сохранением
void setWifiSsid(const char* new_ssid);
void setWifiPassword(const char* new_password);

#endif // SENSITIVE_CONFIG_H