#ifndef WEB_SERVER_HANDLERS_H
#define WEB_SERVER_HANDLERS_H

#include <WebServer.h> // Для WebServer
#include "sensitive_config.h" // Для получения учетных данных веб-сервера

// Initialization
void initWebServerUtils(); // For CSRF token generation
void setupWebServerRoutes();

// Authentication
bool handleAuthentication(); // Declaration moved here

// Page Handlers
void handleRoot();
void handleSettings();
void handleDiagnostics();
void handleDownloadLog();

// Action Handlers
void handleUpdateConfig();
void handleStartDosing();
void handleStopDosing();
void handleStartCalibration();
void handleStopCalibration();
void handleResetStatsWeb();
void handleClearErrorWeb();
void handleStopFlowCalibration(); // Было пропущено в предыдущих изменениях, добавляем для полноты

void handleCalibrateMotorFwd();
void handleCalibrateMotorRev();
void handleCalibrateMotorStop();

void handleFactoryReset();
void handleEmergencyStop();

// Specific handler for /powerToggle, as it's simple and might be called directly
void handlePowerToggleWeb();
void handleWifiSetup(); // Новый обработчик для настройки WiFi

// AP Mode Handlers
void handleApConfigPage();      // Страница для ввода SSID/пароля в режиме AP
void handleApSaveWifiCredentials(); // Обработка POST-запроса с новыми учетными данными WiFi из AP
void handleApSimplePage();      // Test simple page for AP mode

// Helper (can be static in .c if not needed elsewhere, but good to be aware of)
// String get_csrf_input_field(); // Made static in .c
// void generate_new_csrf_token(); // Made static in .c, called by initWebServerUtils

#endif // WEB_SERVER_HANDLERS_H