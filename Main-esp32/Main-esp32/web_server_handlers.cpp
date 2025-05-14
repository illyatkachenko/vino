#include "web_server_handlers.h"
#include "main.h" // For server, system_power_enabled and other global vars/functions
#include <LittleFS.h> // <-- ДОБАВЛЕНО: Для работы с файловой системой
#include <WiFi.h> // For WiFi.status(), WiFi.localIP(), WiFi.macAddress(), WiFi.scanNetworks(), WL_CONNECTED etc.
// Also for log_x functions, getUptimeString, toggleSystemPower
#include "config_manager.h"
#include "error_handler.h"
#include "dosing_logic.h"
#include "calibration_logic.h"
#include "esp_now_handler.h" // <-- ДОБАВЛЕНО: Для isEspNowPeerAvailable и getRemotePeerAddress
#include "pid_controller.h"
#include "sensors.h"
#include "motor_control.h"
#include "utils.h" // Для getUptimeString
#include "sensitive_config.h" // <-- ДОБАВЛЕНО: Для получения учетных данных веб-сервера
#include "localization.h"   // <-- ДОБАВЛЕНО: Для локализации

// WebServer server; // Defined in main.c and extern in main.h (included above)

// Имя лог-файла (должно совпадать с тем, что в Main-esp32.ino)
const char* LOG_FILENAME_WEB_HANDLER = "/system_log.txt";

// --- HTML Templates ---
const char html_start[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Панель управления</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }
        h1, h2, h3 { color: #0056b3; }
        .container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
        .status-item { background-color: #e9ecef; padding: 10px; margin-bottom: 8px; border-radius: 4px; }
        .status-item strong { color: #0056b3; }
        .error-section { border: 2px solid var(--error-red, #dc3545); padding: 15px; margin-top: 15px; border-radius: 4px; background-color: #f8d7da; }
        .error-section h2 { color: var(--error-red, #721c24); }
        .action-buttons a, .action-buttons input[type="submit"], .button-link-small {
            display: inline-block; text-decoration: none; background-color: #007bff; color: white;
            padding: 10px 15px; margin: 5px; border-radius: 5px; border: none; cursor: pointer;
        }
        .action-buttons a:hover, .action-buttons input[type="submit"]:hover, .button-link-small:hover { background-color: #0056b3; }
        .button-link-small { padding: 5px 10px; font-size: 0.9em; }
        .form-group { margin-bottom: 15px; }
        .form-group label { display: block; margin-bottom: 5px; font-weight: bold; }
        .form-group input[type="number"], .form-group input[type="text"] {
            width: calc(100% - 22px); padding: 10px; border: 1px solid #ccc; border-radius: 4px;
        }
        :root { --error-red: #dc3545; --warn-orange: #ffc107; --ok-green: #28a745; }
    </style>
</head>
<body><div class="container">
)rawliteral";

const char html_end[] PROGMEM = R"rawliteral(
</div></body></html>
)rawliteral";

// --- CSRF Token Generation (move from main.c) ---
String csrf_token_value = "";

static String generate_csrf_token_internal() { // Now static
    char token[33]; // 32 hex chars + null terminator
    for (int i = 0; i < 32; i++) {
        sprintf(token + i, "%x", esp_random() % 16);
    }
    token[32] = '\0';
    return String(token);
}

// Helper function to escape HTML attribute values
static String htmlEscapeAttribute(const String& str) {
    String escapedStr = str;
    escapedStr.replace("\"", "&quot;"); // Escape double quotes
    // Add other necessary escapes if needed, e.g., for single quotes, ampersands
    return escapedStr;
}

// Helper function to escape HTML content
static String htmlEscapeContent(const String& str) {
    String escapedStr = str;
    escapedStr.replace("&", "&amp;");
    escapedStr.replace("<", "&lt;");
    escapedStr.replace(">", "&gt;");
    return escapedStr;
}

static String get_csrf_input_field() { // Now static
    // Важно: не кодируем HTML здесь, так как WebServer::sendContent ожидает обычный HTML
    return "<input type='hidden' name='csrf_token' value='" + csrf_token_value + "'>";
}

// --- Authentication (move from main.c) ---
bool handleAuthentication() { // Declaration moved to .h
    if (!server.authenticate(getWebUsername(), getWebPassword())) { // <-- Используем геттеры
        server.requestAuthentication();
        return false;
    }
    return true;
}

// --- Helper functions for request handling ---
static bool preCheckPost() {
    if (!handleAuthentication()) return false;
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return false;
    }
    if (!server.hasArg("csrf_token") || server.arg("csrf_token") != csrf_token_value) {
        server.send(403, "text/plain", "CSRF token mismatch.");
        return false;
    }
    return true;
}

static void sendRedirect(const String& uri) {
    server.sendHeader("Location", uri);
    server.send(303);
}

static void beginHtmlResponse() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", ""); // Отправляем заголовки и начало HTML
    server.sendContent(html_start);
}

static void endHtmlResponse() {
    server.sendContent(html_end);
    server.sendContent(""); // Завершаем передачу
}

void initWebServerUtils() {
    csrf_token_value = generate_csrf_token_internal();
    app_log_i("WEB_UTILS", "CSRF Token generated: %s", csrf_token_value.c_str()); // Changed to app_log_i
}

void handleRoot() {
    if (!handleAuthentication()) return;
    beginHtmlResponse();
    char buffer[200]; 

    // Локальные копии для потокобезопасного чтения
    DosingState local_dosing_state;
    SystemErrorCode local_current_system_error;
    char local_last_error_msg[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE];
    unsigned long local_last_error_time;
    bool local_cal_mode;
    long local_cal_steps;

    portENTER_CRITICAL(&dosing_state_mutex);
    local_dosing_state = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    portENTER_CRITICAL(&error_handler_mutex); 
    local_current_system_error = current_system_error;
    strncpy(local_last_error_msg, last_error_msg_buffer_internal, sizeof(local_last_error_msg) -1);
    local_last_error_msg[sizeof(local_last_error_msg)-1] = '\0';
    local_last_error_time = last_error_time;
    portEXIT_CRITICAL(&error_handler_mutex);

    server.sendContent_P(PSTR("<h1>")); server.sendContent(_T(L_SYSTEM_STATUS)); server.sendContent_P(PSTR("</h1>"));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong> <a href='/powerToggle' class='button-link-small'>%s</a></p>",
             _T(L_POWER_STATUS), system_power_enabled ? _T(L_POWER_ON) : _T(L_POWER_OFF), system_power_enabled ? _T(L_POWER_OFF) : _T(L_POWER_ON)); // Кнопка должна показывать противоположное действие
    server.sendContent(buffer);

    const char* dosing_state_str = _T(L_DOSING_STATE_UNKNOWN); // По умолчанию
    switch (local_dosing_state) {
        case DOSING_STATE_IDLE: dosing_state_str = "Ожидание (IDLE)"; break;
        case DOSING_STATE_REQUESTED: dosing_state_str = "Запрос (REQUESTED)"; break;
        case DOSING_STATE_PRE_COOLING: dosing_state_str = "Предв. охлаждение (PRE_COOLING)"; break;
        case DOSING_STATE_STARTING: dosing_state_str = "Запуск (STARTING)"; break;
        case DOSING_STATE_RUNNING: dosing_state_str = "Выполнение (RUNNING)"; break;
        case DOSING_STATE_STOPPING: dosing_state_str = "Остановка (STOPPING)"; break;
        case DOSING_STATE_PAUSED: dosing_state_str = "Пауза (PAUSED)"; break;
        case DOSING_STATE_FINISHED: dosing_state_str = "Завершено (FINISHED)"; break;
        case DOSING_STATE_ERROR: dosing_state_str = "Ошибка дозирования (ERROR)"; break;
    }
    // Используем _T для состояний, если они добавлены в LangKey
    dosing_state_str = _T((LangKey)(L_DOSING_STATE_IDLE + local_dosing_state));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s (%d)</strong></p>", _T(L_DOSING_STATUS), dosing_state_str, local_dosing_state);
    server.sendContent(buffer);

    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s (сырая): <strong>%.2f °C</strong></p>", _T(L_TEMP_OUT), tOut); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.2f °C</strong></p>", _T(L_TEMP_OUT_FILTERED), tOut_filtered); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.2f °C</strong></p>", _T(L_TEMP_IN), tIn); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.2f °C</strong></p>", _T(L_TEMP_COOLER), tCool); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_COMPRESSOR_STATUS), compressorRunning ? _T(L_COMPRESSOR_ON) : _T(L_COMPRESSOR_OFF)); server.sendContent(buffer);

    local_cal_mode = getCalibrationModeState(); 
    if (local_cal_mode) {
        portENTER_CRITICAL(&motor_cal_steps_mutex);
        local_cal_steps = steps_taken_calibration;
        portEXIT_CRITICAL(&motor_cal_steps_mutex);
        snprintf(buffer, sizeof(buffer), "<p class='status-item' style='color: orange;'>%s. %s: <strong>%ld</strong></p>", _T(L_CALIBRATION_MODE_ACTIVE_MSG), _T(L_STEPS_ACCUMULATED_MSG), local_cal_steps);
        server.sendContent(buffer);
    } else {
        server.sendContent("<p class='status-item'>Режим калибровки: <strong>Выключен</strong></p>");
    }

    if (local_current_system_error != NO_ERROR) {
        server.sendContent("<div class='error-section'>");
        server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_SYSTEM_ERROR_TITLE)); server.sendContent_P(PSTR("</h2>"));
        snprintf(buffer, sizeof(buffer), "<p>%s: <strong>%d</strong></p>", _T(L_ERROR_CODE_LABEL), local_current_system_error); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<p>%s: <strong>%s</strong></p>", _T(L_MESSAGE), local_last_error_msg); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<p>%s: %lu %s</p>", _T(L_ERROR_TIME_LABEL), (millis() - local_last_error_time) / 1000, _T(L_SECONDS_AGO_LABEL)); server.sendContent(buffer);
        server.sendContent("<form action='/clearError' method='POST' style='display:inline-block;'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'>", _T(L_CLEAR_ERROR)); server.sendContent(buffer);
        server.sendContent("</form></div>");
    } else {
        snprintf(buffer, sizeof(buffer), "<p class='status-item' style='color: var(--ok-green);'>%s: <strong>%s</strong></p>", _T(L_STATUS_LABEL), _T(L_OK_STATUS)); server.sendContent(buffer); // Нужны ключи
    }

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_CONTROL_AND_SETTINGS)); server.sendContent_P(PSTR("</h2><div class='action-buttons'>"));
    snprintf(buffer, sizeof(buffer), "<a href='/settings' class='button-link'>%s</a>", _T(L_SETTINGS)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<a href='/diagnostics' class='button-link'>%s</a>", _T(L_DIAGNOSTICS)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<a href='/dosingcontrol' class='button-link' style='background-color: var(--ok-green);'>%s</a>", _T(L_DOSING_CONTROL)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<a href='/errors' class='button-link' style='background-color: var(--warn-orange);'>%s</a>", _T(L_ERROR_LOG)); server.sendContent(buffer);

    server.sendContent("<form action='/settings' method='POST' style='display: inline-block;'>");
    server.sendContent(get_csrf_input_field());
    if (getIsPidTempControlEnabled()) { // getIsPidTempControlEnabled() уже потокобезопасна
        snprintf(buffer, sizeof(buffer), "<input type='hidden' name='pid_control' value='0'><input type='submit' value='%s' class='button-link' style='background-color: var(--warn-orange);'>", _T(L_DISABLE_PID_TEMP_CONTROL_BTN)); server.sendContent(buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "<input type='hidden' name='pid_control' value='1'><input type='submit' value='%s' class='button-link' style='background-color: var(--ok-green);'>", _T(L_ENABLE_PID_TEMP_CONTROL_BTN)); server.sendContent(buffer);
    }
    server.sendContent("</form>");

    server.sendContent("<form action='/resetStats' method='POST' style='display: inline-block;'>");
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s' class='button-link'></form>", _T(L_RESET_STATS)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<a href='/downloadlog' class='button-link'>%s</a>", _T(L_DOWNLOAD_LOG_BTN)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<a href='/emergency' class='button-link' style='background-color: var(--error-red);'>%s</a></div>", _T(L_EMERGENCY_STOP)); server.sendContent(buffer);

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_STATISTICS_TITLE)); server.sendContent_P(PSTR("</h2>"));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_UPTIME), getUptimeString().c_str()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%lu мл</strong></p>", _T(L_TOTAL_VOLUME_DOSED), config.totalVolumeDispensed); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.2f %s</strong></p>", _T(L_COMPRESSOR_RUNTIME_LABEL), config.compressorRunTime / (1000.0 * 60 * 60), _T(L_HOURS_UNIT)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%d</strong></p>", _T(L_COMPRESSOR_STARTS_LABEL), config.compressorStartCount); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%d</strong></p>", _T(L_TOTAL_DOSING_CYCLES), config.totalDosingCycles); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%d</strong></p>", _T(L_TOTAL_ERRORS), config.errorCount); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_LAST_ERROR_MESSAGE), config.lastErrorMsgBuffer); server.sendContent(buffer);

    endHtmlResponse();
}

void handleSettings() {
    if (!handleAuthentication()) return;
    beginHtmlResponse();

    char buffer[256];
    long local_cal_steps_settings;

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_SYSTEM_SETTINGS_TITLE)); server.sendContent_P(PSTR("</h2><form action='/settings' method='POST'>"));
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='tempSetpoint'>%s:</label><input type='number' id='tempSetpoint' name='tempSetpoint' step='0.1' value='%.1f'></div>", _T(L_TEMP_SETPOINT_DOSING), config.tempSetpoint); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='volumeTarget'>%s:</label><input type='number' id='volumeTarget' name='volumeTarget' value='%d'></div>", _T(L_TARGET_VOLUME), config.volumeTarget); server.sendContent(buffer); // Используем L_TARGET_VOLUME
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='motorSpeed'>%s:</label><input type='number' id='motorSpeed' name='motorSpeed' value='%d'></div>", _T(L_MOTOR_SPEED_DOSING), config.motorSpeed); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='flowMlPerPulse'>%s:</label><input type='number' id='flowMlPerPulse' name='flowMlPerPulse' step='0.0001' value='%.4f'></div>", _T(L_ML_PER_PULSE_FLOW_SENSOR_CAL_LABEL), config.flowMlPerPulse); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_SAVE)); server.sendContent(buffer);

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_DOSING_CONTROL_SETTINGS_TITLE)); server.sendContent_P(PSTR("</h2><form action='/startDosing' method='POST'>"));
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='dosingVolume'>%s:</label><input type='number' id='dosingVolume' name='dosingVolume' value='%d'></div>", _T(L_DOSING_VOLUME_ML_LABEL), config.volumeTarget); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_START_DOSING_BTN)); server.sendContent(buffer);
    server.sendContent("<form action='/stopDosing' method='POST' style='margin-top: 10px;'>");
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_STOP_DOSING)); server.sendContent(buffer);

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_CALIBRATION)); server.sendContent_P(PSTR("</h2>"));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.6f</strong></p>", _T(L_CURRENT_ML_PER_STEP_MOTOR_LABEL), config.mlPerStep); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.6f</strong></p>", _T(L_CURRENT_ML_PER_PULSE_FLOW_LABEL), config.flowMlPerPulse); server.sendContent(buffer);


    if (getCalibrationModeState()) { // getCalibrationModeState() потокобезопасна
        portENTER_CRITICAL(&motor_cal_steps_mutex);
        local_cal_steps_settings = steps_taken_calibration;
        portEXIT_CRITICAL(&motor_cal_steps_mutex);
        snprintf(buffer, sizeof(buffer), "<p class='status-item' style='color: orange;'>Режим калибровки активен. Накоплено шагов мотора: <strong>%ld</strong></p>", local_cal_steps_settings); server.sendContent(buffer);
        
        unsigned long local_flow_pulses_cal;
        portENTER_CRITICAL(&cal_pulse_mutex);
        local_flow_pulses_cal = flow_pulses_calibration;
        portEXIT_CRITICAL(&cal_pulse_mutex);
        snprintf(buffer, sizeof(buffer), "<p class='status-item' style='color: orange;'>%s: <strong>%lu</strong></p>", _T(L_ACCUMULATED_FLOW_PULSES_LABEL), local_flow_pulses_cal); server.sendContent(buffer);

        // Кнопки для ручного управления мотором в режиме калибровки
        server.sendContent_P(PSTR("<h4>")); server.sendContent(_T(L_MANUAL_MOTOR_CONTROL_CAL_MODE)); server.sendContent_P(PSTR("</h4>"));
        server.sendContent("<form action='/cal_manual_fwd' method='POST' style='display:inline-block;'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_MOTOR_FORWARD_CAL_BTN)); server.sendContent(buffer);
        server.sendContent("<form action='/cal_manual_rev' method='POST' style='display:inline-block; margin-left:10px;'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_MOTOR_REVERSE_CAL_BTN)); server.sendContent(buffer);
        server.sendContent("<form action='/cal_manual_stop' method='POST' style='display:inline-block; margin-left:10px;'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s' style='background-color: var(--warn-orange);'></form>", _T(L_MOTOR_STOP_CAL_BTN)); server.sendContent(buffer);
        server.sendContent("<br><br>");


        server.sendContent("<form action='/stopCalibration' method='POST'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='actualVolumeMeasured'>%s:</label><input type='number' id='actualVolumeMeasured' name='actualVolumeMeasured' step='0.1' required></div>", _T(L_ACTUAL_VOLUME_MEASURED)); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_FINISH_MOTOR_CALIBRATION_BTN)); server.sendContent(buffer);

        server.sendContent("<form action='/stopFlowCalibration' method='POST' style='margin-top:10px;'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='actualVolumeFlow'>%s:</label><input type='number' id='actualVolumeFlow' name='actualVolumeFlow' step='0.1' required></div>", _T(L_ACTUAL_VOLUME_FLOW_SENSOR)); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_FINISH_FLOW_CALIBRATION_BTN)); server.sendContent(buffer);
    } else {
        server.sendContent("<form action='/startCalibration' method='POST'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='targetCalibrationVolume'>%s:</label><input type='number' id='targetCalibrationVolume' name='targetCalibrationVolume' value='100'></div>", _T(L_TARGET_CALIBRATION_VOLUME_HINT_LABEL)); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_START_CALIBRATION_MOTOR_FLOW_BTN)); server.sendContent(buffer);
    }

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_PID_SETTINGS_TITLE_MOTOR_SPEED)); server.sendContent_P(PSTR("</h2>"));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong> (%s)</p>", _T(L_PID_TEMP_CONTROL_STATUS_LABEL), getIsPidTempControlEnabled() ? _T(L_ENABLED_STATUS) : _T(L_DISABLED_STATUS), _T(L_TOGGLE_ON_MAIN_PAGE_HINT)); server.sendContent(buffer);
    server.sendContent("<form action='/settings' method='POST'>"); 
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='pid_kp'>%s:</label><input type='number' id='pid_kp' name='pid_kp' step='0.1' value='%.2f'></div>", _T(L_PID_KP), getPidKp()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='pid_ki'>%s:</label><input type='number' id='pid_ki' name='pid_ki' step='0.01' value='%.2f'></div>", _T(L_PID_KI), getPidKi()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='pid_kd'>%s:</label><input type='number' id='pid_kd' name='pid_kd' step='0.1' value='%.2f'></div>", _T(L_PID_KD), getPidKd()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_SAVE_PID_COEFFS_BTN)); server.sendContent(buffer);

    // Секция выбора языка
    server.sendContent_P(PSTR("<hr><h2>")); server.sendContent(_T(L_LANGUAGE)); server.sendContent_P(PSTR("</h2>"));
    server.sendContent_P(PSTR("<form action='/settings' method='POST'>"));
    server.sendContent(get_csrf_input_field());
    server.sendContent_P(PSTR("<div class='form-group'><label for='language'>")); server.sendContent(_T(L_LANGUAGE)); server.sendContent_P(PSTR(":</label>"));
    server.sendContent_P(PSTR("<select id='language' name='language'>"));
    snprintf(buffer, sizeof(buffer), "<option value='ru' %s>%s</option>", strcmp(config.currentLanguage, "ru") == 0 ? "selected" : "", _T(L_RUSSIAN)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<option value='en' %s>%s</option>", strcmp(config.currentLanguage, "en") == 0 ? "selected" : "", _T(L_ENGLISH)); server.sendContent(buffer);
    server.sendContent_P(PSTR("</select></div>"));
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_SAVE)); server.sendContent(buffer);

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_FACTORY_RESET)); server.sendContent_P(PSTR("</h2>"));
    snprintf(buffer, sizeof(buffer), "<form action='/factoryReset' method='POST' onsubmit='return confirm(\"%s\");'>", _T(L_ARE_YOU_SURE_FACTORY_RESET_CONFIRM_MSG)); server.sendContent(buffer);
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s' style='background-color: var(--error-red);'></form>", _T(L_PERFORM_FACTORY_RESET)); server.sendContent(buffer);

    server.sendContent("<div class='action-buttons' style='margin-top: 25px;'>");
    snprintf(buffer, sizeof(buffer), "<a href='/' class='button-link'>%s</a>", _T(L_BACK_TO_STATUS)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<a href='/diagnostics' class='button-link'>%s</a></div>", _T(L_DIAGNOSTICS)); server.sendContent(buffer);

    endHtmlResponse();
}

void handleStartDosing() {
    if (!preCheckPost()) return;

    if (server.hasArg("dosingVolume")) {
        int volume = server.arg("dosingVolume").toInt();
        if (volume > 0 && volume <= 10000) {
            startDosingCycle(volume, true);
            sendRedirect("/");
        } else {
            server.send(400, "text/plain", "Invalid dosing volume. Must be between 1 and 10000 ml.");
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid dosing volume via web");
        }
    } else {
        server.send(400, "text/plain", "Missing dosingVolume parameter.");
        setSystemError(INPUT_VALIDATION_ERROR, "Missing dosing volume parameter");
    }
}

void handleStopDosing() {
    if (!preCheckPost()) return;
    stopDosingCycle(true);
    sendRedirect("/");
}

void handleStartCalibration() {
    if (!preCheckPost()) return;

    float targetVol = 100.0f;
    if (server.hasArg("targetCalibrationVolume")) {
        targetVol = server.arg("targetCalibrationVolume").toFloat();
        if (targetVol <= 0 || targetVol > 1000 || isnan(targetVol)) {
            server.send(400, "text/plain", "Invalid calibration target volume. Must be positive and reasonable.");
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid calibration target volume via web");
            return;
        }
    }
    startCalibrationMode(targetVol, true);
    sendRedirect("/settings");
}

void handleStopCalibration() {
    if (!preCheckPost()) return;

    if (server.hasArg("actualVolumeMeasured")) {
        String actualVolumeStr = server.arg("actualVolumeMeasured");
        if (actualVolumeStr.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for decimal numbers, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in float for calibration");
            return;
        }
        float actualVolume = actualVolumeStr.toFloat();

        if (actualVolume > 0 && actualVolume < 10000 && !isnan(actualVolume)) {
            stopCalibrationMode(actualVolume, true);
            sendRedirect("/settings");
        } else {
            server.send(400, "text/plain", "Invalid actual volume measured. Must be positive and reasonable.");
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid actual volume for calibration via web");
        }
    } else {
        server.send(400, "text/plain", "Missing actualVolumeMeasured parameter.");
        setSystemError(INPUT_VALIDATION_ERROR, "Missing actual volume parameter for calibration");
    }
}

void handleStopFlowCalibration() { 
    if (!preCheckPost()) return;

    if (server.hasArg("actualVolumeFlow")) {
        String actualVolumeStr = server.arg("actualVolumeFlow");
        if (actualVolumeStr.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for decimal numbers, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in float for flow calibration");
            return;
        }
        float actualVolume = actualVolumeStr.toFloat();

        if (actualVolume > 0 && actualVolume < 10000 && !isnan(actualVolume)) {
            stopFlowCalibrationMode(actualVolume, true); 
            sendRedirect("/settings");
        } else {
            server.send(400, "text/plain", "Invalid actual volume measured for flow. Must be positive and reasonable.");
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid actual volume for flow calibration via web");
        }
    } else {
        server.send(400, "text/plain", "Missing actualVolumeFlow parameter.");
        setSystemError(INPUT_VALIDATION_ERROR, "Missing actual volume parameter for flow calibration");
    }
}

void handleCalibrateMotorFwd() {
    if (!preCheckPost()) return;

    if (getCalibrationModeState()) {
        manualMotorForward();
        log_i("WEB_CAL", "Manual motor forward initiated for calibration.");
    } else {
        log_w("WEB_CAL", "Attempted manual motor forward, but calibration mode is not active.");
    }
    sendRedirect("/settings");
}

void handleCalibrateMotorRev() {
    if (!preCheckPost()) return;

    if (getCalibrationModeState()) {
        manualMotorReverse();
        log_i("WEB_CAL", "Manual motor reverse initiated for calibration.");
    } else {
        log_w("WEB_CAL", "Attempted manual motor reverse, but calibration mode is not active.");
    }
    sendRedirect("/settings");
}

void handleCalibrateMotorStop() {
    if (!preCheckPost()) return;

    // stopManualMotor() безопасна для вызова, даже если мотор не в ручном режиме или калибровка не активна.
    // Она просто ничего не сделает, если motor_running_manual == false.
    stopManualMotor(); 
    log_i("WEB_CAL", "Manual motor stop initiated for calibration.");
    sendRedirect("/settings");
}

void handleResetStatsWeb() {
    if (!preCheckPost()) return;
    resetStats();
    sendRedirect("/");
}

void handleClearErrorWeb() {
    if (!preCheckPost()) return;
    clearSystemError();
    sendRedirect("/");
}

void handleUpdateConfig() {
    if (!preCheckPost()) return;

    bool config_changed = false;

    if (server.hasArg("tempSetpoint")) {
        String tempArg = server.arg("tempSetpoint");
        if (tempArg.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for decimal numbers, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in TempSetpoint float");
            return;
        }
        float newTemp = tempArg.toFloat();
        if (newTemp >= -10.0f && newTemp <= 30.0f && !isnan(newTemp)) {
            if (fabs(config.tempSetpoint - newTemp) > 0.01f) {
                config.tempSetpoint = newTemp;
                config_changed = true;
                log_i("WEB", "Temp Setpoint updated to: %.1f C", config.tempSetpoint);
            }
        } else {
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid Temp Setpoint via web");
            server.send(400, "text/plain", "Invalid temperature setpoint. Must be between -10 and 30.");
            return;
        }
    }
    if (server.hasArg("volumeTarget")) {
        int newVol = server.arg("volumeTarget").toInt();
        if (newVol > 0 && newVol <= 10000) {
            if (config.volumeTarget != newVol) {
                config.volumeTarget = newVol;
                config_changed = true;
                log_i("WEB", "Volume Target updated to: %d ml", config.volumeTarget);
            }
        } else {
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid Volume Target via web");
            server.send(400, "text/plain", "Invalid volume target. Must be between 1 and 10000 ml.");
            return;
        }
    }
    if (server.hasArg("motorSpeed")) {
        int newSpeed = server.arg("motorSpeed").toInt();
        if (newSpeed >= 0 && newSpeed <= 2000) {
            if (config.motorSpeed != newSpeed) {
                updateMotorSpeed(newSpeed); // Эта функция обновляет config.motorSpeed
                config_changed = true;
                log_i("WEB", "Motor Speed updated to: %d steps/sec", config.motorSpeed);
            }
        } else {
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid Motor Speed via web");
            server.send(400, "text/plain", "Invalid motor speed. Must be between 0 and 2000 steps/sec.");
            return;
        }
    }
    if (server.hasArg("flowMlPerPulse")) {
        String flowArg = server.arg("flowMlPerPulse");
         if (flowArg.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for decimal numbers in flowMlPerPulse, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in flowMlPerPulse float");
            return;
        }
        float newFlowCal = flowArg.toFloat();
        if (newFlowCal > 0.000001f && newFlowCal < 10.0f && !isnan(newFlowCal)) { // Диапазон можно уточнить
            if (fabs(config.flowMlPerPulse - newFlowCal) > 0.0000001f) {
                config.flowMlPerPulse = newFlowCal;
                config_changed = true;
                log_i("WEB", "Flow Sensor ml/Pulse updated to: %.6f", config.flowMlPerPulse);
            }
        } else {
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid Flow ml/Pulse via web");
            server.send(400, "text/plain", "Invalid Flow ml/Pulse. Must be positive and reasonable.");
            return;
        }
    }

    if (server.hasArg("pid_control")) {
        bool new_pid_state = server.arg("pid_control").toInt() == 1;
        if (getIsPidTempControlEnabled() != new_pid_state) {
            enablePidTempControl(new_pid_state);
            log_i("WEB", "PID Temperature Control %s", getIsPidTempControlEnabled() ? "ENABLED" : "DISABLED");
            if (!getIsPidTempControlEnabled()) {
                updateMotorSpeed(config.motorSpeed);
            }
        }
    }

    bool pid_coeffs_changed = false;
    float new_pid_kp = getPidKp(); // getPidKp() потокобезопасна
    float new_pid_ki = getPidKi(); // getPidKi() потокобезопасна
    float new_pid_kd = getPidKd(); // getPidKd() потокобезопасна

    if (server.hasArg("pid_kp")) { 
        String kp_str = server.arg("pid_kp");
        if (kp_str.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for Kp, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in Kp float");
            return;
        }
        new_pid_kp = kp_str.toFloat(); 
        pid_coeffs_changed = true; 
    }
    if (server.hasArg("pid_ki")) { 
        String ki_str = server.arg("pid_ki");
        if (ki_str.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for Ki, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in Ki float");
            return;
        }
        new_pid_ki = ki_str.toFloat(); 
        pid_coeffs_changed = true; 
    }
    if (server.hasArg("pid_kd")) { 
        String kd_str = server.arg("pid_kd");
        if (kd_str.indexOf(',') != -1) {
            server.send(400, "text/plain", "Use dot (.) for Kd, not comma (,).");
            setSystemError(INPUT_VALIDATION_ERROR, "Comma in Kd float");
            return;
        }
        new_pid_kd = kd_str.toFloat(); 
        pid_coeffs_changed = true; 
    }
    
    if (pid_coeffs_changed) {
        // Валидация PID коэффициентов
        // Убедимся, что если отправлен только один коэффициент, остальные сохраняют свои старые значения из getPidKx()
        if (isnan(new_pid_kp) || isnan(new_pid_ki) || isnan(new_pid_kd) || 
            new_pid_kp < 0 || new_pid_ki < 0 || new_pid_kd < 0 ||
            new_pid_kp > 1000 || new_pid_ki > 1000 || new_pid_kd > 1000) { // Примерные пределы
            setSystemError(INPUT_VALIDATION_ERROR, "Invalid PID coefficients via web");
            server.send(400, "text/plain", "Invalid PID coefficients. Must be positive and reasonable.");
            return;
        }
        setPidCoefficients(new_pid_kp, new_pid_ki, new_pid_kd); // Эта функция обновляет config и сохраняет
        log_i("WEB", "PID Coefficients updated via web: Kp=%.2f, Ki=%.2f, Kd=%.2f", new_pid_kp, new_pid_ki, new_pid_kd);
        // config_changed будет true, если setPidCoefficients сохранил config
    }

    // Обработка изменения языка
    if (server.hasArg("language")) {
        String new_lang = server.arg("language");
        if (new_lang == "ru" || new_lang == "en") {
            if (strcmp(config.currentLanguage, new_lang.c_str()) != 0) {
                strncpy(config.currentLanguage, new_lang.c_str(), sizeof(config.currentLanguage) - 1);
                config.currentLanguage[sizeof(config.currentLanguage) - 1] = '\0';
                set_current_language(config.currentLanguage); // Обновляем активный язык в системе локализации
                config_changed = true; // Отмечаем, что конфигурация изменилась
            }
        }
    }

    if (config_changed || pid_coeffs_changed) { // Если изменились PID или другие настройки
        saveConfig();
    }
    sendRedirect("/settings");
}

void handleEmergencyStop() {
    log_w("SYSTEM", "Emergency stop initiated from web!");
    stopMotor();
    compressorOff();
    if (getCalibrationModeState()) {
        log_w("SYSTEM", "Emergency stop during active calibration. Stopping calibration.");
        setCalibrationModeState(false); 
    }
    
    DosingState local_dosing_state_emergency;
    portENTER_CRITICAL(&dosing_state_mutex);
    local_dosing_state_emergency = current_dosing_state;
    portEXIT_CRITICAL(&dosing_state_mutex);

    if (local_dosing_state_emergency != DOSING_STATE_IDLE &&
        local_dosing_state_emergency != DOSING_STATE_FINISHED &&
        local_dosing_state_emergency != DOSING_STATE_ERROR) {
        stopDosingCycle(true); 
    }
    setSystemError(CRIT_MOTOR_FAIL, "EMERGENCY STOP: Motor/Compressor disabled via web!");
    sendRedirect("/");
}

void handleFactoryReset() {
    if (!handleAuthentication()) return;
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }
    if (!server.hasArg("csrf_token") || server.arg("csrf_token") != csrf_token_value) {
        server.send(403, "text/plain", "CSRF token mismatch.");
        return;
    }

    log_w("SYSTEM", "Factory reset initiated via web interface!");
    performFactoryReset();

    server.send(200, "text/plain", "Factory reset successful. System will restart in 5 seconds.");
    delay(5000);
    ESP.restart();
}

void handleDiagnostics() {
    if (!handleAuthentication()) return;
    beginHtmlResponse();

    char buffer[300];

    // Локальные копии для диагностики
    DosingState local_diag_dosing_state;
    unsigned long local_diag_dosing_state_start_time;
    float local_diag_volume_dispensed_cycle;
    bool local_motor_auto, local_motor_manual;
    float local_current_steps_sec;
    unsigned long local_step_interval_us;
    int local_motor_dir;
    long local_motor_cal_steps;
    unsigned long local_flow_pulses_cal_diag;
    SystemErrorCode local_diag_current_system_error;
    char local_diag_last_error_msg[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE];
    unsigned long local_diag_last_error_time;


    portENTER_CRITICAL(&dosing_state_mutex);
    local_diag_dosing_state = current_dosing_state;
    local_diag_dosing_state_start_time = dosing_state_start_time;
    portEXIT_CRITICAL(&dosing_state_mutex);

    portENTER_CRITICAL(&volume_dispensed_mutex);
    local_diag_volume_dispensed_cycle = volume_dispensed_cycle;
    portEXIT_CRITICAL(&volume_dispensed_mutex);

    portENTER_CRITICAL(&motor_state_mutex);
    local_motor_auto = motor_running_auto;
    local_motor_manual = motor_running_manual;
    local_current_steps_sec = current_steps_per_sec;
    local_step_interval_us = step_interval_us;
    local_motor_dir = motor_dir; 
    portEXIT_CRITICAL(&motor_state_mutex);

    portENTER_CRITICAL(&motor_cal_steps_mutex); 
    local_motor_cal_steps = steps_taken_calibration; 
    portEXIT_CRITICAL(&motor_cal_steps_mutex);
    
    portENTER_CRITICAL(&cal_pulse_mutex); 
    local_flow_pulses_cal_diag = flow_pulses_calibration; 
    portEXIT_CRITICAL(&cal_pulse_mutex);

    portENTER_CRITICAL(&error_handler_mutex);
    local_diag_current_system_error = current_system_error;
    strncpy(local_diag_last_error_msg, last_error_msg_buffer_internal, sizeof(local_diag_last_error_msg) -1);
    local_diag_last_error_msg[sizeof(local_diag_last_error_msg)-1] = '\0';
    local_diag_last_error_time = last_error_time;
    portEXIT_CRITICAL(&error_handler_mutex);

    server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_DIAGNOSTICS_INFO_TITLE)); server.sendContent_P(PSTR("</h2>"));

    server.sendContent_P(PSTR("<h3>")); server.sendContent(_T(L_GENERAL_STATUS_TITLE)); server.sendContent_P(PSTR("</h3>"));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_POWER_STATUS), system_power_enabled ? _T(L_POWER_ON) : _T(L_POWER_OFF)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_UPTIME), getUptimeString().c_str()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%u %s</strong></p>", _T(L_FREE_HEAP), ESP.getFreeHeap(), _T(L_BYTES_UNIT)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>CSRF токен: <strong>%s</strong></p>", csrf_token_value.c_str()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>WiFi Статус: <strong>%s</strong> (IP: %s)</p>", WiFi.status() == WL_CONNECTED ? "Подключено" : "Отключено", WiFi.localIP().toString().c_str()); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>WiFi MAC: <strong>%s</strong></p>", WiFi.macAddress().c_str()); server.sendContent(buffer);

    server.sendContent("<h3>Состояние дозирования (Конечный автомат)</h3>");
    const char* dosing_state_str_diag = "Неизвестно";
    switch (local_diag_dosing_state) {
        case DOSING_STATE_IDLE: dosing_state_str_diag = "Ожидание (IDLE)"; break;
        case DOSING_STATE_REQUESTED: dosing_state_str_diag = "Запрос (REQUESTED)"; break;
        // ... (все остальные состояния)
        case DOSING_STATE_PRE_COOLING: dosing_state_str_diag = "Предв. охлаждение (PRE_COOLING)"; break;
        case DOSING_STATE_STARTING: dosing_state_str_diag = "Запуск (STARTING)"; break;
        case DOSING_STATE_RUNNING: dosing_state_str_diag = "Выполнение (RUNNING)"; break;
        case DOSING_STATE_STOPPING: dosing_state_str_diag = "Остановка (STOPPING)"; break;
        case DOSING_STATE_PAUSED: dosing_state_str_diag = "Пауза (PAUSED)"; break;
        case DOSING_STATE_FINISHED: dosing_state_str_diag = "Завершено (FINISHED)"; break;
        case DOSING_STATE_ERROR: dosing_state_str_diag = "Ошибка дозирования (ERROR)"; break;
    }
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Текущее состояние: <strong>%s (%d)</strong></p>", dosing_state_str_diag, local_diag_dosing_state); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Время в текущем состоянии: <strong>%lu мс</strong></p>", millis() - local_diag_dosing_state_start_time); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Объем в цикле (по датчику потока): <strong>%.2f мл</strong></p>", local_diag_volume_dispensed_cycle); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Целевой объем: <strong>%d мл</strong></p>", config.volumeTarget); server.sendContent(buffer);

    server.sendContent("<h3>Датчики температуры</h3>");
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>T_вход: <strong>%.2f °C</strong> (Найден: %s, Ошибок подряд: %d)</p>", tIn, tempInSensorFound ? "Да" : "Нет", consecutive_temp_in_errors); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>T_охладителя: <strong>%.2f °C</strong> (Найден: %s, Ошибок подряд: %d)</p>", tCool, tempCoolerSensorFound ? "Да" : "Нет", consecutive_temp_cooler_errors); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>T_выход (фильтр.): <strong>%.2f °C</strong> (Найден: %s, Ошибок подряд: %d)</p>", tOut_filtered, tempOutSensorFound ? "Да" : "Нет", consecutive_temp_out_errors); server.sendContent(buffer);

    server.sendContent("<h3>Датчик потока</h3>");
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Текущий расход (расчетный): <strong>%.2f мл/мин</strong></p>", current_flow_rate_ml_per_min); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Калибровка датчика потока (мл/импульс): <strong>%.6f</strong></p>", config.flowMlPerPulse); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Проверка отсутствия потока: <strong>%s</strong> (Таймаут с: %lu мс, если активна)</p>", checking_for_flow ? "Активна" : "Неактивна", motor_start_time_with_no_flow); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Импульсов датчика потока (в режиме калибровки): <strong>%lu</strong></p>", local_flow_pulses_cal_diag); server.sendContent(buffer);

    server.sendContent("<h3>Мотор</h3>");
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Работает (авто): <strong>%s</strong></p>", local_motor_auto ? "Да" : "Нет"); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Работает (ручной): <strong>%s</strong></p>", local_motor_manual ? "Да" : "Нет"); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Включен (ENABLE_PIN): <strong>%s</strong></p>", isMotorEnabled() ? "Да (LOW)" : "Нет (HIGH)"); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Направление (motor_dir): <strong>%s</strong></p>", local_motor_dir == HIGH ? "Вперед (HIGH)" : "Назад (LOW)"); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Скорость (шаг/сек): &lt;strong&gt;%.1f&lt;/strong&gt; (Интервал: %lu мкс)&lt;/p&gt;", local_current_steps_sec, local_step_interval_us); server.sendContent(buffer); // Исправлено &lt; и &gt;
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Шагов в цикле дозирования: &lt;strong&gt;%ld&lt;/strong&gt; (Цель: %ld)&lt;/p&gt;", steps_taken_dosing, steps_target_dosing); server.sendContent(buffer); // Исправлено &lt; и &gt;
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Шагов мотора (в режиме калибровки): &lt;strong&gt;%ld&lt;/strong&gt;&lt;/p&gt;", local_motor_cal_steps); server.sendContent(buffer); // Исправлено &lt; и &gt;
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Скорость (шаг/сек): <strong>%.1f</strong> (Интервал: %lu мкс)</p>", local_current_steps_sec, local_step_interval_us); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Шагов в цикле дозирования: <strong>%ld</strong> (Цель: %ld)</p>", steps_taken_dosing, steps_target_dosing); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Шагов мотора (в режиме калибровки): <strong>%ld</strong></p>", local_motor_cal_steps); server.sendContent(buffer);

    server.sendContent("<h3>ESP-NOW</h3>");
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Пир добавлен: <strong>%s</strong></p>", isEspNowPeerAvailable() ? "Да" : "Нет"); server.sendContent(buffer); 
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>MAC адрес пира (из Config): <strong>%s</strong></p>", config.remotePeerMacStr); server.sendContent(buffer);
    uint8_t current_peer_addr[6]; getRemotePeerAddress(current_peer_addr);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>MAC адрес пира (актуальный): <strong>%02X:%02X:%02X:%02X:%02X:%02X</strong></p>", current_peer_addr[0], current_peer_addr[1], current_peer_addr[2], current_peer_addr[3], current_peer_addr[4], current_peer_addr[5]); server.sendContent(buffer);

    server.sendContent("<h3>Системные ошибки</h3>");
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Текущая ошибка (код): <strong>%d</strong></p>", local_diag_current_system_error); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Сообщение: <strong>%s</strong></p>", local_diag_last_error_msg); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Время последней ошибки: <strong>%lu мс</strong> (назад: %lu с)</p>", local_diag_last_error_time, (millis() - local_diag_last_error_time)/1000); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>Всего ошибок (счетчик): <strong>%d</strong></p>", config.errorCount); server.sendContent(buffer);

    server.sendContent("<div class='action-buttons' style='margin-top: 25px;'><a href='/' class='button-link'>Вернуться к статусу</a></div>");
    endHtmlResponse();
}

void handleDownloadLog() {
    if (!handleAuthentication()) return;

    if (!LittleFS.exists(LOG_FILENAME_WEB_HANDLER)) {
        app_log_w("FS_LOG", "Log file %s not found for download.", LOG_FILENAME_WEB_HANDLER);
        server.send(404, "text/plain", "Log file not found.");
        return;
    }

    File logFile = LittleFS.open(LOG_FILENAME_WEB_HANDLER, "r");
    if (!logFile) {
        app_log_e("FS_LOG", "Failed to open log file %s for download.", LOG_FILENAME_WEB_HANDLER);
        server.send(500, "text/plain", "Error opening log file.");
        return;
    }

    server.sendHeader("Content-Disposition", "attachment; filename=\"system_log.txt\"");
    // server.sendHeader("Content-Type", "text/plain"); // streamFile устанавливает это автоматически
    size_t sent = server.streamFile(logFile, "text/plain");
    logFile.close();
    
    // Логируем успешную отправку (если это не переполнит сам лог-файл слишком быстро)
    // Можно закомментировать, если лог отправки лога не нужен
    app_log_i("FS_LOG", "Log file %s sent, %d bytes.", LOG_FILENAME_WEB_HANDLER, sent);
}

void handlePowerToggleWeb() {
    if (!handleAuthentication()) return;
    toggleSystemPower(true); // true indicates it's from the web for redirection
}

void handleWifiSetup() {
    if (!handleAuthentication()) return;

    if (server.method() == HTTP_POST) {
        if (!preCheckPost()) return; // preCheckPost включает проверку CSRF

        String new_ssid = server.arg("wifi_ssid");
        String new_pass = server.arg("wifi_password");

        if (new_ssid.length() == 0 || new_ssid.length() > MAX_SSID_LEN) {
            server.send(400, "text/plain", "SSID не может быть пустым и должен быть не длиннее 32 символов.");
            return;
        }
        if (new_pass.length() > MAX_PASSWORD_LEN) {
            server.send(400, "text/plain", "Пароль должен быть не длиннее 64 символов.");
            return;
        }

        setWifiSsid(new_ssid.c_str());
        if (new_pass.length() > 0) { // Обновляем пароль, только если он введен
            setWifiPassword(new_pass.c_str());
        }
        saveSensitiveConfig();

        String response_msg = "Настройки WiFi обновлены. Система перезагрузится через 5 секунд для применения изменений.";
        server.send(200, "text/plain; charset=UTF-8", response_msg);
        log_i("WEB_WIFI", "WiFi settings updated via web. SSID: %s. Restarting in 5s.", new_ssid.c_str());
        delay(5000);
        ESP.restart();

    } else { // HTTP_GET
        beginHtmlResponse();
        char buffer[512];

        server.sendContent_P(PSTR("<h2>")); server.sendContent(_T(L_WIFI_SETUP_TITLE)); server.sendContent_P(PSTR("</h2>"));
        snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_CURRENT_WIFI_SSID), getWifiSsid()); server.sendContent(buffer);
        server.sendContent("<form action='/wifi_setup' method='POST'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='wifi_ssid'>%s:</label><input type='text' id='wifi_ssid' name='wifi_ssid' required></div>", _T(L_NEW_SSID)); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='wifi_password'>%s:</label><input type='password' id='wifi_password' name='wifi_password'></div>", _T(L_NEW_PASSWORD_LEAVE_EMPTY)); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_SAVE_AND_RESTART)); server.sendContent(buffer);
        snprintf(buffer, sizeof(buffer), "<div class='action-buttons' style='margin-top: 25px;'><a href='/settings' class='button-link'>%s</a></div>", _T(L_BACK_TO_SETTINGS)); server.sendContent(buffer);
        endHtmlResponse();
    }
}

void handleApConfigPage() {
    // Сначала исправляем HTML сущности на реальные символы
    beginHtmlResponse(); // Используем общий html_start
    char buffer[256]; // <-- ДОБАВЛЕНО: Объявление буфера
    server.sendContent_P(PSTR("<h1>")); server.sendContent(_T(L_AP_CONFIG_TITLE)); server.sendContent_P(PSTR("</h1>"));
    server.sendContent_P(PSTR("<p>")); server.sendContent(_T(L_AP_CONFIG_PROMPT)); server.sendContent_P(PSTR("</p>"));

    // Сканирование сетей WiFi
    log_i("AP_CONFIG", "Scanning WiFi networks...");
    int n = WiFi.scanNetworks();
    log_i("AP_CONFIG", "Scan done. Found %d networks.", n);
    char item_buffer[200]; // Объявляем item_buffer здесь, чтобы он был в области видимости для цикла

    server.sendContent_P(PSTR("<form action='/savewifi_ap' method='POST'>"));
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='ap_ssid'>%s</label><input type='text' id='ap_ssid' name='ap_ssid' required></div>", _T(L_AP_SSID_LABEL)); server.sendContent(buffer);

    if (n > 0) {
        snprintf(buffer, sizeof(buffer), "<div class='form-group'><label>%s</label><div id='wifi-scan-results' style='max-height: 200px; overflow-y: auto; border: 1px solid #ccc; padding: 10px; margin-bottom:10px;'>", _T(L_AP_SELECT_FROM_LIST)); server.sendContent(buffer);
        // Логирование найденных сетей в Serial Monitor
        for (int i = 0; i < n; ++i) {
            const char* ssid_raw = WiFi.SSID(i).c_str();
            log_i("AP_CONFIG", "Found network for display: %s, RSSI: %d", ssid_raw, WiFi.RSSI(i)); // Логируем перед экранированием
            String ssid_escaped_for_value = htmlEscapeAttribute(ssid_raw); // Экранируем SSID для атрибута value
            String ssid_escaped_for_display = htmlEscapeContent(ssid_raw); // Экранируем SSID для отображения
            // ... (остальной код для отображения сетей, также нужно перевести "Open", "Encrypted") ...
            
            // Используем JavaScript для автозаполнения поля SSID при выборе радио-кнопки
            snprintf(item_buffer, sizeof(item_buffer), 
                     "<label style='display:block; margin-bottom:5px;'><input type='radio' name='selected_ssid_radio' value='%s' onclick='document.getElementById(\"ap_ssid\").value=this.value;'> %s (%d dBm, %s)</label><br>",
                     ssid_escaped_for_value.c_str(), ssid_escaped_for_display.c_str(), WiFi.RSSI(i), (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? _T(L_AP_SECURITY_OPEN) : _T(L_AP_SECURITY_ENCRYPTED));
            server.sendContent(item_buffer);
        }
        server.sendContent("</div></div>");
    } else {
        server.sendContent_P(PSTR("<p>")); server.sendContent(_T(L_AP_NO_NETWORKS_FOUND)); server.sendContent_P(PSTR("</p>"));
    }
    // Очищаем результаты сканирования, чтобы освободить память
    WiFi.scanDelete();

    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='ap_password'>%s</label><input type='password' id='ap_password' name='ap_password'></div>", _T(L_AP_PASSWORD_LABEL)); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_AP_SAVE_AND_CONNECT_BTN)); server.sendContent(buffer);

    // Добавляем кнопку для повторного сканирования (просто перезагружает страницу)
    snprintf(buffer, sizeof(buffer), "<div class='action-buttons' style='margin-top: 15px;'><a href='/config' class='button-link' style='background-color: #6c757d;'>%s</a></div>", _T(L_AP_UPDATE_NETWORK_LIST)); server.sendContent(buffer);

    endHtmlResponse(); // Используем общий html_end
}

void handleApSaveWifiCredentials() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }
    // CSRF и аутентификация здесь не нужны, так как это режим AP для первоначальной настройки


    String new_ssid = server.arg("ap_ssid");
    String new_pass = server.arg("ap_password");

    if (new_ssid.length() == 0 || new_ssid.length() > MAX_SSID_LEN) {
        server.send(400, "text/plain", "SSID не может быть пустым и должен быть не длиннее 32 символов.");
        return;
    }
    if (new_pass.length() > MAX_PASSWORD_LEN) { // Пароль может быть пустым
        server.send(400, "text/plain", "Пароль должен быть не длиннее 64 символов.");
        return;
    }

    log_i("AP_CONFIG", "Получены новые учетные данные WiFi через AP: SSID='%s', Password_Length=%d", new_ssid.c_str(), new_pass.length());

    setWifiSsid(new_ssid.c_str());
    setWifiPassword(new_pass.c_str()); // setWifiPassword теперь корректно обрабатывает пустой пароль
    saveSensitiveConfig();

    String response_msg = "Настройки Wi-Fi сохранены. Устройство перезагрузится через 5 секунд и попытается подключиться к новой сети.";
    server.send(200, "text/plain; charset=UTF-8", response_msg); // Добавляем charset=UTF-8
    
    delay(5000);
    ESP.restart();
}

// Definition for the simple AP mode page handler
void handleApSimplePage() {
    // Эта страница очень простая, можно оставить как есть или обернуть в begin/endHtmlResponse, если хотим общий стиль
    beginHtmlResponse(); // Используем общий html_start
    server.sendContent_P(PSTR("<h1>")); server.sendContent(_T(L_AP_SIMPLE_PAGE_TITLE)); server.sendContent_P(PSTR("</h1><p>")); server.sendContent(_T(L_AP_SIMPLE_PAGE_PROMPT)); server.sendContent_P(PSTR("</p>"));
    endHtmlResponse();
}

void handleErrorLogPage() {
    if (!handleAuthentication()) return;
    beginHtmlResponse();
    char buffer[ERROR_HANDLER_LAST_MSG_BUFFER_SIZE + 150]; // Увеличил буфер для безопасности

    server.sendContent_P(PSTR("<h1>")); server.sendContent(_T(L_ERROR_LOG_TITLE)); server.sendContent_P(PSTR("</h1>"));
    snprintf(buffer, sizeof(buffer), "<p><a href='/' class='button-link'>%s</a> <a href='/settings' class='button-link'>%s</a></p>", _T(L_BACK_TO_STATUS), _T(L_BACK_TO_SETTINGS)); server.sendContent(buffer);

    StoredError_t errors[MAX_STORED_ERRORS];
    int num_errors = getStoredErrors(errors, MAX_STORED_ERRORS);

    if (num_errors == 0) {
        server.sendContent_P(PSTR("<p>")); server.sendContent(_T(L_NO_STORED_ERRORS)); server.sendContent_P(PSTR("</p>"));
    } else {
        server.sendContent("<table border='1' style='width:100%; border-collapse: collapse;'>");
        server.sendContent("<tr><th style='padding: 8px;'>Время (назад)</th><th style='padding: 8px;'>Код</th><th style='padding: 8px;'>Сообщение</th></tr>");
        unsigned long current_millis = millis();
        for (int i = 0; i < num_errors; i++) {
            unsigned long time_ago_s = (current_millis - errors[i].timestamp) / 1000;
            snprintf(buffer, sizeof(buffer),
                     "<tr><td style='padding: 8px;'>%lu с</td><td style='padding: 8px;'>%d</td><td style='padding: 8px;'>%s</td></tr>",
                     time_ago_s, errors[i].code, htmlEscapeContent(errors[i].message).c_str());
            server.sendContent(buffer);
        }
        server.sendContent("</table>");
    }
    // Кнопка для очистки лога ошибок (опционально)
    // server.sendContent("<form action='/clearAllErrors' method='POST' style='margin-top: 20px;'>");
    // server.sendContent(get_csrf_input_field());
    // server.sendContent("<input type='submit' value='Очистить журнал ошибок' style='background-color: var(--warn-orange);'></form>");

    endHtmlResponse();
}

void handleDosingControlPage() {
    if (!handleAuthentication()) return;

    if (server.method() == HTTP_POST) {
        if (!preCheckPost()) return; // Проверка аутентификации, метода POST и CSRF

        bool settings_changed = false;

        if (server.hasArg("tempSetpoint")) {
            String tempArg = server.arg("tempSetpoint");
            if (tempArg.indexOf(',') != -1) {
                server.send(400, "text/plain", "Use dot (.) for decimal numbers, not comma (,).");
                setSystemError(INPUT_VALIDATION_ERROR, "Comma in TempSetpoint float on Dosing Page");
                return;
            }
            float newTemp = tempArg.toFloat();
            if (newTemp >= -10.0f && newTemp <= 30.0f && !isnan(newTemp)) {
                if (fabs(config.tempSetpoint - newTemp) > 0.01f) {
                    config.tempSetpoint = newTemp;
                    settings_changed = true;
                    app_log_i("WEB_DOSING", "Dosing Page: Temp Setpoint updated to: %.1f C", config.tempSetpoint);
                }
            } else {
                setSystemError(INPUT_VALIDATION_ERROR, "Invalid Temp Setpoint via Dosing Page");
                server.send(400, "text/plain", "Invalid temperature setpoint. Must be between -10 and 30.");
                return;
            }
        }

        if (server.hasArg("volumeTarget")) {
            int newVol = server.arg("volumeTarget").toInt();
            if (newVol > 0 && newVol <= 10000) {
                if (config.volumeTarget != newVol) {
                    config.volumeTarget = newVol; // Это значение будет использовано в startDosingCycle
                    settings_changed = true;
                    app_log_i("WEB_DOSING", "Dosing Page: Volume Target updated to: %d ml", config.volumeTarget);
                }
            } else {
                setSystemError(INPUT_VALIDATION_ERROR, "Invalid Volume Target via Dosing Page");
                server.send(400, "text/plain", "Invalid volume target. Must be between 1 and 10000 ml.");
                return;
            }
        }

        if (settings_changed) {
            saveConfig();
        }

        // Запускаем дозирование с обновленным config.volumeTarget
        startDosingCycle(config.volumeTarget, true); // true - fromWeb
        sendRedirect("/dosingcontrol"); // Перенаправляем обратно на страницу управления дозированием
        return;
    }

    // GET-запрос: отображаем страницу
    beginHtmlResponse();
    char buffer[256];

    server.sendContent_P(PSTR("<h1>")); server.sendContent(_T(L_DOSING_CONTROL_TITLE)); server.sendContent_P(PSTR("</h1>"));
    snprintf(buffer, sizeof(buffer), "<p><a href='/' class='button-link'>%s</a> <a href='/settings' class='button-link'>%s</a></p>", _T(L_BACK_TO_STATUS), _T(L_SETTINGS)); server.sendContent(buffer);

    // Отображение текущего статуса (можно скопировать часть из handleRoot)
    DosingState local_dosing_state = getDosingState(); // Потокобезопасный геттер
    const char* dosing_state_str = _T((LangKey)(L_DOSING_STATE_IDLE + local_dosing_state));
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%s</strong></p>", _T(L_CURRENT_DOSING_STATE), dosing_state_str);
    server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<p class='status-item'>%s: <strong>%.2f °C</strong></p>", _T(L_TEMP_OUT_FILTERED), tOut_filtered); server.sendContent(buffer);

    server.sendContent("<form action='/dosingcontrol' method='POST'>");
    server.sendContent(get_csrf_input_field());
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='tempSetpoint'>%s:</label><input type='number' id='tempSetpoint' name='tempSetpoint' step='0.1' value='%.1f' required></div>", _T(L_TARGET_TEMP_C), config.tempSetpoint); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<div class='form-group'><label for='volumeTarget'>%s:</label><input type='number' id='volumeTarget' name='volumeTarget' value='%d' required></div>", _T(L_TARGET_DOSING_VOLUME_ML), config.volumeTarget); server.sendContent(buffer);
    snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s'></form>", _T(L_SAVE_AND_START_DOSING)); server.sendContent(buffer);

    if (local_dosing_state != DOSING_STATE_IDLE && local_dosing_state != DOSING_STATE_FINISHED && local_dosing_state != DOSING_STATE_ERROR) {
        server.sendContent("<form action='/stopDosing' method='POST' style='margin-top: 10px;'>");
        server.sendContent(get_csrf_input_field());
        snprintf(buffer, sizeof(buffer), "<input type='submit' value='%s' style='background-color: var(--warn-orange);'></form>", _T(L_STOP_DOSING)); server.sendContent(buffer);
    }

    endHtmlResponse();
}

void setupWebServerRoutes() {
    app_log_i("WEB_ROUTES_DBG", "Entering setupWebServerRoutes. ap_mode_active: %s", ap_mode_active ? "true" : "false");
    if (ap_mode_active) { // ap_mode_active - глобальная переменная из main.h
        app_log_i("WEB_ROUTES_DBG", "Setting up AP mode web server routes...");
        server.on("/", HTTP_GET, handleApSimplePage); // Используем простую страницу для /
        server.on("/config", HTTP_GET, handleApConfigPage); // Страница конфигурации теперь на /config
        server.on("/savewifi_ap", HTTP_POST, handleApSaveWifiCredentials);
        // Для всех остальных запросов в режиме AP показываем простую страницу
        server.onNotFound(handleApSimplePage);
        app_log_i("WEB_ROUTES_DBG", "AP mode routes SET.");
    } else {
        app_log_i("WEB_ROUTES_DBG", "Setting up STA mode web server routes...");
        server.on("/", HTTP_GET, handleRoot);
        server.on("/settings", HTTP_GET, handleSettings);
        server.on("/settings", HTTP_POST, handleUpdateConfig);
        server.on("/startDosing", HTTP_POST, handleStartDosing);
        server.on("/stopDosing", HTTP_POST, handleStopDosing);
        server.on("/startCalibration", HTTP_POST, handleStartCalibration);
        server.on("/stopCalibration", HTTP_POST, handleStopCalibration);
        // Добавляем маршруты для ручного управления мотором в режиме калибровки
        server.on("/cal_manual_fwd", HTTP_POST, handleCalibrateMotorFwd);
        server.on("/cal_manual_rev", HTTP_POST, handleCalibrateMotorRev);
        server.on("/cal_manual_stop", HTTP_POST, handleCalibrateMotorStop);

        server.on("/stopFlowCalibration", HTTP_POST, handleStopFlowCalibration); 
        server.on("/resetStats", HTTP_POST, handleResetStatsWeb);
        server.on("/clearError", HTTP_POST, handleClearErrorWeb);
        server.on("/factoryReset", HTTP_POST, handleFactoryReset);
        server.on("/emergency", HTTP_GET, handleEmergencyStop); // Consider making this POST
        server.on("/diagnostics", HTTP_GET, handleDiagnostics);
        server.on("/downloadlog", HTTP_GET, handleDownloadLog);
        server.on("/powerToggle", HTTP_GET, handlePowerToggleWeb);
        server.on("/wifi_setup", HTTP_GET, handleWifiSetup);  // Маршрут для GET
        server.on("/wifi_setup", HTTP_POST, handleWifiSetup); // Маршрут для POST
        server.on("/dosingcontrol", HTTP_GET, handleDosingControlPage);  // Для GET-запросов
        server.on("/dosingcontrol", HTTP_POST, handleDosingControlPage); // Для POST-запросов
        server.on("/errors", HTTP_GET, handleErrorLogPage);   // <-- НОВЫЙ МАРШРУТ
        app_log_i("WEB_ROUTES_DBG", "STA mode routes SET.");
    }

    app_log_i("WEB_ROUTES_DBG", "Calling server.begin()...");
    server.begin();
    app_log_i("WEB_ROUTES_DBG", "server.begin() called.");

    // Check WiFi status again right before logging
    if (WiFi.status() == WL_CONNECTED) {
        app_log_i("WEB_ROUTES_DBG", "HTTP server started. WiFi is CONNECTED. IP: %s", WiFi.localIP().toString().c_str());
    } else {
        app_log_w("WEB_ROUTES_DBG", "HTTP server started. WiFi is NOT CONNECTED. Status: %d", WiFi.status());
    }
}
