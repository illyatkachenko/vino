#include "localization.h"
#include "config_manager.h" // Для DEFAULT_LANGUAGE
#include "main.h" // Для app_log_x

static char current_active_language[3] = "ru"; // Язык по умолчанию - русский

// Таблицы строк (PROGMEM)
// Русский
const char* const translations_ru[L_KEY_COUNT] PROGMEM = {
    [L_SETTINGS] = "Настройки",
    [L_DIAGNOSTICS] = "Диагностика",
    [L_ERROR_LOG] = "Журнал Ошибок",
    [L_DOSING_CONTROL] = "Управление Дозированием",
    [L_BACK_TO_STATUS] = "На главную",
    [L_BACK_TO_SETTINGS] = "К настройкам",
    [L_SAVE_AND_RESTART] = "Сохранить и перезагрузить",
    [L_SAVE] = "Сохранить",
    [L_LANGUAGE] = "Язык",
    [L_RUSSIAN] = "Русский",
    [L_ENGLISH] = "English",
    [L_SYSTEM_STATUS] = "Статус Системы",
    [L_POWER_STATUS] = "Состояние питания",
    [L_POWER_ON] = "Включено",
    [L_POWER_OFF] = "Выключено",
    [L_DOSING_STATUS] = "Состояние дозирования",
    [L_CURRENT_VOLUME_DOSED] = "Текущий выданный объем",
    [L_TARGET_VOLUME] = "Целевой объем",
    [L_TEMP_OUT] = "Температура на выходе",
    [L_TEMP_OUT_FILTERED] = "Температура на выходе (фильтр.)",
    [L_TEMP_IN] = "Температура на входе",
    [L_TEMP_COOLER] = "Температура охладителя",
    [L_COMPRESSOR_STATUS] = "Состояние компрессора",
    [L_COMPRESSOR_ON] = "Включен",
    [L_COMPRESSOR_OFF] = "Выключен",
    [L_MOTOR_STATUS] = "Состояние мотора",
    [L_MOTOR_RUNNING] = "Работает",
    [L_MOTOR_STOPPED] = "Остановлен",
    [L_SYSTEM_ERROR_STATUS] = "Состояние ошибки системы",
    [L_NO_ERROR] = "Нет ошибок",
    [L_TOTAL_DOSING_CYCLES] = "Всего циклов дозирования",
    [L_TOTAL_VOLUME_DOSED] = "Всего выдано объема",
    [L_UPTIME] = "Время работы",
    [L_FREE_HEAP] = "Свободная память",
    [L_WIFI_SSID] = "WiFi SSID",
    [L_WIFI_IP_ADDRESS] = "WiFi IP Адрес",
    [L_WIFI_MAC_ADDRESS] = "WiFi MAC Адрес",
    [L_ESP_NOW_PEER_MAC] = "ESP-NOW MAC Пира",
    [L_ESP_NOW_LAST_SENT_STATUS] = "ESP-NOW Статус последней отправки",
    [L_ESP_NOW_LAST_RECV_STATUS] = "ESP-NOW Статус последнего получения",
    [L_CONTROL_AND_SETTINGS] = "Управление и Настройки",
    [L_TOGGLE_POWER] = "Вкл/Выкл Питание",
    [L_RESET_STATS] = "Сбросить Статистику",
    [L_CLEAR_ERROR] = "Сбросить Ошибку",
    [L_EMERGENCY_STOP] = "Аварийная Остановка",
    [L_TOTAL_ERRORS] = "Всего ошибок",
    [L_LAST_ERROR_MESSAGE] = "Последнее сообщение об ошибке",
    [L_CALIBRATION_MODE_ACTIVE_MSG] = "Режим калибровки активен",
    [L_STEPS_ACCUMULATED_MSG] = "Накоплено шагов",
    [L_SYSTEM_ERROR_TITLE] = "Ошибка системы!",
    [L_ERROR_CODE_LABEL] = "Код ошибки",
    [L_ERROR_TIME_LABEL] = "Время ошибки",
    [L_SECONDS_AGO_LABEL] = "сек назад",
    [L_STATUS_LABEL] = "Состояние",
    [L_OK_STATUS] = "ОК",
    [L_DISABLE_PID_TEMP_CONTROL_BTN] = "Выключить PID T°",
    [L_ENABLE_PID_TEMP_CONTROL_BTN] = "Включить PID T°",
    [L_DOWNLOAD_LOG_BTN] = "Скачать лог",
    [L_STATISTICS_TITLE] = "Статистика",
    [L_COMPRESSOR_RUNTIME_LABEL] = "Время работы компрессора",
    [L_HOURS_UNIT] = "часов",
    [L_COMPRESSOR_STARTS_LABEL] = "Количество запусков компрессора",
    [L_SYSTEM_SETTINGS_TITLE] = "Настройки Системы",
    [L_MOTOR_SETTINGS] = "Настройки Мотора",
    [L_MOTOR_SPEED_DOSING] = "Скорость мотора (дозирование, шагов/с)",
    [L_MOTOR_SPEED_CALIBRATION] = "Скорость мотора (калибровка, шагов/с)",
    [L_ML_PER_STEP] = "мл/шаг",
    [L_ACCELERATION_STEPS] = "Шаги ускорения",
    [L_DECELERATION_STEPS] = "Шаги замедления",
    [L_TEMP_SETTINGS] = "Настройки Температуры",
    [L_TEMP_SETPOINT_DOSING] = "Уставка температуры (дозирование, °C)",
    [L_PID_SETTINGS] = "Настройки PID",
    [L_PID_KP] = "PID Kp",
    [L_PID_KI] = "PID Ki",
    [L_PID_KD] = "PID Kd",
    [L_PID_OUTPUT_MIN] = "PID Выход Min",
    [L_PID_OUTPUT_MAX] = "PID Выход Max",
    [L_ENABLE_PID_TEMP_CONTROL] = "Включить PID контроль температуры",
    [L_FLOW_SENSOR_SETTINGS] = "Настройки Датчика Потока",
    [L_ML_PER_PULSE] = "мл/импульс",
    [L_ESP_NOW_SETTINGS] = "Настройки ESP-NOW",
    [L_REMOTE_PEER_MAC] = "MAC-адрес удаленного пира",
    [L_CALIBRATION] = "Калибровка",
    [L_CALIBRATE_MOTOR_ML_PER_STEP] = "Калибровка мотора (мл/шаг)",
    [L_TARGET_CALIBRATION_VOLUME] = "Целевой объем калибровки (мл)",
    [L_START_CALIBRATION] = "Начать калибровку",
    [L_ACTUAL_VOLUME_MEASURED] = "Фактический измеренный объем (мл)",
    [L_STOP_AND_SAVE_CALIBRATION] = "Остановить и сохранить калибровку",
    [L_CALIBRATE_FLOW_SENSOR] = "Калибровка датчика потока (мл/импульс)",
    [L_TARGET_FLOW_CALIBRATION_VOLUME] = "Целевой объем калибровки потока (мл)",
    [L_START_FLOW_CALIBRATION] = "Начать калибровку потока",
    [L_ACTUAL_VOLUME_FLOW_SENSOR] = "Фактический объем (датчик потока, мл)",
    [L_STOP_AND_SAVE_FLOW_CALIBRATION] = "Остановить и сохранить калибровку потока",
    [L_MANUAL_MOTOR_CONTROL_CAL_MODE] = "Ручное управление мотором (режим калибровки)",
    [L_FORWARD] = "Вперед",
    [L_REVERSE] = "Назад",
    [L_STOP] = "Стоп",
    [L_OTHER_SETTINGS] = "Прочие Настройки",
    [L_WIFI_SETTINGS] = "Настройки WiFi",
    [L_ML_PER_PULSE_FLOW_SENSOR_CAL_LABEL] = "Калибровка датчика потока (мл/импульс)",
    [L_DOSING_CONTROL_SETTINGS_TITLE] = "Управление дозированием (настройки)",
    [L_DOSING_VOLUME_ML_LABEL] = "Объем дозирования (мл)",
    [L_START_DOSING_BTN] = "Начать дозирование",
    [L_CURRENT_ML_PER_STEP_MOTOR_LABEL] = "Текущий мл/Шаг (мотор)",
    [L_CURRENT_ML_PER_PULSE_FLOW_LABEL] = "Текущий мл/Импульс (поток)",
    [L_ACCUMULATED_FLOW_PULSES_LABEL] = "Накоплено импульсов потока",
    [L_MOTOR_FORWARD_CAL_BTN] = "Мотор Вперед (Калибр.)",
    [L_MOTOR_REVERSE_CAL_BTN] = "Мотор Назад (Калибр.)",
    [L_MOTOR_STOP_CAL_BTN] = "Стоп Мотор (Калибр.)",
    [L_FINISH_MOTOR_CALIBRATION_BTN] = "Завершить калибровку мотора",
    [L_FINISH_FLOW_CALIBRATION_BTN] = "Завершить калибровку потока",
    [L_TARGET_CALIBRATION_VOLUME_HINT_LABEL] = "Целевой объем для калибровки (мл, подсказка)",
    [L_START_CALIBRATION_MOTOR_FLOW_BTN] = "Начать калибровку (мотор и поток)",
    [L_PID_SETTINGS_TITLE_MOTOR_SPEED] = "Настройки PID-регулятора температуры (через скорость мотора)",
    [L_PID_TEMP_CONTROL_STATUS_LABEL] = "PID контроль температуры",
    [L_ENABLED_STATUS] = "ВКЛЮЧЕН",
    [L_DISABLED_STATUS] = "ВЫКЛЮЧЕН",
    [L_TOGGLE_ON_MAIN_PAGE_HINT] = "Переключить можно на главной странице",
    [L_SAVE_PID_COEFFS_BTN] = "Сохранить коэфф. PID",
    [L_ARE_YOU_SURE_FACTORY_RESET_CONFIRM_MSG] = "Вы уверены, что хотите сбросить все настройки к заводским? Это действие необратимо и приведет к перезагрузке устройства.",
    [L_FACTORY_RESET] = "Сброс к заводским настройкам",
    [L_PERFORM_FACTORY_RESET] = "Выполнить сброс",
    [L_ARE_YOU_SURE_FACTORY_RESET] = "Вы уверены? Все настройки будут сброшены!",
    [L_DOSING_CONTROL_TITLE] = "Управление Дозированием",
    [L_CURRENT_DOSING_STATE] = "Текущее состояние дозирования",
    [L_TARGET_TEMP_C] = "Целевая температура (°C)",
    [L_TARGET_DOSING_VOLUME_ML] = "Целевой объем дозирования (мл)",
    [L_SAVE_AND_START_DOSING] = "Сохранить и Начать Дозирование",
    [L_STOP_DOSING] = "Остановить Дозирование",
    [L_WIFI_SETUP_TITLE] = "Настройка WiFi",
    [L_CURRENT_WIFI_SSID] = "Текущий SSID",
    [L_NEW_SSID] = "Новый SSID",
    [L_NEW_PASSWORD_LEAVE_EMPTY] = "Новый пароль (оставьте пустым, чтобы не менять)",
    [L_ERROR_LOG_TITLE] = "Журнал ошибок системы",
    [L_NO_STORED_ERRORS] = "Нет сохраненных ошибок.",
    [L_TIME_AGO] = "Время (назад)",
    [L_CODE] = "Код",
    [L_MESSAGE] = "Сообщение",
    [L_AP_CONFIG_TITLE] = "Настройка Wi-Fi подключения",
    [L_AP_CONFIG_PROMPT] = "Пожалуйста, введите данные вашей Wi-Fi сети или выберите из списка ниже.",
    [L_AP_SSID_LABEL] = "Имя сети (SSID):",
    [L_AP_PASSWORD_LABEL] = "Пароль:",
    [L_AP_SELECT_FROM_LIST] = "Или выберите из списка:",
    [L_AP_OR_ENTER_MANUALLY] = "(или введите вручную выше)",
    [L_AP_AVAILABLE_NETWORKS] = "Доступные сети:",
    [L_AP_NETWORK_NAME] = "Имя Сети",
    [L_AP_SIGNAL_STRENGTH] = "Сигнал",
    [L_AP_SECURITY] = "Защита",
    [L_AP_SELECT] = "Выбрать",
    [L_AP_SAVE_WIFI_CREDENTIALS] = "Сохранить учетные данные WiFi",
    [L_AP_SAVE_AND_CONNECT_BTN] = "Сохранить и подключиться",
    [L_AP_UPDATE_NETWORK_LIST] = "Обновить список сетей",
    [L_AP_SIMPLE_PAGE_TITLE] = "Режим Точки Доступа",
    [L_AP_SIMPLE_PAGE_PROMPT] = "Пожалуйста, настройте Wi-Fi, перейдя по ссылке &lt;a href='/config'&gt;/config&lt;/a&gt; для сканирования сетей.",
    [L_AP_SECURITY_OPEN] = "Открытая",
    [L_AP_SECURITY_ENCRYPTED] = "Защищенная",
    [L_AP_NO_NETWORKS_FOUND] = "Доступные сети не найдены. Пожалуйста, введите SSID вручную.",
    [L_DOSING_STATE_IDLE] = "Ожидание (IDLE)",
    [L_DOSING_STATE_REQUESTED] = "Запрос (REQUESTED)",
    [L_DOSING_STATE_PRE_COOLING] = "Предв. охлаждение (PRE_COOLING)",
    [L_DOSING_STATE_STARTING] = "Запуск (STARTING)",
    [L_DOSING_STATE_RUNNING] = "Выполнение (RUNNING)",
    [L_DOSING_STATE_STOPPING] = "Остановка (STOPPING)",
    [L_DOSING_STATE_PAUSED] = "Пауза (PAUSED)",
    [L_DOSING_STATE_FINISHED] = "Завершено (FINISHED)",
    [L_DOSING_STATE_ERROR] = "Ошибка дозирования (ERROR)",
    [L_DOSING_STATE_UNKNOWN] = "Неизвестно",
    [L_DIAGNOSTICS_INFO_TITLE] = "Диагностическая информация",
    [L_GENERAL_STATUS_TITLE] = "Общее состояние",
    [L_BYTES_UNIT] = "байт",
    // New keys
    [L_AP_CONFIGURE_WIFI_NAVIGATE] = "Подключитесь к этой ТД и перейдите на http://%s для настройки WiFi.",
    [L_LITTLEFS_MOUNT_FAILED_LOG_DISABLED] = "Ошибка монтирования LittleFS! Логирование в файл будет отключено.",
    [L_REBOOTED_DUE_TO_FATAL_ERROR] = "!!! Система была перезагружена из-за фатальной ошибки: %u !!!",
    [L_FATAL_ERROR_FLAG_CLEARED_MSG] = "Флаг фатальной ошибки очищен.",
    [L_PREFS_OPEN_FAIL_CLEAR_FATAL_FLAG] = "Не удалось открыть Preferences для очистки флага фатальной ошибки.",
    [L_PREFS_OPEN_FAIL_CHECK_FATAL_FLAG] = "Не удалось открыть Preferences (namespace: %s) для проверки флага фатальной ошибки.",
    [L_RESET_BUTTON_PRESSED_HOLD_MSG] = "Кнопка сброса нажата. Удерживайте %lu мс для перезагрузки...",
    [L_RESET_BUTTON_RELEASED_CANCEL_MSG] = "Кнопка сброса отпущена. Перезагрузка отменена.",
    [L_RESET_BUTTON_HELD_RESTART_MSG] = "Кнопка сброса удерживалась %lu мс. Перезагрузка ESP...",
    [L_NO_ACTIVE_ERROR_MSG] = "Нет активных ошибок.",
    [L_UNKNOWN_ERROR_CODE_DESC_MSG] = "Неизвестный код ошибки.",
    [L_ERROR_DESC_NO_ERROR] = "Нет ошибки",
    [L_ERROR_DESC_WIFI_ERROR] = "Ошибка соединения/связи Wi-Fi",
    [L_ERROR_DESC_CALIBRATION_ERROR] = "Ошибка процесса калибровки",
    [L_ERROR_CAL_ACTIVE_CANNOT_DOSE] = "Калибровка активна. Невозможно начать дозирование.",
    [L_ERROR_SYS_NOT_POWERED_CANNOT_DOSE] = "Система не включена. Невозможно начать дозирование.",
    [L_ERROR_DOSING_CYCLE_BUSY_OR_ERROR] = "Цикл дозирования занят или в состоянии ошибки.",
    [L_ERROR_FLOW_SENSOR_NOT_CALIBRATED] = "Датчик потока не откалиброван (flowMlPerPulse). Откалибруйте.",
    [L_ERROR_DOSING_REQUESTED_SYS_NOT_POWERED] = "Запрошено дозирование, но система не включена.",
    [L_ERROR_PRECOOLING_TIMEOUT] = "Тайм-аут предохлаждения.",
    [L_ERROR_DOSING_START_ABORTED_TOUT_FAIL] = "Запуск дозирования прерван, ошибка датчика T_out.",
    [L_ERROR_MOTOR_SPEED_ZERO_CANNOT_DOSE] = "Скорость мотора 0, невозможно начать дозирование.",
    [L_ERROR_MAX_DOSING_DURATION_TIMEOUT] = "Тайм-аут максимальной длительности дозирования.",
    [L_ERROR_ESPNOW_INIT_FAIL] = "Ошибка инициализации ESP-NOW.",
    [L_ERROR_ESPNOW_RECV_CB_REGISTER_FAIL] = "Ошибка регистрации callback ESP-NOW.",
    [L_ERROR_ESPNOW_PEER_ADD_READD_FAIL] = "Ошибка добавления/передобавления пира ESP-NOW.",
    [L_ERROR_INVALID_PEER_MAC_IN_CONFIG] = "Неверный MAC пира в конфигурации.",
    [L_ERROR_EMERGENCY_STOP_VIA_ESPNOW] = "АВАРИЙНАЯ ОСТАНОВКА через ESP-NOW.",
    [L_WARN_ESPNOW_SEND_FAIL_AFTER_RETRIES] = "Ошибка отправки ESP-NOW после повторных попыток.",
    [L_ERROR_PREFS_DEFAULTS_APPLIED_OPEN_FAIL] = "Применены значения по умолчанию (ошибка открытия Preferences).",
    [L_INFO_STATISTICS_RESET_MSG] = "Статистика сброшена.",
    [L_ERROR_CALIBRATION_ALREADY_ACTIVE] = "Калибровка уже активна.",
    [L_ERROR_SYS_NOT_POWERED_CANNOT_CALIBRATE] = "Система не включена. Невозможно начать калибровку.",
    [L_ERROR_DOSING_CYCLE_ACTIVE_CANNOT_CALIBRATE] = "Цикл дозирования активен/занят. Невозможно начать калибровку.",
    [L_ERROR_INVALID_MOTOR_CALIBRATION_DATA] = "Неверные данные калибровки мотора (Объем/Шаги).",
    [L_ERROR_CALIBRATION_NOT_ACTIVE_MOTOR] = "Калибровка не активна (мотор).",
    [L_ERROR_CALIBRATION_NOT_ACTIVE_FLOW] = "Калибровка не активна (поток).",
    [L_ERROR_INVALID_FLOW_CALIBRATION_DATA] = "Неверные данные калибровки потока (Объем/Импульсы).",
    [L_ERROR_CALIBRATION_TIMED_OUT_MOTOR_STOPPED] = "Тайм-аут калибровки. Мотор остановлен.",
};

// Английский
const char* const translations_en[L_KEY_COUNT] PROGMEM = {
    [L_SETTINGS] = "Settings",
    [L_DIAGNOSTICS] = "Diagnostics",
    [L_ERROR_LOG] = "Error Log",
    [L_DOSING_CONTROL] = "Dosing Control",
    [L_BACK_TO_STATUS] = "Back to Status",
    [L_BACK_TO_SETTINGS] = "Back to Settings",
    [L_SAVE_AND_RESTART] = "Save and Restart",
    [L_SAVE] = "Save",
    [L_LANGUAGE] = "Language",
    [L_RUSSIAN] = "Русский",
    [L_ENGLISH] = "English",
    [L_SYSTEM_STATUS] = "System Status",
    [L_POWER_STATUS] = "Power Status",
    [L_POWER_ON] = "On",
    [L_POWER_OFF] = "Off",
    [L_DOSING_STATUS] = "Dosing Status",
    [L_CURRENT_VOLUME_DOSED] = "Current Volume Dosed",
    [L_TARGET_VOLUME] = "Target Volume",
    [L_TEMP_OUT] = "Output Temperature",
    [L_TEMP_OUT_FILTERED] = "Output Temperature (filtered)",
    [L_TEMP_IN] = "Input Temperature",
    [L_TEMP_COOLER] = "Cooler Temperature",
    [L_COMPRESSOR_STATUS] = "Compressor Status",
    [L_COMPRESSOR_ON] = "On",
    [L_COMPRESSOR_OFF] = "Off",
    [L_MOTOR_STATUS] = "Motor Status",
    [L_MOTOR_RUNNING] = "Running",
    [L_MOTOR_STOPPED] = "Stopped",
    [L_SYSTEM_ERROR_STATUS] = "System Error Status",
    [L_NO_ERROR] = "No Error",
    [L_TOTAL_DOSING_CYCLES] = "Total Dosing Cycles",
    [L_TOTAL_VOLUME_DOSED] = "Total Volume Dosed",
    [L_UPTIME] = "Uptime",
    [L_FREE_HEAP] = "Free Heap",
    [L_WIFI_SSID] = "WiFi SSID",
    [L_WIFI_IP_ADDRESS] = "WiFi IP Address",
    [L_WIFI_MAC_ADDRESS] = "WiFi MAC Address",
    [L_ESP_NOW_PEER_MAC] = "ESP-NOW Peer MAC",
    [L_ESP_NOW_LAST_SENT_STATUS] = "ESP-NOW Last Sent Status",
    [L_ESP_NOW_LAST_RECV_STATUS] = "ESP-NOW Last Received Status",
    [L_CONTROL_AND_SETTINGS] = "Control and Settings",
    [L_TOGGLE_POWER] = "Toggle Power",
    [L_RESET_STATS] = "Reset Statistics",
    [L_CLEAR_ERROR] = "Clear Error",
    [L_EMERGENCY_STOP] = "Emergency Stop",
    [L_TOTAL_ERRORS] = "Total Errors",
    [L_LAST_ERROR_MESSAGE] = "Last Error Message",
    [L_CALIBRATION_MODE_ACTIVE_MSG] = "Calibration mode active",
    [L_STEPS_ACCUMULATED_MSG] = "Steps accumulated",
    [L_SYSTEM_ERROR_TITLE] = "System Error!",
    [L_ERROR_CODE_LABEL] = "Error Code",
    [L_ERROR_TIME_LABEL] = "Error Time",
    [L_SECONDS_AGO_LABEL] = "sec ago",
    [L_STATUS_LABEL] = "Status",
    [L_OK_STATUS] = "OK",
    [L_DISABLE_PID_TEMP_CONTROL_BTN] = "Disable PID T°",
    [L_ENABLE_PID_TEMP_CONTROL_BTN] = "Enable PID T°",
    [L_DOWNLOAD_LOG_BTN] = "Download Log",
    [L_STATISTICS_TITLE] = "Statistics",
    [L_COMPRESSOR_RUNTIME_LABEL] = "Compressor Run Time",
    [L_HOURS_UNIT] = "hours",
    [L_COMPRESSOR_STARTS_LABEL] = "Compressor Starts",
    [L_SYSTEM_SETTINGS_TITLE] = "System Settings",
    [L_MOTOR_SETTINGS] = "Motor Settings",
    [L_MOTOR_SPEED_DOSING] = "Motor Speed (dosing, steps/s)",
    [L_MOTOR_SPEED_CALIBRATION] = "Motor Speed (calibration, steps/s)",
    [L_ML_PER_STEP] = "ml/step",
    [L_ACCELERATION_STEPS] = "Acceleration Steps",
    [L_DECELERATION_STEPS] = "Deceleration Steps",
    [L_TEMP_SETTINGS] = "Temperature Settings",
    [L_TEMP_SETPOINT_DOSING] = "Temp Setpoint (dosing, °C)",
    [L_PID_SETTINGS] = "PID Settings",
    [L_PID_KP] = "PID Kp",
    [L_PID_KI] = "PID Ki",
    [L_PID_KD] = "PID Kd",
    [L_PID_OUTPUT_MIN] = "PID Output Min",
    [L_PID_OUTPUT_MAX] = "PID Output Max",
    [L_ENABLE_PID_TEMP_CONTROL] = "Enable PID Temperature Control",
    [L_FLOW_SENSOR_SETTINGS] = "Flow Sensor Settings",
    [L_ML_PER_PULSE] = "ml/pulse",
    [L_ESP_NOW_SETTINGS] = "ESP-NOW Settings",
    [L_REMOTE_PEER_MAC] = "Remote Peer MAC Address",
    [L_CALIBRATION] = "Calibration",
    [L_CALIBRATE_MOTOR_ML_PER_STEP] = "Calibrate Motor (ml/step)",
    [L_TARGET_CALIBRATION_VOLUME] = "Target Calibration Volume (ml)",
    [L_START_CALIBRATION] = "Start Calibration",
    [L_ACTUAL_VOLUME_MEASURED] = "Actual Volume Measured (ml)",
    [L_STOP_AND_SAVE_CALIBRATION] = "Stop and Save Calibration",
    [L_CALIBRATE_FLOW_SENSOR] = "Calibrate Flow Sensor (ml/pulse)",
    [L_TARGET_FLOW_CALIBRATION_VOLUME] = "Target Flow Calibration Volume (ml)",
    [L_START_FLOW_CALIBRATION] = "Start Flow Calibration",
    [L_ACTUAL_VOLUME_FLOW_SENSOR] = "Actual Volume (flow sensor, ml)",
    [L_STOP_AND_SAVE_FLOW_CALIBRATION] = "Stop and Save Flow Calibration",
    [L_MANUAL_MOTOR_CONTROL_CAL_MODE] = "Manual Motor Control (Calibration Mode)",
    [L_FORWARD] = "Forward",
    [L_REVERSE] = "Reverse",
    [L_STOP] = "Stop",
    [L_OTHER_SETTINGS] = "Other Settings",
    [L_WIFI_SETTINGS] = "WiFi Settings",
    [L_ML_PER_PULSE_FLOW_SENSOR_CAL_LABEL] = "Flow Sensor Calibration (ml/pulse)",
    [L_DOSING_CONTROL_SETTINGS_TITLE] = "Dosing Control (Settings)",
    [L_DOSING_VOLUME_ML_LABEL] = "Dosing Volume (ml)",
    [L_START_DOSING_BTN] = "Start Dosing",
    [L_CURRENT_ML_PER_STEP_MOTOR_LABEL] = "Current ml/Step (motor)",
    [L_CURRENT_ML_PER_PULSE_FLOW_LABEL] = "Current ml/Pulse (flow)",
    [L_ACCUMULATED_FLOW_PULSES_LABEL] = "Accumulated Flow Pulses",
    [L_MOTOR_FORWARD_CAL_BTN] = "Motor Forward (Cal.)",
    [L_MOTOR_REVERSE_CAL_BTN] = "Motor Reverse (Cal.)",
    [L_MOTOR_STOP_CAL_BTN] = "Stop Motor (Cal.)",
    [L_FINISH_MOTOR_CALIBRATION_BTN] = "Finish Motor Calibration",
    [L_FINISH_FLOW_CALIBRATION_BTN] = "Finish Flow Calibration",
    [L_TARGET_CALIBRATION_VOLUME_HINT_LABEL] = "Target Calibration Volume (ml, hint)",
    [L_START_CALIBRATION_MOTOR_FLOW_BTN] = "Start Calibration (Motor & Flow)",
    [L_PID_SETTINGS_TITLE_MOTOR_SPEED] = "PID Temperature Settings (via Motor Speed)",
    [L_PID_TEMP_CONTROL_STATUS_LABEL] = "PID Temperature Control",
    [L_ENABLED_STATUS] = "ENABLED",
    [L_DISABLED_STATUS] = "DISABLED",
    [L_TOGGLE_ON_MAIN_PAGE_HINT] = "Toggle on main page",
    [L_SAVE_PID_COEFFS_BTN] = "Save PID Coeffs.",
    [L_ARE_YOU_SURE_FACTORY_RESET_CONFIRM_MSG] = "Are you sure you want to reset all settings to factory defaults? This action is irreversible and will restart the device.",
    [L_FACTORY_RESET] = "Factory Reset",
    [L_PERFORM_FACTORY_RESET] = "Perform Factory Reset",
    [L_ARE_YOU_SURE_FACTORY_RESET] = "Are you sure? All settings will be reset!",
    [L_DOSING_CONTROL_TITLE] = "Dosing Control",
    [L_CURRENT_DOSING_STATE] = "Current Dosing State",
    [L_TARGET_TEMP_C] = "Target Temperature (°C)",
    [L_TARGET_DOSING_VOLUME_ML] = "Target Dosing Volume (ml)",
    [L_SAVE_AND_START_DOSING] = "Save and Start Dosing",
    [L_STOP_DOSING] = "Stop Dosing",
    [L_WIFI_SETUP_TITLE] = "WiFi Setup",
    [L_CURRENT_WIFI_SSID] = "Current SSID",
    [L_NEW_SSID] = "New SSID",
    [L_NEW_PASSWORD_LEAVE_EMPTY] = "New Password (leave empty to not change)",
    [L_ERROR_LOG_TITLE] = "System Error Log",
    [L_NO_STORED_ERRORS] = "No stored errors.",
    [L_TIME_AGO] = "Time Ago",
    [L_CODE] = "Code",
    [L_MESSAGE] = "Message",
    [L_AP_CONFIG_TITLE] = "Wi-Fi Connection Setup",
    [L_AP_CONFIG_PROMPT] = "Please enter your Wi-Fi network details or select from the list below.",
    [L_AP_SSID_LABEL] = "Network Name (SSID):",
    [L_AP_PASSWORD_LABEL] = "Password:",
    [L_AP_SELECT_FROM_LIST] = "Or select from list:",
    [L_AP_OR_ENTER_MANUALLY] = "(or enter manually above)",
    [L_AP_AVAILABLE_NETWORKS] = "Available Networks:",
    [L_AP_NETWORK_NAME] = "Network Name",
    [L_AP_SIGNAL_STRENGTH] = "Signal",
    [L_AP_SECURITY] = "Security",
    [L_AP_SELECT] = "Select",
    [L_AP_SAVE_WIFI_CREDENTIALS] = "Save WiFi Credentials",
    [L_AP_SAVE_AND_CONNECT_BTN] = "Save and Connect",
    [L_AP_UPDATE_NETWORK_LIST] = "Refresh Network List",
    [L_AP_SIMPLE_PAGE_TITLE] = "Access Point Mode",
    [L_AP_SIMPLE_PAGE_PROMPT] = "Please configure Wi-Fi by navigating to &lt;a href='/config'&gt;/config&lt;/a&gt; to scan networks.",
    [L_AP_SECURITY_OPEN] = "Open",
    [L_AP_SECURITY_ENCRYPTED] = "Encrypted",
    [L_AP_NO_NETWORKS_FOUND] = "No available networks found. Please enter SSID manually.",
    [L_DOSING_STATE_IDLE] = "Idle",
    [L_DOSING_STATE_REQUESTED] = "Requested",
    [L_DOSING_STATE_PRE_COOLING] = "Pre-Cooling",
    [L_DOSING_STATE_STARTING] = "Starting",
    [L_DOSING_STATE_RUNNING] = "Running",
    [L_DOSING_STATE_STOPPING] = "Stopping",
    [L_DOSING_STATE_PAUSED] = "Paused",
    [L_DOSING_STATE_FINISHED] = "Finished",
    [L_DOSING_STATE_ERROR] = "Dosing Error",
    [L_DOSING_STATE_UNKNOWN] = "Unknown",
    [L_DIAGNOSTICS_INFO_TITLE] = "Diagnostic Information",
    [L_GENERAL_STATUS_TITLE] = "General Status",
    [L_BYTES_UNIT] = "bytes",
    // New keys
    [L_AP_CONFIGURE_WIFI_NAVIGATE] = "Connect to this AP and navigate to http://%s to configure WiFi.",
    [L_LITTLEFS_MOUNT_FAILED_LOG_DISABLED] = "LittleFS Mount Failed! Log to file will be disabled.",
    [L_REBOOTED_DUE_TO_FATAL_ERROR] = "!!! System was rebooted due to fatal error: %u !!!",
    [L_FATAL_ERROR_FLAG_CLEARED_MSG] = "Fatal error flag cleared.",
    [L_PREFS_OPEN_FAIL_CLEAR_FATAL_FLAG] = "Failed to open Preferences to clear fatal error flag.",
    [L_PREFS_OPEN_FAIL_CHECK_FATAL_FLAG] = "Failed to open Preferences (namespace: %s) to check fatal error flag.",
    [L_RESET_BUTTON_PRESSED_HOLD_MSG] = "Reset button pressed. Holding for %lu ms to restart...",
    [L_RESET_BUTTON_RELEASED_CANCEL_MSG] = "Reset button released. Reset cancelled.",
    [L_RESET_BUTTON_HELD_RESTART_MSG] = "Reset button held for %lu ms. Restarting ESP...",
    [L_NO_ACTIVE_ERROR_MSG] = "No active error.",
    [L_UNKNOWN_ERROR_CODE_DESC_MSG] = "Unknown error code.",
    [L_ERROR_DESC_NO_ERROR] = "No error",
    [L_ERROR_DESC_WIFI_ERROR] = "WiFi connection/communication error",
    [L_ERROR_DESC_CALIBRATION_ERROR] = "Calibration process error",
    [L_ERROR_CAL_ACTIVE_CANNOT_DOSE] = "Calibration active. Cannot start dosing.",
    [L_ERROR_SYS_NOT_POWERED_CANNOT_DOSE] = "System not powered on. Cannot start dosing.",
    [L_ERROR_DOSING_CYCLE_BUSY_OR_ERROR] = "Dosing cycle busy or in error state.",
    [L_ERROR_FLOW_SENSOR_NOT_CALIBRATED] = "Flow sensor not calibrated (flowMlPerPulse). Calibrate.",
    [L_ERROR_DOSING_REQUESTED_SYS_NOT_POWERED] = "Dosing requested but system not powered.",
    [L_ERROR_PRECOOLING_TIMEOUT] = "Pre-cooling timeout.",
    [L_ERROR_DOSING_START_ABORTED_TOUT_FAIL] = "Dosing start aborted, T_out sensor fail.",
    [L_ERROR_MOTOR_SPEED_ZERO_CANNOT_DOSE] = "Motor speed 0, cannot start dosing.",
    [L_ERROR_MAX_DOSING_DURATION_TIMEOUT] = "Max dosing duration timeout.",
    [L_ERROR_ESPNOW_INIT_FAIL] = "ESP-NOW init failed.",
    [L_ERROR_ESPNOW_RECV_CB_REGISTER_FAIL] = "ESP-NOW recv_cb registration failed.",
    [L_ERROR_ESPNOW_PEER_ADD_READD_FAIL] = "ESP-NOW peer add/re-add failed.",
    [L_ERROR_INVALID_PEER_MAC_IN_CONFIG] = "Invalid Peer MAC in config.",
    [L_ERROR_EMERGENCY_STOP_VIA_ESPNOW] = "EMERGENCY STOP via ESP-NOW.",
    [L_WARN_ESPNOW_SEND_FAIL_AFTER_RETRIES] = "ESP-NOW send failed after retries.",
    [L_ERROR_PREFS_DEFAULTS_APPLIED_OPEN_FAIL] = "Defaults applied (Prefs open fail).",
    [L_INFO_STATISTICS_RESET_MSG] = "Statistics reset.",
    [L_ERROR_CALIBRATION_ALREADY_ACTIVE] = "Calibration already active.",
    [L_ERROR_SYS_NOT_POWERED_CANNOT_CALIBRATE] = "System not powered. Cannot start calibration.",
    [L_ERROR_DOSING_CYCLE_ACTIVE_CANNOT_CALIBRATE] = "Dosing cycle active/busy. Cannot start calibration.",
    [L_ERROR_INVALID_MOTOR_CALIBRATION_DATA] = "Invalid motor calibration data (Volume/Steps).",
    [L_ERROR_CALIBRATION_NOT_ACTIVE_MOTOR] = "Calibration not active (motor).",
    [L_ERROR_CALIBRATION_NOT_ACTIVE_FLOW] = "Calibration not active (flow).",
    [L_ERROR_INVALID_FLOW_CALIBRATION_DATA] = "Invalid flow calibration data (Volume/Pulses).",
    [L_ERROR_CALIBRATION_TIMED_OUT_MOTOR_STOPPED] = "Calibration timed out. Motor stopped.",
};

// Буфер для строк, прочитанных из PROGMEM
static char pgm_buffer[256]; // Увеличьте размер, если нужны очень длинные строки

void localization_init_default_lang() {
    // Initialize with a default language, e.g., English or Russian.
    // This allows _T() to be used before config is loaded.
    strncpy(current_active_language, DEFAULT_LANGUAGE, sizeof(current_active_language) - 1); // DEFAULT_LANGUAGE should be "en" or "ru"
    current_active_language[sizeof(current_active_language) - 1] = '\0';
    app_log_i("LOC", "Localization initialized with default language: %s", current_active_language);
}

void set_current_language(const char* lang_code) {
    if (lang_code && (strcmp(lang_code, "en") == 0 || strcmp(lang_code, "ru") == 0)) {
        strncpy(current_active_language, lang_code, sizeof(current_active_language) - 1);
        current_active_language[sizeof(current_active_language) - 1] = '\0';
    } else {
        app_log_w("LOC", "Invalid language code '%s' provided. Defaulting to '%s'.", lang_code ? lang_code : "NULL", DEFAULT_LANGUAGE);
        strncpy(current_active_language, DEFAULT_LANGUAGE, sizeof(current_active_language) - 1);
        current_active_language[sizeof(current_active_language) - 1] = '\0';
    }
    app_log_i("LOC", "Current language set to: %s", current_active_language);
}

const char* get_current_language_code() {
    return current_active_language;
}

const char* _T(LangKey key) {
    if (key >= L_KEY_COUNT) { // Базовая проверка границ
        app_log_e("LOC", "Invalid LangKey: %d", key);
        return "ERR_KEY";
    }

    const char* const* table_ptr;
    if (strcmp(current_active_language, "en") == 0) {
        table_ptr = translations_en;
    } else { // По умолчанию русский
        table_ptr = translations_ru;
    }

    // Чтение строки из PROGMEM
    const char* str_ptr_pgm = (const char*)pgm_read_ptr(&table_ptr[key]);
    
    if (str_ptr_pgm) {
        // Копируем строку из PROGMEM в статический буфер pgm_buffer
        strncpy_P(pgm_buffer, str_ptr_pgm, sizeof(pgm_buffer) - 1);
        pgm_buffer[sizeof(pgm_buffer) - 1] = '\0'; // Гарантируем нуль-терминацию
        return pgm_buffer;
    }
    app_log_e("LOC", "NULL string for LangKey: %d, Lang: %s", key, current_active_language);
    return "ERR_NULL";
}
