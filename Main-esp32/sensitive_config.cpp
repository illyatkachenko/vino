#include "sensitive_config.h"
#include "main.h" // Для g_preferences_operational и функций логирования
#include "main.h" // Для функций логирования log_x

// Статические переменные для хранения чувствительных данных
// Используем статические массивы для безопасности и фиксированного размера
static char wifi_ssid_buffer[MAX_SSID_LEN + 1] = "";
static char wifi_password_buffer[MAX_PASSWORD_LEN + 1] = "";
static char web_username_buffer[MAX_USERNAME_LEN + 1] = "";
static char web_password_buffer[MAX_PASSWORD_LEN + 1] = ""; // Пароль веб-сервера может быть длинным

// Значения по умолчанию (КРИТИЧНО: Измените эти значения по умолчанию!)
// В идеале, при первом запуске должен быть пустой пароль или требование его установки.
// Для примера оставим те, что были в main.c и web_server_handlers.h
const char* DEFAULT_WIFI_SSID = "Home";       // <-- ЗАМЕНИ НА СВОЙ SSID!
const char* DEFAULT_WIFI_PASSWORD = "nautilus"; // <-- ЗАМЕНИ НА СВОЙ ПАРОЛЬ!
const char* DEFAULT_WEB_USERNAME = "admin";
const char* DEFAULT_WEB_PASSWORD = "YOUR_STRONG_PASSWORD"; // <-- ОБЯЗАТЕЛЬНО ЗАМЕНИТЕ!

void loadSensitiveConfig() {
    if (!g_preferences_operational) {
        log_e("SECRETS_LOAD", "Preferences not operational! Using hardcoded defaults."); // Более явное сообщение об ошибке
        log_w("SECRETS", "Skipping loadSensitiveConfig because Preferences are not operational. Using defaults.");
        strncpy(wifi_ssid_buffer, DEFAULT_WIFI_SSID, MAX_SSID_LEN); wifi_ssid_buffer[MAX_SSID_LEN] = '\0';
        strncpy(wifi_password_buffer, DEFAULT_WIFI_PASSWORD, MAX_PASSWORD_LEN); wifi_password_buffer[MAX_PASSWORD_LEN] = '\0';
        strncpy(web_username_buffer, DEFAULT_WEB_USERNAME, MAX_USERNAME_LEN); web_username_buffer[MAX_USERNAME_LEN] = '\0';
        strncpy(web_password_buffer, DEFAULT_WEB_PASSWORD, MAX_PASSWORD_LEN); web_password_buffer[MAX_PASSWORD_LEN] = '\0';
        return;
    }
    log_i("SECRETS_LOAD", "Attempting to load sensitive config...");
    Preferences preferences;
    bool defaults_applied = false;
    bool prefs_begun_ok = false;

    if (!preferences.begin(SECRETS_NAMESPACE, true)) { // Read-only mode
        g_preferences_operational = false; // Отмечаем, что Preferences не работают
        log_e("SECRETS_LOAD", "Error opening preferences namespace '%s' in R/O mode. Using hardcoded defaults.", SECRETS_NAMESPACE);
        defaults_applied = true;
    } else {
        prefs_begun_ok = true;
        log_i("SECRETS_LOAD", "Preferences namespace '%s' opened successfully in R/O mode.", SECRETS_NAMESPACE);
        // Загрузка WiFi
        String loaded_ssid = preferences.getString("wifi_ssid", DEFAULT_WIFI_SSID);
        String loaded_pass = preferences.getString("wifi_pass", DEFAULT_WIFI_PASSWORD);
        log_i("SECRETS_LOAD", "From NVS: loaded_ssid='%s', loaded_pass_len=%d", loaded_ssid.c_str(), loaded_pass.length());

        strncpy(wifi_ssid_buffer, loaded_ssid.c_str(), MAX_SSID_LEN);
        wifi_ssid_buffer[MAX_SSID_LEN] = '\0';
        log_i("SECRETS_LOAD", "Buffer after strncpy: wifi_ssid_buffer='%s'", wifi_ssid_buffer);

        strncpy(wifi_password_buffer, loaded_pass.c_str(), MAX_PASSWORD_LEN);
        wifi_password_buffer[MAX_PASSWORD_LEN] = '\0';

        // Загрузка Web Auth
        // Логирование для web_user и web_pass можно добавить по аналогии, если потребуется
        String loaded_web_user = preferences.getString("web_user", DEFAULT_WEB_USERNAME);
        String loaded_web_pass = preferences.getString("web_pass", DEFAULT_WEB_PASSWORD);
        strncpy(web_username_buffer, loaded_web_user.c_str(), MAX_USERNAME_LEN);
        web_username_buffer[MAX_USERNAME_LEN] = '\0';
        strncpy(web_password_buffer, loaded_web_pass.c_str(), MAX_PASSWORD_LEN);
        web_password_buffer[MAX_PASSWORD_LEN] = '\0';

        preferences.end();

        // Проверка, были ли загружены значения по умолчанию из-за отсутствия ключей в NVS
        if (strcmp(loaded_ssid.c_str(), DEFAULT_WIFI_SSID) == 0 && strcmp(loaded_pass.c_str(), DEFAULT_WIFI_PASSWORD) == 0) {
             log_w("SECRETS_LOAD", "WiFi credentials loaded from NVS match DEFAULT values. NVS might have been empty or contained defaults.");
             // defaults_applied = true; // Не устанавливаем здесь, если NVS был успешно прочитан, но содержал дефолты
        } else {
             log_i("SECRETS_LOAD", "WiFi credentials loaded from NVS differ from DEFAULTS.");
        }
    }

    if (defaults_applied) { // Это сработает, если preferences.begin() не удался
        log_w("SECRETS_LOAD", "Using hardcoded defaults because preferences.begin() failed.");
        strncpy(wifi_ssid_buffer, DEFAULT_WIFI_SSID, MAX_SSID_LEN); wifi_ssid_buffer[MAX_SSID_LEN] = '\0';
        strncpy(wifi_password_buffer, DEFAULT_WIFI_PASSWORD, MAX_PASSWORD_LEN); wifi_password_buffer[MAX_PASSWORD_LEN] = '\0';
        strncpy(web_username_buffer, DEFAULT_WEB_USERNAME, MAX_USERNAME_LEN); web_username_buffer[MAX_USERNAME_LEN] = '\0';
        strncpy(web_password_buffer, DEFAULT_WEB_PASSWORD, MAX_PASSWORD_LEN); web_password_buffer[MAX_PASSWORD_LEN] = '\0';
        log_i("SECRETS_LOAD", "After applying hardcoded defaults: SSID_buffer='%s'", wifi_ssid_buffer);
    }

    if (defaults_applied && prefs_begun_ok) { // Если preferences.begin() был успешен, но значения были дефолтными (например, NVS пуст)
        // Если использовались значения по умолчанию, сохраним их в NVS для следующего раза
        log_i("SECRETS", "Saving default sensitive config to NVS.");
        saveSensitiveConfig();
    }
}

void saveSensitiveConfig() {
    if (!g_preferences_operational) {
        log_w("SECRETS", "Skipping saveSensitiveConfig because Preferences are not operational.");
        return;
    }

    log_i("SECRETS", "Saving sensitive config...");
    Preferences preferences;
    if (!preferences.begin(SECRETS_NAMESPACE, false)) { // Read-write mode
        g_preferences_operational = false; // Отмечаем, что Preferences не работают
        log_e("SECRETS", "Error opening preferences namespace '%s' for writing. Sensitive config NOT saved.", SECRETS_NAMESPACE);
        // Не вызываем setSystemError, чтобы избежать потенциальных проблем,
        // так как это низкоуровневая функция сохранения.
        return;
    }

    log_i("SECRETS_SAVE", "Saving wifi_ssid_buffer: '%s'", wifi_ssid_buffer);
    preferences.putString("wifi_ssid", wifi_ssid_buffer);
    preferences.putString("wifi_pass", wifi_password_buffer);
    log_i("SECRETS_SAVE", "Saving wifi_password_buffer_len: %d", strlen(wifi_password_buffer));
    preferences.putString("web_user", web_username_buffer);
    preferences.putString("web_pass", web_password_buffer);

    preferences.end();
    log_i("SECRETS", "Sensitive config saved.");
}

void initSensitiveConfig() {
    loadSensitiveConfig(); // При инициализации сразу загружаем
}

// Геттеры
const char* getWifiSsid() {
    log_i("SECRETS_GET", "getWifiSsid called, returning: '%s'", wifi_ssid_buffer);
    return wifi_ssid_buffer;
}

const char* getWifiPassword() {
    log_i("SECRETS_GET", "getWifiPassword called, returning password of length: %d", strlen(wifi_password_buffer));
    return wifi_password_buffer;
}

const char* getWebUsername() {
    return web_username_buffer;
}

const char* getWebPassword() {
    return web_password_buffer;
}

// Сеттеры
void setWifiSsid(const char* new_ssid) {
    if (new_ssid) {
        strncpy(wifi_ssid_buffer, new_ssid, MAX_SSID_LEN);
        wifi_ssid_buffer[MAX_SSID_LEN] = '\0'; // Убедимся в null-терминации
        log_i("SECRETS_SET", "setWifiSsid: input='%s', buffer_after_set='%s'", new_ssid, wifi_ssid_buffer);
    }
}

void setWifiPassword(const char* new_password) {
    // Обновляем пароль, если new_password не NULL. Пустая строка означает "без пароля".
    if (new_password) {
        strncpy(wifi_password_buffer, new_password, MAX_PASSWORD_LEN);
        wifi_password_buffer[MAX_PASSWORD_LEN] = '\0'; // Убедимся в null-терминации
        log_i("SECRETS_SET", "setWifiPassword: input_len=%d, buffer_after_set_len=%d", strlen(new_password), strlen(wifi_password_buffer));
        if (strlen(new_password) == 0) {
            log_i("SECRETS", "WiFi Password buffer updated in memory to EMPTY password.");
        } else {
            log_i("SECRETS", "WiFi Password buffer updated in memory.");
        }
    }
}