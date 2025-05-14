#include "button_handler.h"
#include "hardware_pins.h"       // Для BTN_FWD, BTN_REV, BTN_RESET, BTN_POWER
#include "motor_control.h"       // Для manualMotorForward, manualMotorReverse, stopManualMotor, isMotorRunningManual, motor_dir (если он там)
#include "main.h"                // Для toggleSystemPower, app_log_w, app_log_i, system_power_enabled
#include "localization.h"        // Для _T() и L_* строк
#include <Arduino.h>             // Для digitalRead, millis, ESP.restart, HIGH, LOW

// --- Button Debounce and State Variables (теперь статические в этом файле) ---
static const unsigned long debounce_delay_ms = 50;
static int btn_fwd_state = HIGH, btn_rev_state = HIGH, btn_reset_state = HIGH, btn_power_state = HIGH;
static int last_btn_fwd_state = HIGH, last_btn_rev_state = HIGH, last_btn_reset_state = HIGH, last_btn_power_state = HIGH;
static unsigned long last_btn_fwd_change = 0, last_btn_rev_change = 0, last_btn_reset_change = 0, last_btn_power_change = 0;

static bool reset_requested = false;
static unsigned long reset_request_time = 0;
static const unsigned long reset_delay_ms = 5000; // Hold reset button for 5 seconds to restart

// --- Extern Global Variables (определены в Main-esp32.ino) ---
// system_power_enabled будет доступна через main.h, если она там объявлена как extern
// или если main.h включает другой заголовок, где она объявлена.
// Для прямого доступа, если она не объявлена extern в main.h:
// extern bool system_power_enabled; 
// Но лучше, чтобы main.h корректно экспортировал нужные переменные.

void handleButtons() {
    unsigned long c_ms = millis();

    int fwd = digitalRead(BTN_FWD);
    if (fwd != last_btn_fwd_state) last_btn_fwd_change = c_ms;
    if (c_ms - last_btn_fwd_change > debounce_delay_ms) {
        if (fwd != btn_fwd_state) {
            btn_fwd_state = fwd;
            if (btn_fwd_state == LOW) {
                if (isSystemPowerEnabled()) manualMotorForward(); 
            } else {
                // Предполагаем, что motor_dir и motor_running_manual доступны через motor_control.h или геттеры
                if (isMotorRunningManual() && getMotorDirection() == MOTOR_DIR_FORWARD) stopManualMotor(); 
            }
        }
    }
    last_btn_fwd_state = fwd;

    int rev = digitalRead(BTN_REV);
    if (rev != last_btn_rev_state) last_btn_rev_change = c_ms;
    if (c_ms - last_btn_rev_change > debounce_delay_ms) {
        if (rev != btn_rev_state) {
            btn_rev_state = rev;
            if (btn_rev_state == LOW) {
                if (isSystemPowerEnabled()) manualMotorReverse();
            } else {
                 if (isMotorRunningManual() && getMotorDirection() == MOTOR_DIR_REVERSE) stopManualMotor();
            }
        }
    }
    last_btn_rev_state = rev;

    int pwr = digitalRead(BTN_POWER);
    if (pwr != last_btn_power_state) last_btn_power_change = c_ms;
    if (c_ms - last_btn_power_change > debounce_delay_ms) {
        if (pwr != btn_power_state) {
            btn_power_state = pwr;
            if (btn_power_state == LOW) {
                toggleSystemPower();
            }
        }
    }
    last_btn_power_state = pwr;

    int rst = digitalRead(BTN_RESET);
    if (rst != last_btn_reset_state) last_btn_reset_change = c_ms;
    if (c_ms - last_btn_reset_change > debounce_delay_ms) {
        if (rst != btn_reset_state) {
            btn_reset_state = rst;
            if (btn_reset_state == LOW) {
                reset_requested = true;
                reset_request_time = c_ms;
                app_log_w("SYSTEM", _T(L_RESET_BUTTON_PRESSED_HOLD_MSG), reset_delay_ms);
            } else {
                reset_requested = false;
                app_log_i("SYSTEM", _T(L_RESET_BUTTON_RELEASED_CANCEL_MSG));
            }
        }
    }
    last_btn_reset_state = rst;

    if (reset_requested && (c_ms - reset_request_time > reset_delay_ms)) {
        app_log_w("SYSTEM", _T(L_RESET_BUTTON_HELD_RESTART_MSG), reset_delay_ms);
        ESP.restart();
    }
}