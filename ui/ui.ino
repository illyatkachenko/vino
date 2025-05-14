//Makerfabs MTE21R0 board-template main source file

#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> // For esp_wifi_set_channel
#include <ui.h>

/*Don't forget to set Sketchbook location in File/Preferences to the path of your UI project (the parent foder of this INO file)*/


enum BoardConstants {
    LVGL_BUFFER_RATIO = 6,
    I2C_SDA_PIN=17, I2C_SCL_PIN=18,
    TFT_BL=38,
    BUTTON_PIN=14,
    ESP_NOW_CHANNEL = 1, // Define a channel for ESP-NOW
    ENCODER_CLK=13, // CLK
    ENCODER_DT=10, // DT
};


static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 480;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf [screenWidth * screenHeight / LVGL_BUFFER_RATIO];

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    1 /* CS */, 46 /* SCK */, 0 /* SDA */,
    2 /* DE */, 42 /* VSYNC */, 3 /* HSYNC */, 45 /* PCLK */,
    11 /* R0 */, 15 /* R1 */, 12 /* R2 */, 16 /* R3 */, 21 /* R4 */,
    39 /* G0/P22 */, 7 /* G1/P23 */, 47 /* G2/P24 */, 8 /* G3/P25 */, 48 /* G4/P26 */, 9 /* G5 */,
    4 /* B0 */, 41 /* B1 */, 5 /* B2 */, 40 /* B3 */, 6 /* B4 */
);

// Uncomment for 2.1" round display
Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
    bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */,
    false /* IPS */, 480 /* width */, 480 /* height */,
    st7701_type5_init_operations, sizeof(st7701_type5_init_operations),
    true /* BGR */,
    10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);

//Encoder state-variables:
volatile int encoder_counter = 0; // Renamed from 'counter' for clarity, updated by ISR
int old_Encoder_CLK_State;      // Stores the previous state of ENCODER_CLK for ISR

// --- ESP-NOW Configuration ---
// REPLACE WITH THE MAC ADDRESS OF YOUR MAIN ESP32
uint8_t peerAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // TODO: Замените этим MAC-адресом основного ESP32
esp_now_peer_info_t peerInfo;

// Command types sent FROM screen TO main ESP32
typedef enum {
    CMD_SET_TEMPERATURE,
    CMD_SET_VOLUME,
    CMD_START_PROCESS, // New: to initiate the whole dosing cycle
    CMD_PAUSE,
    CMD_RESUME,
    CMD_STOP_PROCESS,  // New: to stop and reset
} command_type_t;

typedef struct struct_command {
    command_type_t cmd_type;
    int value; // For temperature or volume, or other data
} struct_command_t;

struct_command_t currentCommand; // To hold data to be sent


// Status structure received BY screen FROM main ESP32
typedef enum {
    STATE_UNKNOWN = 0,
    STATE_LOADING,
    STATE_HELLO,
    STATE_IDLE, 
    STATE_SETTINGS_TEMP, 
    STATE_SETTINGS_VOLUME, 
    STATE_COOLING,
    STATE_READY,
    STATE_FILLING,
    STATE_PAUSED,
    STATE_COMPLETE,
    STATE_ERROR,
    STATE_GOODBYE, 
} system_state_t;

typedef struct struct_status {
    float current_temperature;
    int current_volume_ml;
    int target_volume_ml;
    system_state_t system_state;
    uint16_t error_code;
    int current_setpoint_temperature;
    int current_setpoint_volume;
} struct_status_t;

volatile struct_status_t received_status;
volatile bool new_status_received = false;
SemaphoreHandle_t status_mutex; // For protecting received_status

// Forward declaration for UI update function
void handle_ui_update(const struct_status_t* status);

void setup ()
{
    Serial.begin(115200); /* prepare for possible serial debug */

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

        status_mutex = xSemaphoreCreateMutex();
    if (status_mutex == NULL) {
        Serial.println("Failed to create status_mutex!");
        // Handle error: perhaps loop forever or restart
    }


    pin_init();
    gfx->begin();

    lv_init();
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / LVGL_BUFFER_RATIO );


    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    // encoder
    static lv_indev_drv_t indev_drv2;
    lv_indev_drv_init( &indev_drv2 );
    indev_drv2.type = LV_INDEV_TYPE_ENCODER;
    indev_drv2.read_cb = my_encoder_read;
    lv_indev_t *encoder_indev = lv_indev_drv_register( &indev_drv2 );
      
    lv_group_t * g = lv_group_create();
    lv_group_set_default(g); 
    lv_indev_set_group(encoder_indev, g); 

    // Initialize ESP-NOW
    espNowInit();

    ui_init();

    Serial.println( "Setup done" );


    xTaskCreatePinnedToCore( Task_TFT, "Task_TFT", 20480, NULL, 3, NULL, 0 );
}


void loop ()
{
}


// -------------------------------------------------------------------------------------

void Task_TFT (void *pvParameters)
{
    struct_status_t local_status_copy; 

    while (1)
    {
        if (new_status_received) {
            if (xSemaphoreTake(status_mutex, (TickType_t)10) == pdTRUE) {
                memcpy(&local_status_copy, (void*)&received_status, sizeof(struct_status_t));
                new_status_received = false; 
                xSemaphoreGive(status_mutex);
                
                // LVGL operations should be in the LVGL task or protected.
                // Since this IS the LVGL task (lv_timer_handler is here), direct calls are okay.
                handle_ui_update(&local_status_copy);
        lv_timer_handler();
            }
        }
        lv_timer_handler(); // Moved outside the if block
                    }
        }
        lv_timer_handler(); // Moved outside the if block
        vTaskDelay(pdMS_TO_TICKS(10)); // Moved outside the if block
    }
}
//------------------------------------------------------------------------

void pin_init ()
{
    pinMode( TFT_BL, OUTPUT );
    digitalWrite( TFT_BL, HIGH );

    pinMode( ENCODER_CLK, INPUT_PULLUP );
    pinMode( ENCODER_DT, INPUT_PULLUP );
    pinMode( BUTTON_PIN, INPUT_PULLUP ); // Initialize encoder button pin
    old_Encoder_CLK_State = digitalRead( ENCODER_CLK );

    attachInterrupt( ENCODER_CLK, encoder_irq, CHANGE );
    
    Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN );
}


/* Display flushing */
void my_disp_flush (lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap( area->x1, area->y1, (uint16_t*) &color_p->full, w, h );
#else
    gfx->draw16bitRGBBitmap( area->x1, area->y1, (uint16_t*) &color_p->full, w, h );
#endif

    lv_disp_flush_ready(disp);
}

void my_encoder_read (lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    // Encoder Rotation
    noInterrupts(); // Temporarily disable interrupts to safely read shared variable
    data->enc_diff = encoder_counter;
    encoder_counter = 0; // Reset counter after reading
    interrupts(); // Re-enable interrupts

    // Encoder Button
    if (digitalRead(BUTTON_PIN) == LOW) { // Assuming button is active LOW
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_ENTER; // Standard key for encoder press
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// Interrupt Service Routine for the encoder
void encoder_irq()
{
    int current_Encoder_CLK_State = digitalRead( ENCODER_CLK );
    if (current_Encoder_CLK_State != old_Encoder_CLK_State) // If ENCODER_CLK has changed
    {
        // Check ENCODER_DT to determine direction
        // This logic depends on your specific encoder type (CW/CCW pulses)
        // This is a common way: if DT is different from CLK, count one way, else the other.
        if (digitalRead(ENCODER_DT) != current_Encoder_CLK_State) {
            encoder_counter++; // Clockwise
        }
        else {
            encoder_counter--; // Counter-clockwise
        }
    }
    old_Encoder_CLK_State = current_Encoder_CLK_State; // Save current CLK state for next interrupt
}

// --- ESP-NOW Functions ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_status_t)) {
    if (xSemaphoreTake(status_mutex, (TickType_t)10) == pdTRUE) {
        memcpy((void*)&received_status, incomingData, sizeof(received_status));
        new_status_received = true;
        xSemaphoreGive(status_mutex);
        // Serial.printf("Status Recv: Temp: %.1f, State: %d\n", received_status.current_temperature, received_status.system_state);
    } else {
        Serial.println("OnDataRecv: Could not take status_mutex");
    }
  } else {
    Serial.printf("Received data of unexpected length: %d, expected: %d\n", len, sizeof(struct_status_t));
  }
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  Serial.print("Screen ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  if (esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Error setting Wi-Fi channel");
    return;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv); // Register receive callback

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;  // Use the same channel
  peerInfo.encrypt = false; // No encryption for simplicity
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW Initialized for bi-directional communication.");
}

void send_command_via_espnow(command_type_t type, int value) {
  currentCommand.cmd_type = type;
  currentCommand.value = value;
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &currentCommand, sizeof(currentCommand));
  
  if (result == ESP_OK) {
    Serial.printf("ESP-NOW send CMD: %d, VAL: %d - Success\n", type, value);
  } else {
    Serial.printf("ESP-NOW send CMD: %d, VAL: %d - Error: %s\n", type, value, esp_err_to_name(result));
  }
}

// --- C-callable wrapper functions for UI events ---
extern "C" void ui_event_send_temperature_setpoint(int temp) {
    send_command_via_espnow(CMD_SET_TEMPERATURE, temp);
}

extern "C" void ui_event_send_volume_setpoint(int ml) {
    send_command_via_espnow(CMD_SET_VOLUME, ml);
}

extern "C" void ui_event_send_start_process() {
    send_command_via_espnow(CMD_START_PROCESS, 0); 
}

extern "C" void ui_event_send_pause_process() {
    send_command_via_espnow(CMD_PAUSE, 0);
}

extern "C" void ui_event_send_resume_process() {
    send_command_via_espnow(CMD_RESUME, 0);
}

extern "C" void ui_event_send_stop_process() {
    send_command_via_espnow(CMD_STOP_PROCESS, 0);
}

void handle_ui_update(const struct_status_t* status) {
    if (ui_Main_Screen && lv_obj_is_valid(ui_Main_Screen)) { 
        if (ui_Label6 && lv_obj_is_valid(ui_Label6)) {
            lv_label_set_text_fmt(ui_Label6, "%d°C", status->current_setpoint_temperature); 
        }
        if (ui_volume_ && lv_obj_is_valid(ui_volume_)) {
            lv_label_set_text_fmt(ui_volume_, "%d ml", status->current_setpoint_volume); 
        }
    }
    
    if (ui_Temperature && lv_obj_is_valid(ui_Temperature) && ui_Arc1 && lv_obj_is_valid(ui_Arc1)) {
        if (lv_arc_get_value(ui_Arc1) != status->current_setpoint_temperature) {
            lv_arc_set_value(ui_Arc1, status->current_setpoint_temperature);
            if(ui_Label2 && lv_obj_is_valid(ui_Label2)) lv_label_set_text_fmt(ui_Label2, "%d", status->current_setpoint_temperature);
        }
    }

    if (ui_Volume && lv_obj_is_valid(ui_Volume) && ui_Arc3 && lv_obj_is_valid(ui_Arc3)) {
         if (lv_arc_get_value(ui_Arc3) != status->current_setpoint_volume) {
            lv_arc_set_value(ui_Arc3, status->current_setpoint_volume);
            if(ui_Label3 && lv_obj_is_valid(ui_Label3)) lv_label_set_text_fmt(ui_Label3, "%d", status->current_setpoint_volume);
        }
    }

    lv_obj_t* current_screen = lv_scr_act();

    switch (status->system_state) {
        case STATE_IDLE:
            if (current_screen != ui_Main_Screen && current_screen != ui_Temperature && current_screen != ui_Volume) {
                 _ui_screen_change( &ui_Main_Screen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_Main_Screen_screen_init);
            }
            break;
        case STATE_COOLING:
            if (current_screen != ui_Cooling_Down) _ui_screen_change(&ui_Cooling_Down, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_Cooling_Down_screen_init);
            if (ui_Label7 && lv_obj_is_valid(ui_Label7)) lv_label_set_text_fmt(ui_Label7, "Cooling\n%.1f°C", status->current_temperature);
            break;
        case STATE_READY:
            if (current_screen != ui_ready) _ui_screen_change(&ui_ready, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_ready_screen_init);
            if (ui_Label9 && lv_obj_is_valid(ui_Label9)) lv_label_set_text_fmt(ui_Label9, "Ready\n%.1f°C", status->current_temperature);
            break;
        case STATE_FILLING:
            if (current_screen != ui_Filling) _ui_screen_change(&ui_Filling, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_Filling_screen_init);
            if (ui_Label8 && lv_obj_is_valid(ui_Label8)) lv_label_set_text_fmt(ui_Label8, "Filling\n%d/%d ml", status->current_volume_ml, status->target_volume_ml);
            break;
        case STATE_PAUSED:
            if (current_screen != ui_pause) _ui_screen_change(&ui_pause, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_pause_screen_init);
            break;
        case STATE_COMPLETE:
            if (current_screen != ui_goodbye1) _ui_screen_change(&ui_goodbye1, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_goodbye1_screen_init);
            if (ui_Goodbye && lv_obj_is_valid(ui_Goodbye)) lv_label_set_text(ui_Goodbye, "Complete!");
            break;
        case STATE_ERROR:
            if (current_screen != ui_errors1) _ui_screen_change(&ui_errors1, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_errors1_screen_init);
            if (ui_errors && lv_obj_is_valid(ui_errors)) { // ui_errors is the container
                lv_obj_t* error_label = lv_obj_get_child(ui_errors, 0); 
                if (!error_label && lv_obj_is_valid(ui_errors)) { 
                    error_label = lv_label_create(ui_errors);
                    lv_obj_center(error_label);
                    lv_obj_set_style_text_font(error_label, &lv_font_montserrat_24, 0);
                    lv_label_set_long_mode(error_label, LV_LABEL_LONG_WRAP);
                    lv_obj_set_width(error_label, lv_pct(90));
                }
                if(error_label && lv_obj_is_valid(error_label)) lv_label_set_text_fmt(error_label, "Error: %d", status->error_code);
            }
            break;
        case STATE_HELLO:
            if (current_screen != ui_Hello) _ui_screen_change(&ui_Hello, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_Hello_screen_init);
            break;
        case STATE_LOADING: // Should be handled by ui_init
            if (current_screen != ui_Loading_screen && current_screen != ui_Hello) _ui_screen_change(&ui_Loading_screen, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Loading_screen_screen_init);
            break;
        default:
            if (current_screen != ui_Main_Screen && current_screen != ui_Temperature && current_screen != ui_Volume) {
                 _ui_screen_change(&ui_Main_Screen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_Main_Screen_screen_init);
            }
            break;
    }
}