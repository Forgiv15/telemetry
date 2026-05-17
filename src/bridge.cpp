#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "driver/twai.h"
#include <stdlib.h>
#include <string.h>

/* ================= PIN CONFIG ================= */
#define CAN_TX_PIN 14
#define CAN_RX_PIN 13
#define CAN_RS_PIN 38   // Set LOW to enable transceiver (for SN65HVD)
#define STATUS_LED_PIN 48
#define HEARTBEAT_PIN 35

// Mode select pins:
// IO1=HIGH, IO2=LOW  -> bridge mode
// IO1=LOW,  IO2=HIGH -> test generator mode
#define MODE_SEL_IO1 1
#define MODE_SEL_IO2 2

// Test profile select pins (used only in test generator mode):
// IO15=HIGH, IO16=LOW  -> normal test signal
// IO15=LOW,  IO16=HIGH -> complex varying 200..300 kbps load
#define TEST_SEL_IO15 15
#define TEST_SEL_IO16 16

/* ================= SETTINGS ================= */
#define UART_BAUD 921600
#define TEST_LED_ACTIVE_HOLD_MS 250U
#define HEARTBEAT_TOGGLE_MS 250U

/* ================= GLOBAL ================= */
static bool can_ok = false;
static char serial_line_buffer[128];
static size_t serial_line_length = 0;
static Adafruit_NeoPixel status_led(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

enum class RunMode : uint8_t
{
    Invalid = 0,
    Bridge,
    Test
};

enum class TestProfile : uint8_t
{
    Invalid = 0,
    Normal,
    Complex
};

static RunMode active_mode = RunMode::Invalid;
static TestProfile active_profile = TestProfile::Invalid;
static uint32_t test_sequence = 0;
static uint32_t last_test_emit_us = 0;
static uint32_t last_status_ms = 0;
static uint32_t test_frames_in_window = 0;
static uint32_t last_complex_target_kbps = 0;
static uint32_t last_test_send_ms = 0;
static uint32_t last_led_toggle_ms = 0;
static uint32_t last_heartbeat_toggle_ms = 0;
static bool led_blink_phase_on = false;
static bool heartbeat_state = false;
static RunMode requested_mode = RunMode::Invalid;
static TestProfile requested_profile = TestProfile::Invalid;

static void service_heartbeat()
{
    const uint32_t now_ms = millis();

    if ((now_ms - last_heartbeat_toggle_ms) >= HEARTBEAT_TOGGLE_MS)
    {
        last_heartbeat_toggle_ms = now_ms;
        heartbeat_state = !heartbeat_state;
        digitalWrite(HEARTBEAT_PIN, heartbeat_state ? HIGH : LOW);
    }
}

static void set_status_led(uint8_t red, uint8_t green, uint8_t blue)
{
    status_led.setPixelColor(0, status_led.Color(red, green, blue));
    status_led.show();
}

static void update_status_led()
{
    const uint32_t now_ms = millis();
    const bool test_recently_active =
        (active_mode == RunMode::Test) &&
        ((now_ms - last_test_send_ms) <= TEST_LED_ACTIVE_HOLD_MS);

    if (!can_ok)
    {
        set_status_led(32, 0, 0);
        return;
    }

    if (requested_mode == RunMode::Invalid)
    {
        if ((now_ms - last_led_toggle_ms) >= 500U)
        {
            last_led_toggle_ms = now_ms;
            led_blink_phase_on = !led_blink_phase_on;
        }

        if (led_blink_phase_on)
        {
            set_status_led(16, 8, 0);
        }
        else
        {
            set_status_led(0, 0, 0);
        }
        return;
    }

    if (active_mode == RunMode::Bridge)
    {
        set_status_led(0, 0, 24);
        return;
    }

    if (requested_profile == TestProfile::Invalid)
    {
        if ((now_ms - last_led_toggle_ms) >= 200U)
        {
            last_led_toggle_ms = now_ms;
            led_blink_phase_on = !led_blink_phase_on;
        }

        if (led_blink_phase_on)
        {
            set_status_led(24, 0, 24);
        }
        else
        {
            set_status_led(0, 0, 0);
        }
        return;
    }

    if (test_recently_active)
    {
        if ((now_ms - last_led_toggle_ms) >= 100U)
        {
            last_led_toggle_ms = now_ms;
            led_blink_phase_on = !led_blink_phase_on;
        }

        if (active_profile == TestProfile::Complex)
        {
            if (led_blink_phase_on)
            {
                set_status_led(24, 8, 0);
            }
            else
            {
                set_status_led(0, 0, 0);
            }
        }
        else
        {
            if (led_blink_phase_on)
            {
                set_status_led(0, 24, 0);
            }
            else
            {
                set_status_led(0, 0, 0);
            }
        }
        return;
    }

    if (active_profile == TestProfile::Complex)
    {
        set_status_led(8, 4, 0);
    }
    else
    {
        set_status_led(0, 8, 0);
    }
}

static RunMode read_run_mode()
{
    const int io1 = digitalRead(MODE_SEL_IO1);
    const int io2 = digitalRead(MODE_SEL_IO2);

    if ((io1 == HIGH) && (io2 != HIGH))
    {
        return RunMode::Bridge;
    }

    if ((io2 == HIGH) && (io1 != HIGH))
    {
        return RunMode::Test;
    }

    return RunMode::Invalid;
}

static TestProfile read_test_profile()
{
    const int io15 = digitalRead(TEST_SEL_IO15);
    const int io16 = digitalRead(TEST_SEL_IO16);

    if ((io15 == HIGH) && (io16 != HIGH))
    {
        return TestProfile::Normal;
    }

    if ((io16 == HIGH) && (io15 != HIGH))
    {
        return TestProfile::Complex;
    }

    return TestProfile::Invalid;
}

static bool send_test_frame(uint16_t can_id, const uint8_t* data, uint8_t dlc)
{
    twai_message_t msg = {};
    msg.extd = 0;
    msg.rtr = 0;
    msg.identifier = can_id;
    msg.data_length_code = (dlc <= 8U) ? dlc : 8U;

    for (uint8_t i = 0; i < msg.data_length_code; ++i)
    {
        msg.data[i] = data[i];
    }

    if (twai_transmit(&msg, 0) == ESP_OK)
    {
        last_test_send_ms = millis();
        return true;
    }

    return false;
}

static void service_bridge_rx_to_uart()
{
    twai_message_t msg;

    if (twai_receive(&msg, pdMS_TO_TICKS(10)) != ESP_OK)
    {
        return;
    }

    uint32_t ts = millis();

    Serial.print(ts);
    Serial.print(",");

    Serial.print(msg.extd ? "E," : "S,");

    if (msg.extd)
    {
        Serial.printf("%08lX,", (unsigned long)(msg.identifier & 0x1FFFFFFF));
    }
    else
    {
        Serial.printf("%03lX,", (unsigned long)(msg.identifier & 0x7FF));
    }

    Serial.print(msg.data_length_code);
    Serial.print(",");

    for (uint8_t i = 0; i < msg.data_length_code; i++)
    {
        Serial.printf("%02X", msg.data[i]);
        if (i < msg.data_length_code - 1)
        {
            Serial.print(" ");
        }
    }

    Serial.println();
}

static void service_test_generator()
{
    const uint32_t now_us = micros();

    if (active_profile == TestProfile::Normal)
    {
        // Slow and predictable pattern for smoke-testing listeners.
        const uint32_t interval_us = 20000U; // 50 Hz
        if ((now_us - last_test_emit_us) < interval_us)
        {
            return;
        }

        last_test_emit_us = now_us;

        uint8_t data[8];
        for (uint8_t i = 0; i < 8; ++i)
        {
            data[i] = (uint8_t)((test_sequence + i) & 0xFFU);
        }

        if (send_test_frame(0x555, data, 8))
        {
            ++test_sequence;
            ++test_frames_in_window;
        }

        return;
    }

    if (active_profile == TestProfile::Complex)
    {
        // Vary effective generated bus load between ~200 and ~300 kbps.
        // Approximation uses 128 bits per standard 8-byte CAN frame.
        const uint32_t phase_ms = millis() % 4000U; // 4s triangle period
        const uint32_t rise_fall = (phase_ms <= 2000U) ? phase_ms : (4000U - phase_ms);
        const uint32_t target_kbps = 200U + ((rise_fall * 100U) / 2000U); // 200..300
        const uint32_t interval_us = (128U * 1000U) / target_kbps; // 640..426 us
        last_complex_target_kbps = target_kbps;

        uint8_t guard = 0;
        while ((micros() - last_test_emit_us) >= interval_us)
        {
            last_test_emit_us += interval_us;

            uint8_t data[8];
            data[0] = (uint8_t)(test_sequence & 0xFFU);
            data[1] = (uint8_t)((test_sequence >> 8) & 0xFFU);
            data[2] = (uint8_t)(target_kbps & 0xFFU);
            data[3] = (uint8_t)((target_kbps >> 8) & 0xFFU);
            data[4] = (uint8_t)((test_sequence * 3U) & 0xFFU);
            data[5] = (uint8_t)((test_sequence * 7U) & 0xFFU);
            data[6] = (uint8_t)(millis() & 0xFFU);
            data[7] = (uint8_t)((millis() >> 8) & 0xFFU);

            const uint16_t id = (uint16_t)(0x600U + (test_sequence & 0x0FU));
            if (!send_test_frame(id, data, 8))
            {
                break;
            }

            ++test_sequence;
            ++test_frames_in_window;

            // Prevent spending too long in this loop on a slow/blocked bus.
            ++guard;
            if (guard >= 12U)
            {
                break;
            }
        }
    }
}

static void print_status_line()
{
    const uint32_t now_ms = millis();
    if ((now_ms - last_status_ms) < 1000U)
    {
        return;
    }

    twai_status_info_t status;
    twai_get_status_info(&status);

    const char* mode_text = "INVALID";
    if (active_mode == RunMode::Bridge)
    {
        mode_text = "BRIDGE";
    }
    else if (active_mode == RunMode::Test)
    {
        mode_text = "TEST";
    }

    const char* profile_text = "N/A";
    if (active_profile == TestProfile::Normal)
    {
        profile_text = "NORMAL";
    }
    else if (active_profile == TestProfile::Complex)
    {
        profile_text = "COMPLEX";
    }

    Serial.printf("MODE,%s,PROFILE,%s,SEL,%d%d,%d%d,TXFPS,%lu,TGT_KBPS,%lu,State:%d RX:%d TX:%d ErrRX:%d ErrTX:%d\n",
                  mode_text,
                  profile_text,
                  digitalRead(MODE_SEL_IO1),
                  digitalRead(MODE_SEL_IO2),
                  digitalRead(TEST_SEL_IO15),
                  digitalRead(TEST_SEL_IO16),
                  (unsigned long)test_frames_in_window,
                  (unsigned long)last_complex_target_kbps,
                  status.state,
                  status.msgs_to_rx,
                  status.msgs_to_tx,
                  status.rx_error_counter,
                  status.tx_error_counter);

    test_frames_in_window = 0;
    last_status_ms = now_ms;
}

static bool parse_hex_byte(const char* token, uint8_t* value)
{
    char* end_ptr = nullptr;
    unsigned long parsed;

    if ((token == nullptr) || (value == nullptr))
    {
        return false;
    }

    parsed = strtoul(token, &end_ptr, 16);
    if ((end_ptr == token) || (*end_ptr != '\0') || (parsed > 0xFFUL))
    {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}

static void process_serial_command(char* line)
{
    char* tokens[5] = { nullptr };
    char* save_ptr = nullptr;
    char* token = nullptr;
    char* byte_token = nullptr;
    char* byte_save_ptr = nullptr;
    twai_message_t msg = {};
    unsigned long can_id;
    unsigned long dlc;
    size_t index = 0;

    token = strtok_r(line, ",", &save_ptr);
    while ((token != nullptr) && (index < 5U))
    {
        tokens[index++] = token;
        token = strtok_r(nullptr, ",", &save_ptr);
    }

    if ((index < 4U) || (strcmp(tokens[0], "TX") != 0))
    {
        return;
    }

    msg.extd = (strcmp(tokens[1], "E") == 0) ? 1U : 0U;
    if ((msg.extd == 0U) && (strcmp(tokens[1], "S") != 0))
    {
        Serial.println("TXERR,BAD_TYPE");
        return;
    }

    can_id = strtoul(tokens[2], nullptr, 16);
    dlc = strtoul(tokens[3], nullptr, 10);
    if (dlc > 8UL)
    {
        Serial.println("TXERR,BAD_DLC");
        return;
    }

    msg.identifier = (uint32_t)can_id;
    msg.data_length_code = (uint8_t)dlc;
    msg.rtr = 0U;
    msg.ss = 0U;
    msg.self = 0U;
    msg.dlc_non_comp = 0U;

    if (index >= 5U)
    {
        byte_token = strtok_r(tokens[4], " ", &byte_save_ptr);
        index = 0U;
        while ((byte_token != nullptr) && (index < msg.data_length_code))
        {
            if (!parse_hex_byte(byte_token, &msg.data[index]))
            {
                Serial.println("TXERR,BAD_DATA");
                return;
            }
            index++;
            byte_token = strtok_r(nullptr, " ", &byte_save_ptr);
        }

        if (index != msg.data_length_code)
        {
            Serial.println("TXERR,LEN_MISMATCH");
            return;
        }
    }
    else if (msg.data_length_code != 0U)
    {
        Serial.println("TXERR,NO_DATA");
        return;
    }

    if (twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK)
    {
        Serial.println("TXOK");
    }
    else
    {
        Serial.println("TXERR,CAN");
    }
}

static void service_serial_tx_commands(void)
{
    while (Serial.available() > 0)
    {
        char ch = (char)Serial.read();

        if ((ch == '\r') || (ch == '\n'))
        {
            if (serial_line_length > 0U)
            {
                serial_line_buffer[serial_line_length] = '\0';
                process_serial_command(serial_line_buffer);
                serial_line_length = 0U;
            }
        }
        else if (serial_line_length < (sizeof(serial_line_buffer) - 1U))
        {
            serial_line_buffer[serial_line_length++] = ch;
        }
        else
        {
            serial_line_length = 0U;
            Serial.println("TXERR,LINE_TOO_LONG");
        }
    }
}

/* ================= SETUP ================= */
void setup()
{
    Serial.begin(UART_BAUD);
    delay(500);

    pinMode(HEARTBEAT_PIN, OUTPUT);
    digitalWrite(HEARTBEAT_PIN, LOW);

    status_led.begin();
    status_led.setBrightness(32);
    set_status_led(0, 0, 0);

    Serial.println("\nESP32-S3 CAN Bridge Starting...");
    Serial.printf("PINS,CAN_TX,%d,CAN_RX,%d,CAN_RS,%d,STATUS_LED,%d,HEARTBEAT,%d\n",
                  CAN_TX_PIN,
                  CAN_RX_PIN,
                  CAN_RS_PIN,
                  STATUS_LED_PIN,
                  HEARTBEAT_PIN);

    /* Enable transceiver */
    pinMode(CAN_RS_PIN, OUTPUT);
    digitalWrite(CAN_RS_PIN, LOW);   // LOW = normal mode (for SN65HVD)

    pinMode(MODE_SEL_IO1, INPUT_PULLDOWN);
    pinMode(MODE_SEL_IO2, INPUT_PULLDOWN);
    pinMode(TEST_SEL_IO15, INPUT_PULLDOWN);
    pinMode(TEST_SEL_IO16, INPUT_PULLDOWN);

    /* CAN Configuration */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );

    g_config.tx_queue_len = 32;
    g_config.rx_queue_len = 32;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err;

    err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        Serial.printf("Driver install failed: %d\n", err);
        update_status_led();
        return;
    }

    err = twai_start();
    if (err != ESP_OK)
    {
        Serial.printf("CAN start failed: %d\n", err);
        update_status_led();
        return;
    }

    can_ok = true;
    Serial.println("CAN Started @500kbps");
    update_status_led();
}

/* ================= LOOP ================= */
void loop()
{
    service_heartbeat();
    service_serial_tx_commands();

    requested_mode = read_run_mode();
    requested_profile = read_test_profile();

    if (!can_ok)
    {
        active_mode = requested_mode;
        active_profile = requested_profile;
        update_status_led();
        delay(10);
        return;
    }

    active_mode = (requested_mode == RunMode::Invalid) ? RunMode::Test : requested_mode;
    active_profile = (requested_profile == TestProfile::Invalid) ? TestProfile::Normal : requested_profile;

    if (active_mode == RunMode::Bridge)
    {
        service_bridge_rx_to_uart();
        last_complex_target_kbps = 0;
    }
    else if (active_mode == RunMode::Test)
    {
        service_test_generator();
    }
    else
    {
        last_complex_target_kbps = 0;
    }

    update_status_led();
    print_status_line();
}
