#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"

/* =====================================================================
 *  PART 1: DS18B20 temperature sensor over 1-Wire (unchanged logic)
 * ===================================================================== */

#define OW_PIN 4 // GPIO pin where the DS18B20 is connected

static portMUX_TYPE ow_mux = portMUX_INITIALIZER_UNLOCKED; // protects microsecond-timed pulses from being interrupted

// Pull the line low for us microseconds, then let go (the pull-up raises it).
static void ow_low(uint32_t us)
{
    gpio_set_level(OW_PIN, 0);
    esp_rom_delay_us(us);
    gpio_set_level(OW_PIN, 1);
}

// Reset pulse; returns 1 if a sensor answered with a presence pulse
static int ow_reset(void)
{
    int present;
    portENTER_CRITICAL(&ow_mux);
    ow_low(480); // "hello?"
    esp_rom_delay_us(70);
    present = !gpio_get_level(OW_PIN); // sensor pulls low = "I'm here"
    portEXIT_CRITICAL(&ow_mux);
    esp_rom_delay_us(410);
    return present;
}

static void ow_write_bit(int bit)
{
    portENTER_CRITICAL(&ow_mux);
    if (bit)
    {
        ow_low(6);
        esp_rom_delay_us(64);
    } // short low = 1
    else
    {
        ow_low(60);
        esp_rom_delay_us(10);
    } // long low  = 0
    portEXIT_CRITICAL(&ow_mux);
}

static int ow_read_bit(void)
{
    int bit;
    portENTER_CRITICAL(&ow_mux);
    ow_low(6); // start the time slot
    esp_rom_delay_us(9);
    bit = gpio_get_level(OW_PIN); // sensor holds low for 0, releases for 1
    portEXIT_CRITICAL(&ow_mux);
    esp_rom_delay_us(55);
    return bit;
}

static void ow_write_byte(uint8_t b)
{
    for (int i = 0; i < 8; i++)
    {
        ow_write_bit(b & 0x01);
        b >>= 1;
    } // LSB first
}

static uint8_t ow_read_byte(void)
{
    uint8_t b = 0;
    for (int i = 0; i < 8; i++)
        b |= (ow_read_bit() << i);
    return b;
}

static float ds18b20_read_temp(void)
{
    if (!ow_reset())
        return -999.0f; // no sensor on first handshake
    ow_write_byte(0xCC);
    ow_write_byte(0x44);
    vTaskDelay(pdMS_TO_TICKS(800)); // a little margin over 750 ms

    if (!ow_reset())
        return -998.0f; // sensor vanished on second handshake
    ow_write_byte(0xCC);
    ow_write_byte(0xBE);
    uint8_t lsb = ow_read_byte();
    uint8_t msb = ow_read_byte();
    int16_t raw = (msb << 8) | lsb;
    return raw / 16.0f;
}

/* =====================================================================
 *  PART 2: 16x2 LCD through the I2C backpack (unchanged logic)
 * ===================================================================== */

#define I2C_SDA 8
#define I2C_SCL 9
#define LCD_ADDR 0x27
#define LCD_BL 0x08 // backlight bit
#define LCD_EN 0x04 // enable bit
#define LCD_RS 0x01 // register select bit

static i2c_master_dev_handle_t lcd;

static void i2c_lcd_setup(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    i2c_new_master_bus(&bus_cfg, &bus);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus, &dev_cfg, &lcd);
}

// Send one half-byte (upper 4 bits of 'hi'), pulsing EN to latch it
static void lcd_write_nibble(uint8_t hi, uint8_t rs)
{
    uint8_t d = (hi & 0xF0) | LCD_BL | rs;
    uint8_t seq[2] = {d | LCD_EN, d}; // EN high, then EN low
    i2c_master_transmit(lcd, seq, 2, 100);
}

static void lcd_send(uint8_t byte, uint8_t rs)
{
    lcd_write_nibble(byte & 0xF0, rs);        // high nibble first
    lcd_write_nibble((byte << 4) & 0xF0, rs); // then low nibble
}

static void lcd_cmd(uint8_t c)
{
    lcd_send(c, 0); // RS=0: command
    if (c <= 0x03)
        vTaskDelay(pdMS_TO_TICKS(2)); // clear/home are slow
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x30, 0);
    lcd_write_nibble(0x20, 0); // switch the LCD into 4-bit mode
    lcd_cmd(0x28);             // 4-bit, 2 lines, 5x8 font
    lcd_cmd(0x0C);             // display on, cursor off
    lcd_cmd(0x06);             // cursor moves right
    lcd_cmd(0x01);             // clear screen
}

// Write a string on row 0 or 1, padding with spaces to wipe old text.
// NOTE: once the tasks are running, callers must hold lcd_mutex.
static void lcd_print_raw(int row, const char *s)
{
    lcd_cmd(row == 0 ? 0x80 : 0xC0); // cursor to start of line 1 / line 2
    int n = 0;
    while (*s && n < 16)
    {
        lcd_send(*s++, LCD_RS);
        n++;
    }
    while (n < 16)
    {
        lcd_send(' ', LCD_RS);
        n++;
    }
}

/* =====================================================================
 *  PART 3: shared state between tasks
 * ===================================================================== */

// Two tasks want the LCD (sensor task and LED task), so we guard it with a mutex.
static SemaphoreHandle_t lcd_mutex;
static bool lcd_override = false;    // true while a LED message owns the screen
static char last_temp_line[17] = ""; // latest temperature text, used to restore the screen

/* =====================================================================
 *  PART 4: status LEDs
 * ===================================================================== */

#define BLUE_LED 5
#define RED_LED 6

typedef enum
{
    LED_EVT_BLUE,
    LED_EVT_RED
} led_evt_t;

static QueueHandle_t led_queue; // keypad task -> LED task

static void leds_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BLUE_LED) | (1ULL << RED_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cfg);
    gpio_set_level(BLUE_LED, 0);
    gpio_set_level(RED_LED, 0);
}

/* =====================================================================
 *  PART 5: 4x4 keypad, interrupt-driven
 *
 *  Idle ("armed") state: ALL rows driven LOW, columns are inputs with
 *  pull-ups. Pressing any key connects a LOW row to a column, so that
 *  column falls HIGH->LOW and raises a falling-edge interrupt.
 *  The ISR does one tiny thing: drop a note in a queue.
 *  A normal task then works out WHICH key it was.
 * ===================================================================== */

static const int ROW_PINS[4] = {10, 11, 12, 13};
static const int COL_PINS[4] = {14, 15, 16, 17};
static const char KEYS[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

static QueueHandle_t key_evt_queue; // ISR -> keypad task

// The ISR: short, fast, no printf, no LCD, no waiting.
static void IRAM_ATTR key_isr(void *arg)
{
    uint32_t pin = (uint32_t)(uintptr_t)arg;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(key_evt_queue, &pin, &woken);
    portYIELD_FROM_ISR(woken);
}

static void keypad_arm_rows(void)
{
    for (int r = 0; r < 4; r++)
        gpio_set_level(ROW_PINS[r], 0);
}

static void keypad_init(void)
{
    uint64_t row_mask = 0, col_mask = 0;
    for (int i = 0; i < 4; i++)
    {
        row_mask |= 1ULL << ROW_PINS[i];
        col_mask |= 1ULL << COL_PINS[i];
    }
    gpio_config_t rows = {
        .pin_bit_mask = row_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&rows);

    gpio_config_t cols = {
        .pin_bit_mask = col_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // columns rest HIGH
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE}; // fire when a column falls HIGH -> LOW
    gpio_config(&cols);

    keypad_arm_rows();

    gpio_install_isr_service(0);
    for (int c = 0; c < 4; c++)
        gpio_isr_handler_add(COL_PINS[c], key_isr, (void *)(uintptr_t)COL_PINS[c]);
}

// Identify the pressed key by scanning row by row. Returns 0 if none.
// Leaves all rows HIGH when it finishes (caller re-arms them).
static char keypad_scan(void)
{
    for (int r = 0; r < 4; r++)
        gpio_set_level(ROW_PINS[r], 1); // every row idle HIGH first

    for (int r = 0; r < 4; r++)
    {
        gpio_set_level(ROW_PINS[r], 0); // ground just this row
        esp_rom_delay_us(50);           // let the line settle
        for (int c = 0; c < 4; c++)
        {
            if (gpio_get_level(COL_PINS[c]) == 0)
            { // a 0 means pressed
                gpio_set_level(ROW_PINS[r], 1);
                return KEYS[r][c];
            }
        }
        gpio_set_level(ROW_PINS[r], 1); // release the row
    }
    return 0;
}

static void keypad_task(void *arg)
{
    uint32_t pin;
    while (1)
    {
        // Sleep until the ISR drops a note in the queue (uses no CPU while waiting)
        if (xQueueReceive(key_evt_queue, &pin, portMAX_DELAY) != pdTRUE)
            continue;

        vTaskDelay(pdMS_TO_TICKS(30)); // let the contact bounce settle (debounce)

        // Our scan wiggles the row pins, which would trigger the interrupt
        // itself, so silence the column interrupts while we scan.
        for (int c = 0; c < 4; c++)
            gpio_intr_disable(COL_PINS[c]);

        char k = keypad_scan();
        if (k)
        {
            printf("Key pressed: %c\n", k);
            led_evt_t evt;
            if (k == '5')
            {
                evt = LED_EVT_BLUE;
                xQueueSend(led_queue, &evt, 0);
            }
            else if (k == 'D')
            {
                evt = LED_EVT_RED;
                xQueueSend(led_queue, &evt, 0);
            }
        }

        // Wait for the key to be released so one press = one event
        while (keypad_scan() != 0)
            vTaskDelay(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(30)); // release debounce

        // Re-arm: rows LOW, throw away bounce leftovers, interrupts back on
        keypad_arm_rows();
        xQueueReset(key_evt_queue);
        for (int c = 0; c < 4; c++)
            gpio_intr_enable(COL_PINS[c]);
    }
}

/* =====================================================================
 *  PART 6: the LED task (handles the ON / OFF message cycle)
 * ===================================================================== */

static void led_task(void *arg)
{
    led_evt_t evt;
    while (1)
    {
        if (xQueueReceive(led_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        int pin = (evt == LED_EVT_BLUE) ? BLUE_LED : RED_LED;
        const char *on_msg = (evt == LED_EVT_BLUE) ? "BLUE LED ON" : "RED LED ON";
        const char *off_msg = (evt == LED_EVT_BLUE) ? "BLUE LED OFF" : "RED LED OFF";

        // Take over the screen
        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
        lcd_override = true;
        lcd_print_raw(0, on_msg);
        lcd_print_raw(1, "");
        xSemaphoreGive(lcd_mutex);

        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(3000)); // LED stays on for a few seconds

        gpio_set_level(pin, 0);
        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
        lcd_print_raw(0, off_msg);
        xSemaphoreGive(lcd_mutex);
        vTaskDelay(pdMS_TO_TICKS(1500)); // brief "OFF" message

        // Hand the screen back to the live temperature
        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
        lcd_print_raw(0, "Temperature:");
        lcd_print_raw(1, last_temp_line);
        lcd_override = false;
        xSemaphoreGive(lcd_mutex);
    }
}

/* =====================================================================
 *  PART 7: the sensor task (background telemetry, never blocked by keys)
 * ===================================================================== */

static void sensor_task(void *arg)
{
    while (1)
    {
        float t = ds18b20_read_temp();
        char line[17];
        if (t < -900)
            snprintf(line, sizeof line, "Sensor error");
        else
            snprintf(line, sizeof line, "%.2f C", t);

        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
        strcpy(last_temp_line, line);
        if (!lcd_override) // only draw if no LED message is on screen
            lcd_print_raw(1, line);
        xSemaphoreGive(lcd_mutex);

        printf("%s\n", line); // serial telemetry keeps flowing no matter what
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* =====================================================================
 *  app_main: set everything up, start the tasks, then step aside
 * ===================================================================== */

void app_main(void)
{
    // 1-Wire data pin: open-drain, external 4.7k pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OW_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(OW_PIN, 1);

    // Create the synchronization objects BEFORE anything can use them
    lcd_mutex = xSemaphoreCreateMutex();
    key_evt_queue = xQueueCreate(8, sizeof(uint32_t));
    led_queue = xQueueCreate(4, sizeof(led_evt_t));

    leds_init();
    i2c_lcd_setup();
    lcd_init();
    lcd_print_raw(0, "Temperature:");

    xTaskCreate(sensor_task, "sensor", 4096, NULL, 4, NULL);
    xTaskCreate(led_task, "led", 3072, NULL, 5, NULL);
    xTaskCreate(keypad_task, "keypad", 3072, NULL, 6, NULL);

    keypad_init(); // last: interrupts go live once the queue and tasks exist
}
