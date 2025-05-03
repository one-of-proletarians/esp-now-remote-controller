#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define GPIO_UP_BUTTON GPIO_NUM_0
#define GPIO_DOWN_BUTTON GPIO_NUM_4
#define GPIO_BLINK GPIO_NUM_8

#define LONG_PRESS_DURATION 700
#define LONG_PRESS_DURATION_PAUSE 600
#define MAGIC_NUMBER 0xde66
#define GPIO_INPUT_PIN_SEL BIT(GPIO_UP_BUTTON) | BIT(GPIO_DOWN_BUTTON)

void init_isr();
void init_send_sleep(uint8_t *data, size_t len, bool delay);
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void short_press_isr_handle(void *arg);
void sleep_now();
void vTaskBlink(void *);
void vTaskSend(void *);

typedef enum
{
    ON,
    OFF
} Button_t;

typedef struct
{
    int magic;
    Button_t button;
    bool longpress;
} __attribute__((packed)) Cmd_t;

SemaphoreHandle_t xSemaphoreLongPress;
SemaphoreHandle_t xSemaphoreBlink;

void app_main(void)
{
    esp_log_level_set("*", 0);

    if (ESP_SLEEP_WAKEUP_GPIO == esp_sleep_get_wakeup_cause())
    {
        xSemaphoreBlink = xSemaphoreCreateBinary();
        xSemaphoreLongPress = xSemaphoreCreateBinary();
        if (xSemaphoreLongPress)
        {
            init_isr();
            xTaskCreate(vTaskSend, "vTaskSend", 2048 * 4, NULL, 1, NULL);
            xTaskCreate(vTaskBlink, "vTaskBlink", 2048, NULL, 1, NULL);
        }
    }
    else
    {
        sleep_now();
    }
}

void vTaskSend(void *p)
{
    uint64_t bit_mask = esp_sleep_get_gpio_wakeup_status();

    gpio_num_t gpio = -1;

    Cmd_t command = {
        .magic = MAGIC_NUMBER,
        .longpress = false,
    };

    if (bit_mask == 1)
    {
        gpio = GPIO_UP_BUTTON;
        command.button = ON;
    }
    else
    {
        gpio = GPIO_DOWN_BUTTON;
        command.button = OFF;
    }

    if (gpio_get_level(gpio))
        xSemaphoreGive(xSemaphoreLongPress);

    // *********************************

    for (;;)
    {
        if (xSemaphoreTake(xSemaphoreLongPress, pdMS_TO_TICKS(LONG_PRESS_DURATION)))
            portNOP();
        else
            command.longpress = true;

        init_send_sleep((uint8_t *)&command, sizeof(command), command.longpress);
    }
}

void vTaskBlink(void *p)
{
    gpio_config_t blink_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT(GPIO_BLINK),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (;;)
    {
        if (xSemaphoreTake(xSemaphoreBlink, portMAX_DELAY))
        {
            gpio_config(&blink_conf);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(GPIO_BLINK, 1);
        }
    }
}

void sleep_now()
{
    static gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
    esp_deep_sleep_enable_gpio_wakeup(GPIO_INPUT_PIN_SEL, ESP_GPIO_WAKEUP_GPIO_LOW);

    esp_wifi_stop();
    esp_deep_sleep_start();
}

void init_isr()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(GPIO_UP_BUTTON, short_press_isr_handle, NULL);
    gpio_isr_handler_add(GPIO_DOWN_BUTTON, short_press_isr_handle, NULL);
}

void init_send_sleep(uint8_t *data, size_t len, bool delay)
{
    esp_now_peer_info_t peer = {
        .encrypt = true,
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .peer_addr = {0x28, 0x37, 0x2F, 0x69, 0x06, 0x80},
        .lmk = {0x7A, 0x3D, 0xB1, 0x4F, 0x8C, 0x92, 0xE7, 0x1A, 0x55, 0xC3, 0x6E, 0x21, 0xFA, 0x09, 0xD2, 0x88},
    };

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_now_init();
    esp_now_register_send_cb(send_cb);
    esp_now_add_peer(&peer);

    esp_now_send(peer.peer_addr, data, len);

    if (delay)
        vTaskDelay(pdMS_TO_TICKS(LONG_PRESS_DURATION_PAUSE));

    sleep_now();
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    esp_wifi_stop();
    xSemaphoreGive(xSemaphoreBlink);
}

static void IRAM_ATTR short_press_isr_handle(void *arg)
{
    xSemaphoreGiveFromISR(xSemaphoreLongPress, NULL);
}