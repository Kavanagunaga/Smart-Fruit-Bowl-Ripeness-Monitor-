#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "bme68x.h"
#include "driver/gpio.h"
#include "esp_sntp.h"

/* ============================================================
 * CONFIG
 * ============================================================ */
#define SDA_PIN             5
#define SCL_PIN             6
#define LED_EMPTY           GPIO_NUM_1
#define LED_FRESH           GPIO_NUM_2
#define LED_RIPENING        GPIO_NUM_3
#define LED_OVERRIPE        GPIO_NUM_4
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         100000
#define BME_ADDR            BME68X_I2C_ADDR_LOW

#define WIFI_SSID           "kavana"
#define WIFI_PASS           "12345678"
#define WIFI_CONNECT_TIMEOUT_MS 15000

#define MQTT_BROKER_URI     "mqtt://broker.emqx.io:1883"
#define MQTT_TOPIC_DATA     "bme680/data"
#define MQTT_TOPIC_DECISION "bme680/decision"
#define MQTT_QOS            1
#define MQTT_RETAIN         0

#define WARMUP_MS           (2UL * 60UL * 1000UL)
#define SAMPLE_RATE_HZ      1
#define SAMPLE_INTERVAL_US  (1000000ULL / SAMPLE_RATE_HZ)
#define SAMPLES_PER_CYCLE   60
#define SLEEP_DURATION_US   (12ULL * 60ULL * 1000000ULL)

#define DECISION_WINDOW_SAMPLES 120

#define HEATER_TEMP_C       320
#define HEATER_DUR_MS       150
#define AMB_TEMP_C          25

#define BME_OS_TEMP         BME68X_OS_2X
#define BME_OS_HUM          BME68X_OS_1X
#define BME_IIR_FILTER      BME68X_FILTER_SIZE_3

#define MQTT_PAYLOAD_BUF_SIZE   256
#define MQTT_DECISION_BUF_SIZE  256

typedef struct {
    uint32_t timestamp;
    float temperature;
    float humidity;
    float gas_resistance;
} sample_t;

RTC_DATA_ATTR static sample_t sample_buffer[DECISION_WINDOW_SAMPLES];
RTC_DATA_ATTR static uint16_t head = 0;
RTC_DATA_ATTR static uint16_t tail = 0;
RTC_DATA_ATTR static uint16_t count = 0;
RTC_DATA_ATTR static uint32_t elapsed_seconds = 0;
RTC_DATA_ATTR static uint32_t sample_count = 0;
RTC_DATA_ATTR static uint32_t decision_counter = 0;
RTC_DATA_ATTR static uint32_t boot_count = 0;
RTC_DATA_ATTR static float gas_baseline = 0.0f;
RTC_DATA_ATTR static float hum_baseline = 0.0f;
RTC_DATA_ATTR static bool baseline_set = false;

typedef struct {
    float temperature;
    float humidity;
    float gas_resistance;
    uint8_t heat_stable;
    uint8_t gas_valid;
    uint8_t new_data;
} bme_reading_t;

static EventGroupHandle_t wifi_eg;
#define WIFI_CONNECTED_BIT BIT0

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static struct bme68x_dev bme;
static struct bme68x_conf conf;
static struct bme68x_heatr_conf heatr;

static SemaphoreHandle_t sample_sem;
static esp_timer_handle_t sample_timer;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile int decision_msg_id = -1;
static volatile bool decision_published = false;

/* ============================================================
 * TIME
 * ============================================================ */
static void time_sync_notification_cb(struct timeval *tv)
{
    printf("[TIME] SNTP synchronized\n");
}

static void init_ist_timezone(void)
{
    setenv("TZ", "IST-5:30", 1);
    tzset();
}

static void obtain_time(void)
{
    init_ist_timezone();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 15;

    while (timeinfo.tm_year < (2024 - 1900) && retry < retry_count) {
        retry++;
        printf("[TIME] Waiting for SNTP sync... (%d/%d)\n", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2024 - 1900)) {
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        printf("[TIME] IST: %s\n", buf);
    } else {
        printf("[TIME] Failed to get valid wall clock time\n");
    }
}

static void get_current_ist_time(char *buf, size_t len)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &timeinfo);
}

/* ============================================================
 * I2C CALLBACKS
 * ============================================================ */
BME68X_INTF_RET_TYPE bme_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 1000);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

BME68X_INTF_RET_TYPE bme_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(dev_handle, buf, len + 1, 1000);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

void bme_wait_time(uint32_t us, void *intf_ptr)
{
    vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
}

static void sample_timer_cb(void *arg)
{
    xSemaphoreGive(sample_sem);
}

/* ============================================================
 * INIT
 * ============================================================ */
static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

static bool bme_init(void)
{
    bme.read = bme_i2c_read;
    bme.write = bme_i2c_write;
    bme.delay_us = bme_wait_time;
    bme.intf = BME68X_I2C_INTF;
    bme.intf_ptr = NULL;
    bme.amb_temp = AMB_TEMP_C;

    if (bme68x_init(&bme) != BME68X_OK) {
        printf("[BME] Init failed\n");
        return false;
    }

    conf.os_hum = BME_OS_HUM;
    conf.os_temp = BME_OS_TEMP;
    conf.os_pres = BME68X_OS_NONE;
    conf.filter = BME_IIR_FILTER;
    conf.odr = BME68X_ODR_NONE;
    bme68x_set_conf(&conf, &bme);

    heatr.enable = BME68X_ENABLE;
    heatr.heatr_temp = HEATER_TEMP_C;
    heatr.heatr_dur = HEATER_DUR_MS;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr, &bme);

    printf("[BME] Init OK — heater=%dC/%dms\n", HEATER_TEMP_C, HEATER_DUR_MS);
    return true;
}

static bool bme_read_one(bme_reading_t *out)
{
    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

    uint32_t measurement_time_us =
        bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (HEATER_DUR_MS * 1000);

    vTaskDelay(pdMS_TO_TICKS((measurement_time_us + 999) / 1000));

    struct bme68x_data data;
    uint8_t n_fields = 0;

    if (bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme) != BME68X_OK || n_fields == 0) {
        return false;
    }

    out->new_data = (data.status & BME68X_NEW_DATA_MSK) ? 1 : 0;
    out->gas_valid = (data.status & BME68X_GASM_VALID_MSK) ? 1 : 0;
    out->heat_stable = (data.status & BME68X_HEAT_STAB_MSK) ? 1 : 0;
    out->temperature = data.temperature;
    out->humidity = data.humidity;
    out->gas_resistance = data.gas_resistance;
    return true;
}

static void bme_sleep(void)
{
    bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme);
    printf("[BME] Sleep mode set\n");
}

/* ============================================================
 * WIFI + MQTT
 * ============================================================ */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT Connected\n");
            break;

        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT Disconnected\n");
            break;

        case MQTT_EVENT_PUBLISHED:
            printf("MQTT Published, msg_id=%d\n", event->msg_id);
            if (event->msg_id == decision_msg_id) {
                decision_published = true;
            }
            break;

        default:
            break;
    }
}

static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("WiFi disconnected\n");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        printf("WiFi connected\nIP acquired\n");
        xEventGroupSetBits(wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_eg = xEventGroupCreate();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("Connecting to WiFi...\n");
}

/* ============================================================
 * BUFFER + REGRESSION
 * ============================================================ */
static void ring_buffer_push(const sample_t *s)
{
    sample_buffer[tail] = *s;
    tail = (tail + 1) % DECISION_WINDOW_SAMPLES;
    if (count < DECISION_WINDOW_SAMPLES) {
        count++;
    } else {
        head = (head + 1) % DECISION_WINDOW_SAMPLES;
    }
    sample_count++;
}

static float compute_mean_gas(void)
{
    if (count == 0) return 0.0f;

    double sum = 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (head + i) % DECISION_WINDOW_SAMPLES;
        sum += sample_buffer[idx].gas_resistance;
    }

    return (float)(sum / count);
}

static float compute_mean_humidity(void)
{
    if (count == 0) return 0.0f;

    double sum = 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (head + i) % DECISION_WINDOW_SAMPLES;
        sum += sample_buffer[idx].humidity;
    }

    return (float)(sum / count);
}

#define BANANA_GAS_DROP_FRESH       2.0f
#define BANANA_GAS_DROP_RIPENING    5.0f
#define BANANA_GAS_DROP_RIPE        10.0f

#define BANANA_HUM_RISE_CONFIRM     1.5f

static const char *classify_fruit(float gas_drop_pct,
                                  float hum_rise)
{
    gpio_set_level(LED_EMPTY, 0);
    gpio_set_level(LED_FRESH, 0);
    gpio_set_level(LED_RIPENING, 0);
    gpio_set_level(LED_OVERRIPE, 0);

    if (gas_drop_pct < BANANA_GAS_DROP_FRESH)
    {
        gpio_set_level(LED_FRESH, 1);
        return "FRESH";
    }
    else if (gas_drop_pct < BANANA_GAS_DROP_RIPENING)
    {
        if (hum_rise > BANANA_HUM_RISE_CONFIRM) {
            gpio_set_level(LED_RIPENING, 1);
            return "EARLY_RIPENING";
        } else {
            gpio_set_level(LED_FRESH, 1);
            return "FRESH";
        }
    }
    else if (gas_drop_pct < BANANA_GAS_DROP_RIPE)
    {
        gpio_set_level(LED_RIPENING, 1);
        return "RIPENING";
    }
    else
    {
        gpio_set_level(LED_OVERRIPE, 1);
        return "OVERRIPE";
    }
}

/* ============================================================
 * MQTT PUBLISH
 * ============================================================ */
static void mqtt_publish_sample(const sample_t *s)
{
    if (!mqtt_client) return;

    char ist_buf[32];
    char buf[MQTT_PAYLOAD_BUF_SIZE];
    get_current_ist_time(ist_buf, sizeof(ist_buf));

    snprintf(buf, sizeof(buf),
             "{\"timestamp\":%lu,"
             "\"ist\":\"%s\","
             "\"temp\":%.2f,"
             "\"hum\":%.2f,"
             "\"gas\":%.0f}",
             (unsigned long)s->timestamp,
             ist_buf,
             s->temperature,
             s->humidity,
             s->gas_resistance);

    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, buf, 0, MQTT_QOS, MQTT_RETAIN);
    printf("MQTT [data]: %s\n", buf);
}

static void mqtt_publish_decision(uint32_t timestamp,
                                  uint32_t samples,
                                  float gas_drop_pct,
                                  float hum_rise,
                                  const char *condition)
{
    if (!mqtt_client) return;

    char ist_buf[32];
    char buf[MQTT_DECISION_BUF_SIZE];
    get_current_ist_time(ist_buf, sizeof(ist_buf));

    snprintf(buf, sizeof(buf),
         "{\"timestamp\":%lu,"
         "\"ist\":\"%s\","
         "\"samples\":%lu,"
         "\"gas_drop_pct\":%.2f,"
         "\"hum_rise\":%.2f,"
         "\"condition\":\"%s\"}",
         (unsigned long)timestamp,
         ist_buf,
         (unsigned long)samples,
         gas_drop_pct,
         hum_rise,
         condition);

    decision_published = false;
    decision_msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DECISION, buf, 0, MQTT_QOS, MQTT_RETAIN);

    printf("MQTT [decision]: %s\n", buf);
    printf("Decision msg_id=%d\n", decision_msg_id);

    vTaskDelay(pdMS_TO_TICKS(10000));
    printf("[MQTT] Extra 10s wait completed\n");

}

/* ============================================================
 * LED + SLEEP
 * ============================================================ */
static void led_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << LED_EMPTY) |
            (1ULL << LED_FRESH) |
            (1ULL << LED_RIPENING) |
            (1ULL << LED_OVERRIPE)
    };
    gpio_config(&io_conf);

    gpio_set_level(LED_EMPTY, 0);
    gpio_set_level(LED_FRESH, 0);
    gpio_set_level(LED_RIPENING, 0);
    gpio_set_level(LED_OVERRIPE, 0);
}

static void enter_deep_sleep(void)
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);

    printf("[MCU] Deep sleep for 12 min...\n\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    esp_deep_sleep_start();
}

/* ============================================================
 * MAIN
 * ============================================================ */
void app_main(void)
{
    boot_count++;

    if (boot_count > 1) {
        elapsed_seconds += (SLEEP_DURATION_US / 1000000ULL);
    }

    printf("\n========================================\n");
    printf("Boot #%lu\n", (unsigned long)boot_count);
    printf("========================================\n");

    wifi_init();
    xEventGroupWaitBits(wifi_eg, WIFI_CONNECTED_BIT, false, true, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    obtain_time();

    mqtt_init();
    vTaskDelay(pdMS_TO_TICKS(2000));

    i2c_init();
    led_init();

    if (!bme_init()) {
        printf("[ERR] BME680 failed — entering deep sleep\n");
        enter_deep_sleep();
        return;
    }

    printf("\n[WARMUP] Running for 2 min...\n");

    sample_sem = xSemaphoreCreateBinary();

    esp_timer_create_args_t timer_args = {
        .callback = sample_timer_cb,
        .name = "sample_timer",
    };
    esp_timer_create(&timer_args, &sample_timer);
    esp_timer_start_periodic(sample_timer, SAMPLE_INTERVAL_US);

    uint32_t warmup_ticks = WARMUP_MS / (1000 / SAMPLE_RATE_HZ);
    for (uint32_t i = 0; i < warmup_ticks; i++) {
        xSemaphoreTake(sample_sem, portMAX_DELAY);
        bme_reading_t r;
        bme_read_one(&r);
    }

    esp_timer_stop(sample_timer);
    printf("[WARMUP] Done\n");

    printf("\n[READ] Collecting %d valid samples @ %d Hz...\n", SAMPLES_PER_CYCLE, SAMPLE_RATE_HZ);

    esp_timer_start_periodic(sample_timer, SAMPLE_INTERVAL_US);

    uint32_t valid_collected = 0;
    uint32_t skip_count = 0;
    uint32_t read_attempts = 0;

    while (valid_collected < SAMPLES_PER_CYCLE) {
        xSemaphoreTake(sample_sem, portMAX_DELAY);

        bme_reading_t r;
        read_attempts++;

        if (!bme_read_one(&r)) {
            printf("[READ] Measurement failed (attempt %lu)\n", (unsigned long)read_attempts);
            continue;
        }

        if (!r.heat_stable || !r.gas_valid) {
            skip_count++;
            printf("[SKIP] heat_stab=%d gas_valid=%d\n", r.heat_stable, r.gas_valid);
            continue;
        }

        elapsed_seconds++;
        uint32_t ts = elapsed_seconds;

        sample_t s = {
            .timestamp = ts,
            .temperature = r.temperature,
            .humidity = r.humidity,
            .gas_resistance = r.gas_resistance,
        };

        valid_collected++;
        ring_buffer_push(&s);
        mqtt_publish_sample(&s);

        printf("[READ] Sample %lu | rel_ts=%lu | gas=%.0f\n",
               (unsigned long)valid_collected,
               (unsigned long)s.timestamp,
               s.gas_resistance);

        if ((sample_count % DECISION_WINDOW_SAMPLES) == 0) {
            decision_counter++;
            float mean_gas = compute_mean_gas();
            float mean_hum = compute_mean_humidity();

            if (!baseline_set)
            {
                gas_baseline = mean_gas;
                hum_baseline = mean_hum;
                baseline_set = true;

                printf("[BASELINE] Gas=%.0f Hum=%.2f\n",
                    gas_baseline,
                    hum_baseline);

                mqtt_publish_decision(
                    s.timestamp,
                    DECISION_WINDOW_SAMPLES,
                    0.0f,
                    0.0f,
                    "CALIBRATING");

                vTaskDelay(pdMS_TO_TICKS(5000));

                continue;
            }

            float gas_drop_pct =
                ((gas_baseline - mean_gas) / gas_baseline) * 100.0f;

            float hum_rise =
                mean_hum - hum_baseline;

            const char *condition = classify_fruit(gas_drop_pct, hum_rise);

            printf("[DECISION] Window #%lu | MeanGas=%.0f | Drop=%.2f%% | HumRise=%.2f | %s\n",
                    (unsigned long)decision_counter,
                    mean_gas,
                    gas_drop_pct,
                    hum_rise,
                    condition);
                   

            mqtt_publish_decision(s.timestamp,
                      DECISION_WINDOW_SAMPLES,
                      gas_drop_pct,
                      hum_rise,
                      condition);

            TickType_t start = xTaskGetTickCount();
            TickType_t timeout = pdMS_TO_TICKS(3000);

            while (!decision_published && (xTaskGetTickCount() - start < timeout)) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            if (decision_published) {
                printf("[DECISION] Publish ACK received\n");
            } else {
                printf("[DECISION] Publish ACK timeout\n");
            }
        }
    }

    esp_timer_stop(sample_timer);
    esp_timer_delete(sample_timer);
    vSemaphoreDelete(sample_sem);

    printf("\n[READ] Done — %lu valid samples collected, %lu skipped\n",
           (unsigned long)valid_collected,
           (unsigned long)skip_count);

    bme_sleep();
    enter_deep_sleep();
}
