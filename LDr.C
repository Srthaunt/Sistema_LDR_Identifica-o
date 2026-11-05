
#include "ldr_object_detector.h"

#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "ldr_detector";

#define LDR_MUTEX_TIMEOUT_TICKS pdMS_TO_TICKS(100)
#define LDR_MAX_SAMPLES         64
#define LDR_SPI_MAX_CLOCK_HZ    2000000
#define LDR_SPI_SAFE_CLOCK_HZ   1000000   /* limite garantido com VDD = 3,3 V */

#if defined(CONFIG_LDR_SPI_HOST_SPI2)
#define LDR_DEFAULT_SPI_HOST SPI2_HOST
#else
#define LDR_DEFAULT_SPI_HOST SPI3_HOST
#endif

/* Dados de cada sensor */
typedef struct {
    uint8_t  mcp_channel;

    uint16_t threshold;
    uint16_t hysteresis;
    uint16_t baseline;
    bool     baseline_valid;
    bool     presence_when_darker;

    uint16_t adc_value;        /* ultima media */
    bool     present;          /* estado confirmado */
    bool     pending;          /* mudanca aguardando confirmacao */
    int64_t  pending_since_us;
} ldr_sensor_t;

static struct {
    bool                initialized;
    SemaphoreHandle_t   mutex;
    spi_host_device_t   host;
    bool                bus_ready;
    spi_device_handle_t spi;
    ldr_sensor_t        sensor[LDR_SENSOR_COUNT];
    uint8_t             samples;
    uint32_t            confirm_ms;
} s_ctx;

/* ------------------------------ Utilidades ------------------------------ */

#define CHECK_INIT() \
    do { if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE; } while (0)
#define CHECK_ID(id) \
    do { if ((id) >= LDR_SENSOR_COUNT) return ESP_ERR_INVALID_ARG; } while (0)
#define LOCK() \
    do { if (xSemaphoreTake(s_ctx.mutex, LDR_MUTEX_TIMEOUT_TICKS) != pdTRUE) return ESP_ERR_TIMEOUT; } while (0)
#define UNLOCK() xSemaphoreGive(s_ctx.mutex)

static void release_resources(void)
{
    if (s_ctx.spi) {
        spi_bus_remove_device(s_ctx.spi);
        s_ctx.spi = NULL;
    }
    if (s_ctx.bus_ready) {
        spi_bus_free(s_ctx.host);
        s_ctx.bus_ready = false;
    }
    if (s_ctx.mutex) {
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
    }
}

/* ------------------------------ MCP3208 (SPI) ------------------------------ */

static esp_err_t mcp3208_read_channel(uint8_t ch, uint16_t *out)
{
    if (ch >= LDR_MCP3208_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 24,                                   /* bits */
        .tx_data = {
            (uint8_t)(0x06 | (ch >> 2)),                /* START + SGL + D2 */
            (uint8_t)((ch & 0x03) << 6),                /* D1 D0 */
            0x00,
        },
    };

    esp_err_t err = spi_device_polling_transmit(s_ctx.spi, &t);
    if (err != ESP_OK) {
        return err;
    }

    *out = (uint16_t)(((t.rx_data[1] & 0x0F) << 8) | t.rx_data[2]);
    return ESP_OK;
}

/* Media de n amostras de um sensor. Amostras com erro sao ignoradas. */
static esp_err_t sample_sensor(const ldr_sensor_t *s, uint8_t n, int *out)
{
    uint32_t sum = 0;
    uint32_t ok = 0;
    esp_err_t last_err = ESP_FAIL;

    for (uint8_t i = 0; i < n; i++) {
        uint16_t raw = 0;
        esp_err_t err = mcp3208_read_channel(s->mcp_channel, &raw);
        if (err == ESP_OK) {
            sum += raw;
            ok++;
        } else {
            last_err = err;
        }
    }
    if (ok == 0) {
        return last_err;
    }
    *out = (int)((sum + ok / 2) / ok);
    return ESP_OK;
}

/* Histerese + tempo de confirmacao. Chamada com o mutex retido. */
static void update_state(ldr_sensor_t *s, int adc, int64_t now_us)
{
    int thr = s->threshold;
    int hys = s->hysteresis;
    bool candidate;

    if (!s->present) {
        /* Para entrar em PRESENCA, ultrapassa o limite pelo lado da presenca + histerese */
        candidate = s->presence_when_darker ? (adc < thr - hys) : (adc > thr + hys);
    } else {
        /* Para sair de PRESENCA, ultrapassa o limite pelo lado oposto + histerese */
        bool release = s->presence_when_darker ? (adc > thr + hys) : (adc < thr - hys);
        candidate = !release;
    }

    if (candidate == s->present) {
        s->pending = false;
        return;
    }

    if (!s->pending) {
        s->pending = true;
        s->pending_since_us = now_us;
    }
    if ((now_us - s->pending_since_us) >= (int64_t)s_ctx.confirm_ms * 1000LL) {
        s->present = candidate;
        s->pending = false;
    }
}

/* Media longa para calibracao (sem reter o mutex). */
static esp_err_t measure_average(uint8_t id, uint16_t *out)
{
    const ldr_sensor_t *s = &s_ctx.sensor[id];
    uint32_t sum = 0;
    uint32_t ok = 0;
    TickType_t delay = pdMS_TO_TICKS(CONFIG_LDR_CALIB_SAMPLE_DELAY_MS);
    if (delay == 0) {
        delay = 1;
    }

    for (int i = 0; i < CONFIG_LDR_CALIB_SAMPLES; i++) {
        int v = 0;
        if (sample_sensor(s, 1, &v) == ESP_OK) {
            sum += (uint32_t)v;
            ok++;
        }
        vTaskDelay(delay);
    }
    if (ok == 0) {
        return ESP_FAIL;
    }
    *out = (uint16_t)((sum + ok / 2) / ok);
    return ESP_OK;
}

/* ------------------------------ Inicializacao ------------------------------ */

esp_err_t ldr_matrix_get_default_config(ldr_matrix_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t ch[LDR_SENSOR_COUNT] = {
        CONFIG_LDR_MCP_CH_1, CONFIG_LDR_MCP_CH_2, CONFIG_LDR_MCP_CH_3, CONFIG_LDR_MCP_CH_4,
        CONFIG_LDR_MCP_CH_5, CONFIG_LDR_MCP_CH_6, CONFIG_LDR_MCP_CH_7, CONFIG_LDR_MCP_CH_8,
    };
    const uint16_t thr[LDR_SENSOR_COUNT] = {
        CONFIG_LDR_THRESHOLD_1, CONFIG_LDR_THRESHOLD_2, CONFIG_LDR_THRESHOLD_3, CONFIG_LDR_THRESHOLD_4,
        CONFIG_LDR_THRESHOLD_5, CONFIG_LDR_THRESHOLD_6, CONFIG_LDR_THRESHOLD_7, CONFIG_LDR_THRESHOLD_8,
    };
    const uint16_t hys[LDR_SENSOR_COUNT] = {
        CONFIG_LDR_HYSTERESIS_1, CONFIG_LDR_HYSTERESIS_2, CONFIG_LDR_HYSTERESIS_3, CONFIG_LDR_HYSTERESIS_4,
        CONFIG_LDR_HYSTERESIS_5, CONFIG_LDR_HYSTERESIS_6, CONFIG_LDR_HYSTERESIS_7, CONFIG_LDR_HYSTERESIS_8,
    };

    memset(cfg, 0, sizeof(*cfg));
    cfg->spi_host = LDR_DEFAULT_SPI_HOST;
    cfg->gpio_sclk = CONFIG_LDR_SPI_SCLK_GPIO;
    cfg->gpio_miso = CONFIG_LDR_SPI_MISO_GPIO;
    cfg->gpio_mosi = CONFIG_LDR_SPI_MOSI_GPIO;
    cfg->gpio_cs = CONFIG_LDR_SPI_CS_GPIO;
    cfg->spi_clock_hz = CONFIG_LDR_SPI_CLOCK_HZ;

    memcpy(cfg->mcp_channel, ch, sizeof(ch));
    memcpy(cfg->threshold, thr, sizeof(thr));
    memcpy(cfg->hysteresis, hys, sizeof(hys));
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
#ifdef CONFIG_LDR_PRESENCE_WHEN_DARKER
        cfg->presence_when_darker[i] = true;
#else
        cfg->presence_when_darker[i] = false;
#endif
    }

    cfg->samples = CONFIG_LDR_ADC_SAMPLES;
    cfg->confirm_time_ms = CONFIG_LDR_CONFIRM_TIME_MS;
    return ESP_OK;
}

esp_err_t ldr_matrix_init(void)
{
    ldr_matrix_config_t cfg;
    esp_err_t err = ldr_matrix_get_default_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    return ldr_matrix_init_with_config(&cfg);
}

/* GPIO 6..11 estao ligados a flash SPI interna do modulo. */
static bool gpio_is_flash_pin(int gpio)
{
    return gpio >= 6 && gpio <= 11;
}

static esp_err_t validate_config(const ldr_matrix_config_t *cfg)
{
    if (cfg->spi_host != SPI2_HOST && cfg->spi_host != SPI3_HOST) {
        ESP_LOGE(TAG, "spi_host deve ser SPI2_HOST ou SPI3_HOST");
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(cfg->gpio_sclk) || !GPIO_IS_VALID_OUTPUT_GPIO(cfg->gpio_mosi) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(cfg->gpio_cs) || !GPIO_IS_VALID_GPIO(cfg->gpio_miso)) {
        ESP_LOGE(TAG, "GPIO SPI invalido (SCLK/MOSI/CS precisam ser pinos de saida)");
        return ESP_ERR_INVALID_ARG;
    }
    if (gpio_is_flash_pin(cfg->gpio_sclk) || gpio_is_flash_pin(cfg->gpio_miso) ||
        gpio_is_flash_pin(cfg->gpio_mosi) || gpio_is_flash_pin(cfg->gpio_cs)) {
        ESP_LOGE(TAG, "GPIO 6..11 sao da flash interna e nao podem ser usados");
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->gpio_sclk == cfg->gpio_miso || cfg->gpio_sclk == cfg->gpio_mosi ||
        cfg->gpio_sclk == cfg->gpio_cs   || cfg->gpio_miso == cfg->gpio_mosi ||
        cfg->gpio_miso == cfg->gpio_cs   || cfg->gpio_mosi == cfg->gpio_cs) {
        ESP_LOGE(TAG, "GPIOs SPI devem ser todos diferentes");
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->spi_clock_hz == 0 || cfg->spi_clock_hz > LDR_SPI_MAX_CLOCK_HZ) {
        ESP_LOGE(TAG, "spi_clock_hz invalido (1..%d)", LDR_SPI_MAX_CLOCK_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->spi_clock_hz > LDR_SPI_SAFE_CLOCK_HZ) {
        ESP_LOGW(TAG, "Clock SPI %lu Hz acima de 1 MHz: com VDD=3,3 V o MCP3208 nao garante a conversao",
                 (unsigned long)cfg->spi_clock_hz);
    }
    if (cfg->samples < 1 || cfg->samples > LDR_MAX_SAMPLES) {
        ESP_LOGE(TAG, "samples invalido (1..%d)", LDR_MAX_SAMPLES);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        if (cfg->mcp_channel[i] >= LDR_MCP3208_CHANNELS) {
            ESP_LOGE(TAG, "LDR %d: canal MCP3208 invalido (0..7)", i + 1);
            return ESP_ERR_INVALID_ARG;
        }
        if (cfg->threshold[i] > LDR_ADC_MAX_VALUE || cfg->hysteresis[i] > LDR_ADC_MAX_VALUE) {
            ESP_LOGE(TAG, "LDR %d: threshold/histerese fora de 0..4095", i + 1);
            return ESP_ERR_INVALID_ARG;
        }
        for (int j = 0; j < i; j++) {
            if (cfg->mcp_channel[j] == cfg->mcp_channel[i]) {
                ESP_LOGE(TAG, "Canal CH%d usado por mais de um LDR", cfg->mcp_channel[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}

esp_err_t ldr_matrix_init_with_config(const ldr_matrix_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = validate_config(cfg);
    if (err != ESP_OK) {
        return err;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.host = cfg->spi_host;
    s_ctx.samples = cfg->samples;
    s_ctx.confirm_ms = cfg->confirm_time_ms;

    s_ctx.mutex = xSemaphoreCreateMutex();
    if (!s_ctx.mutex) {
        return ESP_ERR_NO_MEM;
    }

    /* 1) Barramento SPI (sem DMA: transferencias de apenas 3 bytes) */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->gpio_mosi,
        .miso_io_num = cfg->gpio_miso,
        .sclk_io_num = cfg->gpio_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
    err = spi_bus_initialize(cfg->spi_host, &bus_cfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar barramento SPI: %s", esp_err_to_name(err));
        release_resources();
        return err;
    }
    s_ctx.bus_ready = true;

    /* 2) MCP3208 como dispositivo SPI modo 0 (CPOL=0, CPHA=0), CS por hardware */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = (int)cfg->spi_clock_hz,
        .mode = 0,
        .spics_io_num = cfg->gpio_cs,
        .queue_size = 1,
    };
    err = spi_bus_add_device(cfg->spi_host, &dev_cfg, &s_ctx.spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar MCP3208 ao SPI: %s", esp_err_to_name(err));
        release_resources();
        return err;
    }

    /* 3) Parametros dos sensores */
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        ldr_sensor_t *s = &s_ctx.sensor[i];
        s->mcp_channel = cfg->mcp_channel[i];
        s->threshold = cfg->threshold[i];
        s->hysteresis = cfg->hysteresis[i];
        s->presence_when_darker = cfg->presence_when_darker[i];
    }

    /* 4) Leitura inicial (apenas valor, sem alterar estado) + diagnostico basico */
    int n_zero = 0;
    int n_full = 0;
    int n_ok = 0;
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        int v = 0;
        if (sample_sensor(&s_ctx.sensor[i], s_ctx.samples, &v) == ESP_OK) {
            s_ctx.sensor[i].adc_value = (uint16_t)v;
            n_ok++;
            if (v == 0) {
                n_zero++;
            }
            if (v == LDR_ADC_MAX_VALUE) {
                n_full++;
            }
        }
    }
    if (n_ok == 0) {
        ESP_LOGE(TAG, "Nenhum canal do MCP3208 respondeu");
    } else if (n_zero == n_ok || n_full == n_ok) {
        ESP_LOGW(TAG, "Todos os canais leram %s: verifique ligacao SPI, VDD/VREF e CS do MCP3208",
                 n_zero == n_ok ? "0" : "4095");
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "Inicializado: MCP3208 em SPI%d (SCLK=%d MISO=%d MOSI=%d CS=%d, %lu Hz), "
             "%u amostras, confirmacao %lu ms",
             (int)cfg->spi_host + 1, cfg->gpio_sclk, cfg->gpio_miso, cfg->gpio_mosi, cfg->gpio_cs,
             (unsigned long)cfg->spi_clock_hz, (unsigned)s_ctx.samples, (unsigned long)s_ctx.confirm_ms);
    return ESP_OK;
}

esp_err_t ldr_matrix_deinit(void)
{
    CHECK_INIT();
    s_ctx.initialized = false;
    release_resources();
    memset(&s_ctx, 0, sizeof(s_ctx));
    return ESP_OK;
}

/* -------------------------------- Leitura -------------------------------- */

esp_err_t ldr_matrix_read_all(void)
{
    CHECK_INIT();
    LOCK();

    esp_err_t result = ESP_OK;
    int64_t now_us = esp_timer_get_time();

    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        ldr_sensor_t *s = &s_ctx.sensor[i];
        int adc = 0;
        esp_err_t err = sample_sensor(s, s_ctx.samples, &adc);
        if (err != ESP_OK) {
            if (result == ESP_OK) {
                result = err;
            }
            continue;   /* mantem ultimo valor/estado */
        }
        s->adc_value = (uint16_t)adc;
        update_state(s, adc, now_us);
    }

    UNLOCK();
    return result;
}

esp_err_t ldr_get_presence(uint8_t sensor_id, bool *presence)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);
    if (!presence) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    *presence = s_ctx.sensor[sensor_id].present;
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_get_all_presence(uint8_t *mask)
{
    CHECK_INIT();
    if (!mask) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    uint8_t m = 0;
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        if (s_ctx.sensor[i].present) {
            m |= (uint8_t)(1u << i);
        }
    }
    UNLOCK();
    *mask = m;
    return ESP_OK;
}

esp_err_t ldr_get_matrix_string(char *buf, size_t buf_len)
{
    if (!buf || buf_len < (LDR_SENSOR_COUNT + 1)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t mask = 0;
    esp_err_t err = ldr_get_all_presence(&mask);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        buf[i] = (mask & (1u << i)) ? '1' : '0';
    }
    buf[LDR_SENSOR_COUNT] = '\0';
    return ESP_OK;
}

esp_err_t ldr_get_adc_value(uint8_t sensor_id, uint16_t *adc_value)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);
    if (!adc_value) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    *adc_value = s_ctx.sensor[sensor_id].adc_value;
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_get_all_adc_values(uint16_t *values)
{
    CHECK_INIT();
    if (!values) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        values[i] = s_ctx.sensor[i].adc_value;
    }
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_get_info(uint8_t sensor_id, ldr_sensor_info_t *info)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    const ldr_sensor_t *s = &s_ctx.sensor[sensor_id];
    info->mcp_channel = s->mcp_channel;
    info->adc_value = s->adc_value;
    info->threshold = s->threshold;
    info->hysteresis = s->hysteresis;
    info->baseline = s->baseline;
    info->baseline_valid = s->baseline_valid;
    info->presence = s->present;
    info->presence_when_darker = s->presence_when_darker;
    UNLOCK();
    return ESP_OK;
}

/* ------------------------------ Configuracao ------------------------------ */

esp_err_t ldr_set_threshold(uint8_t sensor_id, uint16_t threshold)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);
    if (threshold > LDR_ADC_MAX_VALUE) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    s_ctx.sensor[sensor_id].threshold = threshold;
    s_ctx.sensor[sensor_id].pending = false;
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_set_hysteresis(uint8_t sensor_id, uint16_t hysteresis)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);
    if (hysteresis > LDR_ADC_MAX_VALUE) {
        return ESP_ERR_INVALID_ARG;
    }
    LOCK();
    s_ctx.sensor[sensor_id].hysteresis = hysteresis;
    s_ctx.sensor[sensor_id].pending = false;
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_set_polarity(uint8_t sensor_id, bool presence_when_darker)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);
    LOCK();
    s_ctx.sensor[sensor_id].presence_when_darker = presence_when_darker;
    s_ctx.sensor[sensor_id].present = false;
    s_ctx.sensor[sensor_id].pending = false;
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_set_confirm_time(uint32_t confirm_time_ms)
{
    CHECK_INIT();
    LOCK();
    s_ctx.confirm_ms = confirm_time_ms;
    for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
        s_ctx.sensor[i].pending = false;
    }
    UNLOCK();
    return ESP_OK;
}

/* ------------------------------- Calibracao ------------------------------- */

esp_err_t ldr_calibrate(uint8_t sensor_id)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);

    uint16_t base = 0;
    esp_err_t err = measure_average(sensor_id, &base);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LDR %u: falha na leitura durante calibracao", sensor_id + 1);
        return err;
    }

    LOCK();
    ldr_sensor_t *s = &s_ctx.sensor[sensor_id];

    int margin = ((int)base * CONFIG_LDR_CALIB_MARGIN_PERCENT) / 100;
    int thr = s->presence_when_darker ? ((int)base - margin) : ((int)base + margin);
    if (thr < 0) {
        thr = 0;
    }
    if (thr > LDR_ADC_MAX_VALUE) {
        thr = LDR_ADC_MAX_VALUE;
    }

    s->baseline = base;
    s->baseline_valid = true;
    s->adc_value = base;
    s->present = false;
    s->pending = false;

    bool feasible = s->presence_when_darker ? (thr - (int)s->hysteresis >= 0)
                                            : (thr + (int)s->hysteresis <= LDR_ADC_MAX_VALUE);
    if (!feasible) {
        ESP_LOGE(TAG, "LDR %u: referencia=%u inviavel (limite %d, histerese %u). "
                 "Verifique iluminacao/divisor ou reduza a histerese.",
                 sensor_id + 1, base, thr, s->hysteresis);
        UNLOCK();
        return ESP_ERR_INVALID_RESPONSE;
    }

    s->threshold = (uint16_t)thr;
    ESP_LOGI(TAG, "LDR %u calibrado: referencia=%u, limite=%u, histerese=%u",
             sensor_id + 1, base, s->threshold, s->hysteresis);
    UNLOCK();
    return ESP_OK;
}

esp_err_t ldr_calibrate_all(void)
{
    CHECK_INIT();
    esp_err_t result = ESP_OK;
    for (uint8_t i = 0; i < LDR_SENSOR_COUNT; i++) {
        esp_err_t err = ldr_calibrate(i);
        if (err != ESP_OK && result == ESP_OK) {
            result = err;
        }
    }
    return result;
}

esp_err_t ldr_calibrate_with_object(uint8_t sensor_id)
{
    CHECK_INIT();
    CHECK_ID(sensor_id);

    uint16_t occupied = 0;
    esp_err_t err = measure_average(sensor_id, &occupied);
    if (err != ESP_OK) {
        return err;
    }

    LOCK();
    ldr_sensor_t *s = &s_ctx.sensor[sensor_id];

    if (!s->baseline_valid) {
        ESP_LOGE(TAG, "LDR %u: execute ldr_calibrate() (posicao livre) primeiro", sensor_id + 1);
        UNLOCK();
        return ESP_ERR_INVALID_STATE;
    }

    int diff = s->presence_when_darker ? ((int)s->baseline - (int)occupied)
                                       : ((int)occupied - (int)s->baseline);
    if (diff <= 2 * (int)s->hysteresis) {
        ESP_LOGE(TAG, "LDR %u: variacao insuficiente (livre=%u, ocupado=%u, histerese=%u) "
                 "ou polaridade incorreta", sensor_id + 1, s->baseline, occupied, s->hysteresis);
        UNLOCK();
        return ESP_ERR_INVALID_RESPONSE;
    }

    s->threshold = (uint16_t)(((uint32_t)s->baseline + occupied) / 2);
    s->pending = false;
    ESP_LOGI(TAG, "LDR %u refinado: livre=%u, ocupado=%u, limite=%u",
             sensor_id + 1, s->baseline, occupied, s->threshold);
    UNLOCK();
    return ESP_OK;
}