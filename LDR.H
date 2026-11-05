
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "hal/spi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LDR_SENSOR_COUNT   8
#define LDR_ADC_MAX_VALUE  4095     /* MCP3208: 12 bits */
#define LDR_MCP3208_CHANNELS 8

typedef enum {
    LDR_STATE_ABSENT  = 0,
    LDR_STATE_PRESENT = 1,
} ldr_state_t;

typedef struct {
    /* --- SPI --- */
    spi_host_device_t spi_host;          /**< SPI2_HOST ou SPI3_HOST */
    int               gpio_sclk;         /**< SCLK -> MCP3208 CLK */
    int               gpio_miso;         /**< MISO <- MCP3208 DOUT */
    int               gpio_mosi;         /**< MOSI -> MCP3208 DIN */
    int               gpio_cs;           /**< CS   -> MCP3208 CS/SHDN */
    uint32_t          spi_clock_hz;      /**< Clock SPI (<= ~1 MHz com VDD = 3,3 V) */

    /* --- Sensores --- */
    uint8_t           mcp_channel[LDR_SENSOR_COUNT];        /**< Canal do MCP3208 (0..7) de cada LDR */
    uint16_t          threshold[LDR_SENSOR_COUNT];          /**< Limite individual (0..4095) */
    uint16_t          hysteresis[LDR_SENSOR_COUNT];         /**< Histerese individual (contagens) */
    bool              presence_when_darker[LDR_SENSOR_COUNT]; /**< Polaridade individual */

    /* --- Filtro --- */
    uint8_t           samples;           /**< Amostras por leitura (1..64) */
    uint32_t          confirm_time_ms;   /**< Tempo minimo de confirmacao */
} ldr_matrix_config_t;
typedef struct {
    uint8_t  mcp_channel;     /**< Canal do MCP3208 (0..7) */
    uint16_t adc_value;       /**< Ultima media filtrada */
    uint16_t threshold;
    uint16_t hysteresis;
    uint16_t baseline;        /**< Referencia sem objeto (valida se baseline_valid) */
    bool     baseline_valid;
    bool     presence;
    bool     presence_when_darker;
} ldr_sensor_info_t;

/* ----------------------------- Inicializacao ----------------------------- */

esp_err_t ldr_matrix_get_default_config(ldr_matrix_config_t *cfg);
esp_err_t ldr_matrix_init(void);
esp_err_t ldr_matrix_init_with_config(const ldr_matrix_config_t *cfg);
esp_err_t ldr_matrix_deinit(void);

/* -------------------------------- Leitura -------------------------------- */

esp_err_t ldr_matrix_read_all(void);
esp_err_t ldr_get_presence(uint8_t sensor_id, bool *presence);
esp_err_t ldr_get_all_presence(uint8_t *mask);
esp_err_t ldr_get_matrix_string(char *buf, size_t buf_len);
esp_err_t ldr_get_adc_value(uint8_t sensor_id, uint16_t *adc_value);
esp_err_t ldr_get_all_adc_values(uint16_t *values);
esp_err_t ldr_get_info(uint8_t sensor_id, ldr_sensor_info_t *info);

/* ------------------------------ Configuracao ------------------------------ */

esp_err_t ldr_set_threshold(uint8_t sensor_id, uint16_t threshold);
esp_err_t ldr_set_hysteresis(uint8_t sensor_id, uint16_t hysteresis);
esp_err_t ldr_set_polarity(uint8_t sensor_id, bool presence_when_darker);
esp_err_t ldr_set_confirm_time(uint32_t confirm_time_ms);

/* ------------------------------- Calibracao ------------------------------- */

esp_err_t ldr_calibrate(uint8_t sensor_id);

esp_err_t ldr_calibrate_all(void);

esp_err_t ldr_calibrate_with_object(uint8_t sensor_id);

#ifdef __cplusplus
}
#endif