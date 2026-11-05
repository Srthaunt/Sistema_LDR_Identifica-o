#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LDR.H"

static const char *TAG = "app";

#define READ_PERIOD_MS 100
#define READS_PER_LOG  10    /* mostra o estado a cada 1 segundo */

void app_main(void)
{
    ESP_ERROR_CHECK(ldr_matrix_init());

    /* A calibracao assume que todas as posicoes estao livres (sem objeto). */
    ESP_LOGI(TAG, "Calibrando: mantenha todas as posicoes livres");
    esp_err_t err = ldr_calibrate_all();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Calibracao incompleta (%s): os LDRs que falharam mantem o limite configurado",
                 esp_err_to_name(err));
    }

    uint32_t reads = 0;
    while (true) {
        err = ldr_matrix_read_all();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha na leitura: %s", esp_err_to_name(err));
        }

        if (++reads >= READS_PER_LOG) {
            reads = 0;
            char matrix[LDR_SENSOR_COUNT + 1];
            uint16_t adc[LDR_SENSOR_COUNT];
            if (ldr_get_matrix_string(matrix, sizeof(matrix)) == ESP_OK &&
                ldr_get_all_adc_values(adc) == ESP_OK) {
                printf("Presenca: %s | ADC:", matrix);
                for (int i = 0; i < LDR_SENSOR_COUNT; i++) {
                    printf(" %4u", (unsigned)adc[i]);
                }
                printf("\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
    }
}
