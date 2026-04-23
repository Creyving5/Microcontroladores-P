//Creyving Guerrero 2024-1294

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define ADC_CHANNEL ADC1_CHANNEL_6
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define ADC_WIDTH   ADC_WIDTH_BIT_12
#define SAMPLES     1000

#define WIFI_SSID   "Casita"
#define WIFI_PASS   "F@milia00"

static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id)
    {
        case MQTT_EVENT_CONNECTED:    printf("MQTT conectado\n");    break;
        case MQTT_EVENT_DISCONNECTED: printf("MQTT desconectado\n"); break;
        case MQTT_EVENT_PUBLISHED:    printf("Mensaje publicado\n"); break;
        default: break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://mqtt.eclipseprojects.io",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_send_voltage(float voltage)
{
    if (!mqtt_client) return;
    char msg[50];
    snprintf(msg, sizeof(msg), "%.3f", voltage);
    esp_mqtt_client_publish(mqtt_client, "yadiel/voltimetro/vrms", msg, 0, 1, 0);
}

void wifi_init(void)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(3000));
    mqtt_app_start();

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);

    printf("Iniciando medicion RMS...\n");

    while (1)
    {
        uint32_t raw;
        float voltage;
        float sum    = 0;
        float offset = 0;

        for (int i = 0; i < SAMPLES; i++)
        {
            raw     = adc1_get_raw(ADC_CHANNEL);
            voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0f;
            offset += voltage;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        offset /= SAMPLES;

        for (int i = 0; i < SAMPLES; i++)
        {
            raw     = adc1_get_raw(ADC_CHANNEL);
            voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0f;
            float v = voltage - offset;
            sum    += v * v;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        float vrms = sqrtf(sum / SAMPLES);
        printf("Offset: %.3f V | Vrms: %.3f V\n", offset, vrms);
        mqtt_send_voltage(vrms);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}