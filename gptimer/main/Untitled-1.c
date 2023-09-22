// Buscando as bibliotecas necessárias
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

// Definindo as GPIOs
#define botao_0 21
#define botao_1 22
#define botao_2 23
#define gpio_entrada  ((1ULL<<botao_0) | (1ULL<<botao_1) | (1ULL<<botao_2))
#define led_azul 2
#define gpio_saida  1ULL<<led_azul
#define flag_int 0

// Variáveis globais
static const char* TAG = "Teste";       // Nível: info
static const char* SIS = "Sistema";     // Nível: warning
static const char* TIMER = "Timer";     // Nível: info
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t timer_queue = NULL;

typedef struct {
    uint64_t contagem_atual;
    uint64_t valor_alarme;
} timer_queue_t;

typedef struct {
    uint8_t segundo;
    uint8_t minuto;
    uint8_t hora;
} relogio_t;


static void IRAM_ATTR gpio_interrupcao(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR int_alarme_relogio(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    timer_queue_t timer1 = (timer_queue_t) arg;
    xQueueSendFromISR(timer_queue, &timer1, NULL);
}
static bool IRAM_ATTR example_timer_on_alarm_cb_v3(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue = (QueueHandle_t)user_data;
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 1000000, // alarm in next 1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

static void gpio_tarefa_1(void* arg)
{
    uint64_t teste_input = gpio_entrada, teste_output = gpio_saida;
    ESP_LOGI(SIS,"%" PRIu64 "|%" PRIu64, teste_input, teste_output);    //Teste da mascara

    gpio_config_t io_conf = {}; 
    io_conf.intr_type = GPIO_INTR_DISABLE;                              //Desabilitando interrupções
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = gpio_saida;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;                              //Interrupção em borda de descida
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = gpio_entrada;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(flag_int);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(botao_0, gpio_interrupcao, (void*) botao_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(botao_1, gpio_interrupcao, (void*) botao_1);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(botao_2, gpio_interrupcao, (void*) botao_2);

    uint32_t io_num;
    bool flag_1=true;
    
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG,"GPIO[%"PRIu32"]", io_num);
            
            if(io_num==botao_0) gpio_set_level(led_azul,flag_1=true);
            if(io_num==botao_1) gpio_set_level(led_azul,flag_1=false);
            if(io_num==botao_2) {
                flag_1=!flag_1;
                gpio_set_level(led_azul,flag_1);
            }

        }
    }
}

static void tarefa_timer(void* arg)
{
    ESP_LOGI(TIMER, "Create timer handle");
    gptimer_handle_t timer_relogio = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_relogio));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = int_alarme_relogio,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_relogio, &cbs, NULL));

    ESP_LOGI(TIMER, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(timer_relogio));
    ESP_LOGI(TIMER, "Start timer");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 1000000, // period = 1s
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_relogio, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(timer_relogio));

    for(;;) {
        
    }
}

void app_main(void)
{
    // Prática 1: Exibindo informações do módulo
    esp_log_level_set(SIS,ESP_LOG_WARN); //Logging sistema em nivel warning

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(SIS,"This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(SIS,"silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGI(SIS,"Get flash size failed");
        return;
    }

    ESP_LOGI(SIS,"%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(SIS,"Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // for (int i = 10; i >= 0; i--) {
    //     ESP_LOGI(TAG,"Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // ESP_LOGI(TAG,"Restarting now.\n");
    // fflush(stdout);
    // esp_restart();

    // Prática 2: GPIO

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!gpio_evt_queue){
        ESP_LOGE(SIS,"Creating gpio_evt_queue failed");
        return;
    }
    //start gpio task
    xTaskCreate(gpio_tarefa_1, "gpio_tarefa_1", 2048, NULL, 10, NULL);



    // Prática 3: Real Time Clock

    timer_queue = xQueueCreate(10, sizeof(timer_queue_t));
    if (!timer_queue){
        ESP_LOGE(SIS,"Creating timer_queue failed");
        return;
    }
    //start timer task
    xTaskCreate(tarefa_timer, "tarefa_timer", 2048, NULL, 10, NULL);
    

}