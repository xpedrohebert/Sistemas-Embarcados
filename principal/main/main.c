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
#include "FreeRTOSConfig.h"

// Definindo as GPIOs
#define botao_0 21
#define botao_1 22
#define botao_2 23
#define gpio_entrada  ((1ULL<<botao_0) | (1ULL<<botao_1) | (1ULL<<botao_2))
#define led_azul 2
#define gpio_saida  1ULL<<led_azul
#define flag_int 0

// Definindo o tempo de estouro do timer
// #define ESTOURO_RELOGIO 1000             // Estouro do relógio a cada 1 ms
// #define ESTOURO_RELOGIO 10000            // Estouro do relógio a cada 10 ms
#define ESTOURO_RELOGIO 100000              // Estouro do relógio a cada 100 ms
// #define ESTOURO_RELOGIO 1000000          // Estouro do relógio a cada 1 s

// Variáveis globais
static const char* TAG = "Teste";           // Nível: info
static const char* SIS = "Sistema";         // Nível: warning
static const char* TIMER = "Reloginho";     // Nível: info

typedef struct {
    uint64_t campo_contagem_atual;          //campo de contagem atual
    uint64_t campo_valor_do_alarme;         //campo de valor do alarme
} estrutura_de_fila_t;
estrutura_de_fila_t ele;

typedef struct {
    uint8_t dec;                            //campo décimo de segundo
    uint8_t segundo;                        //campo segundo
    uint8_t minuto;                         //campo minuto
    uint8_t hora;                           //campo hora
} estrutura_relogio_t;
estrutura_relogio_t ele1;

static QueueHandle_t gpio_evt_queue = NULL;
gptimer_handle_t timer_relogio;                          //criando a variável do timer   
QueueHandle_t filinha;

// *********INTERRUPÇÕES********************

// Interrupção GPIO (Prática 2)
static void IRAM_ATTR gpio_interrupcao(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Interrupção timer (Prática 3)
static bool IRAM_ATTR interrupt_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + ESTOURO_RELOGIO,            // atualiza o alarme segundo o tempo definido
    };
    gptimer_set_alarm_action(timer_relogio, &alarm_config);
    //ESP_LOGI(TIMER, "entrou na interrupção");
    BaseType_t high_task_awoken = pdFALSE;
    //QueueHandle_t filinha= (QueueHandle_t)user_data;     //dados do usuário na fila
    estrutura_de_fila_t ele2= {
        .campo_valor_do_alarme = edata->alarm_value +1            //atribui o valor do alarme à fila
    };
    ele2.campo_contagem_atual = edata->count_value;             //atribui o valor do contador à fila
    // ele1.segundo= ele.campo_valor_do_alarme;
    xQueueSendFromISR(filinha, &ele2, NULL);  //joga a estrutura ele pra fila
    return (high_task_awoken == pdTRUE);
}

// *********TASKS***********************

// Task GPIO (Prática 2)
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

// Task timer (Prática 3)
static void task_timer(void *arg){
    ESP_LOGI(TIMER, "Configurando o timer");     
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,             //clock de 80Mhz
        .direction = GPTIMER_COUNT_UP,                  //contando pra cima, 
        .resolution_hz = 1000000,                       //clock de 1Mhz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_relogio));                  //enviando tudo pro timer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = interrupt_1,                                //associando callback/interrupção
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_relogio, &cbs, NULL));    //envia esta informação

    gptimer_alarm_config_t alarm_config1 = {    
        .alarm_count = ESTOURO_RELOGIO,                                 //configurando o alarme segundo o tempo definido
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_relogio, &alarm_config1));      //envia configuração do alarme
    ESP_LOGI(TIMER, "Set count value");
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_relogio, 0));   //contagem começando em 100
    ESP_LOGI(TIMER, "Get count value");
    ESP_LOGI(TIMER, "configurou alarme");
    ESP_LOGI(TIMER, "energiza o timer");                      
    ESP_ERROR_CHECK(gptimer_enable(timer_relogio));             //energizando o timer
    ESP_ERROR_CHECK(gptimer_start(timer_relogio));             //inicia contagem do timer
    ESP_LOGI(TIMER, "iniciou a contagem");

    while(1){
        if (xQueueReceive(filinha, &ele, pdMS_TO_TICKS(2000))) {
            ele1.dec++;
            if(ele1.dec==10){
                ele1.segundo++;
                ele1.dec=0;

                if(ele1.segundo==60){
                    ele1.minuto++;
                    ele1.segundo=0;

                    if(ele1.minuto==60){
                        ele1.hora++;
                        ele1.minuto=0;
                    
                        if(ele1.hora==24){
                            ele1.hora=0;
                        }
                    }
                }
                ESP_LOGI(TIMER, "Relógio: %02u:%02u:%02u", ele1.hora,ele1.minuto,ele1.segundo);      //imprime o valor do relogio
                ESP_LOGI(TIMER, "Valor do alarme: %llu", ele.campo_valor_do_alarme);      //imprime o valor do alarme atualizado
                ESP_LOGI(TIMER, "Contagem atual:  %llu", ele.campo_contagem_atual);      //imprime o valor da contagem atual
            }
        }
        else {
            ESP_LOGW(TIMER, "Missed one count event");        //perdeu um evento
        }  
    }
}

//**********MAIN*********************
void app_main(void){

    // Prática 1: Exibindo informações do módulo
    esp_log_level_set(SIS,ESP_LOG_WARN); //Logging sistema em nivel warning

    // Print chip information
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
    filinha = xQueueCreate(3, sizeof(estrutura_de_fila_t)); //cria a fila de 3 elementos de sizeof
    if (!filinha) {
        ESP_LOGE(TAG, "criação da fila falhou");         //só pra ver se a fila foi criada direitim
        return;
    }
    ele.campo_contagem_atual = 0;
    ele.campo_valor_do_alarme = 0 ;
    ele1.dec=0;
    ele1.segundo=0;
    ele1.minuto =0;
    ele1.hora=0;
    timer_relogio = NULL;
    ESP_LOGI(TAG, "passou pelo main A");
    xTaskCreate(task_timer,"task_do_timer", 2048, NULL, 10, NULL);
    ESP_LOGI(TAG, "passou pelo main B");
}