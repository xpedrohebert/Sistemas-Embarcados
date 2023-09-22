#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "FreeRTOSConfig.h"

static const char *TAG = "reloginho";

#define CLOCK 40000000

typedef struct {
    uint64_t campo_contagem_atual;                           //campo de contagem atual
    uint64_t campo_valor_do_alarme;                          //campo de valor do alarme
} estrutura_de_fila_t;
estrutura_de_fila_t ele;

typedef struct {
    uint8_t dec;                                              //campo décimo de segundo
    uint8_t segundo;                                          //campo segundo
    uint8_t minuto;                                           //campo minuto
    uint8_t hora;                                             //campo hora
} estrutura_relogio_t;
estrutura_relogio_t ele1;

gptimer_handle_t timer_relogio;                          //criando a variável do timer   
QueueHandle_t filinha;

//*********interrupção******************
static bool IRAM_ATTR interrupt_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 100000,            // alarm in next 100 ms
    };
    gptimer_set_alarm_action(timer_relogio, &alarm_config);
    //ESP_LOGI(TAG, "entrou na interrupção");
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

//*********TASK***********************
static void task_timer(void *arg){
    ESP_LOGI(TAG, "Configurando o timer");     
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,             //clock de 80Mhz
        .direction = GPTIMER_COUNT_UP,                  //contando pra cima, 
        .resolution_hz = CLOCK,                       //clock de 1Mhz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_relogio));                  //enviando tudo pro timer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = interrupt_1,                                //associando callback/interrupção
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_relogio, &cbs, NULL));    //envia esta informação

    gptimer_alarm_config_t alarm_config1 = {    
        // .reload_count = 0, 
        .alarm_count = 100000,                                 //configurando o alarme de 1s
        // .flags.auto_reload_on_alarm = true,         //recarregar o timer
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_relogio, &alarm_config1));      //envia configuração do alarme
    ESP_LOGI(TAG, "Set count value");
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_relogio, 0));   //contagem começando em 100
    ESP_LOGI(TAG, "Get count value");
    ESP_LOGI(TAG, "configurou alarme");
    ESP_LOGI(TAG, "energiza o timer");                      
    ESP_ERROR_CHECK(gptimer_enable(timer_relogio));             //energizando o timer
    ESP_ERROR_CHECK(gptimer_start(timer_relogio));             //inicia contagem do timer
    ESP_LOGI(TAG, "iniciou a contagem");

    int record=0;

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
                ESP_LOGI(TAG, "Relógio:  %2u:%2u:%2u", ele1.hora,ele1.minuto,ele1.segundo);      //imprime o valor do relogio
                ESP_LOGI(TAG, "Contagem atual:  %llu", ele.campo_contagem_atual);      //imprime o valor do campo recebido
                ESP_LOGI(TAG, "Valor do alarme: %llu", ele.campo_valor_do_alarme);      //imprime o valor do campo recebido

            }
        }
        else {
            ESP_LOGW(TAG, "Missed one count event");        //perdeu um evento
        }
        // uint64_t count;                                         //criando a variável que vai receber a contagem
        // ESP_ERROR_CHECK(gptimer_get_raw_count(timer_relogio, &count));//lê a contagem e colocar ela na variável
        // ESP_LOGI(TAG, "Timer count value=%llu", count);
        // if(record>2000000){
        // ESP_ERROR_CHECK(gptimer_stop(timer_relogio));           //para o timer
        // }   
    }
    //*******loop*****************  
        /*while(1){
            ESP_LOGI(TAG, "deu pau");
            if (xQueueReceive(filinha, &ele, portMAX_DELAY)) {
                ESP_LOGI(TAG, "Timer stopped, count=%llu", ele.campo_contagem_atual);      //imprime o valor do campo recebido
                if (ele.campo_contagem_atual >= 65535) {
                    ele.campo_contagem_atual = 0;  // Reinicia a contagem quando atinge o limite
                }
                ESP_LOGI(TAG, "Timer stopped, count=%llu", ele.campo_contagem_atual);
            } else {
                ESP_LOGW(TAG, "Missed one count event");                   //perdeu um evento
            }
            
            ESP_LOGI(TAG, "Segundos:%u", ele1.segundo);
            ESP_LOGI(TAG, "Minutos:%u", ele1.minuto);
            ESP_LOGI(TAG, "Hora:%u", ele1.hora);
            ESP_LOGI(TAG, "CONTAGEM:%llu", ele.campo_contagem_atual);
            ESP_LOGI(TAG, "ALARME:%llu", ele.campo_valor_do_alarme);
        }*/
}

//**********MAIN*********************
void app_main(void)
{ 
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