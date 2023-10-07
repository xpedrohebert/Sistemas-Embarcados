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
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "FreeRTOSConfig.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Definindo as GPIOs
#define botao_0             21
#define botao_1             22
#define botao_2             23
#define gpio_entrada        ((1ULL<<botao_0) | (1ULL<<botao_1) | (1ULL<<botao_2))
#define led_azul            2
#define rgb_verde           16
#define rgb_vermelho        17
#define rgb_azul            26
#define osciloscopio_32     32
#define osciloscopio_33     33
#define gpio_saida          ((1ULL<<led_azul) | (1ULL<<rgb_vermelho) | (1ULL<<rgb_verde) | (1ULL<<rgb_azul) | (1ULL<<osciloscopio_32) | (1ULL<<osciloscopio_33))
#define flag_int            0
#define adc1_ch0            ADC_CHANNEL_3   // Define canal 3 do ADC (3) como entrada
#define adc_attenuation     ADC_ATTEN_DB_11 // Define a atenuação do ADC

// Pinos PWM da placa utilizada
// Pino 16 -> Verde
// Pino 17 -> Vermelho
// Pino 26 -> Azul
// Pino 32 -> Conectado na entrada do filtro PB
// Pino 33 -> Desconectado

// Definindo o tempo de estouro do timer
// #define ESTOURO_RELOGIO 1000             // Estouro do relógio a cada 1 ms
// #define ESTOURO_RELOGIO 10000            // Estouro do relógio a cada 10 ms
#define ESTOURO_RELOGIO 100000              // Estouro do relógio a cada 100 ms
// #define ESTOURO_RELOGIO 1000000          // Estouro do relógio a cada 1 s

//Definindo parâmetros PWM
#define LEDC_FREQUENCY (5000)               // Frequency in Hertz. Set frequency at 5 kHz

// Logs
static const char* SIS = "Sistema";         // Nível: warning
static const char* TAG = "Teste";           // Nível: info
static const char* TIMER = "Relógio";       // Nível: info
static const char* PWM = "PWM";             // Nível: info
static const char* ADC = "ADC";             // Nível: info

// ******************************** //
// Logging library                  //
// ESP_LOGE - error     (lowest)    //
// ESP_LOGW - warning               //
// ESP_LOGI - info                  //
// ESP_LOGD - debug                 //
// ESP_LOGV - verbose   (highest)   //
// ******************************** //

// Estruturas
typedef struct {
    uint64_t contagem_atual;                // Campo de contagem atual
    uint64_t valor_alarme;                  // Campo de valor do alarme
} estrutura_alarme_t;
estrutura_alarme_t alarme;

typedef struct {
    uint8_t dec;                            // Campo décimo de segundo
    uint8_t segundo;                        // Campo segundo
    uint8_t minuto;                         // Campo minuto
    uint8_t hora;                           // Campo hora
} estrutura_relogio_t;

typedef struct {
    bool modo_PWM;                          // Campo modo do PWM (automático = true ou manual = false)
    uint16_t duty_PWM;                      // Campo duty cycle do PWM (0 a 8191)
} PWM_elements_t;

typedef struct {
    uint16_t valor;                         // Valor bruto do ADC
    uint16_t calibrado;                     // Valor calibrado (em mV)
} ADC_elements_t;

static QueueHandle_t gpio_evt_queue = NULL;
static SemaphoreHandle_t semaphore_pwm = NULL;
static SemaphoreHandle_t semaphore_adc = NULL;
gptimer_handle_t timer_relogio;             // Criando a variável do timer   
QueueHandle_t timer_int_queue;
QueueHandle_t gpio_pwm_queue;
QueueHandle_t timer_adc_queue;

// Variáveis globais
PWM_elements_t pwm;                         // Variável global que armazena o estado do PWM


// ************************** FUNÇÕES ****************************************************************************

// Incrementa o valor atual do duty cycle do PWM e retorna o novo valor
static uint16_t increment_dutycicle(uint16_t duty){
    duty = duty + 1638;
    if(duty > 8191){
        duty = 1;
    }
    return duty;
}

// Atualiza a saída do PWM segundo o duty cycle recebido
static void atualiza_dutycycle(uint16_t duty){
    // Seta o duty cycle dos 2 PWMs
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty));
    // Update duty to apply the new value nos 2 PWMs
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1));
    // ESP_LOGI(PWM, "PWM:  %u", pwm.duty_PWM);            // Imprime o valor do duty cycle do PWM a cada incremento
}

// Calibração ADC
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(ADC, "calibration scheme version is %s", "Curve Fitting");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(ADC, "calibration scheme version is %s", "Line Fitting");
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(ADC, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADC, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADC, "Invalid arg or no memory");
    }

    return calibrated;
}

// **************************** INTERRUPÇÕES *********************************************************************

// Interrupção GPIO (Prática 2)
static void IRAM_ATTR gpio_interrupcao(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Interrupção timer (Prática 3)
static bool IRAM_ATTR interrupt_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata -> alarm_value + ESTOURO_RELOGIO,  // Atualiza o alarme segundo o tempo definido
    };
    gptimer_set_alarm_action(timer_relogio, &alarm_config);
    // ESP_LOGI(TIMER, "entrou na interrupção");
    BaseType_t high_task_awoken = pdFALSE;
    
    estrutura_alarme_t alarme2;
    alarme2.valor_alarme = edata -> alarm_value + 1;            // Atribui o valor do alarme da interrupção pra fila
    alarme2.contagem_atual = edata -> count_value;              // Atribui o valor do contador da interrupção pra fila

    xQueueSendFromISR(timer_int_queue, &alarme2, NULL);         // Joga a estrutura alarme2 pra fila do timer
    return (high_task_awoken == pdTRUE);
}

// ************************** TASKS ******************************************************************************

// Task GPIO (Prática 2)
static void task_gpio(void* arg){
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
    pwm.modo_PWM = true;
    pwm.duty_PWM = 1;
    gpio_set_level(led_azul,pwm.modo_PWM);
    
    for(;;) {
        // Talvez o xSemaphoreTake devesse estar dentro de um if
        xSemaphoreTake(semaphore_pwm, portMAX_DELAY);   // Coloca a task em espera, até ser habilitada novamente pelo estouro do timer
        if (xQueueReceive(gpio_evt_queue, &io_num, 10/(portTICK_PERIOD_MS))){
            ESP_LOGI(TAG,"GPIO[%"PRIu32"]", io_num);
            
            // Prática 2 controle LED azul por meio dos botões 0, 1 e 2
            // if(io_num==botao_0) gpio_set_level(led_azul,pwm.modo_PWM=true);      // Botão 0 liga LED azul
            // if(io_num==botao_1) gpio_set_level(led_azul,pwm.modo_PWM=false);     // Botão 1 desliga LED azul
            // if(io_num==botao_2) {                                                // Botão 2 troca o estado do LED azul
            //     pwm.modo_PWM=!pwm.modo_PWM;
            //     gpio_set_level(led_azul,pwm.modo_PWM);
            // }

            // Prática 4 controle PWM por meio dos botões 0, 1 e 2
            if(io_num==botao_0){
                pwm.modo_PWM = true;
                gpio_set_level(led_azul,pwm.modo_PWM);          // Botão 0 PWM automático
                }
            if(io_num==botao_1){
                pwm.modo_PWM = false;
                gpio_set_level(led_azul,pwm.modo_PWM);          // Botão 1 PWM manual
            }
            if(io_num == botao_2 && pwm.modo_PWM == false){     // Botão 2 incrementa duty cycle em 10%
                pwm.duty_PWM = increment_dutycicle(pwm.duty_PWM);
            }
            ESP_LOGI(PWM, "Duty Cycle do PWM: %u", pwm.duty_PWM);           // Imprime o valor do duty cycle atual do PWM
            ESP_LOGI(PWM, "Modo de Atuação do PWM: %u",pwm.modo_PWM);       // Imprime o modo atual do PWM
        }
        xQueueSendToBack(gpio_pwm_queue, &pwm, NULL);
    }
}

// Task timer (Prática 3)
static void task_timer(void *arg){
    // Declarando as variáveis necessárias e setando em 0
    estrutura_relogio_t relogio;
    relogio.dec = 0;
    relogio.segundo = 0;
    relogio.minuto = 0;
    relogio.hora = 0;
    estrutura_alarme_t alarme;
    alarme.contagem_atual = 0;
    alarme.valor_alarme = 0 ;
    timer_relogio = NULL;
    ADC_elements_t adc;
    adc.calibrado = NULL;
    adc.valor = NULL;

    ESP_LOGI(TIMER, "Configurando o timer");     
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,             // Clock de 80Mhz
        .direction = GPTIMER_COUNT_UP,                  // Contando pra cima, 
        .resolution_hz = 1000000,                       // Clock de 1Mhz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_relogio));                  // Enviando tudo pro timer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = interrupt_1,                                // Associando callback/interrupção
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_relogio, &cbs, NULL));    // Envia esta informação

    gptimer_alarm_config_t alarm_config1 = {    
        .alarm_count = ESTOURO_RELOGIO,                                 // Configurando o alarme segundo o tempo definido
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

    for(;;){
        if (xQueueReceive(timer_int_queue, &alarme, pdMS_TO_TICKS(2000))) {
            relogio.dec++;
            if(relogio.dec==10){
                relogio.segundo++;
                relogio.dec=0;

                if(relogio.segundo==60){
                    relogio.minuto++;
                    relogio.segundo=0;

                    if(relogio.minuto==60){
                        relogio.hora++;
                        relogio.minuto=0;
                    
                        if(relogio.hora==24){
                            relogio.hora=0;
                        }
                    }
                }
                ESP_LOGI(TIMER, "%02u:%02u:%02u", relogio.hora,relogio.minuto,relogio.segundo);     // Imprime o valor do relogio
                ESP_LOGD(TIMER, "Valor do alarme: %llu", alarme.valor_alarme);                      // Imprime o valor do alarme atualizado
                ESP_LOGD(TIMER, "Contagem atual:  %llu", alarme.contagem_atual);                    // Imprime o valor da contagem atual

                if(xQueueReceive(timer_adc_queue, &adc, 1/(portTICK_PERIOD_MS))){                   // Espera 1 ms por uma entrada da fila timer_adc
                    ESP_LOGI(ADC, "Valor Bruto:     %d", adc.valor);
                    ESP_LOGI(ADC, "Valor Calibrado: %d", adc.calibrado);
                }
            }
            xSemaphoreGive(semaphore_pwm);      // Função na TASK Timer para sincronizar com a task PWM
            xSemaphoreGive(semaphore_adc);      // Função na TASK Timer para sincronizar com a task ADC
        }
        else {
            ESP_LOGW(TIMER, "Missed one count event");        // Perdeu um evento
        }  
    }
}

//Task PWM  (Prática 4)
static void task_PWM(void* arg){
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = rgb_verde,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = osciloscopio_32,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));
    
    

    for(;;){
        if(xQueueReceive(gpio_pwm_queue, &pwm, portMAX_DELAY)){
            if(pwm.modo_PWM == true){
                pwm.duty_PWM = increment_dutycicle(pwm.duty_PWM);
                // ESP_LOGI(TAG, "PWM2:  %u", pwm.duty_PWM); 
            }
            atualiza_dutycycle(pwm.duty_PWM);
        } 
        
    }  
}    

//Task ADC (Prática 5)
static void task_ADC(void* arg){
    // ESP_LOGD(ADC, "ENTREI NO ADC");
    // ADC1 Init
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = adc_attenuation,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc1_ch0, &config));

    // ADC1 Calibration Init
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, adc_attenuation, &adc1_cali_handle);

    ADC_elements_t adc;
    adc.calibrado = NULL;
    adc.valor = NULL;

    uint8_t i=0;

    for(;;){
        // ESP_LOGD(ADC, "ENTREI NO FOR");
        xSemaphoreTake(semaphore_adc, portMAX_DELAY);   // Coloca a task em espera, até ser habilitada novamente pelo estouro do timer
            
        // Leitura ADC
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc1_ch0, &adc.valor));

        if (do_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc.valor, &adc.calibrado));
        }
        xQueueSendToBack(timer_adc_queue, &adc, NULL);
    }
}

// ************************** MAIN *******************************************************************************

void app_main(void){
    esp_log_level_set(SIS,ESP_LOG_WARN);                                // Logging sistema em nivel warning
    semaphore_pwm = xSemaphoreCreateBinary();                           // Criação do semaphore binario para o PWM
    semaphore_adc = xSemaphoreCreateBinary();                           // Criação do semaphore binario para o ADC

    // Criação das Filas
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));                // Cria a fila para o tratamento do GPIO por interrupção
    if (!gpio_evt_queue){                                               // Verifica se a fila do GPIO foi criada corretamente
        ESP_LOGE(SIS,"Criação da fila gpio_evt_queue falhou");
        return;
    }
    
    timer_int_queue = xQueueCreate(3, sizeof(estrutura_alarme_t));     // Cria a fila para o tratamento do timer por interrupção
    if (!timer_int_queue){                                              // Verifica se a fila do timer foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila do timer falhou");
        return;
    }

    gpio_pwm_queue = xQueueCreate(10, sizeof(PWM_elements_t));          // Cria a fila para o tratamento do PWM segundo os botões
    if (!gpio_pwm_queue){                                               // Verifica se a fila do PWM foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila gpio-pwm falhou");
        return;
    }

    timer_adc_queue = xQueueCreate(10, sizeof(ADC_elements_t));          // Cria a fila para o tratamento do ADC segundo o timer
    if (!timer_adc_queue){                                               // Verifica se a fila do ADC foi criada corretamente
        ESP_LOGE(SIS, "Criação da fila timer-adc falhou");
        return;
    }

    // Criação das Tasks
    xTaskCreate(task_gpio, "task_gpio", 2048, NULL, 11, NULL);          // Cria task do GPIO
    xTaskCreate(task_timer,"task_timer", 2048, NULL, 10, NULL);         // Cria task do Timer
    xTaskCreate(task_PWM, "task_PWM", 2048, NULL, 10, NULL);            // Cria task do PWM
    xTaskCreate(task_ADC, "task_ADC", 2048, NULL, 10, NULL);            // Cria task do ADC
    
    // Prática 1: Exibindo informações do módulo
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

    // ESP_LOGI(TAG, "passou pelo main A");
    // ESP_LOGI(TAG, "passou pelo main B");

}