# Projeto Final - Medição de ângulo - MCPWM - Dezembro/23
# Disciplina: Sistemas Embarcados - CEFET/MG
# Professor: Túlio Charles de Oliveira Carvalho
# Alunos: Bernardo Neves Lima, Matheus Lima Moreira Martins e Pedro Hebert Moura dos Santos

Este projeto foi desenvolvido como trabalho final da disciplina de Sistemas Embarcados do curso de Engenharia Elétrica do CEFET-MG. O principal objetivo foi aplicar os conhecimentos absorvidos durante o semestre utilizando os principais periféricos do microcontrolador Espressif ESP32, através de sua placa de desenvolvimento. Durante o curso foram implementados diversas funcionalidades através da linguagem C com o módulo ESP32, como os conceitos de tasks, semaphoros, interrupções, timers, display, protocolos de comunicação e transmissão de dados via wifi. Neste trabalho foi escolhido a utilização do módulo de captura MCPWM para auxiliar na detecção de dois sinais para medição de ângulo de giro de um disco dentado, além de associar este ângulo a uma energia absorvida, apresentar os resultados em um display OLED e sincronizar os dados com o MQTT via wifi. 

## Índice
- [Instruções de Instalação](#instruções-de-instalação)
- [Como Usar](#como-usar)
- [Configuração](#configuração)
- [Contribuição](#contribuição)
- [Licença](#licença)

## Instruções de Instalação

Para instalação do programa é necessário que seja baixado o arquivo "projeto_final.rar" e, após descompactá-lo, alocar os arquivos na raiz do diretório definido juntamente com o compilador de sua escolha, de preferência o VScode. Em seguida, é preciso que sejam realizadas as conexões elétricas no módulo ESP32 com as entradas e saídas definidas pelo código. Por fim, se torna possível realizar a compilação do código (Build, Flash and Monitor).

## Como Usar

Para a realização dos testes foi utilizado um disco dentado construído em impressora 3D, este gira em torno de seu próprio eixo, fazendo com que os dentes de sua extremidade sejam detectados por dois sensores de barreira dispostos a 180°. Com a utilização do módulo de captura do ESP32, as variações dos sinais do sensores são identificadas como interrupções, sendo consideradas as bordas de subida e descida dos mesmos. A cada interrupção, é adicionado 7,5° ao valor de ângulo total a ser medido, de forma incremental têm-se o valor final de ângulo. É realizado um ajuste fino deste ângulo final, adicionando um cálculo de tempo ao para definição do ângulo, de modo que a partir da diferença do tempo entre duas interrupções (borda de subida e descida) de um mesmo sensor, é calculado o ângulo que o disco girou, isto para ângulos menores que 7,5°. Este ângulo é recebido por uma função que realiza um cálculo de Energia Absorvida, variável esta que será utilizada em projetos futuros, ao final do cálculo, os valores de ângulo medido e energia são mostrados no display e enviados via MQTT para um dispositivo móvel.

## Configuração

Adicionar aqui os pinos de entradas, saídas e o que ligar em que.

## Contribuição

Os alunos Bernardo, Matheus e Pedro, desenvolveram o código de maneira colaborativa utilizando a estrutura da instituição de ensino CEFET e, sob supervisão do Professor Túlio.


