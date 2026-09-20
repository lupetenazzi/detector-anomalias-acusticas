# Relatório Técnico - Detector de Anomalias

**Projeto do modelo (Edge Impulse Studio):** <https://studio.edgeimpulse.com/public/1114645/live>


## Objetivo

O objetivo da atividade é implementar um sistema embarcado de detecção de anomalias acústicas em tempo real, aplicando conceitos de RTOS, processamento de sinais em edge computing e sincronização de tarefas concorrentes.

### 1. Aplicação e justificativa

O enunciado deixa livre a escolha do som a detectar. Escolhi uma aplicação simples de propósito: **reconhecer um comando de voz ("acende") e acionar um LED**. MInha escolha foi definida assim porque meu foco nesta atividade era aprender e entender de verdade a arquitetura, ligando os aprendizados de sala sobre RTOS a algo que eu conseguisse testar sozinha. 

Do ponto de vista técnico da escolha, isso é um problema de *keyword spotting*: detectar um padrão acústico específico (a palavra) no meio do áudio ambiente comum. Ele é equivalente ao que o enunciado chama de anomalia acústica, ou seja, um padrão de interesse que se destaca do som de fundo. 

É a mesma classe de problema de assistentes de voz de baixo consumo, campainhas inteligentes e comandos de acessibilidade.

O comportamento desejado é:

- Falar **"acende"** → o LED acende, fica aceso 1 segundo e apaga sozinho.
- Falar **"apaga"** → nada acontece; o LED continua como estava. O sistema **não** deve interpretar "apaga" como "acende".
- Qualquer outro som (silêncio, ruído, outras palavras) → nada acontece.

Inicialmente pensei em fazer o LED ficar aceso até eu dizer "apaga". Mudei de ideia e passei a apagá-lo por um **software timer** depois de 1 segundo. A escolha simplifica o comportamento (o LED não depende de um segundo comando reconhecido corretamente) e me permitiu usar mais um mecanismo de RTOS (o timer do FreeRTOS), que roda fora das três tarefas.


## Requisitos do enunciado e onde foram atendidos

| Requisito do enunciado | Como foi atendido |
|---|---|
| Capturar áudio continuamente (ESP32 + INMP441) | I2S a 16 kHz, 16 bits, com DMA, na Task 1 |
| Detectar anomalias com modelo pré-treinado | Modelo do Edge Impulse (features MFE + rede convolucional 1D) embarcado |
| Arquitetura RTOS com no mínimo 3 tarefas sincronizadas | Task 1 (captura), Task 2 (features), Task 3 (detecção) + software timer |
| Medir e documentar a latência de cada etapa | Timestamps em microssegundos por etapa, log por fatia e tabela de estatísticas |
| Alertar via LED/buzzer | LED no GPIO 17 |
| Resolver conflitos de concorrência | Semáforos, filas, três mutexes e `xTimerReset` |
| Diagrama de tarefas RTOS | `diagrama/Diagrama_RTOS.svg` | 
| Modelo `.onnx` |  |
| Código de teste que simula anomalias e mede performance | `teste/script_teste.py` |
| Relatório técnico | Este documento |


## Hardware e montagem

A montagem física usa o **ESP32**, o **microfone INMP441**, um **LED** e um **resistor** em série com o LED. Não usei o buzzer.

| Componente | Pino |
|---|---|
| LED (com resistor) | GPIO 17 |
| INMP441 – WS (LRCLK) | GPIO 25 |
| INMP441 – SCK (BCLK) | GPIO 32 |
| INMP441 – SD (DOUT) | GPIO 33 |
| INMP441 – L/R | GND (canal esquerdo) |
| INMP441 – VDD / GND | 3V3 / GND |

Parâmetros da captura de áudio: 16 kHz, 16 bits, canal esquerdo, interface I2S com DMA (8 buffers de 512 amostras). 

O firmware tem suporte a um buzzer no GPIO 4 (`USE_BUZZER`). No diagrama ele aparece junto ao LED por isso, mas na montagem física usei só o LED.


## Modelo de detecção

Usei o **Edge Impulse Studio** para treinar o modelo. Escolhi a plataforma por ser intuitiva e por ajudar em todo o ciclo: **criar o dataset de áudios** (gravando e rotulando as amostras), **ajustar o processamento e a rede** e **refinar o modelo** olhando as métricas. O Studio também exporta a biblioteca pronta para Arduino, que integrei ao firmware.

### 1. Dataset

O modelo tem três classes, na ordem em que aparecem no firmware: `acende`, `apaga` e `unknown`.

- **acende:** a palavra que deve acionar o LED.
- **apaga:** uma palavra parecida usada como exemplo negativo, para o modelo aprender a distinguir as duas.
- **unknown:** ruído de fundo e outros sons.

As amostras têm 1 segundo de duração e taxa de amostragem de 16 kHz.

No total, utilizei 536 amostras em treinamento e 133 em teste.


### 2. Resultados de validação no Studio

| Métrica (conjunto de validação) | Valor |
|---|---:|
| Acurácia | 85,2 % |
| Loss | 0,40 |
| Área sob a curva ROC | 0,97 |
| Precisão média ponderada | 0,88 |
| Recall médio ponderado | 0,85 |
| F1 médio ponderado | 0,84 |

Matriz de confusão da validação (linhas = classe real; colunas = classe prevista):

| Real \ Previsto | acende | apaga | unknown | F1 |
|---|---:|---:|---:|---:|
| **acende** | 97,6 % | 2,4 % | 0 % | 0,95 |
| **apaga** | 2,8 % | 97,2 % | 0 % | 0,83 |
| **unknown** | 6,5 % | 38,7 % | 54,8 % | 0,71 |



## Arquitetura RTOS

### 1. Diagrama

![Diagrama de tarefas RTOS](diagrama/Diagrama_RTOS.svg)

O sistema roda três tarefas em pipeline. O áudio entra pela Task 1, o índice do buffer cheio passa pela fila para a Task 2, o pacote de features passa pela outra fila para a Task 3, e a Task 3 aciona o LED. A linha tracejada de volta é o semáforo que devolve o buffer à Task 1. Outra linha tracejada mostra o software timer que apaga o LED.

### 2. Tarefas

| Tarefa | Prioridade | Função |
|---|:---:|---|
| **Task 1 – Captura** | 4 (alta) | Lê o INMP441 via I2S/DMA, aplica ganho e preenche o buffer de áudio |
| **Task 2 – Features** | 3 (média) | Calcula RMS e um estimador de centróide espectral (via ZCR), marca silêncio |
| **Task 3 – Detecção** | 2 (baixa) | Roda o classificador, decide e aciona o LED, libera o buffer |
| **Timer Service Task** (FreeRTOS) | do sistema | Executa o callback que apaga o LED após 1 s |

Quem executa primeiro é decidido pelas prioridades, o que torna o escalonamento previsível, e o core 0 fica livre para o sistema.

### 3. Fluxo de dados

1. A Task 1 lê blocos de 512 amostras e os copia para o buffer ativo. Cada buffer tem `EI_CLASSIFIER_SLICE_SIZE` = **4.000 amostras** (250 ms a 16 kHz).
2. Quando o buffer enche, a Task 1 registra o instante, envia o **índice** do buffer para `xCaptureQueue` e troca para o outro buffer.
3. A Task 2 recebe o índice, calcula RMS e ZCR, decide se a fatia é silêncio e envia um pacote com essas features e todos os timestamps para `xFeatureQueue`.
4. A Task 3 recebe o pacote. Se for silêncio, só libera o buffer. Caso contrário, chama `run_classifier_continuous()` sobre o buffer e escolhe a classe de maior probabilidade.
5. Se a classe for `acende`, a confiança superar o limiar e o cooldown tiver passado, liga o LED e reinicia o timer de 1 s. Em seguida, libera o buffer para a Task 1.


### 4. Alerta

O alerta é o LED no GPIO 17. O LED liga na Task 3 e é apagado pelo callback do software timer, 1 segundo depois. Como o apagamento não depende de nenhuma das três tarefas do pipeline, a Task 3 não precisa se bloquear esperando (nada de `delay`), e a captura e a classificação continuam durante o tempo em que o LED está aceso.

### 5. Decisão de acionamento

A decisão usa a classificação de **uma única fatia**: a classe mais provável precisa ser `acende` e a confiança precisa superar `HIGH_CONFIDENCE_THRESHOLD` (**0,50** no firmware que gravei). 


## Testes

Escrevi o script `teste/script_teste.py` (Python). Ele conduz um protocolo fixo, lê a saída serial do ESP32 e classifica automaticamente cada tentativa. Em cada tentativa ele:

1. mostra na tela o que devo fazer ("FALE 'acende'", "FALE 'apaga'", "FALE OUTRA PALAVRA" ou "faça um barulho / fique em silêncio") e faz uma contagem regressiva de 3 s;
2. abre uma janela de observação de 3,5 s (1,5 s para falar + 2 s de espera do pipeline);
3. lê as linhas `[Task3] slice=...` da serial, verifica se apareceu `ACAO EXECUTADA` (o LED foi acionado) e guarda o rótulo e a confiança de cada fatia;
4. espera 1 s antes da próxima tentativa (janela + pausa maior que o tempo do LED aceso, para uma tentativa não contaminar a seguinte).

Só a classe `acende` **deve** acionar o LED. As demais são negativas: se o LED acender, é falso positivo. O script gera a matriz de confusão, as métricas e a tabela de latência por etapa, e salva `resultados.csv`, `resumo.md` e o log bruto da serial.


## Resultados

### 1. Matriz de confusão (acionamento do LED)

| | LED acionou | LED não acionou |
|---|---:|---:|
| **Deveria acionar** ("acende") | TP = 18 | FN = 2 |
| **Não deveria acionar** | FP = 5 | TN = 25 |

| Métrica | Valor | Intervalo de confiança 95 % |
|---|---:|---:|
| Acurácia | 86,0 % (43/50) | 73,8 – 93,0 % |
| Precisão | 78,3 % (18/23) | 58,1 – 90,3 % |
| Recall (sensibilidade) | 90,0 % (18/20) | 69,9 – 97,2 % |
| F1 | 0,837 | — |
| Taxa de falso positivo | 16,7 % (5/30) | 7,3 – 33,6 % |

Os intervalos de confiança foram calculados pelo método de Wilson. Eles são largos por causa do número pequeno de tentativas.

### 2. Resultado por classe

| Classe | Tentativas | LED acionou | Taxa de acionamento |
|---|---:|---:|---:|
| acende | 20 | 18 | 90,0 % |
| apaga | 20 | 1 | 5,0 % |
| outra | 5 | 3 | 60,0 % |
| ruido | 5 | 1 | 20,0 % |

O requisito principal do comportamento, **"apaga" não acender o LED**, foi atendido em 19 das 20 tentativas (95 %).

### 3. Erros

| Tentativa | Classe | Tipo | Maior confiança em "acende" | Observação |
|---:|---|---|---:|---|
| 3 | acende | FN | 0,00 | Nenhuma fatia reconhecida como "acende" |
| 5 | acende | FN | 0,42 | Confiança abaixo do limiar |
| 6 | ruido | FP | 0,51 | Confiança logo acima do limiar |
| 32 | apaga | FP | 0,51 | Confiança logo acima do limiar |
| 21 | outra | FP | 0,96 | Confiança muito alta |
| 30 | outra | FP | 0,83 | Confiança alta |
| 50 | outra | FP | 0,93 | Confiança muito alta |

Nos acionamentos corretos, a maior confiança em "acende" ficou entre 0,59 e 0,85 (média 0,71).


## Análise de latência

A latência de cada etapa é medida no próprio firmware, com `esp_timer_get_time()` (microssegundos). Os timestamps viajam dentro do pacote de features, e a Task 3 calcula as durações e acumula mínimo, média e máximo. Os valores abaixo vêm das 1.499 fatias classificadas durante toda a execução do teste.

| Etapa | n | mín (ms) | média (ms) | p50 (ms) | p95 (ms) | máx (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Fila Task 1 → Task 2 | 1499 | 0,00 | 0,07 | 0,10 | 0,10 | 0,10 |
| Task 2 (RMS + ZCR) | 1499 | 3,64 | 3,65 | 3,65 | 3,65 | 3,65 |
| Fila Task 2 → Task 3 | 1499 | 0,00 | 0,00 | 0,00 | 0,00 | 0,00 |
| Task 3 – DSP (MFE) | 1499 | 33,40 | 33,40 | 33,40 | 33,40 | 33,40 |
| Task 3 – Rede neural | 1499 | 75,40 | 75,41 | 75,40 | 75,50 | 75,50 |
| Task 3 – total | 1499 | 109,10 | 109,10 | 109,10 | 109,10 | 109,20 |
| **Ponta a ponta** (buffer cheio → fim da classificação) | 1499 | 112,80 | 112,83 | 112,80 | 112,90 | 112,90 |
| **Alerta** (buffer cheio → LED acionado) | 23 | 112,80 | 112,85 | 112,80 | 112,90 | 112,90 |

**Etapa de captura.** O tempo de encher um buffer é, por construção, o da duração da fatia: 4.000 amostras ÷ 16.000 Hz = **250 ms**. Esse valor é teórico, e não fez parte da tabela medida.


### 1. Análise dos resultados

O comportamento principal funciona. O sistema acende o LED para "acende" em 90 % das vezes e "apaga" quase nunca o aciona (5 %).

Os falsos positivos têm duas causas diferentes. Os dois casos com confiança em torno de 0,51 (um "ruído" e um "apaga") estão perto do limiar de decisão. Já os três de "outra palavra" tiveram confiança de 0,83 a 0,96: o modelo estava bastante "seguro" e errado. Isso é compatível com a validação do Studio, em que a classe `unknown` era a mais fraca (54,8 % de recall). O modelo não aprendeu bem o que não é "acende".

Usando a maior confiança em "acende" de cada tentativa (do CSV), estimei o que aconteceria com outros limiares. A estimativa ignora o cooldown e usa os mesmos dados do teste, então ela serve para levantar uma hipótese, e não para afirmar que o limiar ideal foi encontrado:

| Limiar | Acionamentos corretos (de 20) | Falsos positivos (de 30) |
|---:|---:|---:|
| 0,40 | ~19 | ~6 |
| 0,45 a 0,50 | ~18 | ~5 |
| 0,55 | ~18 | ~3 |
| 0,60 | ~17 | ~3 |
| 0,70 | ~11 | ~3 |

O limiar usado no teste foi 0,50, e a linha "0,45 a 0,50" da tabela reproduz o resultado observado (18 acionamentos corretos e 5 falsos positivos). Subir o limiar para algo em torno de 0,55 eliminaria os dois falsos positivos de confiança ~0,51 sem perder acionamentos corretos neste teste. Mas os três falsos positivos de "outra palavra" permanecem em qualquer limiar razoável, porque a confiança deles é muito alta. 

A acurácia de validação do Studio (85,2 %) e a acurácia do meu teste (86 %) são parecidas.


## Conclusão

Consegui implementar um detector de comando de voz embarcado com FreeRTOS que atende ao comportamento que defini: "acende" liga o LED por 1 segundo, e "apaga" não faz nada. A arquitetura com três tarefas, dois semáforos, duas filas, três mutexes e um software timer funcionou sem gargalo nas filas (as esperas foram de uma fração de milissegundo), com latência de processamento de ~113 ms para fatias de 250 ms.

O teste manual (acurácia de 86 %, recall de 90 %) mostrou que o sistema funciona, mas que o modelo confunde outras palavras com "acende" com confiança alta. 

A atividade me ajudou a conectar e praticar os conceitos de aula a um sistema e a perceber que, em um sistema embarcado de IA, o desempenho depende tanto da arquitetura de tarefas quanto da qualidade dos dados de treino.

