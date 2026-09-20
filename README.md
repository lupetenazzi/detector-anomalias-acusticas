# Detector de Anomalias Acústicas (ESP32 + INMP441 + FreeRTOS)

Sistema embarcado que escuta o ambiente continuamente e reconhece a palavra **"acende"**: o LED liga por 1 segundo e apaga sozinho. A palavra **"apaga"** existe no modelo como exemplo negativo e **não** aciona o LED.

O firmware usa **FreeRTOS** com três tarefas concorrentes (captura de áudio, extração de features e detecção), filas, semáforos, mutexes e um software timer. O modelo foi treinado no Edge Impulse Studio.

> **Relatório técnico completo:** [relatório técnico](/relatorio.md)

## Arquitetura

![Diagrama de tarefas RTOS](diagrama/Diagrama_RTOS.svg)

| Tarefa | Prioridade | Função |
|---|:---:|---|
| Task 1 – Captura | 4 (alta) | Lê o microfone via I2S/DMA e preenche o double buffer |
| Task 2 – Features | 3 (média) | Calcula RMS e estimativa de centróide (ZCR); marca silêncio |
| Task 3 – Detecção | 2 (baixa) | Roda o classificador e aciona o LED |

## Resultados (teste manual, 50 tentativas)

| Métrica | Valor |
|---|---:|
| Acurácia | 86,0 % |
| Recall ("acende") | 90,0 % |
| Taxa de falso positivo | 16,7 % |
| Latência de processamento (buffer cheio → LED) | ~112,8 ms |

Detalhes e análise no [relatório](relatorio_tecnico.md).

## Estrutura do repositório

```
.
├── firmware/
│   └── detector_anomalias.ino      # firmware do ESP32
├── diagrama/
│   └── Diagrama_RTOS.svg           # diagrama de tarefas e sincronização
├── teste/
│   ├── script_teste.py             # script de teste (falado, via serial)
│   └── resultados_teste/           # resultados do teste executado
├── relatorio_tecnico.md
└── README.md
```

## Detalhes do Hardware

ESP32, microfone INMP441, LED com resistor.

| Componente | Pino |
|---|---|
| LED | GPIO 17 |
| INMP441 WS (LRCLK) | GPIO 25 |
| INMP441 SCK (BCLK) | GPIO 32 |
| INMP441 SD (DOUT) | GPIO 33 |
| INMP441 L/R | GND (canal esquerdo) |
| INMP441 VDD / GND | 3V3 / GND |

## Para gravar o firmware

1. No Arduino IDE, instale o suporte à placa **ESP32**
2. Instale a biblioteca do modelo exportada do Edge Impulse: *Sketch → Include Library → Add .ZIP Library…*
3. Abra `firmware/detector_anomalias.ino`, compile e grave na placa
4. Abra o Monitor Serial a **115200 baud** para ver o log de cada fatia de áudio

## Para rodar o teste

O script pede para você falar perto do microfone, lê a serial do ESP32 e calcula matriz de confusão, métricas e latência por etapa.

```bash
python3 -m venv venv_teste && source venv_teste/bin/activate
pip install pyserial
python -m serial.tools.list_ports # descobrir a porta (ex.: /dev/ttyUSB0)
cd teste
python script_teste.py --port /dev/ttyUSB0 --counts acende:20,apaga:20,outra:5,ruido:5
```

O Monitor Serial da Arduino IDE deve ser encerrado antes da execução dos testes. Os resultados são salvos em `teste/resultados_teste/<data-hora>/` (`resumo.md`, `resultados.csv` e `serial_log.txt`).

## Modelo

Treinado no Edge Impulse Studio: 3 classes (`acende`, `apaga`, `unknown`), features MFE e CNN 1D.

