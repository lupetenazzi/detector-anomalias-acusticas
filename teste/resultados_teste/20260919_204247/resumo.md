# Resultado do teste manual - Detector de Anomalias Acusticas

- **Data:** 19/09/2026 20:49
- **Porta:** /dev/ttyUSB0
- **Tentativas:** 50
- **Protocolo:** contagem de 3 s, 1.5 s para falar, 2.0 s de observacao, ordem embaralhada

## Matriz de confusao (acao do LED)

| | LED acionou | LED nao acionou |
|---|---:|---:|
| **Deveria acionar** ('acende') | TP = 18 | FN = 2 |
| **Nao deveria acionar** | FP = 5 | TN = 25 |

| Metrica | Valor |
|---|---:|
| Tentativas | 50 |
| Acuracia | 86.0 % |
| Precisao | 78.3 % |
| Recall (sensibilidade) | 90.0 % |
| F1 | 0.837 |
| Taxa de falso positivo | 16.7 % |

## Por classe

| Classe | Tentativas | LED acionou | Taxa de acionamento |
|---|---:|---:|---:|
| acende | 20 | 18 | 90.0 % |
| apaga | 20 | 1 | 5.0 % |
| outra | 5 | 3 | 60.0 % |
| ruido | 5 | 1 | 20.0 % |

## Latencia por etapa (slices recebidos durante o teste)

| Etapa | n | min (ms) | media (ms) | p50 (ms) | p95 (ms) | max (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Fila T1->T2 | 1499 | 0.00 | 0.07 | 0.10 | 0.10 | 0.10 |
| Task 2 (RMS + ZCR) | 1499 | 3.64 | 3.65 | 3.65 | 3.65 | 3.65 |
| Fila T2->T3 | 1499 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| Task 3 - DSP (MFCC) | 1499 | 33.40 | 33.40 | 33.40 | 33.40 | 33.40 |
| Task 3 - Rede neural | 1499 | 75.40 | 75.41 | 75.40 | 75.50 | 75.50 |
| Task 3 - total | 1499 | 109.10 | 109.10 | 109.10 | 109.10 | 109.20 |
| Ponta-a-ponta (buffer cheio -> fim da classificacao) | 1499 | 112.80 | 112.83 | 112.80 | 112.90 | 112.90 |
| Alerta: buffer cheio -> LED (firmware) | 23 | 112.80 | 112.85 | 112.80 | 112.90 | 112.90 |

_Slices em silencio (abaixo do gate de RMS) nao chegam a Task 3 e ficam fora destas estatisticas._
